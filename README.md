acousto_ros2_win — Windows build guide (RoboStack + vcpkg)

This project builds a ROS 2 node (acousto_control_node) on Windows using a RoboStack (conda/mamba) environment.
It mirrors the original Visual Studio solution:

• AsierInhoUDP → built as a DLL (excluding UDPDriver.cpp, which is compiled into the node)
• GSPAT_SolverV2 → built as a DLL (uses OpenCL kernels)
• COMToolkit2.cpp (from AsierInhoSerial) and UDPDriver.cpp (from AsierInhoUDP) → compiled directly into the node
• Linked deps: OpenCL, clBLAS, pthreads, ws2_32
• We build in Release/x64 only.

Prerequisites (once per machine)

Visual Studio 2022 (Community is fine)

Install workload: “Desktop development with C++”

This installs MSVC v143 and a recent Windows 10/11 SDK.

Git

Miniforge / Mambaforge (conda/mamba)

We will use the “Miniforge Prompt” terminal.

(Optional) Ninja build system (we’ll install it into the conda env below).

Create or use a RoboStack ROS 2 environment

If you already have a working RoboStack env, just “conda activate” it and skip to section 2.

Open “Miniforge Prompt”, then:

mamba create -n ros2_env python=3.11
conda activate ros2_env
mamba install -c conda-forge cmake ninja colcon-common-extensions vcstool


Install the ROS 2 packages you need for your distro from RoboStack (Iron/Jazzy/etc.).
(Exact packages vary by distro; follow RoboStack’s instructions for your release.)

Install vcpkg and pthreads (MSVC-friendly)

Open any terminal (PowerShell/CMD/Miniforge Prompt):

git clone https://github.com/microsoft/vcpkg C:\path\to\vcpkg
C:\path\to\vcpkg\bootstrap-vcpkg.bat
C:\path\to\vcpkg\vcpkg.exe install pthreads:x64-windows


Verify pthreads is installed:

C:\path\to\vcpkg\vcpkg.exe list | findstr /I pthread

3) Install OpenCL + clBLAS in the conda env

In Miniforge Prompt, with your env active:

conda activate ros2_env
mamba install -c conda-forge opencl-headers khronos-opencl-icd-loader clblas


Files will be placed under:
%CONDA_PREFIX%\Library\include
%CONDA_PREFIX%\Library\lib
%CONDA_PREFIX%\Library\bin

Get the sources into a ROS 2 workspace

cd C:\ws
mkdir acousto_ws && cd acousto_ws
mkdir src && cd src
git clone https://github.com/VModugno/acousto_ros2_win.git
cd ..


Project layout (high level):
acousto_ros2_win/
CMakeLists.txt
package.xml
src/
acousto_control.cpp (ROS 2 node)
AsierInhoSerial/src/COMToolkit2.cpp ← compiled into the node
AsierInhoUDP/src/UDPDriver.cpp ← compiled into the node
AsierInhoUDP/{include, src/.cpp} ← DLL (without UDPDriver.cpp)
GSPAT_SolverV2/{include, src/.cpp, *.cl}← DLL (OpenCL kernels)

Build (Release, x64)

Always build from a shell that has BOTH your conda env and MSVC loaded.

A) Open Miniforge Prompt and activate the env:

conda activate ros2_env


B) Load the MSVC x64 toolchain into this shell:

call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64


C) Tell CMake where vcpkg lives (specify your path here):

set VCPKG_ROOT=C:path\to\vcpkg


D) From the workspace root (the folder that contains “src\”) (here we assume that all the ros envs are in a folder called ws that contains all the ros envs inside):

cd C:\ws\acousto_ws


(Recommended) Clean previous builds:

rmdir /S /Q build install log 2>nul


E) Build only this package, Release, with vcpkg toolchain (verbose):

colcon build --packages-select acousto_control_start ^
  --event-handlers console_direct+ ^
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release ^
               -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake ^
               -DVCPKG_TARGET_TRIPLET=x64-windows ^
               -DCMAKE_VERBOSE_MAKEFILE=ON ^
               -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE


Notes:

If “ninja” is not found, install it into the env: mamba install -c conda-forge ninja

The toolchain file makes CMake find pthreads from vcpkg automatically.

Run

call install\local_setup.bat
ros2 run acousto_control_start acousto_control_node


If Windows asks for firewall permission (UDP sockets), allow it.

Example to trigger the node from another shell:

ros2 topic pub /start_acousto_control std_msgs/Bool "{data: true}"

7) What the CMake build does 

Builds “AsierInhoUDP” as a DLL WITHOUT UDPDriver.cpp (that file is compiled into the node).

Builds “GSPAT_SolverV2” as a DLL and links against OpenCL, clBLAS, and pthreads.

Compiles “COMToolkit2.cpp” (Serial) and “UDPDriver.cpp” (UDP) directly into the node (matching the original WirelessBoard app).

Links the node against: AsierInhoUDP, GSPAT_SolverV2, ws2_32, pthreads.

Installs the solver’s .cl OpenCL kernels and required runtime DLLs into install\bin so you don’t need to edit PATH.

Troubleshooting

Missing DLL at runtime:
• Ensure that after compiling the ros package these exist in install\bin: AsierInhoUDP.dll, GSPAT_SolverV2.dll, pthreadVC*.dll, OpenCL.dll, clBLAS.dll
• If not, re-check sections 2 and 3 and rebuild (or copy the DLLs manually to install\bin).

Mixed Debug/Release CRT:
• Build Release everywhere. Debug DLLs depend on MSVCP140D.dll / ucrtbased.dll and will cause loader issues.
• The provided CMake sets /MD in Release.

MSVC not actually loaded:
• Run: cl /Bv
• It must print the x64 compiler version. If not, re-run VsDevCmd with -arch=x64.

vcpkg not picked up / pthreads not found:
• Pass the toolchain file exactly as shown in section 5.
• Verify install: C:\Users\RoboH\vcpkg\vcpkg.exe list | findstr /I pthread

Super-verbose CMake package discovery (only for debugging):
• Add to “--cmake-args”: -DCMAKE_FIND_DEBUG_MODE=ON

Clean rebuild:
• rmdir /S /Q build install log
• re-run the colcon build command from section 5.

Verifications

Check a built DLL is x64:

dumpbin /headers install\bin\AsierInhoUDP.dll | findstr /I machine
(expect: machine (x64) / 8664)


Check solver DLL dependencies:

dumpbin /dependents install\bin\GSPAT_SolverV2.dll
(expect: OpenCL.dll, clBLAS.dll, pthreadVC*.dll, plus the MSVC/UCRT DLLs without “D” suffix in Release)

10) Quality-of-life: auto-load MSVC with the env (optional)

Create this file:
%CONDA_PREFIX%\etc\conda\activate.d\vsdevcmd.bat

Contents:

@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul


Now every “conda activate ros2_env” primes MSVC automatically.

Notes and customization

• If your vcpkg path is different, set:
set VCPKG_ROOT=C:\Path\To\vcpkg
and keep -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake

• If you must use a different triplet:
-DVCPKG_TARGET_TRIPLET=x64-windows-static (or another)
Make sure all dependencies are consistent.

• The build expects OpenCL and clBLAS from the conda env. If you prefer system-wide installs, adjust the CMake find hints accordingly (or put their DLLs next to install\bin).

End of README