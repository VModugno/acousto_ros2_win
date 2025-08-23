cmake_minimum_required(VERSION 3.16)
project(acousto_control_start)

# --- C++ & MSVC runtime (Release => /MD, Debug => /MDd) -----------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(MSVC)
  add_compile_options(/W4)
  # Ensure Release uses the non-D CRT (no *D.dll)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS 2 --------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- Layout -------------------------------------------------------------------
set(PKG_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src")
# Some GSPAT headers may live here; keep if your code includes from it
set(EXTRA_GSPAT_HEADERS "${PKG_SRC}/GSPAT_Solver")

# --- vcpkg root/triplet hints (so we don't assume C:/tools/vcpkg) ------------
set(VCPKG_ROOT "C:/Users/RoboH/vcpkg" CACHE PATH "Path to your vcpkg root")
if(NOT VCPKG_ROOT AND DEFINED ENV{VCPKG_ROOT})
  set(VCPKG_ROOT "$ENV{VCPKG_ROOT}")
endif()
set(VCPKG_TRIPLET "$ENV{VCPKG_TARGET_TRIPLET}")
if(NOT VCPKG_TRIPLET OR VCPKG_TRIPLET STREQUAL "")
  set(VCPKG_TRIPLET "x64-windows")
endif()

# --- Third-party deps from conda (OpenCL, clBLAS, pthreads) -------------------
# Conda installs typically land in %CONDA_PREFIX%/Library/{include,lib,bin}
if(WIN32)
  # OpenCL (from khronos-opencl-icd-loader + opencl-headers)
  find_package(OpenCL QUIET)
  if(NOT OpenCL_FOUND)
    # Fallback: construct an imported target from conda paths
    find_library(OPENCL_LIBRARY NAMES OpenCL
      HINTS "$ENV{CONDA_PREFIX}/Library/lib" "$ENV{CONDA_PREFIX}/lib")
    find_path(OPENCL_INCLUDE_DIR CL/cl.h
      HINTS "$ENV{CONDA_PREFIX}/Library/include" "$ENV{CONDA_PREFIX}/include")
    if(NOT OPENCL_LIBRARY OR NOT OPENCL_INCLUDE_DIR)
      message(FATAL_ERROR "OpenCL not found. Install khronos-opencl-icd-loader + opencl-headers in the conda env.")
    endif()
    add_library(OpenCL::OpenCL UNKNOWN IMPORTED)
    set_target_properties(OpenCL::OpenCL PROPERTIES
      IMPORTED_LOCATION "${OPENCL_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${OPENCL_INCLUDE_DIR}")
  endif()

  # clBLAS
  find_library(CLBLAS_LIBRARY NAMES clBLAS
    HINTS "$ENV{CONDA_PREFIX}/Library/lib" "$ENV{CONDA_PREFIX}/lib")
  if(NOT CLBLAS_LIBRARY)
    message(FATAL_ERROR "clBLAS.lib not found. Install clblas in the conda env.")
  endif()

  # --- PTHREADS via vcpkg (or fallback to a .lib) ----------------------------
  set(PTHREAD_LIB_TARGET "")
  set(_VCPKG_HINTS "")
  if(VCPKG_ROOT)
    list(APPEND _VCPKG_HINTS
      "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}"
      "${VCPKG_ROOT}/installed/x64-windows")
  endif()
  # Prefer the vcpkg config package/target
  find_package(pthreads CONFIG QUIET HINTS ${_VCPKG_HINTS})
  if(TARGET PThreads4W::PThreads4W)
    set(PTHREAD_LIB_TARGET PThreads4W::PThreads4W)
  else()
    # Fallback: search for a classic import lib
    find_library(PTHREAD_LIBRARY NAMES pthreadVC3 pthreadVC2 pthread
      HINTS
        "$ENV{CONDA_PREFIX}/Library/lib" "$ENV{CONDA_PREFIX}/lib"
        "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/lib"
        "${VCPKG_ROOT}/installed/x64-windows/lib")
    if(NOT PTHREAD_LIBRARY)
      message(FATAL_ERROR "pthreads not found. Either install via vcpkg (pthreads:x64-windows) and pass -DCMAKE_TOOLCHAIN_FILE=\"<vcpkg>/scripts/buildsystems/vcpkg.cmake\", or pass -DPTHREAD_LIBRARY=<path to pthreadVC*.lib>.")
    endif()
    set(PTHREAD_LIB_TARGET "${PTHREAD_LIBRARY}")
  endif()
  # ---------------------------------------------------------------------------

  # Winsock for UDP
  set(WS2_32_LIBRARY "ws2_32.lib")
else()
  find_package(Threads REQUIRED)
  find_package(OpenCL REQUIRED)
  find_library(CLBLAS_LIBRARY NAMES clBLAS)
  if(NOT CLBLAS_LIBRARY)
    message(FATAL_ERROR "clBLAS not found.")
  endif()
endif()

# --- Build vendor DLLs from source --------------------------------------------
# Note: for MSVC, CMake auto-defines <target>_EXPORTS for SHARED libs (dllexport guards).

# 1) AsierInhoSerial (DLL)
file(GLOB_RECURSE ASIERINHOSERIAL_SOURCES
  "${PKG_SRC}/AsierInhoSerial/src/*.c"
  "${PKG_SRC}/AsierInhoSerial/src/*.cpp")
add_library(AsierInhoSerial SHARED ${ASIERINHOSERIAL_SOURCES})
target_include_directories(AsierInhoSerial PUBLIC
  "${PKG_SRC}/AsierInhoSerial/include")
if(WIN32)
  target_link_libraries(AsierInhoSerial PRIVATE "${PTHREAD_LIB_TARGET}")
else()
  target_link_libraries(AsierInhoSerial PRIVATE Threads::Threads)
endif()

# 2) AsierInhoUDP (DLL)
file(GLOB_RECURSE ASIERINHOUDP_SOURCES
  "${PKG_SRC}/AsierInhoUDP/src/*.c"
  "${PKG_SRC}/AsierInhoUDP/src/*.cpp")
add_library(AsierInhoUDP SHARED ${ASIERINHOUDP_SOURCES})
target_include_directories(AsierInhoUDP PUBLIC
  "${PKG_SRC}/AsierInhoUDP/include")
if(WIN32)
  target_link_libraries(AsierInhoUDP PRIVATE "${PTHREAD_LIB_TARGET}" "${WS2_32_LIBRARY}")
else()
  target_link_libraries(AsierInhoUDP PRIVATE Threads::Threads)
endif()

# 3) GSPAT_SolverV2 (DLL)
file(GLOB_RECURSE GSPATV2_SOURCES
  "${PKG_SRC}/GSPAT_SolverV2/src/*.c"
  "${PKG_SRC}/GSPAT_SolverV2/src/*.cpp")
add_library(GSPAT_SolverV2 SHARED ${GSPATV2_SOURCES})
target_include_directories(GSPAT_SolverV2 PUBLIC
  "${PKG_SRC}/GSPAT_SolverV2/include"
  "${EXTRA_GSPAT_HEADERS}")
if(WIN32)
  target_link_libraries(GSPAT_SolverV2 PRIVATE OpenCL::OpenCL "${CLBLAS_LIBRARY}" "${PTHREAD_LIB_TARGET}")
else()
  target_link_libraries(GSPAT_SolverV2 PRIVATE OpenCL::OpenCL "${CLBLAS_LIBRARY}" Threads::Threads)
endif()

# --- ROS 2 node ---------------------------------------------------------------
add_executable(acousto_control_node "${PKG_SRC}/acousto_control.cpp")
target_include_directories(acousto_control_node PRIVATE
  "${PKG_SRC}/AsierInhoUDP/include"
  "${PKG_SRC}/AsierInhoSerial/include"
  "${PKG_SRC}/GSPAT_SolverV2/include"
  "${EXTRA_GSPAT_HEADERS}")
ament_target_dependencies(acousto_control_node rclcpp std_msgs)
target_link_libraries(acousto_control_node
  AsierInhoUDP
  AsierInhoSerial
  GSPAT_SolverV2)

# --- Install: node + our DLLs -------------------------------------------------
install(TARGETS
  AsierInhoSerial
  AsierInhoUDP
  GSPAT_SolverV2
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(TARGETS acousto_control_node
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Also install runtime DLLs so the node runs without PATH tweaks
if(WIN32)
  set(_runtime_copy "")
  set(CONDA_BIN "$ENV{CONDA_PREFIX}/Library/bin")
  if(EXISTS "${CONDA_BIN}/clBLAS.dll")
    list(APPEND _runtime_copy "${CONDA_BIN}/clBLAS.dll")
  endif()
  if(EXISTS "${CONDA_BIN}/OpenCL.dll")
    list(APPEND _runtime_copy "${CONDA_BIN}/OpenCL.dll")
  endif()
  # Try vcpkg pthread DLLs first, then conda's pthreadVC2.dll
  if(VCPKG_ROOT AND EXISTS "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/bin")
    file(GLOB _vcpkg_pthr "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/bin/pthreadVC*.dll")
    list(APPEND _runtime_copy ${_vcpkg_pthr})
  endif()
  if(EXISTS "${CONDA_BIN}/pthreadVC2.dll")
    list(APPEND _runtime_copy "${CONDA_BIN}/pthreadVC2.dll")
  endif()
  if(_runtime_copy)
    install(FILES ${_runtime_copy} DESTINATION bin)
  endif()
endif()
ament_package()
