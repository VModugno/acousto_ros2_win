cmake_minimum_required(VERSION 3.16)
project(acousto_control_start)

# --- C++ & MSVC runtime -------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(MSVC)
  add_compile_options(/W4)
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
set(EXTRA_GSPAT_HEADERS "${PKG_SRC}/GSPAT_Solver")  # common interface headers if used

# --- vcpkg hints --------------------------------------------------------------
set(VCPKG_ROOT "C:/Users/RoboH/vcpkg" CACHE PATH "Path to your vcpkg root")
set(VCPKG_TRIPLET "x64-windows" CACHE STRING "vcpkg triplet")

# --- Deps: OpenCL & clBLAS from conda; pthread via vcpkg (fallback to .lib) ---
if(WIN32)
  # OpenCL
  find_package(OpenCL QUIET)
  if(NOT OpenCL_FOUND)
    find_library(OPENCL_LIBRARY NAMES OpenCL
      HINTS "$ENV{CONDA_PREFIX}/Library/lib" "$ENV{CONDA_PREFIX}/lib")
    find_path(OPENCL_INCLUDE_DIR CL/cl.h
      HINTS "$ENV{CONDA_PREFIX}/Library/include" "$ENV{CONDA_PREFIX}/include")
    if(NOT OPENCL_LIBRARY OR NOT OPENCL_INCLUDE_DIR)
      message(FATAL_ERROR "OpenCL not found in conda env.")
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
    message(FATAL_ERROR "clBLAS.lib not found in conda env.")
  endif()

  # pthreads (prefer vcpkg)
  set(PTHREAD_LIB_TARGET "")
  find_package(pthreads CONFIG QUIET
               HINTS "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}"
                     "${VCPKG_ROOT}/installed/x64-windows")
  if(TARGET PThreads4W::PThreads4W)
    set(PTHREAD_LIB_TARGET PThreads4W::PThreads4W)
  else()
    find_library(PTHREAD_LIBRARY NAMES pthreadVC3 pthreadVC2 pthread
      HINTS "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/lib"
            "${VCPKG_ROOT}/installed/x64-windows/lib"
            "$ENV{CONDA_PREFIX}/Library/lib" "$ENV{CONDA_PREFIX}/lib")
    if(NOT PTHREAD_LIBRARY)
      message(FATAL_ERROR "pthreads not found (install via vcpkg or pass -DPTHREAD_LIBRARY=...).")
    endif()
    set(PTHREAD_LIB_TARGET "${PTHREAD_LIBRARY}")
  endif()

  set(WS2_32_LIBRARY ws2_32)
else()
  find_package(Threads REQUIRED)
  find_package(OpenCL REQUIRED)
  find_library(CLBLAS_LIBRARY NAMES clBLAS REQUIRED)
endif()

# ===== Build libraries exactly like VS ========================================

# AsierInhoUDP library WITHOUT UDPDriver.cpp (that one is compiled into the node)
add_library(AsierInhoUDP SHARED
  "${PKG_SRC}/AsierInhoUDP/src/AsierInhoUDP.cpp"
  "${PKG_SRC}/AsierInhoUDP/src/AsierInhoUDP_CWrapper.cpp"
  "${PKG_SRC}/AsierInhoUDP/src/AsierInhoUDPImpl.cpp"
)
target_include_directories(AsierInhoUDP PUBLIC
  "${PKG_SRC}/AsierInhoUDP/include"
  "${PKG_SRC}/AsierInhoUDP/src"      # for AsierInhoUDPImpl.h
)
if(WIN32)
  target_link_libraries(AsierInhoUDP PRIVATE ${PTHREAD_LIB_TARGET} ${WS2_32_LIBRARY})
else()
  target_link_libraries(AsierInhoUDP PRIVATE Threads::Threads)
endif()

# GSPAT_SolverV2 library (compile the core .cpp files)
add_library(GSPAT_SolverV2 SHARED
  "${PKG_SRC}/GSPAT_SolverV2/src/GSPAT_SolverV2.cpp"
  "${PKG_SRC}/GSPAT_SolverV2/src/GSPATV2_CWrapper.cpp"
  "${PKG_SRC}/GSPAT_SolverV2/src/HologramProfiler.cpp"
  "${PKG_SRC}/GSPAT_SolverV2/src/HologramSolution.cpp"
  "${PKG_SRC}/GSPAT_SolverV2/src/HologramSolverCL.cpp"
  "${PKG_SRC}/GSPAT_SolverV2/src/SolutionPool.cpp"
)
target_include_directories(GSPAT_SolverV2 PUBLIC
  "${PKG_SRC}/GSPAT_SolverV2/include"
  "${EXTRA_GSPAT_HEADERS}"
)
if(WIN32)
  target_link_libraries(GSPAT_SolverV2 PRIVATE OpenCL::OpenCL ${CLBLAS_LIBRARY} ${PTHREAD_LIB_TARGET})
else()
  target_link_libraries(GSPAT_SolverV2 PRIVATE OpenCL::OpenCL ${CLBLAS_LIBRARY} Threads::Threads)
endif()

# ===== ROS 2 node (replicates WirelessBoard exe composition) ==================
# Compile these vendor sources directly into the node, like VS:
set(EMBEDDED_VENDOR_SRCS
  "${PKG_SRC}/AsierInhoSerial/src/COMToolkit2.cpp"
  "${PKG_SRC}/AsierInhoUDP/src/UDPDriver.cpp"
)

add_executable(acousto_control_node
  "${PKG_SRC}/acousto_control.cpp"
  ${EMBEDDED_VENDOR_SRCS}
)
target_include_directories(acousto_control_node PRIVATE
  "${PKG_SRC}/AsierInhoSerial/src"   # COMToolkit2.h lives here
  "${PKG_SRC}/AsierInhoUDP/include"
  "${PKG_SRC}/AsierInhoUDP/src"      # UDPDriver.h, Impl headers
  "${PKG_SRC}/GSPAT_SolverV2/include"
  "${EXTRA_GSPAT_HEADERS}"
)
ament_target_dependencies(acousto_control_node rclcpp std_msgs)

# Use the PLAIN signature to avoid mixed-signature errors
target_link_libraries(acousto_control_node
  AsierInhoUDP
  GSPAT_SolverV2
  ${PTHREAD_LIB_TARGET}
  ${WS2_32_LIBRARY}
)

# ===== Install ================================================================
install(TARGETS
  AsierInhoUDP
  GSPAT_SolverV2
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(TARGETS acousto_control_node
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Install OpenCL kernels used by the solver
file(GLOB _gspat_kernels "${PKG_SRC}/GSPAT_SolverV2/src/*.cl")
if(_gspat_kernels)
  install(FILES ${_gspat_kernels} DESTINATION bin)
endif()

# Also install runtime DLLs so the node runs without PATH edits
if(WIN32)
  set(_runtime_copy "")
  set(CONDA_BIN "$ENV{CONDA_PREFIX}/Library/bin")
  foreach(_dll IN ITEMS
      "${CONDA_BIN}/clBLAS.dll"
      "${CONDA_BIN}/OpenCL.dll")
    if(EXISTS "${_dll}")
      list(APPEND _runtime_copy "${_dll}")
    endif()
  endforeach()
  if(VCPKG_ROOT AND EXISTS "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/bin")
    file(GLOB _vcpkg_pthr "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/bin/pthreadVC*.dll")
    list(APPEND _runtime_copy ${_vcpkg_pthr})
  endif()
  if(_runtime_copy)
    install(FILES ${_runtime_copy} DESTINATION bin)
  endif()
endif()

ament_package()
