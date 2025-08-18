# acousto_ros2_win


set VCPKG_ROOT=C:\Users\RoboH\vcpkg
colcon build --packages-select acousto_control_start --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release  -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows





colcon build --packages-select acousto_control_start --event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE