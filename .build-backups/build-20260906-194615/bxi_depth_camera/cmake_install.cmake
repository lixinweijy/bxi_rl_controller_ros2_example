# Install script for directory: /home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbxi_depth_camera_core.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbxi_depth_camera_core.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbxi_depth_camera_core.so"
         RPATH "$ORIGIN")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/libbxi_depth_camera_core.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbxi_depth_camera_core.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbxi_depth_camera_core.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbxi_depth_camera_core.so"
         OLD_RPATH "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/lib:/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/lib:/opt/ros/jazzy/lib:"
         NEW_RPATH "$ORIGIN")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbxi_depth_camera_core.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras"
         RPATH "$ORIGIN/..")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera" TYPE EXECUTABLE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/cameras")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras"
         OLD_RPATH "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera:/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/lib:/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/lib:/opt/ros/jazzy/lib:"
         NEW_RPATH "$ORIGIN/..")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-inspect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-inspect")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-inspect"
         RPATH "$ORIGIN/..")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera" TYPE EXECUTABLE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/cameras-inspect")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-inspect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-inspect")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-inspect"
         OLD_RPATH "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/lib:/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/lib:"
         NEW_RPATH "$ORIGIN/..")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-inspect")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-validate" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-validate")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-validate"
         RPATH "$ORIGIN/..")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera" TYPE EXECUTABLE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/cameras-validate")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-validate" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-validate")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-validate"
         OLD_RPATH "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/lib:/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/lib:"
         NEW_RPATH "$ORIGIN/..")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-validate")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-fps-test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-fps-test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-fps-test"
         RPATH "$ORIGIN/..")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera" TYPE EXECUTABLE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/cameras-fps-test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-fps-test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-fps-test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-fps-test"
         OLD_RPATH "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/lib:/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/lib:"
         NEW_RPATH "$ORIGIN/..")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera/cameras-fps-test")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/launch" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/launch/cameras.launch.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/README.md")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bxi_depth_camera" TYPE PROGRAM RENAME "install-udev-rules" FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/tools/install_udev_rules.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/lib/" USE_SOURCE_PERMISSIONS)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/lib/" USE_SOURCE_PERMISSIONS)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/vendor/orbbec_sdk_v2" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/shared/" USE_SOURCE_PERMISSIONS)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/vendor/realsense2" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/shared/" USE_SOURCE_PERMISSIONS)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/vendor/licenses/orbbec_sdk_v2" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/orbbec_sdk_v2/licenses/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/vendor/licenses/realsense2" TYPE DIRECTORY FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/realsense2/licenses/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/vendor" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/vendor/cpp/linux-aarch64/MANIFEST.md")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/gtest/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/bxi_depth_camera")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/bxi_depth_camera")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/environment" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/environment" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_index/share/ament_index/resource_index/packages/bxi_depth_camera")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera/cmake" TYPE FILE FILES
    "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_core/bxi_depth_cameraConfig.cmake"
    "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_core/bxi_depth_cameraConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bxi_depth_camera" TYPE FILE FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
