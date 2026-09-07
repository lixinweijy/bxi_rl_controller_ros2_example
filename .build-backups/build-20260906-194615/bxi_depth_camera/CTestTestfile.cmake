# CMake generated Testfile for 
# Source directory: /home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera
# Build directory: /home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_common "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/test_results/bxi_depth_camera/test_common.gtest.xml" "--package-name" "bxi_depth_camera" "--output-file" "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/ament_cmake_gtest/test_common.txt" "--command" "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/test_common" "--gtest_output=xml:/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/test_results/bxi_depth_camera/test_common.gtest.xml")
set_tests_properties(test_common PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera/test_common" TIMEOUT "60" WORKING_DIRECTORY "/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/build/bxi_depth_camera" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/CMakeLists.txt;211;ament_add_gtest;/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_depth_camera/CMakeLists.txt;0;")
subdirs("gtest")
