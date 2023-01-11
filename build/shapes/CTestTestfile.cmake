# CMake generated Testfile for 
# Source directory: /home/p3zz/Documents/uni/robot-planning-project/shapes
# Build directory: /home/p3zz/Documents/uni/robot-planning-project/build/shapes
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_shapes "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/p3zz/Documents/uni/robot-planning-project/build/shapes/test_results/shapes/test_shapes.gtest.xml" "--package-name" "shapes" "--output-file" "/home/p3zz/Documents/uni/robot-planning-project/build/shapes/ament_cmake_gtest/test_shapes.txt" "--command" "/home/p3zz/Documents/uni/robot-planning-project/build/shapes/test_shapes" "--gtest_output=xml:/home/p3zz/Documents/uni/robot-planning-project/build/shapes/test_results/shapes/test_shapes.gtest.xml")
set_tests_properties(test_shapes PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/p3zz/Documents/uni/robot-planning-project/build/shapes/test_shapes" TIMEOUT "60" WORKING_DIRECTORY "/home/p3zz/Documents/uni/robot-planning-project/build/shapes" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/p3zz/Documents/uni/robot-planning-project/shapes/CMakeLists.txt;45;ament_add_gtest;/home/p3zz/Documents/uni/robot-planning-project/shapes/CMakeLists.txt;0;")
subdirs("gtest")
