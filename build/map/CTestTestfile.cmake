# CMake generated Testfile for 
# Source directory: /home/p3zz/Documents/uni/robot-planning-project/map
# Build directory: /home/p3zz/Documents/uni/robot-planning-project/build/map
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_map "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/p3zz/Documents/uni/robot-planning-project/build/map/test_results/map/test_map.gtest.xml" "--package-name" "map" "--output-file" "/home/p3zz/Documents/uni/robot-planning-project/build/map/ament_cmake_gtest/test_map.txt" "--command" "/home/p3zz/Documents/uni/robot-planning-project/build/map/test_map" "--gtest_output=xml:/home/p3zz/Documents/uni/robot-planning-project/build/map/test_results/map/test_map.gtest.xml")
set_tests_properties(test_map PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/p3zz/Documents/uni/robot-planning-project/build/map/test_map" TIMEOUT "60" WORKING_DIRECTORY "/home/p3zz/Documents/uni/robot-planning-project/build/map" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/p3zz/Documents/uni/robot-planning-project/map/CMakeLists.txt;46;ament_add_gtest;/home/p3zz/Documents/uni/robot-planning-project/map/CMakeLists.txt;0;")
subdirs("gtest")
