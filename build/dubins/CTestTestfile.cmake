# CMake generated Testfile for 
# Source directory: /home/p3zz/Documents/uni/robot-planning-project/dubins
# Build directory: /home/p3zz/Documents/uni/robot-planning-project/build/dubins
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_dubins "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/p3zz/Documents/uni/robot-planning-project/build/dubins/test_results/dubins/test_dubins.gtest.xml" "--package-name" "dubins" "--output-file" "/home/p3zz/Documents/uni/robot-planning-project/build/dubins/ament_cmake_gtest/test_dubins.txt" "--command" "/home/p3zz/Documents/uni/robot-planning-project/build/dubins/test_dubins" "--gtest_output=xml:/home/p3zz/Documents/uni/robot-planning-project/build/dubins/test_results/dubins/test_dubins.gtest.xml")
set_tests_properties(test_dubins PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/p3zz/Documents/uni/robot-planning-project/build/dubins/test_dubins" TIMEOUT "60" WORKING_DIRECTORY "/home/p3zz/Documents/uni/robot-planning-project/build/dubins" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/p3zz/Documents/uni/robot-planning-project/dubins/CMakeLists.txt;46;ament_add_gtest;/home/p3zz/Documents/uni/robot-planning-project/dubins/CMakeLists.txt;0;")
subdirs("gtest")
