cmake_minimum_required(VERSION 3.8)
project(main_tester)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dubins REQUIRED)
find_package(utils REQUIRED)
find_package(map REQUIRED)
find_package(shapes REQUIRED)
find_package(planner REQUIRED)

add_executable(main_tester src/main_tester.cpp)

ament_target_dependencies(main_tester rclcpp nav_msgs geometry_msgs dubins utils map shapes planner)

install(
  TARGETS main_tester
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
