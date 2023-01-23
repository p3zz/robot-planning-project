cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shapes REQUIRED)
find_package(dubins REQUIRED)
find_package(map REQUIRED)
find_package(utils REQUIRED)
find_package(planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs)
find_package(nav_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(graph_msgs REQUIRED)

include_directories(include)

add_executable(navigation "src/path.cpp" src/pose.cpp src/follow_client.cpp src/obstacles.cpp src/gates.cpp src/roadmap.cpp src/walls.cpp src/main.cpp)

ament_target_dependencies(navigation rclcpp rclcpp_action utils shapes dubins map planner geometry_msgs tf2 tf2_ros nav2_msgs nav_msgs obstacles_msgs graph_msgs)

install(
  TARGETS navigation
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
