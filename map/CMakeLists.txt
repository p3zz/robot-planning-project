cmake_minimum_required(VERSION 3.8)
project(map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)
find_package(shapes REQUIRED)
find_package(dubins REQUIRED)

include_directories(include)
 
add_library(map src/map.cpp)
 
ament_export_targets(map HAS_LIBRARY_TARGET)

ament_target_dependencies(map utils shapes dubins)
 
install(
  DIRECTORY include/map
  DESTINATION include
)
 
install(
  TARGETS map
  EXPORT map
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
