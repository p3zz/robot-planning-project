cmake_minimum_required(VERSION 3.8)
project(path_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)

include_directories(include)
 
add_library(path_solver src/path_solver.cpp)
 
ament_export_targets(path_solver HAS_LIBRARY_TARGET)

ament_target_dependencies(path_solver utils)
 
install(
  DIRECTORY include/path_solver
  DESTINATION include
)
 
install(
  TARGETS path_solver
  EXPORT path_solver
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
