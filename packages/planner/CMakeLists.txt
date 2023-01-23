cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)
find_package(map REQUIRED)
find_package(dubins REQUIRED)
find_package(shapes REQUIRED)

include_directories(include)
add_library(planner src/planner.cpp)
ament_export_targets(planner HAS_LIBRARY_TARGET)
ament_target_dependencies(planner utils map dubins shapes)

install(
  DIRECTORY include/planner
  DESTINATION include
)
 
install(
  TARGETS planner
  EXPORT planner
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
