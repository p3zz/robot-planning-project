cmake_minimum_required(VERSION 3.8)
project(decisions)

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
add_library(decisions src/decisions.cpp)
ament_export_targets(decisions HAS_LIBRARY_TARGET)
ament_target_dependencies(decisions utils map dubins shapes)

install(
  DIRECTORY include/decisions
  DESTINATION include
)
 
install(
  TARGETS decisions
  EXPORT decisions
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
