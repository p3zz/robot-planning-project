cmake_minimum_required(VERSION 3.8)
project(shapes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)

include_directories(include)

add_library(shapes src/shapes.cpp)

ament_export_targets(shapes HAS_LIBRARY_TARGET)

ament_target_dependencies(shapes utils)

install(
  DIRECTORY include/shapes
  DESTINATION include
)

install(
  TARGETS shapes
  EXPORT shapes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # gtest section
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES test/segment.cpp test/circle.cpp test/polygon.cpp test/main.cpp)
  ament_add_gtest(test_shapes ${TEST_FILES})
  
  target_link_libraries(test_shapes shapes)
  
  install(TARGETS test_shapes DESTINATION lib/shapes)

endif()

ament_package()
