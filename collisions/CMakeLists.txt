cmake_minimum_required(VERSION 3.8)
project(collisions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(collisions src/collisions.cpp)

ament_export_targets(collisions HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/collisions
  DESTINATION include
)

install(
  TARGETS collisions
  EXPORT collisions
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

  ament_add_gtest(test_collisions test/main.cpp)
  
  target_link_libraries(test_collisions collisions)
  
  install(TARGETS test_collisions DESTINATION lib/collisions)

endif()

ament_package()
