cmake_minimum_required(VERSION 3.8)
project(dubins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)
find_package(shapes REQUIRED)

include_directories(include)
 
add_library(dubins src/dubins.cpp)
 
ament_export_targets(dubins HAS_LIBRARY_TARGET)

ament_target_dependencies(dubins utils shapes)

 
install(
  DIRECTORY include/dubins
  DESTINATION include
)
 
install(
  TARGETS dubins
  EXPORT dubins
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

  set(TEST_FILES test/performance.cpp test/collisions.cpp test/main.cpp)
  ament_add_gtest(test_dubins ${TEST_FILES})
  
  target_link_libraries(test_dubins dubins)
  
  install(TARGETS test_dubins DESTINATION lib/dubins)

endif()

ament_package()
