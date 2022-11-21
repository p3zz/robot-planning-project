cmake_minimum_required(VERSION 3.8)
project(lib_dubins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)
 
add_library(lib_dubins src/dubins.cpp)
 
ament_export_targets(lib_dubins HAS_LIBRARY_TARGET)
 
install(
  DIRECTORY include/lib_dubins
  DESTINATION include
)
 
install(
  TARGETS lib_dubins
  EXPORT lib_dubins
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
