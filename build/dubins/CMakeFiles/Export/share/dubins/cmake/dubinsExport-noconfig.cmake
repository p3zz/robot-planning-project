#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dubins::dubins" for configuration ""
set_property(TARGET dubins::dubins APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(dubins::dubins PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdubins.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS dubins::dubins )
list(APPEND _IMPORT_CHECK_FILES_FOR_dubins::dubins "${_IMPORT_PREFIX}/lib/libdubins.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
