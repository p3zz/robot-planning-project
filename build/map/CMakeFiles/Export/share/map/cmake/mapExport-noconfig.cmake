#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "map::map" for configuration ""
set_property(TARGET map::map APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(map::map PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmap.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS map::map )
list(APPEND _IMPORT_CHECK_FILES_FOR_map::map "${_IMPORT_PREFIX}/lib/libmap.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
