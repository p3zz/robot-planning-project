#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "shapes::shapes" for configuration ""
set_property(TARGET shapes::shapes APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(shapes::shapes PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libshapes.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS shapes::shapes )
list(APPEND _IMPORT_CHECK_FILES_FOR_shapes::shapes "${_IMPORT_PREFIX}/lib/libshapes.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
