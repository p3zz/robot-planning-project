#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "utils::utils" for configuration ""
set_property(TARGET utils::utils APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(utils::utils PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libutils.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS utils::utils )
list(APPEND _IMPORT_CHECK_FILES_FOR_utils::utils "${_IMPORT_PREFIX}/lib/libutils.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
