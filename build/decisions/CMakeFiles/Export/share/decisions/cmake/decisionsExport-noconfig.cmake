#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "decisions::decisions" for configuration ""
set_property(TARGET decisions::decisions APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(decisions::decisions PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdecisions.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS decisions::decisions )
list(APPEND _IMPORT_CHECK_FILES_FOR_decisions::decisions "${_IMPORT_PREFIX}/lib/libdecisions.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
