# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_decisions_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED decisions_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(decisions_FOUND FALSE)
  elseif(NOT decisions_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(decisions_FOUND FALSE)
  endif()
  return()
endif()
set(_decisions_CONFIG_INCLUDED TRUE)

# output package information
if(NOT decisions_FIND_QUIETLY)
  message(STATUS "Found decisions: 0.0.0 (${decisions_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'decisions' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${decisions_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(decisions_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${decisions_DIR}/${_extra}")
endforeach()
