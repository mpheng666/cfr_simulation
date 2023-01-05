# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cfr_actuation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cfr_actuation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cfr_actuation_FOUND FALSE)
  elseif(NOT cfr_actuation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cfr_actuation_FOUND FALSE)
  endif()
  return()
endif()
set(_cfr_actuation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cfr_actuation_FIND_QUIETLY)
  message(STATUS "Found cfr_actuation: 0.0.0 (${cfr_actuation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cfr_actuation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cfr_actuation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cfr_actuation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${cfr_actuation_DIR}/${_extra}")
endforeach()
