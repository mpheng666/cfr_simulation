#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cfr_manager::cfr_feedback_broadcastor" for configuration "Debug"
set_property(TARGET cfr_manager::cfr_feedback_broadcastor APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(cfr_manager::cfr_feedback_broadcastor PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libcfr_feedback_broadcastor.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS cfr_manager::cfr_feedback_broadcastor )
list(APPEND _IMPORT_CHECK_FILES_FOR_cfr_manager::cfr_feedback_broadcastor "${_IMPORT_PREFIX}/lib/libcfr_feedback_broadcastor.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
