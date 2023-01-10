#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cfr_manager::cfr_manager" for configuration "Debug"
set_property(TARGET cfr_manager::cfr_manager APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(cfr_manager::cfr_manager PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libcfr_manager.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS cfr_manager::cfr_manager )
list(APPEND _IMPORT_CHECK_FILES_FOR_cfr_manager::cfr_manager "${_IMPORT_PREFIX}/lib/libcfr_manager.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
