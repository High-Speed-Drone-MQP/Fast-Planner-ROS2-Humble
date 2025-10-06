#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "plan_env::plan_env" for configuration ""
set_property(TARGET plan_env::plan_env APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(plan_env::plan_env PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libplan_env.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS plan_env::plan_env )
list(APPEND _IMPORT_CHECK_FILES_FOR_plan_env::plan_env "${_IMPORT_PREFIX}/lib/libplan_env.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
