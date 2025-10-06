#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "path_searching::path_searching" for configuration ""
set_property(TARGET path_searching::path_searching APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(path_searching::path_searching PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpath_searching.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS path_searching::path_searching )
list(APPEND _IMPORT_CHECK_FILES_FOR_path_searching::path_searching "${_IMPORT_PREFIX}/lib/libpath_searching.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
