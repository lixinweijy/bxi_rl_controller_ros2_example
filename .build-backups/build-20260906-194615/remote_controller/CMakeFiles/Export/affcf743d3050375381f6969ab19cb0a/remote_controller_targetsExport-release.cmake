#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "remote_controller::remote_controller_core" for configuration "Release"
set_property(TARGET remote_controller::remote_controller_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(remote_controller::remote_controller_core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libremote_controller_core.a"
  )

list(APPEND _cmake_import_check_targets remote_controller::remote_controller_core )
list(APPEND _cmake_import_check_files_for_remote_controller::remote_controller_core "${_IMPORT_PREFIX}/lib/libremote_controller_core.a" )

# Import target "remote_controller::remote_controller" for configuration "Release"
set_property(TARGET remote_controller::remote_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(remote_controller::remote_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/remote_controller/remote_controller"
  )

list(APPEND _cmake_import_check_targets remote_controller::remote_controller )
list(APPEND _cmake_import_check_files_for_remote_controller::remote_controller "${_IMPORT_PREFIX}/lib/remote_controller/remote_controller" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
