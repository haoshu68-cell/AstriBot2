#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pinocchio::pinocchio_default" for configuration "Release"
set_property(TARGET pinocchio::pinocchio_default APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pinocchio::pinocchio_default PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpinocchio_default.so.3.7.0"
  IMPORTED_SONAME_RELEASE "libpinocchio_default.so.3.7.0"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio_default )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio_default "${_IMPORT_PREFIX}/lib/libpinocchio_default.so.3.7.0" )

# Import target "pinocchio::pinocchio_visualizers" for configuration "Release"
set_property(TARGET pinocchio::pinocchio_visualizers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pinocchio::pinocchio_visualizers PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpinocchio_visualizers.so.3.7.0"
  IMPORTED_SONAME_RELEASE "libpinocchio_visualizers.so.3.7.0"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio_visualizers )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio_visualizers "${_IMPORT_PREFIX}/lib/libpinocchio_visualizers.so.3.7.0" )

# Import target "pinocchio::pinocchio_parsers" for configuration "Release"
set_property(TARGET pinocchio::pinocchio_parsers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pinocchio::pinocchio_parsers PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpinocchio_parsers.so.3.7.0"
  IMPORTED_SONAME_RELEASE "libpinocchio_parsers.so.3.7.0"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio_parsers )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio_parsers "${_IMPORT_PREFIX}/lib/libpinocchio_parsers.so.3.7.0" )

# Import target "pinocchio::pinocchio_python_parser" for configuration "Release"
set_property(TARGET pinocchio::pinocchio_python_parser APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pinocchio::pinocchio_python_parser PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpinocchio_python_parser.so.3.7.0"
  IMPORTED_SONAME_RELEASE "libpinocchio_python_parser.so.3.7.0"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio_python_parser )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio_python_parser "${_IMPORT_PREFIX}/lib/libpinocchio_python_parser.so.3.7.0" )

# Import target "pinocchio::pinocchio_pywrap_default" for configuration "Release"
set_property(TARGET pinocchio::pinocchio_pywrap_default APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pinocchio::pinocchio_pywrap_default PROPERTIES
  IMPORTED_LOCATION_RELEASE "/opt/astribot_ros/third_pkg/pinocchio/lib/python3.10/site-packages/pinocchio/pinocchio_pywrap_default.cpython-310-x86_64-linux-gnu.so.3.7.0"
  IMPORTED_SONAME_RELEASE "pinocchio_pywrap_default.cpython-310-x86_64-linux-gnu.so.3.7.0"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio_pywrap_default )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio_pywrap_default "/opt/astribot_ros/third_pkg/pinocchio/lib/python3.10/site-packages/pinocchio/pinocchio_pywrap_default.cpython-310-x86_64-linux-gnu.so.3.7.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
