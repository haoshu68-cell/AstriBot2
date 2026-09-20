cmake_minimum_required(VERSION 3.16)
project(astribot_eigen_vendor VERSION 3.4.0 LANGUAGES NONE)

option(ASTRIBOT_EIGEN_STANDALONE "Install Eigen before building non-ROS dependencies" OFF)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/verify_source.py"
  RESULT_VARIABLE _astribot_eigen_verified)
if(NOT _astribot_eigen_verified EQUAL 0)
  message(FATAL_ERROR "Pinned Eigen source validation failed")
endif()
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/Eigen3Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/Eigen3Config.cmake"
  INSTALL_DESTINATION share/eigen3/cmake)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/Eigen3ConfigVersion.cmake"
  VERSION 3.4.0 COMPATIBILITY SameMajorVersion ARCH_INDEPENDENT)
install(DIRECTORY vendor/eigen/Eigen vendor/eigen/unsupported DESTINATION include/eigen3)
install(FILES vendor/eigen/signature_of_eigen3_matrix_library DESTINATION include/eigen3)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/Eigen3Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/Eigen3ConfigVersion.cmake" DESTINATION share/eigen3/cmake)
install(DIRECTORY vendor/eigen/ DESTINATION share/${PROJECT_NAME}/licenses
  FILES_MATCHING PATTERN "COPYING.*" PATTERN "Eigen" EXCLUDE PATTERN "unsupported" EXCLUDE)
install(FILES vendor.lock.json README.md DESTINATION share/${PROJECT_NAME})

if(NOT ASTRIBOT_EIGEN_STANDALONE)
  find_package(ament_cmake REQUIRED)
  ament_package(CONFIG_EXTRAS cmake/astribot_eigen_vendor-extras.cmake)
endif()
