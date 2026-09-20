# Resolve before PCL/MoveIt/GTSAM so their dependency discovery uses this copy.
get_filename_component(_astribot_eigen_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(_astribot_eigen_headers "${_astribot_eigen_prefix}/include/eigen3")
if(TARGET Eigen3::Eigen)
  get_target_property(_astribot_eigen_existing Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT "${_astribot_eigen_existing}" STREQUAL "${_astribot_eigen_headers}")
    message(FATAL_ERROR "Foreign Eigen target already loaded: ${_astribot_eigen_existing}. Find astribot_eigen_vendor before other dependencies.")
  endif()
endif()
set(Eigen3_DIR "${_astribot_eigen_prefix}/share/eigen3/cmake" CACHE PATH "Repository Eigen" FORCE)
find_package(Eigen3 3.4.0 EXACT REQUIRED CONFIG PATHS "${Eigen3_DIR}" NO_DEFAULT_PATH)
# PCL's FindEigen module also uses this legacy cache key.
set(EIGEN_INCLUDE_DIR "${_astribot_eigen_headers}" CACHE PATH "Repository Eigen" FORCE)
set(EIGEN3_INCLUDE_DIR "${_astribot_eigen_headers}" CACHE PATH "Repository Eigen" FORCE)
function(astribot_target_eigen target)
  get_target_property(_headers Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  target_link_libraries(${target} Eigen3::Eigen)
  # Some binary dependencies export /usr/include/eigen3; prefer our headers
  # only on targets which explicitly opt into this dependency.
  target_include_directories(${target} SYSTEM BEFORE PRIVATE "${_headers}")
endfunction()
