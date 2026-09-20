cmake_minimum_required(VERSION 3.16)
project(astribot_s1_slam)
cmake_policy(SET CMP0074 NEW)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # project() may already have created an empty cache entry. Replace that entry
  # while preserving an explicitly selected configuration such as Debug.
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(astribot_eigen_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(astribot_slam_msgs REQUIRED)

find_package(astribot_autonomy_core REQUIRED)
find_package(Eigen3 3.4.0 EXACT REQUIRED CONFIG)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)
# Used to decode General.image_topic (sensor_msgs/CompressedImage) and write
# the matched frames out as PNG under <save_path>/<mapname>/images/.
find_package(OpenCV REQUIRED)

find_package(GTSAM REQUIRED CONFIG)

include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES gtsam Eigen3::Eigen)
unset(ASTRIBOT_GTSAM_SYSTEM_EIGEN CACHE)
check_cxx_source_compiles("#include <gtsam/base/Matrix.h>
#ifndef GTSAM_USE_SYSTEM_EIGEN
#error GTSAM must be built with GTSAM_USE_SYSTEM_EIGEN=ON
#endif
int main() { return 0; }" ASTRIBOT_GTSAM_SYSTEM_EIGEN)
unset(CMAKE_REQUIRED_LIBRARIES)
if(NOT ASTRIBOT_GTSAM_SYSTEM_EIGEN)
  message(FATAL_ERROR "GTSAM/Eigen configuration mismatch. Run tools/robot/build_slam_dependencies.sh and select its GTSAM_DIR.")
endif()

# Make sure libgtsam.so can be found at runtime without LD_LIBRARY_PATH tricks.
set(CMAKE_INSTALL_RPATH "$<TARGET_FILE_DIR:gtsam>")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(voxelslam src/voxelslam.cpp src/BTC.cpp)
target_include_directories(voxelslam SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_options(voxelslam PRIVATE -pthread -fexceptions)

ament_target_dependencies(voxelslam
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
  visualization_msgs
  astribot_slam_msgs
)

target_link_libraries(voxelslam astribot_autonomy_core::perception_geometry ${PCL_LIBRARIES} ${OpenCV_LIBS} gtsam Eigen3::Eigen spdlog::spdlog)
# DT_RPATH also resolves GTSAM's indirect METIS dependency in its install prefix.
target_link_options(voxelslam PRIVATE -pthread "-Wl,--disable-new-dtags")

install(TARGETS voxelslam
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

astribot_target_eigen(voxelslam)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(slam_logging_probe test/logging_probe.cpp)
  target_include_directories(slam_logging_probe PRIVATE src)
  target_link_libraries(slam_logging_probe spdlog::spdlog)
  ament_target_dependencies(slam_logging_probe rclcpp)
  ament_add_pytest_test(slam_logging_contract test/test_logging.py
    ENV SLAM_LOG_PROBE=${CMAKE_CURRENT_BINARY_DIR}/slam_logging_probe
    TIMEOUT 90)
endif()
ament_package()
