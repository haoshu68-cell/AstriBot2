cmake_minimum_required(VERSION 3.8)
project(astribot_s1_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(astribot_slam_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(height_slice_map_node src/height_slice_map_node.cpp)
target_include_directories(height_slice_map_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(height_slice_map_node ${PCL_LIBRARIES} OpenSSL::Crypto nlohmann_json::nlohmann_json yaml-cpp)
ament_target_dependencies(height_slice_map_node rclcpp ament_index_cpp nav_msgs astribot_slam_msgs)

add_executable(nav_prob_grid_node src/nav_prob_grid_node.cpp)

ament_target_dependencies(nav_prob_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  astribot_slam_msgs
  tf2
  tf2_ros
)

install(TARGETS nav_prob_grid_node height_slice_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(height_slice_grid_test test/height_slice_grid_test.cpp)
  target_include_directories(height_slice_grid_test PRIVATE include)
endif()
ament_package()
