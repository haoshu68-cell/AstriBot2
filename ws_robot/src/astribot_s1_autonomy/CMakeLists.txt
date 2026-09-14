cmake_minimum_required(VERSION 3.8)
project(astribot_s1_autonomy)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

find_path(LIVOX_MSG_INCLUDE_DIR
  NAMES livox_ros_driver2/msg/custom_msg.hpp
  PATH_SUFFIXES include/livox_ros_driver2 include)
find_library(LIVOX_MSG_TYPESUPPORT_CPP
  NAMES livox_ros_driver2__rosidl_typesupport_cpp)
find_library(LIVOX_MSG_GENERATOR_C
  NAMES livox_ros_driver2__rosidl_generator_c)

if(LIVOX_MSG_INCLUDE_DIR AND LIVOX_MSG_TYPESUPPORT_CPP AND LIVOX_MSG_GENERATOR_C)
  set(LIVOX_MSG_AVAILABLE TRUE)
  message(STATUS "找到 livox CustomMsg：将构建 livox_custom_to_pc2_node")
  message(STATUS "  头文件 ${LIVOX_MSG_INCLUDE_DIR}")
else()
  set(LIVOX_MSG_AVAILABLE FALSE)
  message(STATUS "未找到 livox CustomMsg：跳过 livox_custom_to_pc2_node（实机上缺它意味着没有 /scan，也就没有动态避障）")
endif()

set(ASTRIBOT_AUTONOMY_DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  rcl_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  nav2_msgs
  nav2_costmap_2d
  rclcpp_action
  std_srvs
)

add_library(${PROJECT_NAME}_components SHARED
  src/slice_projector.cpp
  src/self_filter.cpp
  src/frontier_search.cpp
  src/path_validator.cpp
  src/costmap_adapter.cpp
  src/escape_logic.cpp
  src/failure_budget.cpp
  src/pointcloud_slice_scan_node.cpp
  src/frontier_explorer_node.cpp
  src/exploration_coordinator_node.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_components SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_components ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_components ${ASTRIBOT_AUTONOMY_DEPS})

rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "astribot_s1_autonomy::PointcloudSliceScanNode"
  "astribot_s1_autonomy::FrontierExplorerNode"
  "astribot_s1_autonomy::ExplorationCoordinatorNode"
)

add_executable(pointcloud_slice_scan_node src/pointcloud_slice_scan_main.cpp)
target_link_libraries(pointcloud_slice_scan_node ${PROJECT_NAME}_components)
ament_target_dependencies(pointcloud_slice_scan_node ${ASTRIBOT_AUTONOMY_DEPS})

add_executable(frontier_explorer_node src/frontier_explorer_main.cpp)
target_link_libraries(frontier_explorer_node ${PROJECT_NAME}_components)
ament_target_dependencies(frontier_explorer_node ${ASTRIBOT_AUTONOMY_DEPS})

add_executable(exploration_coordinator_node src/exploration_coordinator_main.cpp)
target_link_libraries(exploration_coordinator_node ${PROJECT_NAME}_components)
ament_target_dependencies(exploration_coordinator_node ${ASTRIBOT_AUTONOMY_DEPS})

if(LIVOX_MSG_AVAILABLE)
  add_executable(livox_custom_to_pc2_node
    src/livox_custom_to_pc2_main.cpp
    src/livox_custom_to_pc2_node.cpp
    src/livox_custom_convert.cpp)
  target_include_directories(livox_custom_to_pc2_node
    PRIVATE include ${LIVOX_MSG_INCLUDE_DIR})
  ament_target_dependencies(livox_custom_to_pc2_node ${ASTRIBOT_AUTONOMY_DEPS})
  target_link_libraries(livox_custom_to_pc2_node
    ${LIVOX_MSG_TYPESUPPORT_CPP} ${LIVOX_MSG_GENERATOR_C})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS pointcloud_slice_scan_node frontier_explorer_node exploration_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(LIVOX_MSG_AVAILABLE)
  install(
    TARGETS livox_custom_to_pc2_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_components)
ament_export_dependencies(${ASTRIBOT_AUTONOMY_DEPS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
