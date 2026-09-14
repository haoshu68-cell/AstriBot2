cmake_minimum_required(VERSION 3.16)
project(astribot_s1_autonomy)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(astribot_autonomy_core REQUIRED)
find_package(astribot_s1_perception_components REQUIRED)
find_package(astribot_s1_exploration REQUIRED)
# Compatibility headers are exported transitively, without recompiling any implementation.
ament_export_dependencies(astribot_autonomy_core astribot_s1_perception_components astribot_s1_exploration)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES
  "${astribot_s1_perception_components_DIR}/../config/pointcloud_slice_scan_params.yaml"
  "${astribot_s1_perception_components_DIR}/../config/livox_custom_to_pc2_params.yaml"
  "${astribot_s1_exploration_DIR}/../config/frontier_explorer_params.yaml"
  "${astribot_s1_exploration_DIR}/../config/exploration_coordinator_params.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/pointcloud_slice_scan_node scripts/livox_custom_to_pc2_node
  scripts/frontier_explorer_node scripts/exploration_coordinator_node DESTINATION lib/${PROJECT_NAME})
# Historical component IDs resolve to the new libraries; no duplicate implementation.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/compat_components" CONTENT
"astribot_s1_autonomy::PointcloudSliceScanNode;$<TARGET_FILE:astribot_s1_perception_components::astribot_s1_perception_components_components>
astribot_s1_autonomy::FrontierExplorerNode;$<TARGET_FILE:astribot_s1_exploration::astribot_s1_exploration_components>
astribot_s1_autonomy::ExplorationCoordinatorNode;$<TARGET_FILE:astribot_s1_exploration::astribot_s1_exploration_components>")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/compat_components"
  DESTINATION share/ament_index/resource_index/rclcpp_components RENAME ${PROJECT_NAME})
ament_package()
