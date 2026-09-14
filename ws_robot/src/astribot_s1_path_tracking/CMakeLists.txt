cmake_minimum_required(VERSION 3.8)
project(astribot_s1_path_tracking)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
foreach(dependency astribot_navigation_msgs nav2_controller nav2_behavior_tree behaviortree_cpp_v3 nav2_msgs std_msgs geometry_msgs nav2_core nav2_costmap_2d nav2_mppi_controller nav2_smac_planner nav2_util
    nav_msgs pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/arrival_controller.cpp src/exact_goal_planner.cpp src/policy_progress_checker.cpp
  src/three_phase_controller.cpp src/align_math.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  astribot_navigation_msgs nav2_controller nav2_msgs std_msgs geometry_msgs nav2_core nav2_costmap_2d nav2_smac_planner nav2_util nav_msgs pluginlib
  rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

add_library(astribot_s1_mppi_critics SHARED
  src/curvature_speed_limit_critic.cpp src/curvature_speed_math.cpp)
target_include_directories(astribot_s1_mppi_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(astribot_s1_mppi_critics
  nav2_core nav2_costmap_2d nav2_mppi_controller pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

add_library(astribot_path_guard_bt_node SHARED src/path_guard_bt.cpp)
target_compile_definitions(astribot_path_guard_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(astribot_path_guard_bt_node PRIVATE include)
ament_target_dependencies(astribot_path_guard_bt_node nav2_behavior_tree behaviortree_cpp_v3
  astribot_navigation_msgs nav2_msgs std_msgs nav_msgs geometry_msgs rclcpp)

install(TARGETS ${PROJECT_NAME} astribot_s1_mppi_critics astribot_path_guard_bt_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml critics.xml README.md DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} astribot_s1_mppi_critics)
ament_export_dependencies(astribot_navigation_msgs nav2_core nav2_costmap_2d geometry_msgs nav_msgs)
ament_package()
