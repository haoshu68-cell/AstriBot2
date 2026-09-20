cmake_minimum_required(VERSION 3.16)
project(nonhome_planner_benchmark)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
foreach(dep nav2_core nav2_costmap_2d nav2_smac_planner rclcpp rclcpp_lifecycle
    pluginlib tf2_ros tf2_geometry_msgs astribot_s1_robot_geometry)
  find_package(${dep} REQUIRED)
endforeach()
add_executable(planner_benchmark main.cpp)
ament_target_dependencies(planner_benchmark nav2_core nav2_costmap_2d nav2_smac_planner
  rclcpp rclcpp_lifecycle pluginlib tf2_ros tf2_geometry_msgs astribot_s1_robot_geometry)
