cmake_minimum_required(VERSION 3.16)
project(astribot_s1_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(astribot_eigen_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ompl REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(astribot_transport_msgs REQUIRED)
find_package(astribot_perception_msgs REQUIRED)

set(ASTRIBOT_MANIPULATION_DEPS
  astribot_eigen_vendor
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  joint_trajectory_controller
  tf2
  tf2_eigen
  tf2_geometry_msgs
)

add_library(${PROJECT_NAME}_core SHARED
  src/error_codes.cpp
  src/singularity_monitor.cpp
  src/collision_validator.cpp
  src/controller_trajectory.cpp
  src/closed_chain_constraint.cpp
  src/trajectory_metrics.cpp
  src/trajectory_time_optimizer.cpp
  src/dual_arm_planner.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core ${ASTRIBOT_MANIPULATION_DEPS})

add_library(${PROJECT_NAME}_ompl_extension SHARED
  src/ompl_planner_extension.cpp
)

target_include_directories(${PROJECT_NAME}_ompl_extension PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_ompl_extension SYSTEM PUBLIC
  ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_ompl_extension
  ${OMPL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_ompl_extension
  ${ASTRIBOT_MANIPULATION_DEPS}
  moveit_planners_ompl
)
pluginlib_export_plugin_description_file(
  moveit_core ompl_planner_extension_plugin_description.xml)

add_library(${PROJECT_NAME}_gripper SHARED
  src/gripper_commander.cpp
)
target_include_directories(${PROJECT_NAME}_gripper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_gripper ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_gripper
  ${ASTRIBOT_MANIPULATION_DEPS}
  rclcpp_action
  control_msgs
  sensor_msgs
)

add_executable(planning_demo_node src/planning_demo_node.cpp)
target_link_libraries(planning_demo_node ${PROJECT_NAME}_core ${PROJECT_NAME}_gripper)
ament_target_dependencies(planning_demo_node
  ${ASTRIBOT_MANIPULATION_DEPS}
  rclcpp_action
  nav2_msgs
  tf2_ros
  control_msgs
  sensor_msgs
)

add_executable(self_collision_pair_generator src/self_collision_pair_generator.cpp)
add_executable(transport_skill_planner src/transport_skill_planner.cpp)
target_link_libraries(transport_skill_planner ${PROJECT_NAME}_core ${PROJECT_NAME}_gripper)
ament_target_dependencies(transport_skill_planner
  ${ASTRIBOT_MANIPULATION_DEPS} astribot_transport_msgs control_msgs sensor_msgs rclcpp_action)
astribot_target_eigen(transport_skill_planner)
target_link_libraries(self_collision_pair_generator ${PROJECT_NAME}_core)
ament_target_dependencies(self_collision_pair_generator ${ASTRIBOT_MANIPULATION_DEPS})

add_executable(grasp_candidate_gate_node src/grasp_candidate_gate_node.cpp)
ament_target_dependencies(grasp_candidate_gate_node
  rclcpp geometry_msgs astribot_perception_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ompl_extension ${PROJECT_NAME}_gripper
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS planning_demo_node self_collision_pair_generator transport_skill_planner
  grasp_candidate_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_gripper)
ament_export_dependencies(astribot_eigen_vendor ${ASTRIBOT_MANIPULATION_DEPS} astribot_perception_msgs)
ament_export_targets(export_${PROJECT_NAME})

astribot_target_eigen(${PROJECT_NAME}_core)
astribot_target_eigen(${PROJECT_NAME}_ompl_extension)
astribot_target_eigen(${PROJECT_NAME}_gripper)
astribot_target_eigen(planning_demo_node)
astribot_target_eigen(self_collision_pair_generator)
astribot_target_eigen(grasp_candidate_gate_node)
find_package(moveit_ros_move_group REQUIRED)
add_library(astribot_cancellable_execution SHARED src/cancellable_execution.cpp)
ament_target_dependencies(astribot_cancellable_execution rclcpp rclcpp_action pluginlib moveit_ros_move_group moveit_ros_planning moveit_ros_planning_interface moveit_msgs)
astribot_target_eigen(astribot_cancellable_execution)
pluginlib_export_plugin_description_file(moveit_ros_move_group cancellable_execution_plugins.xml)
install(TARGETS astribot_cancellable_execution LIBRARY DESTINATION lib)
add_library(astribot_owned_trajectory_controller SHARED src/owned_trajectory_controller.cpp)
target_include_directories(astribot_owned_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(astribot_owned_trajectory_controller
  joint_trajectory_controller astribot_transport_msgs pluginlib)
pluginlib_export_plugin_description_file(controller_interface owned_trajectory_controller_plugins.xml)
install(TARGETS astribot_owned_trajectory_controller LIBRARY DESTINATION lib)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(controller_trajectory_test test/controller_trajectory_test.cpp)
  target_link_libraries(controller_trajectory_test ${PROJECT_NAME}_core)
  ament_target_dependencies(controller_trajectory_test ${ASTRIBOT_MANIPULATION_DEPS})
  astribot_target_eigen(controller_trajectory_test)
  ament_add_gtest(controller_stop_test test/controller_stop_test.cpp TIMEOUT 30)
  ament_target_dependencies(controller_stop_test joint_trajectory_controller rclcpp_action)
  ament_add_gtest(owned_controller_test test/owned_controller_test.cpp TIMEOUT 30)
  target_link_libraries(owned_controller_test astribot_owned_trajectory_controller)
  ament_target_dependencies(owned_controller_test joint_trajectory_controller astribot_transport_msgs rclcpp_action)
  ament_add_gtest(grasp_candidate_gate_test test_grasp_candidate_gate.cpp)
  ament_target_dependencies(grasp_candidate_gate_test rclcpp geometry_msgs astribot_perception_msgs)
  ament_add_gtest(gripper_planning_only_test test_gripper_planning_only.cpp)
  target_link_libraries(gripper_planning_only_test ${PROJECT_NAME}_gripper)
  ament_target_dependencies(gripper_planning_only_test ${ASTRIBOT_MANIPULATION_DEPS} rclcpp_action control_msgs sensor_msgs)
  astribot_target_eigen(gripper_planning_only_test)
endif()
ament_package()
