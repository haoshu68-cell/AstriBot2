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
find_package(sensor_msgs REQUIRED)

set(ASTRIBOT_MANIPULATION_DEPS
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
)

add_library(${PROJECT_NAME}_core SHARED
  src/error_codes.cpp
  src/singularity_monitor.cpp
  src/collision_validator.cpp
  src/closed_chain_constraint.cpp
  src/trajectory_metrics.cpp
  src/trajectory_time_optimizer.cpp
  src/dual_arm_planner.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
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
  ${EIGEN3_INCLUDE_DIRS}
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
target_include_directories(${PROJECT_NAME}_gripper SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
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
target_link_libraries(self_collision_pair_generator ${PROJECT_NAME}_core)
ament_target_dependencies(self_collision_pair_generator ${ASTRIBOT_MANIPULATION_DEPS})

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
  TARGETS planning_demo_node self_collision_pair_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_gripper)
ament_export_dependencies(${ASTRIBOT_MANIPULATION_DEPS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
