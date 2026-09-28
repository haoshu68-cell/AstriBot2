cmake_minimum_required(VERSION 3.16)
project(astribot_transport_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/PlanSkill.srv" "msg/ManipulationStage.msg" "action/PlanManipulation.action"
  "srv/SetExecutionGuard.srv" "srv/RevalidateManipulation.srv" "srv/RevalidatePayloadTransition.srv"
  "msg/ExecutionGuardStatus.msg" "msg/ExecutionHeartbeat.msg" "msg/ObservedOctomap.msg"
  "msg/ControllerTrajectoryStart.msg"
  DEPENDENCIES geometry_msgs moveit_msgs builtin_interfaces std_msgs octomap_msgs trajectory_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
