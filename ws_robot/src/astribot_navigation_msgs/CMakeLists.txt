cmake_minimum_required(VERSION 3.8)
project(astribot_navigation_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/RobotEnvelope.msg" "srv/SetRobotEnvelope.srv" "msg/SensorHealth.msg" "msg/SensorHealthArray.msg" "msg/WorldVersion.msg" "msg/NavigationPolicyStatus.msg" "msg/NavigationExecutionStatus.msg" "msg/MotionConstraint.msg" "msg/CorridorAlignment.msg" "msg/PathRisk.msg" "srv/PlanCandidate.srv" "srv/ResolveRoute.srv"
  "msg/SocialAgent.msg" "msg/SocialAgentArray.msg" "msg/SocialObservationStatus.msg"
  "msg/EnvelopeSlice.msg" "msg/RobotGeometryState.msg" "msg/NavigationEnvelopeV2.msg"
  "msg/EnvelopeApplyStatus.msg" "msg/ArmHoldStatus.msg" "msg/PassageAssessment.msg"
  "srv/SetFixedEnvelope.srv" "srv/ReserveArmMotion.srv" "srv/AssessNavigationStart.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs sensor_msgs trajectory_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
