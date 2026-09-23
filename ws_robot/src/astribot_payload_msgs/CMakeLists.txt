cmake_minimum_required(VERSION 3.8)
project(astribot_payload_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AttachmentObservation.msg" "msg/AttachmentState.msg"
  DEPENDENCIES builtin_interfaces moveit_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
