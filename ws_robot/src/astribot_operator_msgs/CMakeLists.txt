cmake_minimum_required(VERSION 3.16)
project(astribot_operator_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OperatorCommand.srv" "srv/ExplorationCommand.srv" "srv/StartLoopRoute.srv" "srv/CancelLoopRoute.srv" DEPENDENCIES geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
