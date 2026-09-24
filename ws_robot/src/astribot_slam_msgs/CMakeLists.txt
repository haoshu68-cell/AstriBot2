cmake_minimum_required(VERSION 3.8)
project(astribot_slam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyframeSubmap.msg"
  "msg/KeyframePoseUpdate.msg"
  "msg/KeyframePoseArray.msg"
  "msg/HeightSliceMaps.msg"
  DEPENDENCIES geometry_msgs sensor_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
