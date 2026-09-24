#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/scene_binding_first_stage/overlay.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/scene_binding_first_stage/capture_install/astribot_s1_transport_native/share/astribot_s1_transport_native/local_setup.bash
export LD_LIBRARY_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/scene_binding_first_stage/capture_install/astribot_s1_transport_native/lib:"${LD_LIBRARY_PATH:-}"
