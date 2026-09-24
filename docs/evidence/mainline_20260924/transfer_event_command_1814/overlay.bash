#!/usr/bin/env bash
# Keep the verified navigation/MTC baseline; select the event-command native build.
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_resume_1528/overlay.bash
export AMENT_PREFIX_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_event_command_1814/native_install:"${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_event_command_1814/native_install:"${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_event_command_1814/native_install/lib:"${LD_LIBRARY_PATH:-}"
export PYTHONPATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_event_command_1814/native_install/local/lib/python3.10/dist-packages:"${PYTHONPATH:-}"
