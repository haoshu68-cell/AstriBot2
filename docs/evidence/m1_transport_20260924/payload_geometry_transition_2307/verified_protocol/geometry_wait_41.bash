#!/usr/bin/env bash
set -e
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/payload_geometry_transition_2307/overlay.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=41
export IGN_PARTITION=m1_full_41_geometry_2307 GZ_PARTITION=m1_full_41_geometry_2307
exec python3 /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/test/verify_full_manipulation.py --mode delayed_ledger --operation PICK --domain 41 --executable /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/payload_geometry_transition_2307/native_install/lib/astribot_s1_transport_native/trajectory_executor --physical-fixture /home/yjh/WorkSpace/astribot_sdk_ros2/runs/m1_transport_20260924/continuous_build/full_physical_fixture --ledger-executable /home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_20260923_065115/install/astribot_s1_payload_state/lib/astribot_s1_payload_state/payload_state --output /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/payload_geometry_transition_2307/protocol/geometry_wait_41.json
