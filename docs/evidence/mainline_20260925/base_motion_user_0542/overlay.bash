# User-fixed idle hold and explicitly relaxed simulation angular limits.
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/payload_replan_0502/overlay.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/base_motion_user_0542/mtc_install/share/astribot_s1_transport_mtc/local_setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/base_motion_user_0542/native_install/share/astribot_s1_transport_native/local_setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/base_motion_user_0542/chassis_install/share/astribot_s1_chassis_effort_drive_native/local_setup.bash
export AMENT_PREFIX_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/base_motion_user_0542/config_install:"${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/base_motion_user_0542/native_install/lib:/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/base_motion_user_0542/mtc_install/lib:/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260925/base_motion_user_0542/chassis_install/lib:"${LD_LIBRARY_PATH:-}"
