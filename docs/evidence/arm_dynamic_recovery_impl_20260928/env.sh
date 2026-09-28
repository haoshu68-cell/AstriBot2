source /opt/ros/humble/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/task_chain_20260921/moveit_sensor_plugin/install/local_setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/deps/mtc_install/local_setup.bash
source /tmp/astribot_arm_dynamic_recovery/install/local_setup.bash
if test -f /tmp/astribot_arm_dynamic_recovery_impl/install/local_setup.bash; then
  source /tmp/astribot_arm_dynamic_recovery_impl/install/local_setup.bash
fi
