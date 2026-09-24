# Navigation decisions are upstream; no final velocity filter is installed here.
# This overlay does not start ROS or Gazebo and does not enable hardware motion.
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/payload_transition/overlay.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/navigation_install/local_setup.bash
for _nav_boundary_prefix in \
  /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/canonical_scene/install/astribot_s1_transport_mtc \
  /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/install/astribot_s1_path_tracking \
  /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/install/astribot_s1_navigation_policy_native \
  /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/install/astribot_s1_dynamics_coupling \
  /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/install/astribot_s1_task_arbiter_native
do
  export AMENT_PREFIX_PATH="$_nav_boundary_prefix:$AMENT_PREFIX_PATH"
  export CMAKE_PREFIX_PATH="$_nav_boundary_prefix:$CMAKE_PREFIX_PATH"
  export LD_LIBRARY_PATH="$_nav_boundary_prefix/lib:$LD_LIBRARY_PATH"
done
unset _nav_boundary_prefix
export PYTHONPATH="/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/install/astribot_s1_navigation_policy_native/local/lib/python3.10/dist-packages:$PYTHONPATH"
