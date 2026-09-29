# Read-only environment composition for the combined parent candidate.
source /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/nav_boundary/overlay.bash
for _transfer_candidate_prefix in \
  /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_resume_1528/mtc_install \
  /home/yjh/WorkSpace/astribot_sdk_ros2/runs/m1_transport_20260924/continuous_install
do
  export AMENT_PREFIX_PATH="$_transfer_candidate_prefix:$AMENT_PREFIX_PATH"
  export CMAKE_PREFIX_PATH="$_transfer_candidate_prefix:$CMAKE_PREFIX_PATH"
  export LD_LIBRARY_PATH="$_transfer_candidate_prefix/lib:$LD_LIBRARY_PATH"
  export PYTHONPATH="$_transfer_candidate_prefix/local/lib/python3.10/dist-packages:$PYTHONPATH"
done
unset _transfer_candidate_prefix
