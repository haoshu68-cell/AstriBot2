#!/usr/bin/env bash
# From /home/yjh/WorkSpace/astribot_sdk_ros2, using the prepared isolated header overlay.
source runs/normal_grasp_20260923/env.bash
source runs/mainline_20260924/payload_transition/install/local_setup.bash
set -e
export LD_LIBRARY_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/trajectory_time_scaling/mtc_build:/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/trajectory_time_scaling/install/astribot_s1_manipulation/lib:${LD_LIBRARY_PATH:-}
cmake -S ws_robot/src/astribot_s1_transport_mtc -B runs/mainline_20260924/trajectory_time_scaling/mtc_build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/trajectory_time_scaling/install/astribot_s1_transport_mtc -Dastribot_s1_manipulation_DIR=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/trajectory_time_scaling/install/astribot_s1_manipulation/share/astribot_s1_manipulation/cmake -Dastribot_transport_msgs_DIR=/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/payload_transition/install/astribot_transport_msgs/share/astribot_transport_msgs/cmake
cmake --build runs/mainline_20260924/trajectory_time_scaling/mtc_build --parallel 1
ctest --test-dir runs/mainline_20260924/trajectory_time_scaling/mtc_build --output-on-failure
cmake --install runs/mainline_20260924/trajectory_time_scaling/mtc_build
