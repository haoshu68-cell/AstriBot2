#!/usr/bin/env bash
# Isolated build + synthetic ROS / local model tests. Does not start a robot stack.
set -e
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_root="${VISION_BUILD_ROOT:-/tmp/vision_cpp_build}"
if [[ -z "${VISION_TEST_MODELS:-}" ]]; then
  echo 'Set VISION_TEST_MODELS to the prepared ONNX + labels.txt + bus.jpg directory.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
cd "$repo_dir"
colcon --log-base "$build_root/log" build --base-paths ws_robot/src \
  --build-base "$build_root/build" --install-base "$build_root/install" \
  --packages-select astribot_perception_msgs --parallel-workers 1
source "$build_root/install/local_setup.bash"
colcon --log-base "$build_root/log" build --base-paths ws_robot/src \
  --build-base "$build_root/build" --install-base "$build_root/install" \
  --packages-select astribot_s1_perception_components astribot_s1_manipulation \
  --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON \
  "-Dastribot_perception_msgs_DIR=$build_root/install/astribot_perception_msgs/share/astribot_perception_msgs/cmake"
source "$build_root/install/local_setup.bash"
export ROS_DOMAIN_ID="${VISION_TEST_DOMAIN:-181}"
export ROS_LOCALHOST_ONLY=1
ctest --test-dir "$build_root/build/astribot_s1_perception_components" --output-on-failure
ctest --test-dir "$build_root/build/astribot_s1_manipulation" --output-on-failure
python3 tools/vision/verify_vision_launch.py
