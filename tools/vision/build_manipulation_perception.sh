#!/usr/bin/env bash
# Build all consumers of the versioned perception interfaces into one overlay.
# No simulation is launched, stopped or cleaned by this script.
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_root="${1:-${repo_dir}/runs/grasp_pose_build}"
if [[ "${build_root}" != /* ]]; then
  echo 'Build output must be an absolute path.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
for underlay in "${repo_dir}/ws_robot/install/local_setup.bash" "${repo_dir}/install/local_setup.bash"; do
  if [[ -f "${underlay}" ]]; then source "${underlay}"; fi
done
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon --log-base "${build_root}/log" build --base-paths "${repo_dir}/ws_robot/src" \
  --packages-select astribot_perception_msgs astribot_object_pose_core \
  astribot_s1_manipulation_perception astribot_s1_perception_components astribot_s1_manipulation astribot_s1_gazebo_bringup \
  --build-base "${build_root}/build" --install-base "${build_root}/install" \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
echo "Perception overlay: ${build_root}/install/local_setup.bash"
echo 'Model preparation is separate: tools/vision/graspnet_setup.sh.'
