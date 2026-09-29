#!/usr/bin/env bash
set -euo pipefail
# Run from a checkout of the recorded migration branch. Inputs are read-only.
: "${GEOMETRY_CPP_PREFIX:?Provide the earlier native geometry install prefix}"
: "${NAVIGATION_MSGS_PREFIX:?Provide the generated navigation messages prefix}"
: "${POLICY_OBSERVER_GEOMETRY_BINDING:?Provide the recorded old geometry binding for Python comparison}"
run_dir=${1:?Provide a NEW private build directory}
test ! -e "$run_dir"
mkdir -p "$run_dir"
repo_dir=$(git rev-parse --show-toplevel)
package_dir="$repo_dir/ws_robot/src/astribot_s1_navigation_policy_native"
# ROS setup hooks assume some optional variables can be unset.
set +u
source /opt/ros/humble/setup.bash
source "$NAVIGATION_MSGS_PREFIX/share/astribot_navigation_msgs/local_setup.bash"
set -u
export AMENT_PREFIX_PATH="$POLICY_CONFIG_PREFIX:${AMENT_PREFIX_PATH:-}"
cmake -S "$package_dir" -B "$run_dir/build" \
  -DASTRIBOT_BUILD_PYBIND=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_INSTALL_PREFIX="$run_dir/install" \
  -DCMAKE_PREFIX_PATH="$GEOMETRY_CPP_PREFIX" \
  -Dastribot_navigation_msgs_DIR="$NAVIGATION_MSGS_PREFIX/share/astribot_navigation_msgs/cmake" \
  -DPOLICY_FUSION_GEOMETRY_INCLUDE="$repo_dir/ws_robot/src/astribot_s1_robot_geometry/include"
cmake --build "$run_dir/build" -j2
ctest --test-dir "$run_dir/build" --output-on-failure
cmake --install "$run_dir/build"
export POLICY_OBSERVER_CPP="$run_dir/install/lib/astribot_s1_navigation_policy_native/policy_observer_cpp"
export POLICY_OBSERVER_DOMAIN=164 PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 -m pytest -q -p no:cacheprovider "$package_dir/test/test_policy_observer_ros.py" --basetemp "$run_dir/ros"
POLICY_OBSERVER_DOMAIN=165 /usr/bin/python3 "$package_dir/test/benchmark_policy_observer_ros.py" --output "$run_dir/benchmark" --samples 100 --warmup 20 --pairs 4
