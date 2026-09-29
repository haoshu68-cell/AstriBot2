#!/usr/bin/env bash
set -eo pipefail
# Run from the recorded commit, using an unused durable experiment directory.
run_dir=${1:?Provide a NEW absolute experiment directory}
: "${NAVIGATION_MSGS_PREFIX:?Generated navigation message install prefix}"
: "${POLICY_CONFIG_PREFIX:?Existing policy configuration install prefix}"
test ! -e "$run_dir"
mkdir -p "$run_dir"
repo_dir=$(git rev-parse --show-toplevel)
evidence="$repo_dir/docs/evidence/pybind_removal_20260921"
source /opt/ros/humble/setup.bash
source "$NAVIGATION_MSGS_PREFIX/share/astribot_navigation_msgs/local_setup.bash"
export AMENT_PREFIX_PATH="$POLICY_CONFIG_PREFIX:${AMENT_PREFIX_PATH:-}"
mkdir -p "$run_dir/oracle/include/astribot_s1_robot_geometry"
cp "$evidence/geometry/baseline_sources/"*.hpp "$run_dir/oracle/include/astribot_s1_robot_geometry/"
cp "$evidence/geometry/baseline_sources/geometry_native.cpp" "$run_dir/oracle/"
cp "$evidence/policy_json_integer_runtime/oracle_CMakeLists.txt" "$run_dir/oracle/CMakeLists.txt"
cmake -S "$run_dir/oracle" -B "$run_dir/oracle_build" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$run_dir/oracle_build" -j2
cmake -S "$repo_dir/ws_robot/src/astribot_s1_robot_geometry" -B "$run_dir/geometry_build" \
  -DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$run_dir/geometry_install" \
  -Dastribot_navigation_msgs_DIR="$NAVIGATION_MSGS_PREFIX/share/astribot_navigation_msgs/cmake"
cmake --build "$run_dir/geometry_build" -j2
cmake --install "$run_dir/geometry_build"
package_dir="$repo_dir/ws_robot/src/astribot_s1_navigation_policy_native"
cmake -S "$package_dir" -B "$run_dir/build" \
  -DASTRIBOT_BUILD_PYBIND=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_INSTALL_PREFIX="$run_dir/install" \
  -DCMAKE_PREFIX_PATH="$run_dir/geometry_install" \
  -Dastribot_navigation_msgs_DIR="$NAVIGATION_MSGS_PREFIX/share/astribot_navigation_msgs/cmake"
cmake --build "$run_dir/build" -j2
PYTHONDONTWRITEBYTECODE=1 ctest --test-dir "$run_dir/build" --output-on-failure
cmake --install "$run_dir/build"
export POLICY_OBSERVER_CPP="$run_dir/install/lib/astribot_s1_navigation_policy_native/policy_observer_cpp"
export POLICY_OBSERVER_GEOMETRY_BINDING="$run_dir/oracle_build/_geometry_native.cpython-310-x86_64-linux-gnu.so"
export ROS_LOG_DIR="$run_dir/ros_logs" PYTHONDONTWRITEBYTECODE=1
POLICY_OBSERVER_DOMAIN=164 /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  "$package_dir/test/test_policy_observer_ros.py" --basetemp "$run_dir/ros"
POLICY_OBSERVER_DOMAIN=165 /usr/bin/python3 "$package_dir/test/benchmark_policy_observer_ros.py" \
  --output "$run_dir/benchmark" --samples 100 --warmup 20 --pairs 4
