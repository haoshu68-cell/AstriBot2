#!/usr/bin/env bash
set -eo pipefail
# Use a checkout of this recorded checkpoint and a NEW absolute directory.
# Prerequisites: generated navigation messages and installed policy configs.
run_dir=${1:?Provide a NEW absolute experiment directory}
: "${CPP_BASELINE:?Path to the prior strings-checkpoint installed native observer}"
test -x "$CPP_BASELINE"
repo_dir=$(git rev-parse --show-toplevel)
evidence="$repo_dir/docs/evidence/pybind_removal_20260921"
# Builds a fresh test-only Python oracle, native geometry and native policy;
# runs CTest, installed ROS differentials and the Python/native benchmark.
bash "$evidence/policy_json_integer_runtime/reproduce.sh" "$run_dir"
source /opt/ros/humble/setup.bash
source "$NAVIGATION_MSGS_PREFIX/share/astribot_navigation_msgs/local_setup.bash"
export AMENT_PREFIX_PATH="$POLICY_CONFIG_PREFIX:${AMENT_PREFIX_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR="$run_dir/ros_logs"
export POLICY_OBSERVER_CPP="$run_dir/install/lib/astribot_s1_navigation_policy_native/policy_observer_cpp"
package_dir="$repo_dir/ws_robot/src/astribot_s1_navigation_policy_native"
# A native/native comparison must work without any Python geometry binding.
env -u POLICY_OBSERVER_GEOMETRY_BINDING POLICY_OBSERVER_DOMAIN=165 \
  /usr/bin/python3 "$package_dir/test/benchmark_policy_observer_ros.py" \
  --cpp-baseline "$CPP_BASELINE" --output "$run_dir/benchmark_native" \
  --samples 100 --warmup 20 --pairs 4

cmake -S "$package_dir" -B "$run_dir/sanitizer" \
  -DASTRIBOT_BUILD_PYBIND=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_PREFIX_PATH="$run_dir/geometry_install" \
  -Dastribot_navigation_msgs_DIR="$NAVIGATION_MSGS_PREFIX/share/astribot_navigation_msgs/cmake" \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -g -fno-omit-frame-pointer -fno-pie -no-pie -fsanitize=address,undefined -ffp-contract=off' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined -no-pie'
cmake --build "$run_dir/sanitizer" -j2 --target \
  policy_fusion_probe policy_health_probe policy_observation_adapters_probe policy_risk_probe
mkdir -p "$run_dir/sanitizer_wrappers"
for probe in policy_fusion_probe policy_health_probe policy_observation_adapters_probe policy_risk_probe; do
  cat > "$run_dir/sanitizer_wrappers/$probe" <<EOF
#!/bin/sh
exec env LD_PRELOAD="$(g++ -print-file-name=libasan.so)\${LD_PRELOAD:+:\$LD_PRELOAD}" "$run_dir/sanitizer/$probe" "\$@"
EOF
  chmod +x "$run_dir/sanitizer_wrappers/$probe"
done
export POLICY_FUSION_PROBE="$run_dir/sanitizer_wrappers/policy_fusion_probe"
export POLICY_HEALTH_PROBE="$run_dir/sanitizer_wrappers/policy_health_probe"
export POLICY_OBSERVATION_ADAPTERS_PROBE="$run_dir/sanitizer_wrappers/policy_observation_adapters_probe"
export POLICY_RISK_PROBE="$run_dir/sanitizer_wrappers/policy_risk_probe"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  "$package_dir/test/test_policy_fusion.py" "$package_dir/test/test_policy_health.py" \
  "$package_dir/test/test_policy_observation_adapters.py" \
  "$package_dir/test/test_policy_json_strings.py" "$package_dir/test/test_policy_risk.py"

# Geometry tests explicitly load retired test-only modules; no production hook.
(
  export PYTHONPATH="$evidence/policy_stamp_runtime/geometry_test_support:$repo_dir/tools/migration/python_reference:${PYTHONPATH:-}"
  cmake -S "$repo_dir/ws_robot/src/astribot_s1_robot_geometry" -B "$run_dir/geometry_build" \
    -DBUILD_TESTING=ON -DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF
  cmake --build "$run_dir/geometry_build" -j2
  PYTEST_ADDOPTS='-p geometry_reference_plugin' \
    ctest --test-dir "$run_dir/geometry_build" --output-on-failure
)
