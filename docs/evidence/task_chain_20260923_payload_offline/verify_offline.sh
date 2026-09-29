#!/usr/bin/env bash
# Recheck this recorded offline candidate using its already configured, isolated builds.
# No ROS node, simulator, lifecycle command, daemon or robot action is launched.
set -e -o pipefail
cd "$(dirname "$0")/../../.."
task_run="$PWD/runs/task_chain_20260923_payload_offline"
source runs/task_chain_20260922_gpu_recovery/query_env_navigation.sh
source "$task_run/install/local_setup.bash"
export CMAKE_PREFIX_PATH="$task_run/hold_install:$task_run/geometry_install:$task_run/payload_install:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="$task_run/hold_install:$task_run/geometry_install:$task_run/payload_install:${AMENT_PREFIX_PATH:-}"

cmake --build "$task_run/payload_build" -j4 > "$task_run/final_payload_build.log" 2>&1
ctest --test-dir "$task_run/payload_build" -V > "$task_run/final_payload_tests.log" 2>&1
cmake --install "$task_run/payload_build" > "$task_run/final_payload_install.log" 2>&1

cmake --build "$task_run/geometry_build" -j4 > "$task_run/final_geometry_build.log" 2>&1
ctest --test-dir "$task_run/geometry_build" -V \
  -R '^(attachment_geometry_test|lease_time|filled_collision|fusion_snapshot|geometry_state_core|cpp_geometry_differential)$' \
  > "$task_run/final_geometry_tests.log" 2>&1
cmake --install "$task_run/geometry_build" > "$task_run/final_geometry_install.log" 2>&1

cmake --build "$task_run/hold_build" -j4 > "$task_run/final_hold_build.log" 2>&1
ctest --test-dir "$task_run/hold_build" -V > "$task_run/final_hold_tests.log" 2>&1
cmake --install "$task_run/hold_build" > "$task_run/final_hold_install.log" 2>&1

cmake --build "$task_run/navigation_build" --target fixed_hold_flow_test fixed_envelope_core_test fixed_envelope_cpp -j4 \
  > "$task_run/final_navigation_build.log" 2>&1
ctest --test-dir "$task_run/navigation_build" -V -R '^(fixed_hold_flow_test|fixed_envelope_authority)$' \
  > "$task_run/final_navigation_tests.log" 2>&1
/usr/bin/python3 -m pytest -q ws_robot/src/astribot_s1_navigation/test/test_nonhome_launch.py \
  --junitxml="$task_run/final_launch_tests.xml" > "$task_run/final_launch_tests.log" 2>&1
