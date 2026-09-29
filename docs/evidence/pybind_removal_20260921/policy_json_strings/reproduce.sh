#!/usr/bin/env bash
set -eo pipefail
run_dir=${1:?Provide a NEW absolute experiment directory}
repo_dir=$(git rev-parse --show-toplevel)
evidence="$repo_dir/docs/evidence/pybind_removal_20260921"
# This common recipe builds the current recorded sources, including the new
# string CTest suite, installed 108-case ROS suite and paired benchmark.
bash "$evidence/policy_json_integer_runtime/reproduce.sh" "$run_dir"
POLICY_ADAPTERS_BUILD_DIR="$run_dir/sanitizer" POLICY_ADAPTERS_SANITIZE=1 \
  bash "$evidence/policy_observation_adapters/reproduce.sh"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  POLICY_OBSERVATION_ADAPTERS_PROBE="$run_dir/sanitizer/probe_sanitized_wrapper" \
  PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  "$repo_dir/ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_json_strings.py"
