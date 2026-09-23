#!/usr/bin/env bash
# Inside one isolated, disposable container only. No host robot/DDS mounts.
# P0_MESH and P0_FIXTURE_KIND are explicit; /inputs and /engines are read-only.
set -euo pipefail
test "${ROS_LOCALHOST_ONLY:-}" = 1
test -n "${P0_MESH:-}"
test -f "$P0_MESH"
case "${P0_FIXTURE_KIND:-}" in official|project) ;; *) exit 2;; esac
test -s /engines/refine.plan
test -s /engines/score.plan
test -d /output
test ! -e /output/result.json

# ROS-generated setup files may reference unset optional variables.
set +u
source /opt/ros/humble/setup.bash
set -u
backend_pid=''
finish() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$backend_pid" ]]; then
    kill -INT "$backend_pid" 2>/dev/null || true
    # ros2 launch owns its component child. The caller's bounded disposable
    # container provides the final cleanup boundary if upstream shutdown hangs.
    local backend_result=0
    wait "$backend_pid" || backend_result=$?
    printf '%s\n' "$backend_result" > /output/backend_exit_code.txt
  fi
  printf '%s\n' "$result" > /output/runner_exit_code.txt
  exit "$result"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch /tools/p0_fixture.launch.py > /output/backend.log 2>&1 &
backend_pid=$!
printf '%s\n' "$backend_pid" > /output/backend_container_pid.txt
python3 -B /tools/validate_fixture.py --input /inputs --kind "$P0_FIXTURE_KIND" \
  --output /output/result.json --timeout 120 > /output/validator.log 2>&1
