#!/usr/bin/env bash
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO/ws_robot/install/setup.bash"
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export DISPLAY="${DISPLAY:-:1}"
exec python3 "$REPO/tools/sim_stack_supervisor.py" "$@"
