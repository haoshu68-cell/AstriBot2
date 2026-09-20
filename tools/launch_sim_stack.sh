#!/usr/bin/env bash
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ASTRIBOT_OVERLAY_SETUP:-}" ]]; then
    source "$REPO/tools/ros_overlay_env.sh"
    astribot_prepare_overlay_environment
fi
source /opt/ros/humble/setup.bash
source "$REPO/ws_robot/install/setup.bash"
if [[ -n "${ASTRIBOT_OVERLAY_SETUP:-}" ]]; then
    source "$ASTRIBOT_OVERLAY_SETUP"
fi
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export DISPLAY="${DISPLAY:-:1}"
# Default ROS launch entry; legacy --flags keep their existing CLI contract.
if [[ $# -eq 0 || "$1" != --* ]]; then
    export ASTRIBOT_LOG_CAPTURE=1
    exec python3 -m astribot_logging.launch_entry launch astribot_s1_navigation \
        sim_stack.launch.py "repo_dir:=$REPO" "$@"
fi
exec python3 "$REPO/tools/sim_stack_supervisor.py" "$@"
