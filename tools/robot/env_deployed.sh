#!/usr/bin/env bash
# Source on the robot: the application overlay and native vendor SDK have separate roots.
export ASTRIBOT_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
export ASTRIBOT_SDK_ROOT="${ASTRIBOT_VENDOR_SDK_ROOT:-/home/astribot/Downloads/astribot_sdk_aarch64}"
if [ "$(uname -m)" != aarch64 ] || [ ! -f "$ASTRIBOT_SDK_ROOT/env.sh" ]; then
    echo '[env_deployed] 需要机器人 ARM64 环境及厂家 SDK；开发机请勿 source 此文件。' >&2
    return 1 2>/dev/null || exit 1
fi
source "$ASTRIBOT_PROJECT_ROOT/tools/robot/env_robot.sh" || return 1
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${ASTRIBOT_HARDWARE_DDS_PROFILE:-/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml}"
if [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "[env_deployed] DDS 配置不存在: $FASTRTPS_DEFAULT_PROFILES_FILE" >&2
    return 1 2>/dev/null || exit 1
fi
export PATH="/opt/ros/humble/bin:$PATH"
echo "[env_deployed] PROJECT=$ASTRIBOT_PROJECT_ROOT"
echo "[env_deployed] SDK=$ASTRIBOT_SDK_ROOT DDS=$FASTRTPS_DEFAULT_PROFILES_FILE"
