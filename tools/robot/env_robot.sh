#!/usr/bin/env bash
_ENV_ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -n "${ASTRIBOT_SDK_ROOT:-}" ]; then
    SDK_ROOT="$ASTRIBOT_SDK_ROOT"
elif [ -f "$_ENV_ROBOT_DIR/env.sh" ]; then
    SDK_ROOT="$_ENV_ROBOT_DIR"
else
    SDK_ROOT="$(cd "$_ENV_ROBOT_DIR/../.." && pwd)"
fi
if [ ! -f "$SDK_ROOT/env.sh" ]; then
    echo "[env_robot][ERROR] SDK env.sh 不存在: $SDK_ROOT；请设置 ASTRIBOT_SDK_ROOT" >&2
    return 1 2>/dev/null || exit 1
fi
export ASTRIBOT_SDK_ROOT="$SDK_ROOT"

source /opt/ros/humble/setup.bash

source "${SDK_ROOT}/env.sh"

_MW="${SDK_ROOT}/third_party/software/astribot_ros_middleware/lib/python3.10/site-packages"
if [ -d "$_MW" ]; then
    export PYTHONPATH="${_MW}:${PYTHONPATH}"
    export ASTRIBOT_MIDDLEWARE_PY="$_MW"
else
    echo "[env_robot][ERROR] middleware 缺失: $_MW"
fi

export PYTHONPATH="${SDK_ROOT}/astribot_sdk/core/common:${PYTHONPATH}"

export ASTRIBOT_LOG=1
export ROBOT_TYPE="${ROBOT_TYPE:-S1}"   # 非 S1 时底盘自由度是 2，enable 会被拒

if [ -f "${SDK_ROOT}/ws_robot/install/setup.bash" ]; then
    source "${SDK_ROOT}/ws_robot/install/setup.bash"
    echo "[env_robot] ws_robot overlay 已挂载"
else
    echo "[env_robot] ws_robot 尚未构建（见 docs/sync_to_aarch64_sdk.md 第 4 节）"
fi

echo "[env_robot] DOMAIN=$ROS_DOMAIN_ID  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  RMW=$RMW_IMPLEMENTATION"
