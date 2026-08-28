#!/usr/bin/env bash
# =====================================================================
# 实机环境（aarch64）。**不修改厂商 env.sh**，只在其之上补齐。
#
# 为什么不直接改 env.sh：
#   · 它属于厂商 git 仓库(public_repo/astribot_sdk_aarch64)，改了每次 pull 都冲突
#   · 开发机那份 env.sh 默认 ASTRIBOT_NET_MODE=localhost -> ROS_LOCALHOST_ONLY=1，
#     会切断 SDK 与控制器(192.168.0.10)之间的 DDS，覆盖过来是严重回归
# =====================================================================
SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 1. 先 source ROS 2 ----
# 厂商 env.sh **不 source /opt/ros/humble**（实测：它只 source SDK 内部的
# third_party/software、astribot_msgs、third_pkg 三个 setup.bash）。
# 不先 source ROS 的话 `import rclpy` 直接 ModuleNotFoundError，
# 而 astribot_interface.py:17 就在 import rclpy，SDK 整个起不来。
# 顺序也重要：astribot_msgs 的 local_setup.bash 是 ROS overlay，要 ROS 在前。
source /opt/ros/humble/setup.bash

# ---- 2. 再用厂商 env.sh（其 ROS_DOMAIN_ID=25 / LOCALHOST_ONLY=0 / RMW / Fast DDS
#         白名单 192.168.0.11 都是对的，不要覆盖）----
source "${SDK_ROOT}/env.sh"

# ---- 3. 补厂商 env.sh 的两处缺陷 ----
# (a) env.sh:17 指向的 third_party/astribot_ros_middleware_py 根本不存在，
#     真实路径在 third_party/software/... 下。不存在的路径挂 PYTHONPATH 上
#     不报错、静默忽略，所以这条缺陷能活很久。
_MW="${SDK_ROOT}/third_party/software/astribot_ros_middleware/lib/python3.10/site-packages"
if [ -d "$_MW" ]; then
    export PYTHONPATH="${_MW}:${PYTHONPATH}"
    export ASTRIBOT_MIDDLEWARE_PY="$_MW"
else
    echo "[env_robot][ERROR] middleware 缺失: $_MW"
fi

# (b) 编译过的 util.py:26 用**裸名** import robotics_library_py.robotics_library_py，
#     需要 common 这一层可见。只有 SDK_ROOT 不够。
export PYTHONPATH="${SDK_ROOT}/astribot_sdk/core/common:${PYTHONPATH}"

# ---- 4. SDK 运行期约定 ----
# ASTRIBOT_LOG 必须在 import SDK **之前**生效：astribot_interface.py 在 import 时
# 就 os.dup2 重定向 fd 1/2，晚了会把我们自己的 ERROR 一起吞掉。
export ASTRIBOT_LOG=1
export ROBOT_TYPE="${ROBOT_TYPE:-S1}"   # 非 S1 时底盘自由度是 2，enable 会被拒

# ---- 5. 我们的工作空间 ----
if [ -f "${SDK_ROOT}/ws_robot/install/setup.bash" ]; then
    source "${SDK_ROOT}/ws_robot/install/setup.bash"
    echo "[env_robot] ws_robot overlay 已挂载"
else
    echo "[env_robot] ws_robot 尚未构建（见 docs/sync_to_aarch64_sdk.md 第 4 节）"
fi

echo "[env_robot] DOMAIN=$ROS_DOMAIN_ID  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  RMW=$RMW_IMPLEMENTATION"
