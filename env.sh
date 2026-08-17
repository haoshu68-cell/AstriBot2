#!/usr/bin/env bash

# ===========================================
#   Astribot SDK Environment Setup (relative)
# ===========================================

# 1. 自动获取 env.sh 所在目录 = SDK_ROOT
SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[env.sh] SDK_ROOT = $SDK_ROOT"

# -----------------------------------------------------
# 2. 设置 PYTHONPATH（全部从 SDK_ROOT 自动推算）
# -----------------------------------------------------

# astribot_ros_middleware_py
export PYTHONPATH="${SDK_ROOT}/third_party/astribot_ros_middleware_py:$PYTHONPATH"

# astribot_sdk 本体
export PYTHONPATH="${SDK_ROOT}:$PYTHONPATH"

echo "[env.sh] PYTHONPATH configured."


# -----------------------------------------------------
# 3. source 各种 setup.bash
# -----------------------------------------------------

# third_party/software/setup.bash
if [ -f "${SDK_ROOT}/third_party/software/setup.bash" ]; then
    source "${SDK_ROOT}/third_party/software/setup.bash"
else
    echo "[WARN] ${SDK_ROOT}/third_party/software/setup.bash not found."
fi

# astribot_msgs
if [ -f "${SDK_ROOT}/astribot_msgs/share/astribot_msgs/local_setup.bash" ]; then
    source "${SDK_ROOT}/astribot_msgs/share/astribot_msgs/local_setup.bash"
else
    echo "[WARN] local_setup.bash for astribot_msgs not found."
fi

# third_pkg
if [ -f "${SDK_ROOT}/third_party/third_pkg/local_setup.bash" ]; then
    source "${SDK_ROOT}/third_party/third_pkg/local_setup.bash"
else
    echo "[WARN] ${SDK_ROOT}/third_party/third_pkg/local_setup.bash not found."
fi


# -----------------------------------------------------
# 4. 设置 LD_LIBRARY_PATH
# -----------------------------------------------------

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${SDK_ROOT}/astribot_sdk/core/common/robotics_library_py"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${SDK_ROOT}/astribot_sdk/core/common/whole_body_control/third_party"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${SDK_ROOT}/third_party/drake/lib"

echo "[env.sh] LD_LIBRARY_PATH updated."


# -----------------------------------------------------
# 5. 其他环境变量
# -----------------------------------------------------

export ROBOT_TYPE='S1'
export ASTRIBOT_SDK_ROOT="$SDK_ROOT"

# -----------------------------------------------------
# 6. ROS2 网络配置（只走 192.168.0.x 网段）
# -----------------------------------------------------

# 固定 Domain ID
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0

# 强制使用 Fast DDS 作为 RMW
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 自动检测本机的 192.168.0.x 地址（无论是 0.75 还是 0.80 都能适配）
LOCAL_192_IP=$(ip -4 addr show | awk '/inet 192\.168\.0\./ {print $2}' | head -n1 | cut -d/ -f1)

if [ -n "$LOCAL_192_IP" ]; then
    if [ "$LOCAL_192_IP" = "192.168.0.10" ]; then
        # 本机（机器人端）运行 SDK，无需设置白名单
        echo "[env.sh] 检测到本机 IP = 192.168.0.10，跳过 Fast DDS 白名单设置"
    else
        # 远程机器连接机器人，需要设置白名单限制网段
        FASTDDS_TEMPLATE="${SDK_ROOT}/config/fastdds_whitelist_192.xml.template"
        FASTDDS_XML="${SDK_ROOT}/config/fastdds_whitelist_192.xml"

        # 从模板生成配置文件，替换 IP 占位符
        sed "s/__LOCAL_192_IP__/${LOCAL_192_IP}/g" "$FASTDDS_TEMPLATE" > "$FASTDDS_XML"

        export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML"
        echo "[env.sh] Fast DDS whitelist = ${LOCAL_192_IP} (只用 192.168.0.x 网段)"
    fi
else
    echo "[env.sh][WARN] 未检测到 192.168.0.x 地址，不设置 Fast DDS 接口白名单。"
fi

echo "[env.sh] ROS_DOMAIN_ID      = $ROS_DOMAIN_ID"
echo "[env.sh] RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "[env.sh] Environment setup completed."
