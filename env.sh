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
# 6. ROS2 / Gazebo 网络配置
#
#   ASTRIBOT_NET_MODE=localhost（默认）
#       所有话题只在本机传播。DDS 走「共享内存 + 127.0.0.1 回环 UDP」，
#       Gazebo/ign-transport 也绑回环，局域网内既发现不到也抓不到本机话题。
#   ASTRIBOT_NET_MODE=lan
#       跨机模式（连实机 192.168.0.10、多机联调），沿用 192.168.0.x 网段白名单。
#       用法：ASTRIBOT_NET_MODE=lan source env.sh
#
#   注意：Domain ID 只是频道号，不具备任何隔离或安全作用（同网段任何人
#   export 相同 Domain ID 即可读写你的话题）。真正的隔离靠下面的传输层白名单。
# -----------------------------------------------------

# 固定 Domain ID
export ROS_DOMAIN_ID=25

# 强制使用 Fast DDS 作为 RMW
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 收敛到「仅本机」。
#
# 实测结论（2026-08-19，Humble + Fast DDS，用 /proc/net/igmp 逐网卡数多播加入次数验证）：
#   · 只设 ROS_LOCALHOST_ONLY=1                     -> eno1 加入数不变，确实封住 ✔
#   · 额外挂一个 interfaceWhiteList=127.0.0.1 的 XML -> eno1 加入数照涨，反而泄漏 ✘
#     原因：Fast DDS 的 interfaceWhiteList 只过滤单播 locator，不阻止 SPDP 发现报文
#     从物理网卡多播出去；而带 is_default_profile 的 XML 又会盖掉 rmw 层的 localhost 处理。
#   所以这里只用 ROS_LOCALHOST_ONLY，并且必须主动 unset FASTRTPS_DEFAULT_PROFILES_FILE
#   （否则先 source 过 lan 模式残留的 profile 会把本机限制解掉）。
#   保留内建传输的额外好处：共享内存(SHM)仍然可用，Livox 点云等大消息不掉性能。
#
# Gazebo 的 ign-transport 不走 DDS，是完全独立的第二条通道，必须单独设 IGN_IP/GZ_IP，
# 否则即使 DDS 封死了，Gazebo 仍会在局域网多播 239.255.0.7。
_astribot_lock_to_localhost() {
    export ROS_LOCALHOST_ONLY=1
    export IGN_IP=127.0.0.1
    export GZ_IP=127.0.0.1
    unset FASTRTPS_DEFAULT_PROFILES_FILE
}

ASTRIBOT_NET_MODE="${ASTRIBOT_NET_MODE:-localhost}"

if [ "$ASTRIBOT_NET_MODE" = "lan" ]; then
    # 自动检测本机的 192.168.0.x 地址（无论是 0.75 还是 0.80 都能适配）
    LOCAL_192_IP=$(ip -4 addr show | awk '/inet 192\.168\.0\./ {print $2}' | head -n1 | cut -d/ -f1)

    if [ -n "$LOCAL_192_IP" ]; then
        export ROS_LOCALHOST_ONLY=0
        unset IGN_IP GZ_IP

        if [ "$LOCAL_192_IP" = "192.168.0.10" ]; then
            # 本机（机器人端）运行 SDK，无需设置白名单
            unset FASTRTPS_DEFAULT_PROFILES_FILE
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
        # 关键：探测失败必须收紧而不是放开。旧版本此处什么都不设，
        # 导致机器换到别的网段（如 10.x）时 DDS 在所有网卡上裸奔。
        _astribot_lock_to_localhost
        echo "[env.sh][WARN] 请求 lan 模式但未检测到 192.168.0.x 地址，已回退为「仅本机」。"
    fi
else
    _astribot_lock_to_localhost
    echo "[env.sh] 网络模式 = localhost（话题仅本机可见：DDS 限回环+SHM，Gazebo 绑 127.0.0.1）"
fi

unset -f _astribot_lock_to_localhost

echo "[env.sh] ROS_DOMAIN_ID      = $ROS_DOMAIN_ID"
echo "[env.sh] RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "[env.sh] ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY"
echo "[env.sh] FASTRTPS_PROFILES  = ${FASTRTPS_DEFAULT_PROFILES_FILE:-<none>}"
echo "[env.sh] Environment setup completed."
