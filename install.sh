#!/usr/bin/env bash
set -e

echo "=========================================="
echo " Astribot SDK Install Script (Ubuntu 22.04)"
echo "=========================================="
echo ""

# 1. 检查系统版本 ---------------------------------------------------------
UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" != "22.04" ]]; then
    echo "[WARN] 当前系统不是 Ubuntu 22.04，而是: $UBUNTU_VERSION"
    echo "       脚本仍会继续执行，但依赖包版本可能不同。"
fi

# 2. 先检查 ROS2 环境 -----------------------------------------------------
echo ""
echo ">>> 检查 ROS2 环境..."

# 2.1 是否存在 ros2 命令
if command -v ros2 >/dev/null 2>&1; then
    echo "[OK] 已检测到 ROS2 命令: $(command -v ros2)"
else
    echo "============================================================"
    echo "[ERROR] 未检测到 ROS2 Humble 环境。"
    echo "        Astribot SDK 需要 ROS2 Humble 才能正常工作。"
    echo ""
    echo ">>> 推荐最简单的 ROS2 安装方式（鱼香ROS，一键安装）:"
    echo ""
    echo "    wget http://fishros.com/install -O fishros && . fishros"
    echo ""
    echo ">>> 选择菜单:"
    echo "    1) 安装 ROS"
    echo "    2) 选择 ROS2 Humble"
    echo ""
    echo "安装完成后重新运行本脚本即可。"
    echo "============================================================"
fi

# 2.2 如果有 /opt/ros/humble，尝试自动加载
if [ -d "/opt/ros/humble" ]; then
    if [ "${ROS_DISTRO:-}" != "humble" ]; then
        echo "[INFO] 自动加载 ROS2 Humble 环境..."
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    fi

    # 再次验证 ros2 pkg 是否可用
    if command -v ros2 >/dev/null 2>&1 && ros2 pkg list >/dev/null 2>&1; then
        echo "[OK] ROS2 Humble 环境正确加载 (ROS_DISTRO=$ROS_DISTRO)"
    else
        echo "[WARN] ros2 命令存在，但 'ros2 pkg list' 失败，请手动检查 ROS2 安装。"
    fi
else
    echo "[WARN] 未找到 /opt/ros/humble，ROS2 Humble 可能尚未安装。"
fi

# 2.3 将 ROS2 环境写入 bashrc（避免每次手动 source）
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "[INFO] 正在将 ROS2 环境加入 ~/.bashrc ..."
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# 3. 更新 apt 源 -----------------------------------------------------------
echo ""
echo ">>> 更新 apt 软件源..."
sudo apt update

# 4. 安装基础依赖 ---------------------------------------------------------
echo ""
echo ">>> 安装基础依赖 (apt 包)..."

sudo apt install -y git python3-pip libgsl27 libgsl-dev libmumps-seq-dev ros-humble-tf-transformations python3-colcon-ros  || echo "[WARN] 部分 apt 包未成功安装，请稍后手动检查。"

# 5. Python 依赖安装 ------------------------------------------------------
echo ""
echo ">>> 升级 pip..."
python3 -m pip install --upgrade pip

echo ""
echo ">>> 安装 Python 依赖..."
python3 -m pip install -U \
    tabulate \
    h5py \
    filterpy \
    colcon-common-extensions \
    numpy==1.22.4 \
    opencv-python==4.10.0.82

# 6. SDK 安装准备 ---------------------------------------------------------
echo ""
echo ">>> 创建 /opt/astribot_ros 及子目录(log, robot_config)..."

sudo mkdir -p /opt/astribot_ros/log
sudo mkdir -p /opt/astribot_ros/robot_config

echo ">>> 设置权限，使普通用户也能写入 log 与 robot_config..."
sudo chown -R "$USER:$USER" /opt/astribot_ros

echo "[OK] /opt/astribot_ros 已配置完成"



echo ""
echo "=========================================="
echo "   Astribot SDK Install Completed!"
echo "=========================================="
