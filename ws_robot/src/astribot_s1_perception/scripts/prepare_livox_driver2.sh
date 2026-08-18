#!/usr/bin/env bash
# 文件用途：一次性准备 livox_ros_driver2 源码，让它能被 colcon 当成普通 ROS2 包构建。
#
# 为什么需要这一步：livox_ros_driver2 官方仓库里没有现成的 package.xml，
# 只有 package_ROS1.xml / package_ROS2.xml 两份、按 ROS1/ROS2 区分的 manifest，
# 需要手动复制一份改名成 package.xml；launch 文件同理放在 launch_ROS2/ 里。
# 官方自带的 build.sh 也会做这两步，但它还会顺手
# `rm -rf ../../build/ ../../devel/ ../../install/`——这个路径在我们的工作空间布局下
# 精确等于 ws_robot/build、ws_robot/install，会把本仓库其它所有包（机器人描述、
# 仿真bringup、感知包）已经编译好的产物全部删掉！所以这里不用官方 build.sh，
# 只手动做它"复制文件"这一步，编译交给我们自己工作空间统一的 colcon build 命令。
#
# 用法（在 ws_robot 下执行一次，clone完submodule之后、colcon build之前）：
#   bash src/astribot_s1_perception/scripts/prepare_livox_driver2.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DRIVER_DIR="${WS_ROOT}/src/livox_ros_driver2"

if [ ! -d "${DRIVER_DIR}" ]; then
  echo "[prepare_livox_driver2] 错误：找不到 ${DRIVER_DIR}" >&2
  echo "  请先执行：git submodule update --init --recursive" >&2
  exit 1
fi

cd "${DRIVER_DIR}"

if [ ! -f package_ROS2.xml ]; then
  echo "[prepare_livox_driver2] 错误：${DRIVER_DIR}/package_ROS2.xml 不存在，"
  echo "  livox_ros_driver2 仓库结构可能变了，请对照官方 build.sh 手动核对。" >&2
  exit 1
fi

cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

echo "[prepare_livox_driver2] 完成：已生成 package.xml 和 launch/ 目录。"
echo "接下来正常执行（注意 --cmake-args，livox_ros_driver2 的 CMakeLists 靠这两个变量"
echo "区分 ROS1/ROS2、区分发行版）："
echo "  cd ${WS_ROOT}"
echo "  colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble"
