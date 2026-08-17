#!/usr/bin/env bash
# 文件用途：补充导出仿真所需环境变量的便捷脚本。
#
# 用法（在 source 完 ws_robot/install/setup.bash 之后再 source 本脚本）：
#   source ws_robot/install/setup.bash
#   source ws_robot/src/astribot_s1_gazebo_bringup/scripts/setup_gazebo_env.sh
#
# 为什么需要这个脚本：
#   aws_robomaker_small_warehouse_world 的官方 README 只给了经典 Gazebo(Gazebo Classic) 的
#   GAZEBO_MODEL_PATH 配置方式，没有为新版 Gazebo(Ignition/Gz Sim) 配置资源搜索路径。
#   如果不额外导出 GZ_SIM_RESOURCE_PATH / IGN_GAZEBO_RESOURCE_PATH，
#   仓储场景里的货架、托盘、纸箱等模型会加载失败或贴图丢失（在 Gazebo 里表现为“看不到模型”
#   或“模型是灰色/纯色方块”）。
#
#   warehouse_sim.launch.py 内部已经用 SetEnvironmentVariable 动态设置了同样的路径（双保险，
#   即使不手动 source 这个脚本，用 ros2 launch 启动也不会有资源路径问题）；
#   本脚本主要用于：
#     1) 你想脱离 launch 文件、手动执行 `gz sim <world文件>` 单独调试场景时；
#     2) 想在当前终端里用 `gz topic -l` / `ros2 topic list` 等命令行工具排查问题时，
#        保证环境变量是导出好的。

set -euo pipefail

# 找不到包时给出清晰报错，而不是让后面的 gz sim 静默报"资源缺失"却不知道原因
if ! WAREHOUSE_SHARE_DIR="$(ros2 pkg prefix aws_robomaker_small_warehouse_world 2>/dev/null)/share/aws_robomaker_small_warehouse_world"; then
  echo "[setup_gazebo_env] 错误：找不到 ament 包 aws_robomaker_small_warehouse_world。" >&2
  echo "  请确认已经在 ws_robot 下执行过 colcon build 并 source 了 install/setup.bash。" >&2
  return 1 2>/dev/null || exit 1
fi

if [ ! -d "${WAREHOUSE_SHARE_DIR}" ]; then
  echo "[setup_gazebo_env] 错误：${WAREHOUSE_SHARE_DIR} 不存在，colcon build 可能没有正确安装该包。" >&2
  return 1 2>/dev/null || exit 1
fi

WAREHOUSE_MODELS_DIR="${WAREHOUSE_SHARE_DIR}/models"
WAREHOUSE_WORLDS_DIR="${WAREHOUSE_SHARE_DIR}/worlds"

# 新版 Gazebo(Garden/Fortress，命令为 `gz sim`) 使用的变量名
export GZ_SIM_RESOURCE_PATH="${WAREHOUSE_MODELS_DIR}:${WAREHOUSE_WORLDS_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
# 更旧的 Ignition Gazebo(命令为 `ign gazebo`) 使用的变量名，双写保证兼容
export IGN_GAZEBO_RESOURCE_PATH="${WAREHOUSE_MODELS_DIR}:${WAREHOUSE_WORLDS_DIR}:${IGN_GAZEBO_RESOURCE_PATH:-}"

echo "[setup_gazebo_env] 已导出 GZ_SIM_RESOURCE_PATH / IGN_GAZEBO_RESOURCE_PATH:"
echo "  ${WAREHOUSE_MODELS_DIR}"
echo "  ${WAREHOUSE_WORLDS_DIR}"
