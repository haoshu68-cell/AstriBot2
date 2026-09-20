#!/usr/bin/env bash
# Prepare the upstream driver's ROS2 manifest, launch files, and project Eigen dependency.
# Do not use its build.sh: its cleanup paths erase this workspace's build/install.
# Run after submodule initialization, before colcon package discovery.

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
python3 - <<'PY'
from pathlib import Path
p = Path('package.xml')
text = p.read_text()
anchor = '<buildtool_depend>ament_cmake_auto</buildtool_depend>'
if '<depend>astribot_eigen_vendor</depend>' not in text:
    if text.count(anchor) != 1:
        raise SystemExit('Unsupported Livox manifest; inspect the upstream update')
    p.write_text(text.replace(anchor, anchor + '\n  <depend>astribot_eigen_vendor</depend>'))
p = Path('CMakeLists.txt')
text = p.read_text()
anchor = '  find_package(ament_cmake_auto REQUIRED)'
if 'find_package(astribot_eigen_vendor REQUIRED)' not in text:
    if text.count(anchor) != 1:
        raise SystemExit('Unsupported Livox CMake; inspect the upstream update')
    p.write_text(text.replace(anchor, '  find_package(astribot_eigen_vendor REQUIRED)\n' + anchor))
text = p.read_text()
anchor = '  ament_auto_package(INSTALL_TO_SHARE'
if 'astribot_target_eigen(${PROJECT_NAME})' not in text:
    if text.count(anchor) != 1:
        raise SystemExit('Unsupported Livox target layout; inspect the upstream update')
    p.write_text(text.replace(anchor, '  astribot_target_eigen(${PROJECT_NAME})\n' + anchor))
PY

echo "[prepare_livox_driver2] 完成：已生成 package.xml 和 launch/ 目录。"
echo "接下来正常执行（注意 --cmake-args，livox_ros_driver2 的 CMakeLists 靠这两个变量"
echo "区分 ROS1/ROS2、区分发行版）："
echo "  cd ${WS_ROOT}"
echo "  colcon build --base-paths src --symlink-install --packages-up-to livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble"
