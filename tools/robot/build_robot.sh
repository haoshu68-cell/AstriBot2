#!/usr/bin/env bash
set -euo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${SDK_ROOT}/ws_robot"
ROS_UNDERLAY=/opt/ros/humble
VENDOR_ROS=/opt/astribot_ros/middle_ware

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; RST=$'\033[0m'
die() { echo "${RED}[build_robot][FATAL]${RST} $*" >&2; exit 1; }
ok()  { echo "${GRN}[build_robot][OK]${RST} $*"; }
warn(){ echo "${YEL}[build_robot][WARN]${RST} $*"; }

if [[ "${AMENT_PREFIX_PATH:-}" == *"${VENDOR_ROS}"* ]]; then
    die "当前 shell 的 AMENT_PREFIX_PATH 里已经有厂商 ROS（${VENDOR_ROS}）。
  这说明你 source 过 env_robot.sh 或厂商 env.sh。在这种 shell 里构建会让
  CMake 找到厂商的头文件，而运行时又是厂商的库——看似自洽，实则与既有产物
  混编，且**不会有任何报错**。
  改法：开一个**干净的新终端**，直接跑 ./build_robot.sh（脚本自己会 source
  正确的 underlay，不需要你先 source 任何东西）。"
fi

if [ -n "${ASTRIBOT_MIDDLEWARE_PY:-}" ]; then
    die "检测到 ASTRIBOT_MIDDLEWARE_PY 已设置（env_robot.sh 的痕迹）。同上：请用干净终端。"
fi
ok "起始 shell 干净：没有厂商 ROS 的痕迹"

[ -f "${ROS_UNDERLAY}/setup.bash" ] || die "找不到 ${ROS_UNDERLAY}/setup.bash"

set +u
# shellcheck disable=SC1091
source "${ROS_UNDERLAY}/setup.bash"
set -u

[ "${ROS_DISTRO:-}" = "humble" ] \
    || die "ROS_DISTRO='${ROS_DISTRO:-（空）}'，期望 'humble'。
  空值几乎一定是缺 ros_environment 包：
      sudo apt install -y ros-humble-ros-environment
  没有它 rosdep 无法解析**任何** key，而报错看起来像是包不存在。"
ok "ROS_DISTRO=${ROS_DISTRO}  ROS_VERSION=${ROS_VERSION:-?}"

first_prefix="${AMENT_PREFIX_PATH%%:*}"
[ "$first_prefix" = "$ROS_UNDERLAY" ] \
    || die "AMENT_PREFIX_PATH 首段是 '${first_prefix}'，期望 '${ROS_UNDERLAY}'"
ok "AMENT_PREFIX_PATH 首段 = ${first_prefix}"

[[ "${AMENT_PREFIX_PATH}" != *"${VENDOR_ROS}"* ]] \
    || die "source underlay 之后厂商 ROS 仍在 AMENT_PREFIX_PATH 里，环境不干净"
[[ "${CMAKE_PREFIX_PATH:-}" != *"${VENDOR_ROS}"* ]] \
    || die "CMAKE_PREFIX_PATH 里有厂商 ROS，CMake 会找到厂商头文件"
ok "厂商 ROS 未混入构建环境"

missing=()
for p in ament_cmake ament_cmake_python rosidl_default_generators \
         rclcpp rclpy tf2_ros nav_msgs sensor_msgs geometry_msgs \
         nav2_costmap_2d nav2_msgs moveit_core moveit_ros_planning_interface \
         pointcloud_to_laserscan control_msgs; do
    [ -d "${ROS_UNDERLAY}/share/$p" ] || missing+=("$p")
done
if [ ${#missing[@]} -gt 0 ]; then
    die "underlay 缺少构建必需包：${missing[*]}
  补齐：sudo apt install -y $(printf 'ros-humble-%s ' "${missing[@]}" | tr '_' '-')"
fi
ok "构建必需包齐全（14 项）"

IGNORE=(
    livox_ros_driver2
    astribot_s1_gazebo_bringup
)

CLEAN=0
COLCON_EXTRA=()
for arg in "$@"; do
    case "$arg" in
        --clean) CLEAN=1 ;;
        *) COLCON_EXTRA+=("$arg") ;;
    esac
done

cd "$WS" || die "找不到工作空间 $WS"

if [ "$CLEAN" = 1 ]; then
    warn "--clean：删除 build/ install/ log/"
    rm -rf build install log
fi

echo
echo "===== colcon build ====="
echo "  工作空间 : $WS"
echo "  underlay : $ROS_UNDERLAY"
echo "  不构建   : ${IGNORE[*]}"
echo

set +e
colcon build --symlink-install \
    --packages-ignore "${IGNORE[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "${COLCON_EXTRA[@]}"
rc=$?
set -e

echo
if [ $rc -ne 0 ]; then
    die "colcon build 失败（退出码 $rc）。看 ${WS}/log/latest_build/ 里的具体包日志。"
fi

echo "===== 构建后校验 ====="
cache=$(find "${WS}/build" -maxdepth 2 -name CMakeCache.txt 2>/dev/null | head -1)
if [ -n "$cache" ]; then
    if grep -q "${VENDOR_ROS}" "$cache"; then
        die "产物污染：${cache##*/build/} 里出现厂商 ROS 路径。
  说明这次（或之前某次增量）构建是在带厂商环境的 shell 里做的。
  必须 ./build_robot.sh --clean 重建。"
    fi
    ok "CMakeCache 未引用厂商 ROS"
fi

ok "构建完成。运行前请 source ${SDK_ROOT}/env_robot.sh（那才是运行环境）"
echo
echo "  提醒：构建用 ${ROS_UNDERLAY}，运行用 env_robot.sh（厂商栈 + 我们的 overlay）。"
echo "        这两个环境**刻意不同**，不要在同一个终端里混用。"
