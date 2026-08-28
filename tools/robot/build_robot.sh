#!/usr/bin/env bash
# =====================================================================
# 实机构建脚本（aarch64 Orin）。**必须用它构建，不要手敲 colcon build。**
#
# ============ 为什么需要这个脚本，而不是记住一条命令 ============
# 机器人上有**两套各自完整的 ROS 2 Humble**：
#
#   /opt/ros/humble                284 包   apt 装，nav2/MoveIt/slam_toolbox 在这
#   /opt/astribot_ros/middle_ware   341 包   厂商源码编译，厂商栈实际跑的是它
#
# 两者重叠 184 个包，其中 150 个（83%）版本不同。而**决定性的危险在于：
# ROS 2 的库 SONAME 不带版本号**——
#
#   librclcpp.so   SONAME = "librclcpp.so"     ← 两套完全相同
#   librcl.so      SONAME = "librcl.so"
#   librmw.so      SONAME = "librmw.so"
#
# 对比 libignition-math6.so.6：那个带版本，装错会在**加载时响亮失败**。
# ROS 的库不带，于是"谁在搜索路径前面就用谁"，**静默生效、零报错**。
#
# 已实测的既成事实（不是推测）：
#   编译期  CMakeCache 里 6 个 include 路径全部 /opt/ros/humble
#   运行期  ldd 我们的 C++ 二进制 → 35 个库来自厂商 middle_ware，仅 1 个来自 humble
#
# 也就是说：**对着 humble 编译、在厂商库上运行**。这个组合已验证可用
# （3 个各链 31 个 ROS 库的测试二进制，44 项全通过），但它成立的前提是
# **构建时必须只有 humble 在路径里**。若某次先 source 了 env_robot.sh 再构建，
# 就变成对厂商头文件编译，而增量构建会让产物一半对着一套、一半对着另一套——
# 那种状态没有任何报错，只会在运行时表现成难以归因的怪问题。
#
# 所以本脚本做的事就一件：**构建前把环境断言死，不对就拒绝构建。**
#
# 用法：
#   ./build_robot.sh              增量构建
#   ./build_robot.sh --clean      先删 build/ install/ 再构建
#   ./build_robot.sh --packages-select <pkg>...   透传给 colcon
# =====================================================================
set -euo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${SDK_ROOT}/ws_robot"
ROS_UNDERLAY=/opt/ros/humble
VENDOR_ROS=/opt/astribot_ros/middle_ware

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; RST=$'\033[0m'
die() { echo "${RED}[build_robot][FATAL]${RST} $*" >&2; exit 1; }
ok()  { echo "${GRN}[build_robot][OK]${RST} $*"; }
warn(){ echo "${YEL}[build_robot][WARN]${RST} $*"; }

# ---------------------------------------------------------------------
# 断言 0：不能在已经 source 过 env_robot.sh / 厂商栈的 shell 里构建
#
# 这是本脚本最重要的一条。放在最前面是因为后面的 source 会掩盖它。
# ---------------------------------------------------------------------
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

# ---------------------------------------------------------------------
# 断言 1：underlay 存在且完整
# ---------------------------------------------------------------------
[ -f "${ROS_UNDERLAY}/setup.bash" ] || die "找不到 ${ROS_UNDERLAY}/setup.bash"

# source ROS 期间必须关掉 `set -u`：ROS 的 setup.bash 会引用未定义的
# AMENT_TRACE_SETUP_FILES（实测 /opt/ros/humble/setup.bash:8），
# 在 `set -u` 下直接 "unbound variable" 退出。
# 这是 ament setup 脚本的通病，不是本机的问题，所以只能在这里让路。
set +u
# shellcheck disable=SC1091
source "${ROS_UNDERLAY}/setup.bash"
set -u

# ROS_DISTRO 为空是这台机器上踩过的真实坑：精简版 ROS 没装 ros_environment，
# 于是 setup.bash 不导出 ROS_DISTRO，而 rosdep 靠它选规则集——
# 表象是"无法解析任何 rosdep key"，极易误判成 rosdep 数据库坏了。
[ "${ROS_DISTRO:-}" = "humble" ] \
    || die "ROS_DISTRO='${ROS_DISTRO:-（空）}'，期望 'humble'。
  空值几乎一定是缺 ros_environment 包：
      sudo apt install -y ros-humble-ros-environment
  没有它 rosdep 无法解析**任何** key，而报错看起来像是包不存在。"
ok "ROS_DISTRO=${ROS_DISTRO}  ROS_VERSION=${ROS_VERSION:-?}"

# AMENT_PREFIX_PATH 的**首段**必须是 humble：首段决定 CMake 的搜索优先级
first_prefix="${AMENT_PREFIX_PATH%%:*}"
[ "$first_prefix" = "$ROS_UNDERLAY" ] \
    || die "AMENT_PREFIX_PATH 首段是 '${first_prefix}'，期望 '${ROS_UNDERLAY}'"
ok "AMENT_PREFIX_PATH 首段 = ${first_prefix}"

# 再确认厂商 ROS 没有从任何途径混进来
[[ "${AMENT_PREFIX_PATH}" != *"${VENDOR_ROS}"* ]] \
    || die "source underlay 之后厂商 ROS 仍在 AMENT_PREFIX_PATH 里，环境不干净"
[[ "${CMAKE_PREFIX_PATH:-}" != *"${VENDOR_ROS}"* ]] \
    || die "CMAKE_PREFIX_PATH 里有厂商 ROS，CMake 会找到厂商头文件"
ok "厂商 ROS 未混入构建环境"

# ---------------------------------------------------------------------
# 断言 2：构建必需的包在位
#
# 只查**构建期**真正需要的。仿真/GUI 依赖（rviz2、gz_*）全是 exec_depend，
# 构建不需要它们——这一点是踩过坑才弄清的：一开始用 --packages-skip 跳过
# 仿真包，结果 perception 的 exec_depend 指向被跳过的包，colcon 去找它的
# package.sh 找不到就报错，连带 5 个包 Aborted。改用 --packages-ignore 才对。
# ---------------------------------------------------------------------
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

# ---------------------------------------------------------------------
# 不构建的包，及其理由
#
# ⚠️ 用 --packages-ignore 而**不是** --packages-skip：ignore 是"当它不存在"，
#    skip 是"存在但不建"，后者会让依赖它的包去找不存在的 package.sh。
# ---------------------------------------------------------------------
IGNORE=(
    # 缺 Livox-SDK2 编不过。而且机器人上**不该**用我们这份：厂商已在跑自己的
    # 雷达驱动，再起一个等于同一台雷达两个驱动。
    livox_ros_driver2
    # 缺 aws-robomaker-small-warehouse-world submodule。纯仿真世界，机器人上无意义。
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

# ---------------------------------------------------------------------
# 构建后校验：产物是否真的对着 humble 编译的
#
# 这一步是"断言"的闭环——前面查的是环境，这里查的是**产物**。
# ---------------------------------------------------------------------
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
