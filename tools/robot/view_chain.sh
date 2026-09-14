#!/usr/bin/env bash
set -euo pipefail

RVIZ_PKG=astribot_s1_perception
RVIZ_CFG_NAME=chain_view.rviz
ALLOW_GOAL_TOOL=0

die() { echo "[view_chain][ERROR] $*" >&2; exit 1; }
ok()  { echo -e "\033[32m[view_chain][OK]\033[0m $*"; }

usage() {
    cat <<'EOF'
用法: view_chain.sh [选项] [-- rviz 额外参数...]

  (无选项)              用只读配置 chain_view.rviz —— 工具栏没有任何会发布话题的工具
  --allow-goal-tool     放行含 2D Goal Pose 的配置（默认切到 nav_view.rviz）
  --config <名字.rviz>  指定包内 rviz/ 目录下的配置文件名
  -h | --help           本帮助

⚠️ --allow-goal-tool 之后 rviz 工具栏会有 "2D Goal Pose"，点一下就往 /goal_pose
   发目标。nav2 的 bt_navigator + controller_server 起着时，机器人会真的移动。
   用之前确认：周围有空间、手放在物理急停上。
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --allow-goal-tool) ALLOW_GOAL_TOOL=1; RVIZ_CFG_NAME=nav_view.rviz; shift ;;
        --config)          [ $# -ge 2 ] || die "--config 后面要跟文件名"
                           RVIZ_CFG_NAME="$2"; shift 2 ;;
        -h|--help)         usage; exit 0 ;;
        --)                shift; break ;;
        *)                 die "未知选项 $1（-h 看用法）" ;;
    esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_SH=""
for cand in "${HERE}/env_robot.sh" "${HERE}/../../env_robot.sh"; do
    [ -f "$cand" ] && { ENV_SH="$(cd "$(dirname "$cand")" && pwd)/$(basename "$cand")"; break; }
done
[ -n "$ENV_SH" ] || die "找不到 env_robot.sh（在 ${HERE} 与 ${HERE}/../../ 都没有）"

if [ -z "${DISPLAY:-}" ]; then
    cat >&2 <<'EOF'
[view_chain][ERROR] DISPLAY 未设置 —— rviz2 需要真实的 X display（Ogre 走 GLX，
                    QT_QPA_PLATFORM=offscreen 无效，会直接 core dump）。

拿到显示的方式：

  (A) NoMachine —— **唯一实测可用**，GPU 加速（OpenGL 4.6）
      PC 上装 NoMachine 客户端，连 10.249.22.137:4000，用 astribot 登录，
      接到物理桌面 :0，然后在那个桌面的终端里跑本脚本。

  (B) ssh -X / -Y —— ❌ **实测不可用，不要浪费时间**
      转发的 display 上 Ogre 建不出 GLX 窗口：
        InvalidParametersException: Window with name 'OgreWindow(0)' already exists
        Unable to create the rendering window after 100 tries  → core dump
      根因是现代 Xorg 默认关闭 indirect GLX（要 +iglx 才开），
      而 rviz 的 Ogre 必须要直接 GLX 上下文。ssh 转发本身是通的
      （DISPLAY=localhost:10.0 已正确设置），卡的是 GL 而不是 X。

  (C) 已经 ssh 进来了、想画在机器人自己的屏幕上（调试用）：
      export DISPLAY=:0
      export XAUTHORITY=/run/user/1000/gdm/Xauthority
EOF
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ENV_SH"
set -u

ok "DISPLAY=${DISPLAY}  DOMAIN=${ROS_DOMAIN_ID:-unset}  rviz2=$(command -v rviz2 || echo 'NOT FOUND')"
command -v rviz2 >/dev/null || die "rviz2 不在 PATH 上（env_robot.sh 没生效？）"

CFG=""
if PREFIX="$(ros2 pkg prefix "$RVIZ_PKG" 2>/dev/null)"; then
    cand="${PREFIX}/share/${RVIZ_PKG}/rviz/${RVIZ_CFG_NAME}"
    [ -f "$cand" ] && CFG="$cand"
fi
[ -n "$CFG" ] || die "找不到 ${RVIZ_CFG_NAME}。确认它在包的 rviz/ 目录下（不是 config/），并重新 colcon build ${RVIZ_PKG}"
ok "配置：${CFG}"

CFG_CODE="$(grep -vE '^[[:space:]]*#' "$CFG")"
if ! grep -q "^  Tools:" <<<"$CFG_CODE"; then
    die "配置里没有 Tools 段 —— rviz2 会装载默认工具集，其中含 2D Goal Pose（发 /goal_pose）。拒绝启动。"
fi
if grep -qE "rviz_default_plugins/(SetGoal|SetInitialPose)" <<<"$CFG_CODE"; then
    if [ "$ALLOW_GOAL_TOOL" -ne 1 ]; then
        die "配置 ${RVIZ_CFG_NAME} 含 SetGoal / SetInitialPose —— 点一下就会发目标。
拒绝启动。确实要用请显式加 --allow-goal-tool。"
    fi
    cat >&2 <<'EOF'

  ############################################################
  ##  ⚠️  工具栏含 2D Goal Pose —— 机器人会真的移动  ⚠️     ##
  ############################################################
  · 点一下就往 /goal_pose 发目标 → bt_navigator → controller_server → cmd_vel
  · 底盘是**位置指令开环积分**（examples/202）：轮子打滑时指令位置持续超前，
    唯一硬保护是桥接的 leash（leash_xy_m 0.25 / leash_theta_rad 0.35）
  · 确认：周围有空间、手放在物理急停上
  · 停止写通路： pgrep -a -f bridge_container   拿到 pid 后 kill <pid>
    （**不要** pkill -f "bridge_container" —— 模式串会命中你自己的命令行，
      把当前 shell 一起杀掉。本项目已栽三次。）

EOF
else
    ok "工具栏只读（无 SetGoal / SetInitialPose）"
fi

echo "===== rviz2 启动（Ctrl-C 退出）====="
exec rviz2 -d "$CFG" "$@"
