#!/usr/bin/env bash
# =====================================================================
# 实机可视化启动脚本（在**机器人上**跑，画面经 NoMachine 或 ssh -X 传到 PC）
#
# 为什么必须在机器人上跑 rviz2，而不能在 PC 上跑：
#   厂商 Fast DDS 配置 useBuiltinTransports=false + interfaceWhiteList
#   只有 192.168.0.11 与 127.0.0.1；且 iptables 在 wlP1p1s0 上 DROP 了
#   239.255.0.1（DDS 多播发现）。PC 走 WiFi 段根本发现不到任何话题。
#   rviz2 与话题同机时 DDS 走 loopback —— 而 127.0.0.1 恰在白名单里。
#
# 只读：本脚本不发布任何话题，不触发机器人运动。
#   rviz 配置里 Tools 段被显式限定为相机/选择/测量，
#   2D Goal Pose（会往 /goal_pose 发目标）已从工具栏移除。
# =====================================================================
set -euo pipefail

RVIZ_PKG=astribot_s1_perception
RVIZ_CFG_NAME=chain_view.rviz

die() { echo "[view_chain][ERROR] $*" >&2; exit 1; }
ok()  { echo -e "\033[32m[view_chain][OK]\033[0m $*"; }

# ---- 1. 找 env_robot.sh ----------------------------------------------------
# 仓库里它在 tools/robot/ 下；下发到机器人后被放在 SDK 根目录。两处都找。
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_SH=""
for cand in "${HERE}/env_robot.sh" "${HERE}/../../env_robot.sh"; do
    [ -f "$cand" ] && { ENV_SH="$(cd "$(dirname "$cand")" && pwd)/$(basename "$cand")"; break; }
done
[ -n "$ENV_SH" ] || die "找不到 env_robot.sh（在 ${HERE} 与 ${HERE}/../../ 都没有）"

# ---- 2. 检查有没有可用的 X display ---------------------------------------
# 放在 source 之前：环境没问题却没显示时，rviz2 会抛 Ogre RenderingAPIException
# 并 core dump，报错指向 OgreGLXGLSupport.cpp，很难联想到"只是没有 DISPLAY"。
# 注意 QT_QPA_PLATFORM=offscreen **救不了**：Ogre 绕过 Qt 直接开 GLX。
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

# ---- 3. 运行环境 ----------------------------------------------------------
# env_robot.sh 里 source 的 ROS setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES，
# 与 set -u 冲突，所以这一段临时关掉。
set +u
# shellcheck disable=SC1090
source "$ENV_SH"
set -u

ok "DISPLAY=${DISPLAY}  DOMAIN=${ROS_DOMAIN_ID:-unset}  rviz2=$(command -v rviz2 || echo 'NOT FOUND')"
command -v rviz2 >/dev/null || die "rviz2 不在 PATH 上（env_robot.sh 没生效？）"

# ---- 4. 找 rviz 配置 ------------------------------------------------------
# setup.py 只 glob rviz/*.rviz —— 放在 config/ 下的 .rviz **不会被安装**，
# 那时这里会找不到文件（曾经踩过）。
CFG=""
if PREFIX="$(ros2 pkg prefix "$RVIZ_PKG" 2>/dev/null)"; then
    cand="${PREFIX}/share/${RVIZ_PKG}/rviz/${RVIZ_CFG_NAME}"
    [ -f "$cand" ] && CFG="$cand"
fi
[ -n "$CFG" ] || die "找不到 ${RVIZ_CFG_NAME}。确认它在包的 rviz/ 目录下（不是 config/），并重新 colcon build ${RVIZ_PKG}"
ok "配置：${CFG}"

# ---- 5. 只读自检：确认工具栏里没有会发目标的工具 --------------------------
# 与"不触发运动"这条约束对应的可执行检查。配置被改回默认工具集时这里会拦下。
# 注意必须先剥掉注释行：配置里的说明文字本身就写着 SetGoal 这个名字，
# 直接 grep 整个文件会命中注释、把正确的配置误判成违规（第一版就是这么错的）。
CFG_CODE="$(grep -vE '^[[:space:]]*#' "$CFG")"
if ! grep -q "^  Tools:" <<<"$CFG_CODE"; then
    die "配置里没有 Tools 段 —— rviz2 会装载默认工具集，其中含 2D Goal Pose（发 /goal_pose）。拒绝启动。"
fi
if grep -qE "rviz_default_plugins/(SetGoal|SetInitialPose)" <<<"$CFG_CODE"; then
    die "配置里含 SetGoal / SetInitialPose —— 点一下就会发目标/重定位。拒绝启动。"
fi
ok "工具栏只读（无 SetGoal / SetInitialPose）"

# ---- 6. 起 rviz -----------------------------------------------------------
echo "===== rviz2 启动（Ctrl-C 退出）====="
exec rviz2 -d "$CFG" "$@"
