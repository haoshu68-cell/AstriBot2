#!/usr/bin/env bash
# =====================================================================
# 在机器人上起一个 VNC 服务，把**已有的物理桌面 :0** 共享出去。
# 用途：在 PC 上看 rviz（建图/导航/探索），替代跑不通的 NoMachine。
#
# 关键设计：**只绑 127.0.0.1**（x11vnc -localhost）
#   · 不在 WiFi 或业务网段上开任何新端口 —— 与"WiFi 只做 ssh 运维"一致
#   · PC 侧用 ssh 隧道过来，画面流量跑在 ssh 里，顺带被 ssh 加密
#   · VNC 协议的口令上限只有 8 个字符，让它完全不面对网络是更稳的做法
#
# 只读：本脚本不发布任何 ROS 话题，不触发机器人运动。
#   （注意：VNC 给的是完整桌面，桌面里当然能开终端做任何事 ——
#     这一点和 NoMachine 没有区别，闸门不在这里。）
# =====================================================================
set -euo pipefail

DISPLAY_NUM=:0
XAUTH=/run/user/1000/gdm/Xauthority
PORT=5900
PASSWD_FILE="${HOME}/.vnc/passwd"
LOG=/tmp/x11vnc.log

die() { echo "[vnc_view][ERROR] $*" >&2; exit 1; }
ok()  { echo -e "\033[32m[vnc_view][OK]\033[0m $*"; }

# ---- 1. 依赖 ----
command -v x11vnc >/dev/null || die "x11vnc 未安装。装它只动 1 个包、不碰任何 ROS 包：
    sudo apt-get install -y x11vnc"

# ---- 2. 口令文件（由你自己设，脚本不代劳）----
if [ ! -f "$PASSWD_FILE" ]; then
    die "还没有 VNC 口令。先跑一次（会交互问两遍）：
    x11vnc -storepasswd
它会写到 ${PASSWD_FILE}。
注意 VNC 协议规定口令**最多 8 个字符**，第 9 位起被静默截断 ——
所以别用长口令然后以为它生效了。真正的防线是 -localhost + ssh。"
fi

# ---- 3. 目标显示必须真的在 ----
[ -e "$XAUTH" ] || die "找不到 X 认证文件 ${XAUTH}（图形会话没起？）"
DISPLAY="$DISPLAY_NUM" XAUTHORITY="$XAUTH" xdpyinfo >/dev/null 2>&1 \
    || die "打不开 ${DISPLAY_NUM}。确认 gdm3 在跑：systemctl is-active gdm3"
ok "物理桌面 ${DISPLAY_NUM} 可访问"

# ---- 4. 幂等：已经在跑就不要再起一个 ----
# 两个 x11vnc 抢同一个端口时，后起的那个会失败但前一个还在，
# 很容易误读成"起不来"。
if pgrep -u "$(id -u)" -x x11vnc >/dev/null 2>&1; then
    ok "x11vnc 已在运行（pid $(pgrep -u "$(id -u)" -x x11vnc | tr '\n' ' ')）"
    ss -tln 2>/dev/null | grep ":${PORT}" || true
    echo
    echo "要重起就先： pkill -x x11vnc"
    exit 0
fi
if ss -tln 2>/dev/null | grep -q ":${PORT}\b"; then
    die "端口 ${PORT} 已被别的进程占用：$(ss -tlnp 2>/dev/null | grep ":${PORT}" | head -1)"
fi

# ---- 5. 起服务 ----
#   -localhost  只监听 127.0.0.1（这是本方案的核心，别去掉）
#   -forever    客户端断开后继续等下一个，不退出
#   -shared     允许多个查看者
#   -bg         转后台并把日志写到 -o 指定的文件
echo "===== 启动 x11vnc（仅 127.0.0.1:${PORT}）====="
x11vnc -display "$DISPLAY_NUM" -auth "$XAUTH" \
       -rfbauth "$PASSWD_FILE" -rfbport "$PORT" \
       -localhost -forever -shared -bg -o "$LOG" -q

# -bg 立刻返回，端口不一定马上就绑好，所以要等一下再判定
for _ in 1 2 3 4 5 6 7 8 9 10; do
    ss -tln 2>/dev/null | grep -q "127.0.0.1:${PORT}" && break
    read -r -t 0.3 _ < /dev/zero 2>/dev/null || true
done

if ss -tln 2>/dev/null | grep -q "127.0.0.1:${PORT}"; then
    ok "已监听 $(ss -tln | grep ":${PORT}" | awk '{print $4}' | tr '\n' ' ')"
else
    echo "[vnc_view][ERROR] ${PORT} 没绑上。日志：" >&2
    tail -20 "$LOG" >&2
    exit 1
fi

# ---- 6. 自检：确认它没有暴露到网络上 ----
# 这一条对应"不在 WiFi/业务网段上开新端口"这个约束，必须是可执行的检查。
if ss -tln 2>/dev/null | grep ":${PORT}" | grep -qvE "127\.0\.0\.1|\[::1\]"; then
    echo "[vnc_view][ERROR] ${PORT} 绑到了非回环地址，与设计不符。已杀掉。" >&2
    ss -tln | grep ":${PORT}" >&2
    pkill -x x11vnc || true
    exit 1
fi
ok "仅回环，网络上不可见"

cat <<EOF

===== PC 端怎么连（在你的开发机上跑）=====

  # ① 开隧道（这个终端保持不关）
  ssh -N -L ${PORT}:localhost:${PORT} astribot@10.249.22.137

  # ② 另开一个终端，用已经装好的 remmina
  remmina -c vnc://localhost:${PORT}
  #   口令就是你 x11vnc -storepasswd 设的那个

===== 连上后在那个桌面里起 rviz =====

  ${HOME}/Downloads/astribot_sdk_aarch64/view_chain.sh

日志：${LOG}
停止：pkill -x x11vnc
EOF
