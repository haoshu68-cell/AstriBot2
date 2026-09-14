#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM=:0
XAUTH=/run/user/1000/gdm/Xauthority
PORT=5900
PASSWD_FILE="${HOME}/.vnc/passwd"
LOG=/tmp/x11vnc.log

die() { echo "[vnc_view][ERROR] $*" >&2; exit 1; }
ok()  { echo -e "\033[32m[vnc_view][OK]\033[0m $*"; }

command -v x11vnc >/dev/null || die "x11vnc 未安装。装它只动 1 个包、不碰任何 ROS 包：
    sudo apt-get install -y x11vnc"

if [ ! -f "$PASSWD_FILE" ]; then
    die "还没有 VNC 口令。先跑一次（会交互问两遍）：
    x11vnc -storepasswd
它会写到 ${PASSWD_FILE}。
注意 VNC 协议规定口令**最多 8 个字符**，第 9 位起被静默截断 ——
所以别用长口令然后以为它生效了。真正的防线是 -localhost + ssh。"
fi

[ -e "$XAUTH" ] || die "找不到 X 认证文件 ${XAUTH}（图形会话没起？）"
DISPLAY="$DISPLAY_NUM" XAUTHORITY="$XAUTH" xdpyinfo >/dev/null 2>&1 \
    || die "打不开 ${DISPLAY_NUM}。确认 gdm3 在跑：systemctl is-active gdm3"
ok "物理桌面 ${DISPLAY_NUM} 可访问"

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

echo "===== 启动 x11vnc（仅 127.0.0.1:${PORT}）====="
x11vnc -display "$DISPLAY_NUM" -auth "$XAUTH" \
       -rfbauth "$PASSWD_FILE" -rfbport "$PORT" \
       -localhost -forever -shared -bg -o "$LOG" -q

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
