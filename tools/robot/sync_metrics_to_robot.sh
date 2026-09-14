#!/usr/bin/env bash

set -eo pipefail   # 不要加 set -u（source ROS setup.bash 会静默退出）

ROBOT="${ROBOT:-astribot@10.249.22.137}"   # 别名 orin/astribot 都不解析，只能用 IP
REMOTE_ROOT="${REMOTE_ROOT:-/home/astribot/Downloads/astribot_sdk_aarch64}"
REMOTE_WS="$REMOTE_ROOT/ws_robot"
PKG_SRC="ws_robot/src/astribot_s1_navigation"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

RSYNC_FLAGS=(-az --no-owner --no-group --exclude '__pycache__' --exclude '*.pyc')
[ -n "$DRY_RUN" ] && RSYNC_FLAGS+=(-n -v)

echo "==== [1/4] 同步包内文件 ===="
ITEMS=(
  "$PKG_SRC/astribot_s1_navigation/explore_metrics"
  "$PKG_SRC/astribot_s1_navigation/explore_metrics_recorder_node.py"
  "$PKG_SRC/astribot_s1_navigation/posture_monitor_policy.py"
  "$PKG_SRC/setup.py"
  "$PKG_SRC/scripts"
  "$PKG_SRC/test"
)
for it in "${ITEMS[@]}"; do
  if [ ! -e "$it" ]; then
    echo "🔴 本地缺 $it —— 清单与仓库不符，停"; exit 1
  fi
  src="$it"; [ -d "$it" ] && src="$it/"
  dst="$REMOTE_ROOT/${it}"; [ -d "$it" ] && dst="$REMOTE_ROOT/${it}/"
  ssh "$ROBOT" "mkdir -p \"\$(dirname '$dst')\"" </dev/null
  rsync "${RSYNC_FLAGS[@]}" "$src" "$ROBOT:$dst"
  echo "  → $it"
done

echo
echo "==== [2/4] 同步工具脚本 ===="
for f in tools/report_explore_metrics.py tools/robot/robot_metrics_session.sh \
         tools/robot/check_metrics_inputs.py; do
  ssh "$ROBOT" "mkdir -p '$REMOTE_ROOT/$(dirname "$f")'" </dev/null
  rsync "${RSYNC_FLAGS[@]}" "$f" "$ROBOT:$REMOTE_ROOT/$f"
  echo "  → $f"
done
[ -n "$DRY_RUN" ] && { echo; echo "DRY_RUN：到此为止，没有重建。"; exit 0; }
ssh "$ROBOT" "chmod +x '$REMOTE_ROOT/tools/robot/robot_metrics_session.sh' \
  '$REMOTE_ROOT/tools/robot/check_metrics_inputs.py' \
  '$REMOTE_ROOT/tools/report_explore_metrics.py'" </dev/null

echo
echo "==== [3/4] 实机重建 astribot_s1_navigation ===="
ssh "$ROBOT" "bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  cd $REMOTE_WS
  colcon build --symlink-install --packages-select astribot_s1_navigation \
    2>&1 | tail -25
'" </dev/null || echo "  (构建返回非 0，看下面校验)"

echo
echo "==== [4/4] 校验（判据=装出来的可执行文件在不在 + 失败词）===="
ssh "$ROBOT" "bash -lc '
  W=$REMOTE_WS
  L=\$W/install/astribot_s1_navigation/lib/astribot_s1_navigation
  echo \"--- 装出来的可执行文件 ---\"; ls \$L 2>&1
  echo \"--- 关键项 ---\"
  for x in explore_metrics_recorder_node; do
    [ -e \"\$L/\$x\" ] && echo \"  ✅ \$x\" || echo \"  🔴 \$x 缺失\"
  done
  echo \"--- 构建日志里的失败词 ---\"
  grep -riE \"failed|aborted|not processed|Traceback\" \$W/log/latest_build/*/stderr.log \
       \$W/log/latest_build/events.log 2>/dev/null | head -10 || echo \"  (无)\"
'" </dev/null

echo
echo "下一步（在实机上）："
echo "  bash $REMOTE_ROOT/tools/robot/robot_metrics_session.sh start"
