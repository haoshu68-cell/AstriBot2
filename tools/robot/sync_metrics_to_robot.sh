#!/usr/bin/env bash
# 把"探索评价指标"这套东西从开发机同步到实机，并在实机重建那一个包。
#
# 在**开发机**上跑：
#   bash tools/robot/sync_metrics_to_robot.sh            # 同步 + 重建 + 校验
#   DRY_RUN=1 bash tools/robot/sync_metrics_to_robot.sh  # 只看会传哪些文件
#
# 只传文件、只重建一个包。不启动任何节点、不动机器人。
#
# --------------------------------------------------------------------------
# 为什么必须重建，而且必须校验重建结果
#
#   · **src 同步了不等于 install 重建了。** `ros2 run` 读的是 install。
#     只比 src 的 md5 会得出"已同步"的假结论，而症状是报错与改动前逐字相同。
#   · --symlink-install 下纯 .py 改动确实不用重建（build/<pkg>/<pkg> 是指向
#     src 的符号链接），**但这次不是纯 .py 改动**：console_scripts 新增了
#     explore_metrics_recorder_node，那个可执行包装脚本必须由构建生成，
#     不重建就永远是"命令找不到"。
#   · setup.py 的 scripts= 列了 scripts/ 下两个文件。**它们不在实机上就直接
#     构建失败**（不是警告），所以同步清单里必须带上整个 scribts 目录。
#   · 校验判据是"install 下那个可执行文件在不在"，不是"colcon 说 finished"。
#     只 grep "packages finished" 曾让 8 笔提交全报 ✅，而每笔都有 failed。
# --------------------------------------------------------------------------

set -eo pipefail   # 不要加 set -u（source ROS setup.bash 会静默退出）

ROBOT="${ROBOT:-astribot@10.249.22.137}"   # 别名 orin/astribot 都不解析，只能用 IP
REMOTE_ROOT="${REMOTE_ROOT:-/home/astribot/Downloads/astribot_sdk_aarch64}"
REMOTE_WS="$REMOTE_ROOT/ws_robot"
PKG_SRC="ws_robot/src/astribot_s1_navigation"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

RSYNC_FLAGS=(-az --no-owner --no-group --exclude '__pycache__' --exclude '*.pyc')
[ -n "$DRY_RUN" ] && RSYNC_FLAGS+=(-n -v)
# 注意**没有 --delete**：实机那棵树上有开发机没有的东西（厂商件、之前的
# 排障脚本），--delete 会连它们一起抹掉。

echo "==== [1/4] 同步包内文件 ===="
# 逐项列出而不是整目录同步：整目录会把开发机的 build/ install/ 和实机
# 不适用的仿真件一起推过去。
ITEMS=(
  "$PKG_SRC/astribot_s1_navigation/explore_metrics"
  "$PKG_SRC/astribot_s1_navigation/explore_metrics_recorder_node.py"
  "$PKG_SRC/astribot_s1_navigation/posture_monitor_policy.py"
  "$PKG_SRC/setup.py"
  "$PKG_SRC/scripts"
  "$PKG_SRC/test"
)
# ⚠️ **清单里刻意没有 config/。** 实机那两个 nav2_params_*.yaml 是按硬件单独
# 调过的，与开发机差 110 行，其中两处一覆盖就是安全回退：
#   · use_sim_time: 实机 False / 开发机 True —— 实机 /clock 没有发布者，
#     误设 True 会让六个节点的时钟恒为 0 永不前进，而 costmap 照发、零告警。
#   · vx_max/vy_max: 实机 0.2（用户为首轮实机指定的上限）/ 开发机 1.0 —— 覆盖
#     等于把实机限速悄悄放大 5 倍。
# 新参数要上实机，必须**逐项手工并进实机那份 yaml**，不能整文件推。
for it in "${ITEMS[@]}"; do
  if [ ! -e "$it" ]; then
    echo "🔴 本地缺 $it —— 清单与仓库不符，停"; exit 1
  fi
  # 目录用尾斜杠传内容，文件直接传
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
# 非交互 ssh 里 PATH 没有 ros2，必须先 source。且**不要 source 厂商 env.sh**：
# 这台机器 IP 是 .11，env.sh 会因此生成并覆盖 DDS profile。
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
  for x in explore_metrics_recorder_node run_speed_sweep.sh; do
    [ -e \"\$L/\$x\" ] && echo \"  ✅ \$x\" || echo \"  🔴 \$x 缺失\"
  done
  echo \"--- 构建日志里的失败词 ---\"
  grep -riE \"failed|aborted|not processed|Traceback\" \$W/log/latest_build/*/stderr.log \
       \$W/log/latest_build/events.log 2>/dev/null | head -10 || echo \"  (无)\"
'" </dev/null

echo
echo "下一步（在实机上）："
echo "  bash $REMOTE_ROOT/tools/robot/robot_metrics_session.sh start"
