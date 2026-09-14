#!/usr/bin/env bash

set -eo pipefail

WS="${WS:-/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot}"
ACTION="${1:-}"

STATE_ROOT="${STATE_ROOT:-/home/astribot/explore_metrics}"
CURRENT_LABEL_FILE="$STATE_ROOT/.current_label"

if [ -n "$RUN_LABEL" ]; then
  :                                     # 调用者显式指定，照用
elif [ "$ACTION" = start ]; then
  RUN_LABEL="hw$(date +%m%d_%H%M)"      # 新会话，只在这里生成
elif [ -s "$CURRENT_LABEL_FILE" ]; then
  RUN_LABEL="$(cat "$CURRENT_LABEL_FILE")"
  echo "  会话标签 $RUN_LABEL（取自 $CURRENT_LABEL_FILE）"
elif [ -n "$ACTION" ] && [ "$ACTION" != status ] && [ "$ACTION" != stop ] && [ "$ACTION" != report ]; then
  RUN_LABEL="hw$(date +%m%d_%H%M)"      # 用法提示分支，标签用不到
else
  echo "🔴 没有 $CURRENT_LABEL_FILE，也没给 RUN_LABEL —— 不知道该操作哪个会话。"
  echo "   现有目录：$(ls -1 "$STATE_ROOT" 2>/dev/null | tr '\n' ' ')"
  echo "   显式指定：RUN_LABEL=hw0908_2024 bash $0 $ACTION"
  exit 2
fi

OUT_ROOT="${OUT_ROOT:-$STATE_ROOT/$RUN_LABEL}"
BAG_DIR="$OUT_ROOT/bag"
BAG_DIR_ACTIONS="$OUT_ROOT/bag_actions"
METRICS_DIR="$OUT_ROOT/metrics"
PIDS_DIR="$OUT_ROOT/pids"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"

MAP_FRAME="${MAP_FRAME:-map}"
BASE_FRAME="${BASE_FRAME:-astribot_torso_base}"

ACTION="${1:-}"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -e

BAG_TOPICS=(
  /tf /tf_static /odom /joint_states
  /map_nav /map_scan_filtered_prob
  /scan /scan_from_cloud
  /cmd_vel /cmd_vel_nav_body /cmd_vel_nav_body_raw /cmd_vel_pre_arm_coupling
  /speed_limit
  /plan /plan_smoothed /unsmoothed_plan /transformed_global_plan /local_plan
  /exploration/state /exploration/current_goal /exploration/complete
  /global_costmap/costmap /local_costmap/costmap
  /global_costmap/published_footprint /local_costmap/published_footprint
  /rosout /diagnostics
)

ACTION_TOPICS=(
  /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
  /compute_path_to_pose/_action/status /follow_path/_action/status
)

read_pid() { cat "$PIDS_DIR/$1.pid" 2>/dev/null || true; }


do_start() {
  mkdir -p "$METRICS_DIR" "$PIDS_DIR"
  printf '%s\n' "$RUN_LABEL" > "$CURRENT_LABEL_FILE"

  echo "==== [1/3] 前置检查（判据=实收帧数，不是话题存在）===="
  if ! python3 "$WS/../tools/robot/check_metrics_inputs.py" \
        --base-frame "$BASE_FRAME" --map-frame "$MAP_FRAME"; then
    echo "🔴 前置检查未过（见上表）。没有开录，也没有产生任何数据。"
    echo "   注意：/exploration/* 只有在探索协调器在跑时才有；若你打算手动遥控，"
    echo "   那几条是空的，逐轮指标就出不来（bag 仍然完整可用）。"
    return 1
  fi

  echo
  echo "==== [2/3] 开录 ===="
  nohup ros2 bag record -o "$BAG_DIR" --max-bag-size 2000000000 \
    "${BAG_TOPICS[@]}" > "$OUT_ROOT/bag.log" 2>&1 &
  echo $! > "$PIDS_DIR/bag.pid"
  echo "  bag pid=$(cat "$PIDS_DIR/bag.pid")  ->  $BAG_DIR"

  nohup ros2 bag record -o "$BAG_DIR_ACTIONS" --include-hidden-topics \
    "${ACTION_TOPICS[@]}" > "$OUT_ROOT/bag_actions.log" 2>&1 &
  echo $! > "$PIDS_DIR/bag_actions.pid"
  echo "  bag(action) pid=$(cat "$PIDS_DIR/bag_actions.pid")  ->  $BAG_DIR_ACTIONS"

  RECORDER="$WS/install/astribot_s1_navigation/lib/astribot_s1_navigation/explore_metrics_recorder_node"
  if [ ! -x "$RECORDER" ]; then
    echo "🔴 没有 $RECORDER —— 包没重建。在开发机上跑 tools/robot/sync_metrics_to_robot.sh"
    return 1
  fi
  nohup "$RECORDER" --ros-args \
    -p use_sim_time:=false \
    -p output_dir:="$METRICS_DIR" \
    -p run_label:="$RUN_LABEL" \
    -p map_frame:="$MAP_FRAME" \
    -p base_frame:="$BASE_FRAME" \
    > "$OUT_ROOT/recorder.log" 2>&1 &
  echo $! > "$PIDS_DIR/recorder.pid"
  echo "  recorder pid=$(cat "$PIDS_DIR/recorder.pid")  ->  $METRICS_DIR"

  echo
  echo "==== [3/3] 复核（看字节在长，不看进程存活）===="
  sleep 8
  echo "  bag: $(du -sh "$BAG_DIR" 2>/dev/null | cut -f1)   action: $(du -sh "$BAG_DIR_ACTIONS" 2>/dev/null | cut -f1)"
  if grep -qi 'Hidden topics are not recorded' "$OUT_ROOT/bag_actions.log" 2>/dev/null; then
    echo "  🔴 action bag 仍在跳过隐藏话题 —— 检查 --include-hidden-topics"
  fi
  for f in bag bag_actions recorder; do
    p=$(read_pid "$f")
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
      echo "  $f 活着 (pid=$p)"
    else
      echo "  🔴 $f 已退出，看 $OUT_ROOT/$f.log"
    fi
  done
  echo
  echo "  标签=$RUN_LABEL  目录=$OUT_ROOT"
  echo "  机器人**不会**因为这个脚本动 —— 该由你或协调器去驱动它。"
  echo "  停止：bash $0 stop     出报告：bash $0 report"
}

do_status() {
  echo "==== 进程 ===="
  for f in bag bag_actions recorder; do
    p=$(read_pid "$f")
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
      echo "  $f 活着 (pid=$p)"
    else
      echo "  $f 不在"
    fi
  done
  echo "==== 体积 ===="
  du -sh "$BAG_DIR" "$BAG_DIR_ACTIONS" 2>/dev/null
  echo "==== 已完成轮次 ===="
  c="$METRICS_DIR/${RUN_LABEL}_rounds.csv"
  if [ -f "$c" ]; then
    echo "  $(( $(wc -l < "$c") - 1 )) 轮   ($c)"
  else
    echo "  还没有 ${RUN_LABEL}_rounds.csv（目录里：$(ls "$METRICS_DIR" 2>/dev/null | tr '\n' ' '))"
  fi
  echo "==== 录制器自检 ===="
  grep -E "自检|订阅|收到|TF" "$OUT_ROOT/recorder.log" 2>/dev/null | tail -8
}

do_stop() {
  echo "==== 停录（SIGINT，让 bag 正常封口）===="
  echo "  会话 $RUN_LABEL  ->  $OUT_ROOT"
  if [ ! -d "$PIDS_DIR" ]; then
    echo "🔴 没有 $PIDS_DIR —— 这个标签下没开过录。现有目录："
    echo "   $(ls -1 "$STATE_ROOT" 2>/dev/null | tr '\n' ' ')"
    return 1
  fi
  sent=0
  for f in bag bag_actions recorder; do
    p=$(read_pid "$f")
    if [ -n "$p" ] && kill -INT "$p" 2>/dev/null; then
      echo "  SIGINT -> $p ($f)"
      sent=$((sent + 1))
    else
      echo "  ⚠️  $f 没有可用 pid 或进程已不在（pid='${p:-空}'）"
    fi
  done
  [ "$sent" -eq 0 ] && echo "  🔴 一个信号都没发出去 —— 下面的封口确认不代表这次 stop 起了作用"
  sleep 10
  echo "==== 封口确认（metadata.yaml 出来了才算）===="
  for d in "$BAG_DIR" "$BAG_DIR_ACTIONS"; do
    if [ -f "$d/metadata.yaml" ]; then
      echo "  ✅ $(basename "$d") 已封口，$(du -sh "$d" | cut -f1)"
    else
      echo "  🔴 $(basename "$d") 没有 metadata.yaml —— bag 未封口，回放会失败"
    fi
  done
  for f in bag bag_actions recorder; do
    p=$(read_pid "$f")
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
      kill -9 "$p" 2>/dev/null || true
      echo "  强杀残留 $p ($f)"
    fi
  done
  echo
  echo "  出报告：bash $0 report"
}

do_report() {
  c="$METRICS_DIR/${RUN_LABEL}_rounds.csv"
  if [ ! -f "$c" ]; then
    echo "🔴 没有 $c —— recorder 没落盘过。目录里：$(ls "$METRICS_DIR" 2>/dev/null | tr '\n' ' ')"
    return 1
  fi
  rounds=$(( $(wc -l < "$c") - 1 ))   # 减掉表头
  echo "==== 轮次数 $rounds ===="
  if [ "$rounds" -le 0 ]; then
    echo "⚠️  表里 0 个已完成轮次。报告只会给出链路自检，不含任何路径指标。"
    echo "    最常见原因：协调器处于 PAUSED/IDLE，整段时间没派发过目标"
    echo "    （轮次是按 /exploration/current_goal 切分的）。bag 仍然完整可用。"
  fi
  python3 "$WS/../tools/report_explore_metrics.py" "$METRICS_DIR" \
          --md "$OUT_ROOT/report.md"
  echo
  if [ -f "$OUT_ROOT/report.md" ]; then
    echo "  ✅ 报告：$OUT_ROOT/report.md  ($(wc -l < "$OUT_ROOT/report.md") 行)"
  else
    echo "  🔴 report.md 没有生成 —— 上面的表格只在终端里，没有落盘"
    return 1
  fi
  echo "  ⚠️ 先看报告第 ① 节的数据可用性自检。位姿覆盖率低的轮次，"
  echo "     它们的里程/横偏/净空都不是测量结果，不要当跟踪质量的证据。"
}

case "$ACTION" in
  start)  do_start ;;
  status) do_status ;;
  stop)   do_stop ;;
  report) do_report ;;
  *)
    echo "用法: bash $0 {start|status|stop|report}"
    echo "  环境变量: RUN_LABEL(默认 hw<日期时分>)  OUT_ROOT  WS  MAP_FRAME  BASE_FRAME"
    exit 2 ;;
esac
