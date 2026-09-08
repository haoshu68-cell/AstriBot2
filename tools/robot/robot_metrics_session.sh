#!/usr/bin/env bash
# 实机：一次"录 bag + 采指标 + 跑完出报告"的完整会话。
#
# 只读。这个脚本**不发任何速度指令、不使能任何写通路、不动机器人**：
# 它只订阅话题、写文件。机器人由谁来开（你手动遥控、还是探索协调器派发目标）
# 由你决定，脚本只负责把那段时间的数据完整记下来。
#
# 用法（在实机上跑，或从开发机 ssh 过去跑）：
#   bash tools/robot/robot_metrics_session.sh start            # 开录
#   bash tools/robot/robot_metrics_session.sh status           # 看在录什么、收到多少帧
#   bash tools/robot/robot_metrics_session.sh stop             # 停录并封口 bag
#   bash tools/robot/robot_metrics_session.sh report            # 出指标报告
#   RUN_LABEL=hw01 bash tools/robot/robot_metrics_session.sh start   # 指定标签
#
# 产物：/home/astribot/explore_metrics/<RUN_LABEL>/
#   bag/                   白名单话题
#   bag_actions/           action 状态（隐藏话题，必须单独一个 bag）
#   metrics/<label>_rounds.csv   逐轮指标
#   report.md              报告
#
# --------------------------------------------------------------------------
# 实机与仿真的差异（这几条都是实测出来的，每一条都能让部署静默失败）
#
#   1. **/clock 没有发布者**。所以 use_sim_time 必须是 false。
#      误开的后果是所有时间戳恒为 0，而 costmap 照发、判据照过、零告警。
#   2. **ros2 的子命令只有 7 个**。实机 ros2 只有
#      bag/daemon/extension_points/extensions/node/param/service ——
#      `ros2 topic`、`ros2 run`、`ros2 launch` 全部报 invalid choice。
#      所以：话题判据走 rclpy 直接问图；节点按 install 下的可执行文件路径直接起。
#      `ros2 bag` 在（rosbag2_py 也在），录 bag 没问题。
#   3. **地图话题不是 /map**。实机 /map pub=0，真正在发的是 /map_nav
#      （OccupancyGrid, pub=1）和 /map_scan_filtered_prob。录制器本身不订阅
#      地图（它只用 TF 和 costmap），但 bag 要录对，否则回放时没有地图。
#   4. **非交互 ssh 里 PATH 没有 ros2**，必须先 source。且
#      **不要 source 厂商 env.sh** —— 这台机器 IP 是 .11，env.sh 会因此生成并
#      覆盖 DDS profile（跳过覆盖的条件是 IP==192.168.0.10）。
#   5. ROS_DOMAIN_ID 是 **25**（不是仿真那侧的 42）。查询 shell 不带，
#      症状是"节点全都 Node not found"，看着像整个系统没起来。
# --------------------------------------------------------------------------

# 不要开 set -u：set -u 下 source ROS setup.bash 会**一个字都不打印**地退出，
# 看起来像后面的探针坏了。
set -eo pipefail

WS="${WS:-/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot}"
RUN_LABEL="${RUN_LABEL:-hw$(date +%m%d_%H%M)}"
OUT_ROOT="${OUT_ROOT:-/home/astribot/explore_metrics/$RUN_LABEL}"
BAG_DIR="$OUT_ROOT/bag"
BAG_DIR_ACTIONS="$OUT_ROOT/bag_actions"
METRICS_DIR="$OUT_ROOT/metrics"
PIDS_DIR="$OUT_ROOT/pids"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"

# 实机坐标系。base_frame 是 astribot_torso_base —— 本项目没有 base_link，
# 根 frame 就叫这个名字，写成 base_link 的后果是 TF 全查不到、位姿列全空。
MAP_FRAME="${MAP_FRAME:-map}"
BASE_FRAME="${BASE_FRAME:-astribot_torso_base}"

ACTION="${1:-}"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -e

# 录进 bag 的话题。用白名单不用 -a：-a 会把 livox 原始点云（每帧两万点、
# 四路）一起吸进来，实机磁盘和 CPU 都不划算。
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
# 注意这里**没有 /clock**：实机它没有发布者，点名录一个不存在的话题只会
# 让 bag 多一条无用的空记录。回放时用 use_sim_time:=false。

ACTION_TOPICS=(
  /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
  /compute_path_to_pose/_action/status /follow_path/_action/status
)

# ---------------------------------------------------------------- 子命令

do_start() {
  mkdir -p "$METRICS_DIR" "$PIDS_DIR"

  echo "==== [1/3] 前置检查（判据=实收帧数，不是话题存在）===="
  # 为什么不能只看话题在不在：本项目实测过五个话题全部 pub=1 但 0Hz
  # （BEST_EFFORT 发布者 + RELIABLE 订阅者，只有一条 WARNING）。
  # 也实测过节点发现到了而话题 pub 恒 0。所以判据只能是实收帧数。
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

  # action 话题是**隐藏话题**：显式点名也不够，ros2 bag record 会直接跳过
  # 并只打一条 `Hidden topics are not recorded`。"每目标耗时/失败码"全靠它们。
  nohup ros2 bag record -o "$BAG_DIR_ACTIONS" --include-hidden-topics \
    "${ACTION_TOPICS[@]}" > "$OUT_ROOT/bag_actions.log" 2>&1 &
  echo $! > "$PIDS_DIR/bag_actions.pid"
  echo "  bag(action) pid=$(cat "$PIDS_DIR/bag_actions.pid")  ->  $BAG_DIR_ACTIONS"

  # ⚠️ 不能用 `ros2 run` —— 实机的 ros2 CLI **连 run 子命令都没装**
  # （只有 bag/daemon/extension_points/extensions/node/param/service；
  #  topic、launch、run 全部报 invalid choice）。install 下的入口点包装脚本
  # 本身就是可执行文件，直接按路径调，行为与 ros2 run 完全一致。
  RECORDER="$WS/install/astribot_s1_navigation/lib/astribot_s1_navigation/explore_metrics_recorder_node"
  if [ ! -x "$RECORDER" ]; then
    echo "🔴 没有 $RECORDER —— 包没重建。在开发机上跑 tools/robot/sync_metrics_to_robot.sh"
    return 1
  fi
  # ⚠️ use_sim_time 必须显式 false（实机 /clock 无发布者）。
  # ⚠️ 参数名是 output_dir / run_label，写错会被静默忽略、零告警。
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
    p=$(cat "$PIDS_DIR/$f.pid" 2>/dev/null)
    kill -0 "$p" 2>/dev/null && echo "  $f 活着 (pid=$p)" || echo "  🔴 $f 已退出，看 $OUT_ROOT/$f.log"
  done
  echo
  echo "  标签=$RUN_LABEL  目录=$OUT_ROOT"
  echo "  机器人**不会**因为这个脚本动 —— 该由你或协调器去驱动它。"
  echo "  停止：bash $0 stop     出报告：bash $0 report"
}

do_status() {
  echo "==== 进程 ===="
  # 不用 pgrep -f：命令行含有该模式时会匹配到自己（本项目已踩三次，
  # 一次 kill 循环杀掉了自己的 shell）。按 pid 文件查。
  for f in bag bag_actions recorder; do
    p=$(cat "$PIDS_DIR/$f.pid" 2>/dev/null)
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
    # 文件名是 <label>_rounds.csv，盯裸 rounds.csv 会永远报 0 轮
    echo "  还没有 ${RUN_LABEL}_rounds.csv（目录里：$(ls "$METRICS_DIR" 2>/dev/null | tr '\n' ' '))"
  fi
  echo "==== 录制器自检 ===="
  grep -E "自检|订阅|收到|TF" "$OUT_ROOT/recorder.log" 2>/dev/null | tail -8
}

do_stop() {
  echo "==== 停录（SIGINT，让 bag 正常封口）===="
  for f in bag bag_actions recorder; do
    p=$(cat "$PIDS_DIR/$f.pid" 2>/dev/null)
    [ -n "$p" ] && kill -INT "$p" 2>/dev/null && echo "  SIGINT -> $p ($f)"
  done
  sleep 10
  echo "==== 封口确认（metadata.yaml 出来了才算）===="
  for d in "$BAG_DIR" "$BAG_DIR_ACTIONS"; do
    if [ -f "$d/metadata.yaml" ]; then
      echo "  ✅ $(basename "$d") 已封口，$(du -sh "$d" | cut -f1)"
    else
      echo "  🔴 $(basename "$d") 没有 metadata.yaml —— bag 未封口，回放会失败"
    fi
  done
  # SIGKILL 兜底，避免留下往已删目录写的僵尸录制进程（本项目实测过
  # 两个 bag record 活了 108 分钟，一直往已经 mv 走的目录里写）
  for f in bag bag_actions recorder; do
    p=$(cat "$PIDS_DIR/$f.pid" 2>/dev/null)
    [ -n "$p" ] && kill -0 "$p" 2>/dev/null && { kill -9 "$p" 2>/dev/null; echo "  强杀残留 $p ($f)"; }
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
    # 空表和"跑得不好"是两回事，必须先说清楚，否则下面那份报告会被当成结论
    echo "⚠️  表里 0 个已完成轮次。报告只会给出链路自检，不含任何路径指标。"
    echo "    最常见原因：协调器处于 PAUSED/IDLE，整段时间没派发过目标"
    echo "    （轮次是按 /exploration/current_goal 切分的）。bag 仍然完整可用。"
  fi
  python3 "$WS/../tools/report_explore_metrics.py" "$METRICS_DIR" \
          --md "$OUT_ROOT/report.md"
  echo
  # 判据是**文件真的在**，不是上一条命令返回 0。曾经这里直接打印路径，
  # 而 0 轮那条早退路径根本没写文件，于是打印出一个不存在的报告路径。
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
