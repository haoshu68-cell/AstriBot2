#!/usr/bin/env bash
# 用途：一条命令跑完"清栈 -> 起仿真(Gazebo+rviz) -> 立刻开录 -> 感知链硬门禁 -> 探索"。
#
# 为什么要有这个脚本（不是图省事，是三次踩过的坑）：
#   1. 感知链断了会**静默空转**。run8 里 livox_fusion_node 端点全无，链条从
#      /livox/fused_points 起就是 0 帧，栈却活得好好的：Gazebo 在步进、rviz 进程在、
#      18 个节点全在。表现只有"rviz 一片空白 + 探索 dispatched=0"，
#      空转了 900 秒没有任何一处报错。所以这里必须有**主动门禁**。
#   2. 门禁判据只能是"实收帧数"。进程存活、发布者数、`ros2 topic hz` 三个都骗过我：
#      · 发布者数：/livox/fused_points 有 SUB 无 PUB 也照样出现在 topic list 里；
#      · `ros2 topic hz --qos-reliability best_effort`：对实测 47Hz 的 /odom 报 0。
#      所以门禁走 tools/gate_chain.py，用显式 BEST_EFFORT 订阅数真帧。
#   3. 录制器必须在**第一个目标派发之前**就位。run9 上我在探索开始约 2.5 分钟后
#      才手工挂录制器，它订阅 /exploration/current_goal 时收到**锁存的历史目标**，
#      给早已开始的目标开窗：轮 0 窗口 211.9~241.6，而 pose 样本却是 244.35~245.00,
#      整体错位一格。结果 9 轮里 6 轮位姿覆盖率 < 0.16，只有 3 轮可用。
#
# 启动顺序：清栈 -> 起仿真 -> **验它真在步进（不在就自动重启）** -> 立刻开录 -> 观察感知链。
#   验步进必须排在开录之前：Gazebo 那个加载期死锁下 /clock 一帧不发，
#   录下来的 bag 是空的、录制器的 use_sim_time 时钟永远停在 0，纯属白录。
#   而开录必须排在感知链观察之前：链条走通要 90~120s（等 SLAM 出图），
#   探索的第一个目标就在出图后不久派发，录制器排在后面必然赶不上第 0 轮。
#   没有 /map 就没有前沿、就不会派发目标，所以提前开录不会脏数据（录制器只读）。
#
# 用法：
#   bash tools/run_explore_with_bag.sh                  # 标签取当前时刻
#   RUN_LABEL=run10 bash tools/run_explore_with_bag.sh  # 指定标签
#
# 退出码：0=仿真在步进且已在录制；1=连续 3 次都是加载期死锁，仿真起不来。
#         感知链没走通**不算失败**，只打印逐跳表，录制照常保留。

# 注意：**不要**开 `set -u`。本仓踩过：set -u 下 source ROS setup.bash 会静默退出，
# 一个字都不打印，看起来像后面的探针坏了。见 memory set-u-plus-ros-setup-bash-kills-script。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

RUN_LABEL="${RUN_LABEL:-run$(date +%H%M)}"
OUT_ROOT="${OUT_ROOT:-/tmp/explore_${RUN_LABEL}}"
BAG_DIR="$OUT_ROOT/bag"
BAG_DIR_ACTIONS="$OUT_ROOT/bag_actions"
METRICS_DIR="$OUT_ROOT/metrics"
STACK_LOG="$OUT_ROOT/stack.log"
BAG_LOG="$OUT_ROOT/bag.log"
BAG_ACT_LOG="$OUT_ROOT/bag_actions.log"
REC_LOG="$OUT_ROOT/recorder.log"

# 仿真"是否在步进"的等待上限。Gazebo 起进程 + 加载世界实测 20~60s，给 100s。
# 超了就判加载期死锁并重启（不是继续等 —— 实测等 240s 也是 0 帧）。
STEP_TIMEOUT_SEC="${STEP_TIMEOUT_SEC:-100}"
# 感知链观察的上限。只用于打报告，不影响流程，所以可以给得宽松些。
CHAIN_TIMEOUT_SEC="${CHAIN_TIMEOUT_SEC:-180}"

export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1

mkdir -p "$OUT_ROOT" "$METRICS_DIR"

echo "==== [1/5] 清栈 ===="
timeout 180 bash tools/clean_sim_stack.sh 2>&1 | tail -6

# 0 字节的 fastrtps_port*_el 残留文件会让**新起的**参与者报
# `open_and_lock_file failed`，而老进程不受影响 —— 于是症状是"只有我的探针连不上"。
# 清栈已经清过一遍，这里兜底再扫一次（清栈脚本的模式曾漏过 sem.fastrtps_* 前缀）。
find /dev/shm -maxdepth 1 -name 'fastrtps_*' -size 0 -delete 2>/dev/null || true
find /dev/shm -maxdepth 1 -name 'sem.fastrtps_*' -delete 2>/dev/null || true

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/ws_robot/install/setup.bash"

echo
echo "==== [2/5] 起仿真栈（Gazebo + rviz 都起，用户要求）+ 步进自检与自动重启 ===="
# Gazebo 有一种**间歇性**加载期死锁：进程全在、server 在烧 CPU、rviz 在，
# 但物理一步不走（/clock 有发布者、0 帧）。run10 实测等满 240s 一帧都没有，
# 而一小时前同一份代码是好的 —— 所以它是启动竞态，不是配置错误。
# 对启动竞态，正确处置是**重启重试**，不是把人卡在启动界面上等。
STACK_PID=""
for attempt in 1 2 3; do
  echo "  --- 第 $attempt/3 次启动尝试 ---"
  echo "  日志: $STACK_LOG"
  nohup ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
    env:=sim mode:=mapping exploration:=true controller_plugin:=mppi \
    use_rviz:=true headless:=false \
    > "$STACK_LOG" 2>&1 &
  STACK_PID=$!
  echo "  launch pid=$STACK_PID"
  echo "$STACK_PID" > "$OUT_ROOT/stack.pid"

  if python3 "$REPO_ROOT/tools/wait_sim_stepping.py" --timeout "$STEP_TIMEOUT_SEC"; then
    break
  fi

  # 死锁了。留证据再杀，否则下一轮日志会把现场覆盖掉。
  cp "$STACK_LOG" "$OUT_ROOT/stack_deadlock_attempt${attempt}.log" 2>/dev/null || true
  echo "  ⚠️ 第 $attempt 次是加载期死锁，现场留在 stack_deadlock_attempt${attempt}.log，清栈重来"
  timeout 180 bash tools/clean_sim_stack.sh 2>&1 | tail -3
  find /dev/shm -maxdepth 1 -name 'fastrtps_*' -size 0 -delete 2>/dev/null || true
  find /dev/shm -maxdepth 1 -name 'sem.fastrtps_*' -delete 2>/dev/null || true
  STACK_PID=""
  sleep 5
done

if [ -z "$STACK_PID" ]; then
  echo "🔴 连续 3 次都是 Gazebo 加载期死锁，仿真起不来。没有开录，也没有产生任何数据。"
  exit 1
fi

echo
echo "==== [3/5] 立刻开录（必须早于第一个目标派发，见文件头说明）===="
# 白名单而非 -a：-a 会把四路原始点云(每帧两万点)一起录进来，体积和回放开销都不划算。
# /clock 必须录：不录它，回放时 use_sim_time 的一侧全部冻在 0。
BAG_TOPICS=(
  /clock /tf /tf_static /odom /pose /joint_states
  /map /map_metadata /map_updates
  /scan /scan_from_cloud
  /cmd_vel /cmd_vel_nav_body_raw /cmd_vel_pre_arm_coupling /speed_limit
  /plan /local_plan /transformed_global_plan /plan_smoothed /unsmoothed_plan
  /goal_pose /initialpose
  /exploration/state /exploration/current_goal /exploration/complete
  /global_costmap/costmap /local_costmap/costmap
  /global_costmap/published_footprint /local_costmap/published_footprint
  /rosout /diagnostics
)
nohup ros2 bag record -o "$BAG_DIR" --max-bag-size 2000000000 \
  "${BAG_TOPICS[@]}" > "$BAG_LOG" 2>&1 &
BAG_PID=$!
echo "$BAG_PID" > "$OUT_ROOT/bag.pid"
echo "  bag pid=$BAG_PID  ->  $BAG_DIR"

# action status 必须单独一个 bag，因为它们是**隐藏话题**：
# 显式点名也不够，`ros2 bag record` 会直接跳过并只打一条
#   [WARN] [ROSBAG2_TRANSPORT]: Hidden topics are not recorded.
#                              Enable them with --include-hidden-topics
# 而"派发次数 / 失败码 / 每目标耗时"这几个指标全靠这几条话题。run9 实测踩到。
# 不把 --include-hidden-topics 加到上面那个 bag 上，是因为它会连
# /parameter_events 之类的隐藏话题一起吸进来，白白放大体积。
nohup ros2 bag record -o "$BAG_DIR_ACTIONS" --include-hidden-topics \
  /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback \
  /compute_path_to_pose/_action/status /follow_path/_action/status \
  > "$BAG_ACT_LOG" 2>&1 &
BAG_ACT_PID=$!
echo "$BAG_ACT_PID" > "$OUT_ROOT/bag_actions.pid"
echo "  bag(action) pid=$BAG_ACT_PID  ->  $BAG_DIR_ACTIONS"

# ⚠️ 参数名是 output_dir / run_label。run6 上写成 out_dir / run_tag，
# 两个都被静默忽略、零告警，数据落到默认目录且标签是空串。
nohup ros2 run astribot_s1_navigation explore_metrics_recorder_node --ros-args \
  -p use_sim_time:=true \
  -p output_dir:="$METRICS_DIR" \
  -p run_label:="$RUN_LABEL" \
  > "$REC_LOG" 2>&1 &
REC_PID=$!
echo "$REC_PID" > "$OUT_ROOT/recorder.pid"
echo "  recorder pid=$REC_PID  ->  $METRICS_DIR"

echo
echo "==== [4/5] 感知链观察（只报告，不再阻塞、不再删数据）===="
# 这里原来是一道**阻塞式硬门禁**：不通就等满 240s，然后把 bag 与 metrics 一起删掉。
# 改掉的原因有两条，都是实测：
#   · 它挡不住真正的启动故障 —— 加载期死锁已经由上面第 2 步的步进自检+重启处理了，
#     到这一步仿真必然在走，剩下的差异只是 SLAM 出图早晚；
#   · 它把已经录到的数据删了。/map 出得晚不代表这段录制没用，删掉纯粹是损失。
# 所以现在只观察一次、如实打印断在哪一跳，不影响后续流程。
python3 "$REPO_ROOT/tools/gate_chain.py" --timeout "$CHAIN_TIMEOUT_SEC" \
  > "$OUT_ROOT/chain_check.log" 2>&1 && CHAIN_OK=1 || CHAIN_OK=0
tail -20 "$OUT_ROOT/chain_check.log"
if [ "$CHAIN_OK" != "1" ]; then
  echo "  ⚠️ 感知链在 ${CHAIN_TIMEOUT_SEC}s 内没走通（详见 chain_check.log 的逐跳表，第一个 🔴 是断点）。"
  echo "     录制**继续保留**。若最终 0 次派发，就以这份逐跳表为排查起点。"
fi

echo
echo "==== [5/5] 录制状态复核 ===="
# 只看进程活着不够（这条判据本轮已经骗过我一次）：bag 要看**字节在长**，
# 录制器要看它自己打印的订阅自检。
sleep 5
echo "  bag 体积: $(du -sh "$BAG_DIR" 2>/dev/null | cut -f1)   action bag: $(du -sh "$BAG_DIR_ACTIONS" 2>/dev/null | cut -f1)"
if grep -qi 'Hidden topics are not recorded' "$BAG_ACT_LOG" 2>/dev/null; then
  echo "  🔴 action bag 仍在跳过隐藏话题 —— 检查 --include-hidden-topics 是否生效"
fi
echo "  录制器输出目录内容: $(ls "$METRICS_DIR" 2>/dev/null | tr '\n' ' ')"

echo
echo "======================================================="
echo "✅ 已在录制。标签=$RUN_LABEL"
echo "   bag        : $BAG_DIR"
echo "   action bag : $BAG_DIR_ACTIONS"
echo "   指标        : $METRICS_DIR/${RUN_LABEL}_rounds.csv"
echo "   栈日志      : $STACK_LOG"
echo "   出报告      : python3 tools/report_explore_metrics.py $METRICS_DIR --md $OUT_ROOT/report.md"
echo "   停止        : bash tools/clean_sim_stack.sh   (会一并停 bag 与录制器)"
echo "======================================================="
