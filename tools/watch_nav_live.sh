#!/usr/bin/env bash
# 起一套**带 Gazebo GUI + RViz** 的栈，用来肉眼看自主探索建图与路径跟踪。
#
# 不循环、不切参数、不杀栈，起来就一直跑，供人观察。
# （2026-09-07：本脚本原名 watch_narrow_live.sh，随窄通道层一起删掉了切臂逻辑。）
#
# 纪律（每条都来自实测）：
#  · 起栈前必须清干净，判据是 /clock 发布者数为 0，然后必须变成 1。
#    陈旧的 parameter_bridge 会重连新 Gazebo ⇒ 双时钟 ⇒ 1ms 回跳 ⇒
#    tf2 清空整个 TF buffer ⇒ planner 反复 abort ⇒ 0 次到位。
#  · headless 保持 false（headless:=true 是 gz_ros2_control 加载期死锁的触发条件）。
#  · 门控不能只看「/map 有发布者」——有发布者 ≠ 有消息。实测过一轮
#    「/map 发布者=1 而协调器一直报 尚未收到占据栅格地图」，
#    那轮 0 次到位跟策略毫无关系。所以这里等**真的收到一帧 /map**。
REPO=/home/yjh/WorkSpace/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
source "$REPO/ws_robot/install/setup.bash"
set -u
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=1

OUT=/tmp/nav_watch
mkdir -p "$OUT"
INST="$REPO/ws_robot/install/astribot_s1_navigation/share/astribot_s1_navigation/config/nav2_params_mppi.yaml"

say() { echo "[$(date +%H:%M:%S)] $*"; }

# ---- 起栈前核对**安装态**足迹是正方形（launch 读的是这份，不是 src）----
# 只比 src 会得出"已同步"的假结论，而症状与改动前逐字相同。
n=$(grep -c 'footprint: "\[\[0.31, 0.31\], \[-0.31, 0.31\], \[-0.31, -0.31\], \[0.31, -0.31\]\]"' "$INST")
[ "$n" = "2" ] || { say "!! 安装态正方形足迹只有 $n 处(应 2) —— 先 colcon build"; exit 1; }
say "足迹: 安装态正方形 a=0.31 x2 (内切 0.310 / 外接 0.438)"

bash "$REPO/tools/clean_sim_stack.sh" > "$OUT/clean.log" 2>&1 || {
  say "!! 清理未通过，拒绝起栈（脏状态下看到的现象不可信）"; tail -4 "$OUT/clean.log"; exit 1; }
sleep 3

T0=$(date +%s)
# max_linear_speed 必须显式传：它的 launch 默认值是 1.0，不传就跑 vx_max=1.0。
# 2026-09-07 实测漏传的后果是 max|vx|=0.456，看着像限速失效，其实是默认值。
nohup ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  exploration:=true controller_plugin:=mppi use_rviz:=true headless:=false \
  max_linear_speed:=0.2 \
  > "$OUT/launch.log" 2>&1 &
LP=$!
say "launch pid=$LP  (mppi + exploration + rviz + gazebo gui)"
echo "$LP" > "$OUT/launch.pid"

# ---- 门控 ----
for i in $(seq 1 40); do
  sleep 5
  cp=$(timeout 20 ros2 topic info /clock 2>/dev/null | awk '/Publisher count:/ {print $3}')
  case "$cp" in ''|*[!0-9]*) cp=-1 ;; esac
  # 判据是**真的收到一帧地图**，不是"有发布者"。
  got_map=no
  timeout 12 ros2 topic echo /map --once > /dev/null 2>&1 && got_map=yes
  rviz=$(ps -eo args --no-headers | awk '$0 ~ /rviz2/ && $0 !~ /awk/' | wc -l)
  gz=$(ps -eo args --no-headers | awk '$0 ~ /ign gazebo|gz sim|ign-gazebo/ && $0 !~ /awk/' | wc -l)
  say "  门控 $((i*5))s: /clock 发布者=$cp  收到/map=$got_map  rviz2=$rviz  gazebo=$gz"
  # rviz 与 gazebo 都必须核对进程数：use_rviz:=true 不保证它真起来了
  # （setup.py 只 glob rviz/*.rviz，.rviz 放错目录会静默不装）。
  if [ "$cp" = "1" ] && [ "$got_map" = "yes" ] && [ "$rviz" -ge 1 ] && [ "$gz" -ge 1 ]; then
    say "门控通过：时钟唯一、地图已到、rviz 与 gazebo 都在"
    break
  fi
  if [ "$cp" -gt 1 ] 2>/dev/null; then
    say "!! /clock 有 $cp 个发布者 —— 仿真时间会非单调，看到的现象不可信"; exit 1
  fi
done
say "现在可以看了。观察要点见终端输出。"
