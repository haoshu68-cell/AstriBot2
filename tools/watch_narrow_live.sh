#!/usr/bin/env bash
# 起一套**带 Gazebo GUI + RViz** 的栈，用来肉眼看窄通道贴边通行是否有效。
#
# 与 run_narrow_verification.sh 的区别：这个不循环、不切臂、不杀栈，
# 起来就一直跑，供人观察。窄通道**固定为开**。
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

OUT=/tmp/narrow_watch
mkdir -p "$OUT"
INST="$REPO/ws_robot/install/astribot_s1_navigation/share/astribot_s1_navigation/config/nav2_params_mppi.yaml"

say() { echo "[$(date +%H:%M:%S)] $*"; }

# ---- 窄通道固定为开，并从**安装态**确认（launch 读的是这份，不是 src）----
sed -i 's/^\(      narrow_enabled: \).*/\1true/' "$INST"
sed -i 's/^\(      narrow_enabled: \).*/\1true/' \
  "$REPO/ws_robot/src/astribot_s1_navigation/config/nav2_params_mppi.yaml"
n=$(grep -c "^      narrow_enabled: true$" "$INST")
[ "$n" = "2" ] || { say "!! 安装态 narrow_enabled=true 只有 $n 处(应 2)"; exit 1; }
say "窄通道: 安装态 narrow_enabled=true x2"

bash "$REPO/tools/clean_sim_stack.sh" > "$OUT/clean.log" 2>&1 || {
  say "!! 清理未通过，拒绝起栈（脏状态下看到的现象不可信）"; tail -4 "$OUT/clean.log"; exit 1; }
sleep 3

T0=$(date +%s)
nohup ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  exploration:=true controller_plugin:=mppi use_rviz:=true headless:=false \
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
  say "  门控 $((i*5))s: /clock 发布者=$cp  收到/map=$got_map  rviz2 进程=$rviz"
  if [ "$cp" = "1" ] && [ "$got_map" = "yes" ] && [ "$rviz" -ge 1 ]; then
    say "门控通过：时钟唯一、地图已到、rviz 已起"
    break
  fi
  if [ "$cp" -gt 1 ] 2>/dev/null; then
    say "!! /clock 有 $cp 个发布者 —— 仿真时间会非单调，看到的现象不可信"; exit 1
  fi
done
say "现在可以看了。观察要点见终端输出。"
