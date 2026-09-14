#!/usr/bin/env bash
set -u

DOMAIN="${ROS_DOMAIN_ID:-25}"

NAMES=(
  "ign" "ruby" "parameter_bridge" "robot_state_publisher"
  "controller_server" "planner_server" "smoother_server" "behavior_server"
  "bt_navigator" "waypoint_follower" "velocity_smoother" "lifecycle_manager"
  "map_server" "amcl" "collision_monitor"
  "async_slam_toolbox_node" "sync_slam_toolbox_node"
  "pointcloud_to_laserscan_node" "pointcloud_slice_scan_node"
  "static_transform_publisher"
  "livox_preprocess_node" "livox_fusion_node" "livox_custom_to_pc2_node"
  "rviz2"
  "omni_effort_drive_node" "cmd_vel_body_to_world_node"
  "arm_speed_limiter_node" "arm_chassis_speed_coupling_node"
  "exploration_coordinator_node" "frontier_explorer_node"
  "explore_metrics_recorder_node"
)

RESIDUAL_PAT='^rviz2$|^ign$|^ruby$|^gzserver$|^gzclient$|^parameter_bridg|^robot_state_pub|^joint_state_pub|^controller_serv|^planner_serv|^smoother_serv|^behavior_serv|^map_serv|^bt_navigator|^lifecycle_manager|^waypoint_follower|^velocity_smoother|^collision_monitor|slam_toolbox|^pointcloud_|^livox_|^omni_effort|^cmd_vel_body|^arm_speed|^arm_chassis|^exploration_coo|^frontier_expl|^static_transform_pub|^amcl$|^explore_metrics'

collect_pids() {
  local out=()
  local n
  local pat
  pat=$(printf '%s\n' "${NAMES[@]}" | paste -sd'|')
  while read -r pid; do
    [ -n "$pid" ] && out+=("$pid")
  done < <(ps -eo pid,args --no-headers | awk -v pat="^(${pat})$" '
    {
      exe = $2
      k = split(exe, parts, "/")
      base = parts[k]
      if (base ~ pat) { print $1 }
    }')
  while read -r pid; do
    [ -n "$pid" ] && out+=("$pid")
  done < <(ps -eo pid,args --no-headers | awk \
    '$0 ~ /ros2 (launch|run)/ && $0 ~ /astribot_s1_/ && $0 !~ /awk/ {print $1}')
  while read -r pid; do
    [ -n "$pid" ] && out+=("$pid")
  done < <(ps -eo pid,args --no-headers | awk \
    '$0 ~ /astribot_sdk_ros2\/ws_robot\/install\// && $0 !~ /awk/ {print $1}')
  printf '%s\n' "${out[@]:-}" | sort -u | grep -E '^[0-9]+$' || true
}

echo "==== 清理前 ===="
BEFORE=$(collect_pids | wc -l)
echo "命中进程 $BEFORE 个"

for sig in TERM TERM KILL; do
  mapfile -t pids < <(collect_pids)
  [ "${#pids[@]}" -eq 0 ] && break
  for p in "${pids[@]}"; do
    [ "$p" = "$$" ] && continue
    [ "$p" = "$PPID" ] && continue
    kill "-$sig" "$p" 2>/dev/null
  done
  sleep 2
done

mapfile -t left < <(collect_pids)
echo "清理后残留 ${#left[@]} 个"
if [ "${#left[@]}" -gt 0 ]; then
  ps -o pid,args -p "$(IFS=,; echo "${left[*]}")" 2>/dev/null | cut -c1-120
fi

rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

shm_sweep() {
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  local n
  n=$(ls /dev/shm 2>/dev/null | grep -c "fastrtps" || true)
  echo "残留 fastrtps 共享内存对象: ${n:-0}  (验证段之后重新清理并计数)"
}

set +u
source /opt/ros/humble/setup.bash 2>/dev/null
set -u
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
timeout 15 ros2 daemon stop >/dev/null 2>&1
sleep 1

echo "==== 验证 (ROS_DOMAIN_ID=$DOMAIN) ===="
CLOCK_PUBS=$(timeout 20 ros2 topic info /clock 2>/dev/null | awk '/Publisher count:/ {print $3}')
CLOCK_PUBS="${CLOCK_PUBS:-0}"
echo "/clock 发布者数 = $CLOCK_PUBS  (期望 0)"

residual=$(ps -eo pid,args --no-headers | awk -v pat="$RESIDUAL_PAT" '
  $0 !~ /awk/ {
    exe = $2; k = split(exe, parts, "/"); base = parts[k]
    if (base ~ pat) { print $1"  "base }
  }')
if [ -n "$residual" ]; then
  echo "🔴 宽口径自检发现残留进程（NAMES 表漏了它们）:"
  echo "$residual" | sed 's/^/     /'
  echo "   ⇒ 请把上面的可执行名补进本脚本的 NAMES 数组。"
  echo "   ⇒ 在这个状态下跑测量，数据不可信（实测：残留 rviz2 让一整轮 0 次到位）"
  shm_sweep
  echo "结果: **未清干净**"
  exit 1
fi
echo "宽口径残留自检 = 0 个进程"
shm_sweep

if [ "${#left[@]}" -eq 0 ] && [ "$CLOCK_PUBS" = "0" ]; then
  echo "结果: 干净"
  exit 0
fi
echo "结果: **未清干净** —— 不要在这个状态下跑任何测量，数据不可信"
exit 1
