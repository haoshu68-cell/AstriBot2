#!/usr/bin/env bash
# 清干净一次仿真栈，并**验证**清干净了。
#
# 为什么需要这个脚本（两次实测教训）：
#
# 1) 按工作区路径杀进程会漏掉 /opt/ros/humble 下的二进制 ——
#    parameter_bridge、nav2_* 都不在工作区路径里。漏下的进程会被
#    ppid 重挂到别的父进程上，看起来像是"没有残留"。
#
# 2) 漏下的 parameter_bridge 会**重新连上新起的 Gazebo**（ign-transport
#    按话题名发现，不认进程）。于是 /clock 有两个发布者，两路独立投递交错
#    到达 ⇒ 仿真时间非单调。实测 60 个样本里回跳 2 次、每次仅 1ms，
#    但 tf2 对**任何**回跳都会清空整个 TF buffer：
#      Detected jump back in time. Clearing TF buffer.   x16934
#      Could not find a connection between 'map' and 'astribot_torso_base'  x234
#    表现是 planner_server 反复 abort、机器人 0 次到位，而 Gazebo 和 SLAM
#    看起来都正常。这个现象从日志几乎不可能反推到"有个陈旧的桥"。
#
# 所以本脚本的判据不是"kill 命令返回成功"，而是
#   **/clock 的发布者数必须为 0**（清理后）/ **为 1**（重启后）。
set -u

DOMAIN="${ROS_DOMAIN_ID:-25}"

# ---- 要杀的进程名。刻意用「基名精确匹配」而不是 -f 模式匹配：
#      pkill -f 的模式会命中调用它的这个 shell 自身，循环从中间静默断掉
#      （本项目已踩过，日志文件不存在被误读成"启动失败"）。
NAMES=(
  # 仿真与桥
  "ign" "ruby" "parameter_bridge" "robot_state_publisher"
  # nav2
  "controller_server" "planner_server" "smoother_server" "behavior_server"
  "bt_navigator" "waypoint_follower" "velocity_smoother" "lifecycle_manager"
  "map_server" "amcl" "collision_monitor"
  # slam / 感知
  "async_slam_toolbox_node" "sync_slam_toolbox_node"
  "pointcloud_to_laserscan_node" "pointcloud_slice_scan_node"
  "static_transform_publisher"
  "livox_preprocess_node" "livox_fusion_node" "livox_custom_to_pc2_node"
  # 可视化。⚠️ rviz2 是 /opt/ros/humble 的二进制，最容易被漏掉 ——
  #    2026-09-03 实测漏了它：cycle 1 的 3 个 rviz2 活到 cycle 2，
  #    那一轮 686s 内 0 次到位、足迹回读也失败，整轮数据作废。
  #    而 /clock 判据抓不到它（rviz2 不发 /clock），所以"清理成功"是假的。
  "rviz2"
  # 底盘控制链。🔴 残留会**持续下发力矩/速度指令**，比残留 rviz 危险得多。
  "omni_effort_drive_node" "cmd_vel_body_to_world_node"
  "arm_speed_limiter_node" "arm_chassis_speed_coupling_node"
  # 本仓自研节点（可执行名，不是路径）
  "exploration_coordinator_node" "frontier_explorer_node"
)

# ---- 残留自检用的**宽口径**探测集。
#      刻意与 NAMES 分开：NAMES 是"杀谁"，这个是"验证还剩谁"。
#      两者共用一张表时，表漏了一项就同时漏掉杀与验，遗漏永远发现不了 ——
#      这次漏 rviz2 就是这么过关的。宽口径宁可误报也不能漏报。
# ⚠️ 不要用 `_server$` 这种通配：实测它命中了系统里无关的 screenshot_server，
#    误报会让人开始无视这条自检 —— 那比没有自检更糟。nav2 的 server 逐个列。
RESIDUAL_PAT='^rviz2$|^ign$|^ruby$|^gzserver$|^gzclient$|^parameter_bridg|^robot_state_pub|^joint_state_pub|^controller_serv|^planner_serv|^smoother_serv|^behavior_serv|^map_serv|^bt_navigator|^lifecycle_manager|^waypoint_follower|^velocity_smoother|^collision_monitor|slam_toolbox|^pointcloud_|^livox_|^omni_effort|^cmd_vel_body|^arm_speed|^arm_chassis|^exploration_coo|^frontier_expl|^static_transform_pub|^amcl$'


collect_pids() {
  # 只按可执行**基名**匹配，避免误杀编辑器/浏览器等无关进程。
  #
  # !!! 不能用 pgrep -x !!! Linux 的 comm 字段被截断到 15 个字符，
  # 于是 `pgrep -x parameter_bridge` 永远匹配不到（真实 comm 是
  # "parameter_bridg"），而 `controller_server` 变成 "controller_serv"。
  # 这个坑很毒：脚本报告"命中 1 个进程 / 已清理"，而两个 /clock 桥还活着。
  # 实测就是这样让"清理成功"变成假成功的。
  # 改成对 args 的第一个字段取 basename 做精确比较。
  local out=()
  local n
  local pat
  pat=$(printf '%s\n' "${NAMES[@]}" | paste -sd'|')
  while read -r pid; do
    [ -n "$pid" ] && out+=("$pid")
  done < <(ps -eo pid,args --no-headers | awk -v pat="^(${pat})$" '
    {
      # $2 是可执行路径（$1 是 pid）。取它的 basename。
      exe = $2
      k = split(exe, parts, "/")
      base = parts[k]
      if (base ~ pat) { print $1 }
    }')
  # launch 的 python 进程：只认命令行里同时含 ros2 launch 与本仓包名的。
  #
  # `$0 !~ /awk/` 不是多余的：awk 自己的命令行里就含有上面那些模式字符串，
  # 于是它会**匹配到自己**，凭空多出一个已经不存在的 pid。表现是脚本报
  # 「残留 1 个」而 ps 打不出任何行。这与 `pkill -f` 命中自身是同一类坑。
  while read -r pid; do
    [ -n "$pid" ] && out+=("$pid")
  done < <(ps -eo pid,args --no-headers | awk \
    '$0 ~ /ros2 launch/ && $0 ~ /astribot_s1_/ && $0 !~ /awk/ {print $1}')
  # 本仓 install 目录下的 python 节点。
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
    # 绝不杀自己或自己的父进程。
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

# ---- 共享内存与信号量。前缀必须两种都清：
#      只清 fastrtps_ 会漏掉 sem.fastrtps_*（实测漏下 73 个）。
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
SHM_LEFT=$(ls /dev/shm 2>/dev/null | grep -c "fastrtps" || true)
echo "残留 fastrtps 共享内存对象: ${SHM_LEFT:-0}"

# ---- daemon 陈旧时 `ros2 topic list` 会给出完全过时的结果
#      （实测话题数 2 vs 80），必须重启。
#
# set +u 是必须的：ROS 的 setup.bash 里引用了未定义变量，在 set -u 下
# 会让**整个脚本**当场退出。而上面 kill 已经跑完了，于是现象是
# 「清理看着成功、验证段一行都没输出」—— 那正是本脚本要避免的那种假成功。
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
set -u
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
timeout 15 ros2 daemon stop >/dev/null 2>&1
sleep 1

# ---- 真正的判据：/clock 不能还有发布者。
echo "==== 验证 (ROS_DOMAIN_ID=$DOMAIN) ===="
CLOCK_PUBS=$(timeout 20 ros2 topic info /clock 2>/dev/null | awk '/Publisher count:/ {print $3}')
CLOCK_PUBS="${CLOCK_PUBS:-0}"
echo "/clock 发布者数 = $CLOCK_PUBS  (期望 0)"

# ---- 🔴 宽口径残留自检：抓 NAMES 表**自己的遗漏** ----
#
# 为什么必须与 NAMES 分开：上一版只有 "/clock 发布者=0" 和 "NAMES 命中数=0"
# 两条判据，而漏掉的 rviz2 既不发 /clock、也不在 NAMES 里 ——
# 两条判据同时对它视而不见，脚本照报"干净"。3 个 rviz2 就这样活到下一轮，
# 那一轮 686s 内 0 次到位、整轮作废。
#
# 判据必须能抓住清理表的遗漏，否则它验证的只是"我记得杀的那些确实杀了"。
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
  echo "结果: **未清干净**"
  exit 1
fi
echo "宽口径残留自检 = 0 个进程"

if [ "${#left[@]}" -eq 0 ] && [ "$CLOCK_PUBS" = "0" ]; then
  echo "结果: 干净"
  exit 0
fi
echo "结果: **未清干净** —— 不要在这个状态下跑任何测量，数据不可信"
exit 1
