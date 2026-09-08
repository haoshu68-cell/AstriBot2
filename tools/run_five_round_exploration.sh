#!/usr/bin/env bash
# 五轮自主探索建图验证（Gazebo + rviz 都起，用户要求）。
#
# 设计上最重要的一条：**每一轮在开始计量之前必须先通过感知链拍率闸门**。
# 理由是实测的：连续三次启动，各有一个不同的节点进程活着但在 DDS 图里
# 匹配不上（第3轮 livox_fusion_node+smoother_server、第4轮左雷达 alias 静态 TF、
# 第5轮 livox_preprocess_left 累计 CPU 1s/365s 而右路 1m42s）。
# 这类故障下 /scan 与 /map 全程为空，探索必然 0 次派发 —— 若不设闸门，
# 五轮会得到五份一模一样的"探索坏了"假结论。所以闸门不过的轮次记为
# INVALID(基础设施)，重试；重试仍不过就如实记为无效，绝不混进有效轮次。
#
# 判据一律用"窗口内帧数"，不用 count_publishers：上面第5轮那次
# /livox/left/cloud_filtered 的 pub 恒为 1 而拍率为 0。
set -uo pipefail

REPO=/home/yjh/WorkSpace/astribot_sdk_ros2
ROOT=${ROOT:-/tmp/explore_5rounds}
ROUNDS=${ROUNDS:-5}
EXPLORE_SEC=${EXPLORE_SEC:-360}   # 每轮探索时长
GATE_SEC=${GATE_SEC:-200}         # 闸门最长等待
RETRY=${RETRY:-1}                 # 闸门不过时的重试次数

mkdir -p "$ROOT"
SUMMARY="$ROOT/summary.tsv"
[ -f "$SUMMARY" ] || printf '轮次\t状态\t闸门耗时s\trviz\tscanHz\tmapHz\t备注\n' > "$SUMMARY"

set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$REPO/ws_robot/install/setup.bash" >/dev/null 2>&1
set -u
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1

# ---- 闸门：按拍率判协调器派发目标所依赖的四条通路是否真的在出数据 --------
cat > "$ROOT/gate.py" <<'PY'
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import Costmap

# 四条都必须真出数据才算就绪。第2轮的教训：只闸 /scan 与 /map 时，
# 全局代价地图整轮没起来也能过闸 —— 协调器 83 次报"校验图不可用"、
# 一个目标都没派发，而汇总表看起来像"探索一个目标都产不出"的逻辑缺陷。
# 少一条判据，就会把一次基础设施故障读成一次探索故障。
# QoS 一律用 sensor_data(BEST_EFFORT/VOLATILE)：它对 RELIABLE 与
# TRANSIENT_LOCAL 的发布者都兼容，反过来会一帧都收不到且只有一条 WARNING。
NEED = [
    ('scan', LaserScan, '/scan_from_cloud'),
    ('map', OccupancyGrid, '/map'),
    ('costmap', Costmap, '/global_costmap/costmap_raw'),
    ('odom', Odometry, '/odom'),
]

DEADLINE = float(sys.argv[1])
rclpy.init()
n = Node('gate')
c = {k: 0 for k, _, _ in NEED}
for key, typ, topic in NEED:
    n.create_subscription(
        typ, topic,
        (lambda k: (lambda _m: c.__setitem__(k, c[k] + 1)))(key),
        qos_profile_sensor_data)

t0 = time.time()
ok = False
# 每 5s 一个计数窗口；四路都在同一个窗口内出过数据才算通过。
while time.time() - t0 < DEADLINE:
    for k in c:
        c[k] = 0
    w0 = time.time()
    while time.time() - w0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if all(v > 0 for v in c.values()):
        ok = True
        break

rate = ' '.join('%s=%.1f' % (k, c[k] / 5.0) for k, _, _ in NEED)
print('%s %.0f %s' % ('GATE_OK' if ok else 'GATE_FAIL', time.time() - t0, rate),
      flush=True)
sys.stdout.flush()
# 绝不走 destroy_node()/rclpy.shutdown()：实测它会挂死(78 分钟只烧掉 1s CPU)，
# 而闸门是被 $(...) 命令替换捕获的，它一挂整个驱动脚本就永久卡住、后续轮次
# 一轮都不会跑 —— 判据已经打印完了，此处直接退进程，不给关闭路径任何机会。
import os
os._exit(0)
PY

launch_stack() {   # $1=日志路径
  cd "$REPO"
  nohup ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
    env:=sim mode:=mapping exploration:=true controller_plugin:=mppi \
    use_rviz:=true headless:=false > "$1" 2>&1 &
  echo $!
}

for r in $(seq 1 "$ROUNDS"); do
  RD="$ROOT/round_$r"; rm -rf "$RD"; mkdir -p "$RD"
  echo "################ 第 $r 轮 ################"

  status=INVALID; gate_t=-1; scan_hz=0; map_hz=0; note=''
  for attempt in $(seq 0 "$RETRY"); do
    echo "---- 第 $r 轮 第 $((attempt+1)) 次启动 ----"
    bash "$REPO/tools/clean_sim_stack.sh" > "$RD/clean_$attempt.log" 2>&1
    pid=$(launch_stack "$RD/stack_$attempt.log")
    echo "launch pid=$pid"

    gate=$(python3 "$ROOT/gate.py" "$GATE_SEC" 2>/dev/null | tail -1)
    gate_t=$(echo "$gate" | awk '{print $2}')
    scan_hz=$(echo "$gate" | awk '{print $3}')
    map_hz=$(echo "$gate" | awk '{print $4}')
    rviz=$(pgrep -x rviz2 | wc -l)
    echo "闸门: $gate   rviz2 进程=$rviz"

    if [[ "$gate" == GATE_OK* && "$rviz" -ge 1 ]]; then
      status=VALID
      break
    fi
    # 闸门不过：把"哪一跳断的"记下来，便于事后按跳统计而不是只知道"又坏了"
    {
      echo "== 闸门不过，逐跳拍率与各节点 CPU =="
      ps -eo pid,etimes,time,pcpu,args | grep -E 'livox_(preprocess|fusion)|slice_scan' | grep -v grep
    } > "$RD/gate_fail_$attempt.txt" 2>&1
    note="闸门不过(第$((attempt+1))次)"
  done

  if [ "$status" = VALID ]; then
    echo "---- 闸门通过，开始 ${EXPLORE_SEC}s 探索计量 ----"
    nohup ros2 run astribot_s1_navigation explore_metrics_recorder_node --ros-args \
      -p use_sim_time:=true -p output_dir:="$RD/metrics" \
      -p run_label:="round$r" > "$RD/recorder.log" 2>&1 &
    rec=$!
    sleep "$EXPLORE_SEC"
    kill -INT "$rec" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$rec" 2>/dev/null || break; sleep 1; done
    kill -9 "$rec" 2>/dev/null
    note="探索 ${EXPLORE_SEC}s 完成"
  else
    echo "---- 第 $r 轮判为无效（基础设施故障，不计入探索结论）----"
  fi

  printf '%d\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$r" "$status" "$gate_t" "${rviz:-0}" "$scan_hz" "$map_hz" "$note" >> "$SUMMARY"

  bash "$REPO/tools/clean_sim_stack.sh" > "$RD/clean_final.log" 2>&1
done

echo
echo "==================== 五轮汇总 ===================="
cat "$SUMMARY"
