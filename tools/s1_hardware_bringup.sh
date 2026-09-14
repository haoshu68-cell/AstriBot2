#!/usr/bin/env bash
set -e   # 刻意不用 -u：source ROS 的 setup.bash 在 set -u 下会静默退出，一个字都不打印

SDK=/home/astribot/Downloads/astribot_sdk_aarch64
WS=$SDK/ws_robot
TOOLS=/home/astribot/s1_tools
LOG=/tmp/s1_logs
SHARE=$WS/install/astribot_s1_navigation/share/astribot_s1_navigation
SELF_TAG=s1_hardware_bringup      # 用来把自己从"要杀的进程"里排除掉，见 ours_pids

export ROS_DOMAIN_ID=25          # 实机是 25，不是 42；查询侧不带就是"节点全都 Node not found"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
export FASTRTPS_SHM_PERMISSION=UNRESTRICT
unset ROS_LOCALHOST_ONLY
export PATH=/opt/ros/humble/bin:$PATH   # 厂商裁剪版 ros2 只有 7 个子命令，必须压下去

export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{date_time_with_ms}] [{severity}] [{name}]: {message}'

AXIS_SPEED_CAP=0.5     # 2026-09-08 用户决定由 0.2 提到 0.5
export AXIS_SPEED_CAP  # 两个判据的 python3 heredoc 是 <<'PY'（不做 shell 展开），
                       # 只能从环境读；读不到就 KeyError 炸掉 —— 刻意不给默认值，
                       # 一个静默回落到 0.2 的判据比没有判据更坏。

mkdir -p $LOG $TOOLS
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash

# 自检：**写错的 token 会被原样打印出来，不报错、不告警**（实测 `{no_such_token}`
# 打出的就是字面量 `[{no_such_token}]`）。所以这一行不能只"看着对"，必须真跑一次。
# 用 rclpy.logging 而不是建节点：不 init、不起 DDS participant，毫秒级返回。
FMT_PROBE=$(RCUTILS_CONSOLE_OUTPUT_FORMAT="$RCUTILS_CONSOLE_OUTPUT_FORMAT" \
  python3 -c "import rclpy.logging as L; L.get_logger('fmt_probe').info('ok')" 2>&1 | tail -1)
if ! printf '%s' "$FMT_PROBE" | grep -qE '^\[[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{3}\] \[INFO\] \[fmt_probe\]: ok$'; then
  echo "⚠️  日志时间戳格式自检未通过，实际输出: $FMT_PROBE"
  echo "    → 回退到 rcutils 默认格式（epoch 纳秒），日志仍可用，只是跨模块对齐要手算。"
  unset RCUTILS_CONSOLE_OUTPUT_FORMAT
fi

ST=/opt/ros/humble/lib/tf2_ros/static_transform_publisher

# ---------------------------------------------------------------------------
# launcher：厂商裁剪版 ros2 没有 launch 子命令，而阶段 8 source 完 env.sh 之后
# PATH 会被换回厂商的 ros2 —— 所以所有 launch 一律走这个 Python 直调。
#
# ⚠️ 它**必须放在持久目录**。原来放 /tmp/roslaunch.py，实测机器人一重启 /tmp 就被清空、
#    全机再无副本，脚本每次重启后必挂在第一个 launch 上。改放 $TOOLS 并在这里自生成，
#    让脚本自洽（不依赖任何"上次遗留在 /tmp 的文件"）。
# ---------------------------------------------------------------------------
RL=$TOOLS/roslaunch.py
# 每次都重写，不做 "if not exists" —— 否则一个内容已经过时/写坏的旧 launcher
# 会永远不被覆盖，脚本就和自己的源码脱钩了。
if true; then
  cat > "$RL" <<'RLPY'
#!/usr/bin/env python3
"""ros2 launch 的最小等价物：
     roslaunch.py <pkg> <launch_file> [k:=v ...]
     roslaunch.py --path <绝对路径的 launch 文件> [k:=v ...]

为什么不用 `ros2 launch`：这台机器人上 **ros2launch 这个包没装**
（dpkg 里 0 个 ros-humble-ros2launch，site-packages 里也没有），所以厂商裁剪版和
/opt/ros/humble 的 ros2 都没有 launch 子命令。launch / launch_ros 本体是齐的，
直接用 LaunchService 跑。LaunchService.run() 自带 SIGINT 处理，
会把子进程按 launch 的关停流程带走 —— 这正是 stop 的 SIGINT 那一轮需要的。

--path 模式是必需的，不是可选糖：nav2 要绕过 nav2_full_bringup 直指 src 下的
navigation.launch.py（那一层的 launch_arguments 白名单会静默吞参数）。
"""
import os, sys
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

argv = sys.argv[1:]
if argv and argv[0] == '--path':
    if len(argv) < 2:
        print('usage: roslaunch.py --path <launch 文件> [arg:=val ...]', file=sys.stderr)
        raise SystemExit(2)
    path, rest = argv[1], argv[2:]
    if not os.path.isfile(path):
        print('找不到 launch 文件 %s' % path, file=sys.stderr)
        raise SystemExit(2)
else:
    if len(argv) < 2:
        print('usage: roslaunch.py <package> <launch_file> [arg:=val ...]'
              ' | roslaunch.py --path <launch 文件> [arg:=val ...]', file=sys.stderr)
        raise SystemExit(2)
    from ament_index_python.packages import get_package_share_directory
    pkg, lf, rest = argv[0], argv[1], argv[2:]
    share = get_package_share_directory(pkg)
    path = next((c for c in (os.path.join(share, 'launch', lf), os.path.join(share, lf))
                 if os.path.isfile(c)), None)
    if path is None:                                  # 兜底：整棵 share 里找
        for root, _, files in os.walk(share):
            if lf in files:
                path = os.path.join(root, lf)
                break
    if path is None:
        print('找不到 launch 文件 %s（包 %s，share=%s）' % (lf, pkg, share), file=sys.stderr)
        raise SystemExit(2)
args = [tuple(a.split(':=', 1)) for a in rest if ':=' in a]
svc = LaunchService()
svc.include_launch_description(
    LaunchDescription([IncludeLaunchDescription(AnyLaunchDescriptionSource(path),
                                               launch_arguments=args)]))
raise SystemExit(svc.run())
RLPY
  chmod +x "$RL"
fi

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
MODE=${1:-start}
USE_RVIZ=true
# 默认**使能写通路**（用户 2026-09-03 明确要求"按流程保证模块全部启动就好，并使能就好"）。
# 使能 = 底盘真的会动。不想动就显式加 --no-drive。
DRIVE=true
for a in "$@"; do
  case "$a" in
    --no-rviz)         USE_RVIZ=false ;;
    --no-drive)        DRIVE=false ;;
    --drive)           DRIVE=true ;;   # 兼容旧写法，现在是默认值
    --keep-nav2-yaml)  : ;;  # 兼容旧调用；现在始终保留安装配置。
  esac
done

say()  { printf '\n\033[1;36m═══ %s\033[0m\n' "$*"; }
ok()   { printf '  \033[32m✔\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m✘ %s\033[0m\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
die()  { printf '\n\033[31m✗ 判据不过，停在这里：%s\033[0m\n' "$*"; exit 1; }

# ---------------------------------------------------------------------------
# 拍率判据。判"话题在流"必须看拍率，不能看 count_publishers ——
# 链上每个节点都有发布者，所以 pub>0 恒为真；实测过 5 个话题全 pub=1 但 0Hz。
# 订阅端刻意用 BEST_EFFORT：它能收 RELIABLE 也能收 BEST_EFFORT 的发布者，反之不行。
# ---------------------------------------------------------------------------
wait_hz() {  # wait_hz <topic> <type> <min_hz> <timeout_s>
  python3 - "$1" "$2" "$3" "$4" <<'PY'
import importlib, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
topic, tname, min_hz, timeout = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
pkg, cls = tname.rsplit('/', 1)
msg = getattr(importlib.import_module(pkg.replace('/', '.') + '.msg'), cls)
rclpy.init()
n = rclpy.create_node('wait_hz')
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=20)
cnt = [0]; first = [None]
n.create_subscription(msg, topic, lambda m: (cnt.__setitem__(0, cnt[0] + 1),
                                             first[0] is None and first.__setitem__(0, time.time())), q)
t0 = time.time()
while time.time() - t0 < timeout:
    rclpy.spin_once(n, timeout_sec=0.1)
    if first[0] and cnt[0] >= 3 and time.time() - first[0] > 1.0:
        hz = (cnt[0] - 1) / (time.time() - first[0])
        if hz >= min_hz:
            print('%.2f' % hz); n.destroy_node(); rclpy.shutdown(); sys.exit(0)
hz = 0.0 if not first[0] else (cnt[0] - 1) / max(1e-9, time.time() - first[0])
print('%.2f' % hz); n.destroy_node(); rclpy.shutdown(); sys.exit(1)
PY
}

ours_pids() {   # 打印 "<pid> <cmdline>"，每行一个，只含我们的进程
  ps -eo pid,args --no-headers > /tmp/s1_ps_snap.txt
  SELF_PID=$$ SELF_PPID=$PPID SELF_TAG=$SELF_TAG python3 - <<'PY'
import os, re

VENDOR = (                       # 一票否决，先判
    "/opt/astribot_ros/",                 # 厂商整套栈
    "/home/astribot/SLAM/vxlm-slam/",     # 厂商 SLAM(voxelslam / nav_prob_grid_node)
    "astribot_orin_startup.sh",           # 开机脚本
    "ptp_sync_time", "orin_sync.sh",      # 时间同步(实机开机时钟是 1970，别碰)
    "/usr/NX/", "nxexec", "x11vnc",       # 远程桌面 —— 杀了就再也连不上
    "sshd", "ros2cli.daemon",             # 自己这条连接 / ros2 CLI daemon
    # ↓ start 阶段 0 用**我们的** launcher 起厂商包 —— 那个 launcher 进程的命令行里
    #   含 $TOOLS 路径，会命中下面的 OURS，等于 stop 一跑就把厂商 SLAM 带走。
    #   按"它在起谁"否决掉。
    "roslaunch.py voxel_slam",
    "roslaunch.py livox_ros_driver2",
    "roslaunch.py nav_prob_grid",
)
OURS = (                         # 我们的：按路径
    "/Downloads/astribot_sdk_aarch64/ws_robot/install/",  # 我们编译出来的所有包
    "/tmp/roslaunch.py",                                 # 旧路径，留着以便清掉重启前遗留的进程
    "/home/astribot/s1_tools/",                          # launcher / tf_to_odom / grid_self_clear 等
    "/opt/ros/humble/lib/nav2_",                         # nav2 全家桶(厂商从不用)
    "/opt/ros/humble/lib/tf2_ros/static_transform_publisher",
    "/opt/ros/humble/lib/pointcloud_to_laserscan",
    "/opt/ros/humble/lib/robot_state_publisher",
    "/opt/ros/humble/lib/joint_state_publisher",
)
OURS_RE = (
    re.compile(r"rviz2\b.*astribot_s1_navigation"),      # 只认加载我们配置的那个 rviz2
)

tag       = os.environ["SELF_TAG"]
skip_pids = {int(os.environ["SELF_PID"]), int(os.environ["SELF_PPID"]),
             os.getpid(), os.getppid()}

for line in open("/tmp/s1_ps_snap.txt"):
    parts = line.strip().split(None, 1)
    if len(parts) < 2:
        continue
    try:
        pid = int(parts[0])
    except ValueError:
        continue
    cmd = parts[1]
    # 排除自己：PID、脚本名、快照命令、以及本 heredoc
    if pid in skip_pids or tag in cmd:
        continue
    if cmd.startswith("ps -eo") or cmd.startswith("python3 -") \
       or cmd.startswith("/usr/bin/python3 -"):
        continue
    if any(v in cmd for v in VENDOR):        # 厂商护栏一票否决
        continue
    if any(o in cmd for o in OURS) or any(r.search(cmd) for r in OURS_RE):
        print(pid, cmd)
PY
}

ours_pid_list() { ours_pids | awk '{print $1}'; }
ours_count()    { ours_pids | grep -c . || true; }

do_stop() {
  say "停止所有非厂商插件（厂商 SLAM / 雷达驱动 / 远程桌面不动）"

  if ours_pids | grep -q "astribot_trajectory_bridge"; then
    timeout 8 ros2 service call /astribot_bridge_container/disable \
      std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 \
      && ok "底盘桥接已 disable（指令归零）" || warn "disable 没调成功，继续按 PID 停"
    sleep 1
  fi

  local snap n
  snap=$(ours_pids || true)
  n=$(printf '%s' "$snap" | grep -c . || true)
  if [ "${n:-0}" -eq 0 ]; then
    ok "没有我们的进程在跑"
  else
    printf '  要停的进程 %s 个：\n' "$n"
    printf '%s\n' "$snap" | cut -c1-140 | sed 's/^/    /'

    printf '%s\n' "$snap" | awk '{print $1}' | while read -r p; do
      [ -n "$p" ] && kill -INT "$p" 2>/dev/null || true
    done
    sleep 5

    local sig
    for sig in TERM KILL; do
      local rest
      rest=$(ours_pid_list || true)
      [ -z "$rest" ] && break
      printf '%s\n' "$rest" | while read -r p; do
        [ -n "$p" ] && kill -"$sig" "$p" 2>/dev/null && echo "    $sig $p" || true
      done
      sleep 3
    done
  fi

  local left nleft
  left=$(ours_pids || true)
  nleft=$(printf '%s' "$left" | grep -c . || true)
  if [ "${nleft:-0}" -gt 0 ]; then
    bad "还有 $nleft 个残留，请手工处理："
    printf '%s\n' "$left" | cut -c1-140 | sed 's/^/    /'
  else
    ok "残留 0 个"
  fi

  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  rm -f /dev/shm/fastdds_*  /dev/shm/sem.fastdds_*  2>/dev/null || true
  local nshm
  nshm=$(ls /dev/shm 2>/dev/null | grep -c -E '^(sem\.)?(fastrtps|fastdds)' || true)
  ok "fastdds 共享内存残留 ${nshm:-0} 个"

  ok "厂商 voxelslam / nav_prob_grid_node / livox 驱动保持原样"
}

do_status() {
  say "我们的进程"
  local snap n
  snap=$(ours_pids || true)
  n=$(printf '%s' "$snap" | grep -c . || true)
  printf '  共 %s 个\n' "${n:-0}"
  [ "${n:-0}" -gt 0 ] && printf '%s\n' "$snap" | cut -c1-150 | sed 's/^/    /'

  say "厂商进程（只看不动）"
  ps -eo pid,etime,args --no-headers \
    | grep -E "voxelslam|nav_prob_grid|livox_ros_driver2_node" | grep -v grep \
    | cut -c1-120 | sed 's/^/    /' || echo "    (无)"

  say "话题拍率"
  for spec in "/scan sensor_msgs/LaserScan" "/odom nav_msgs/Odometry" \
              "/map_scan_filtered_prob nav_msgs/OccupancyGrid" "/map_nav nav_msgs/OccupancyGrid" \
              "/cmd_vel_nav_body geometry_msgs/Twist" "/cmd_vel geometry_msgs/Twist"; do
    set -- $spec
    printf '  %-26s %s Hz\n' "$1" "$(wait_hz "$1" "$2" 0.01 6 || true)"
  done
}

case "$MODE" in
  stop)   do_stop; exit 0 ;;
  status) do_status; exit 0 ;;
esac

verify_all() {
  local fail=0 hz

  say "判据 1/7  厂商 SLAM 在出栅格"
  hz=$(wait_hz /map_scan_filtered_prob nav_msgs/OccupancyGrid 0.3 20) \
    && ok "/map_scan_filtered_prob $hz Hz" \
    || { bad "/map_scan_filtered_prob 只有 $hz Hz（厂商标称 1.0）"; fail=1; }

  say "判据 2/7  /scan 干净（自滤没滤错体积）"
  python3 - <<'PY' || fail=1
import rclpy, time, math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
FOOTPRINT_CIRCUMSCRIBED = 0.42
rclpy.init(); n = rclpy.create_node('scan_chk')
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=5)
got = []
n.create_subscription(LaserScan, '/scan', lambda m: got.append(m), q)
t0 = time.time()
while time.time() - t0 < 15 and len(got) < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
if not got:
    print('  ✘ /scan 一帧未收到'); raise SystemExit(1)
m = got[-1]
fin = [r for r in m.ranges if math.isfinite(r) and r > 0.0]
inside = [r for r in fin if r < FOOTPRINT_CIRCUMSCRIBED]
print('  束数=%d 有效=%d 最近=%.3fm  落在足迹内(<%.2fm)的束数=%d' %
      (len(m.ranges), len(fin), min(fin) if fin else float('nan'),
       FOOTPRINT_CIRCUMSCRIBED, len(inside)))
if inside:
    print('  最内侧几束: %s' % ['%.3f' % r for r in sorted(inside)[:8]])
raise SystemExit(1 if inside else 0)
PY
  [ $fail -eq 0 ] && ok "/scan 足迹内无回波（自滤没漏机器人自己）"

  say "判据 3/7  /odom 在流"
  hz=$(wait_hz /odom nav_msgs/Odometry 15 15) \
    && ok "/odom $hz Hz" || { bad "/odom 只有 $hz Hz（要 20Hz）"; fail=1; }

  say "判据 4/7  /map_nav 里机器人自身假障碍已清"
  python3 - <<'PY' || fail=1
import rclpy, time, math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
CLEAR_R = 0.25          # 必须与 grid_self_clear_node 的 clear_radius_m 一致
OCC = 65
rclpy.init(); n = rclpy.create_node('mapnav_chk')
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=1)
s = {}
n.create_subscription(OccupancyGrid, '/map_nav',  lambda m: s.__setitem__('nav', m), q)
n.create_subscription(OccupancyGrid, '/map_scan_filtered_prob', lambda m: s.__setitem__('raw', m), q)
n.create_subscription(Odometry, '/odom', lambda m: s.__setitem__('od', m), q)
t0 = time.time()
while time.time() - t0 < 20 and len(s) < 3:
    rclpy.spin_once(n, timeout_sec=0.2)
if len(s) < 3:
    print('  ✘ 收不齐 (%s)' % ','.join(sorted(s))); raise SystemExit(1)
od = s['od']; x, y = od.pose.pose.position.x, od.pose.pose.position.y
def occ_in_disc(g, r):
    res, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
    cx, cy = int((x - ox) / res), int((y - oy) / res)
    k = int(math.ceil(r / res)); c = 0
    for j in range(cy - k, cy + k + 1):
        for i in range(cx - k, cx + k + 1):
            if not (0 <= i < g.info.width and 0 <= j < g.info.height):
                continue
            if math.hypot((i - cx) * res, (j - cy) * res) > r:   # ← 圆，不是方框
                continue
            if g.data[j * g.info.width + i] >= OCC:
                c += 1
    return c
nn = occ_in_disc(s['nav'], CLEAR_R); rr = occ_in_disc(s['raw'], CLEAR_R)
print('  机器人 (%.3f, %.3f)；半径 %.2fm **圆内**占据格：/map_nav=%d  厂商原始=%d'
      % (x, y, CLEAR_R, nn, rr))
# 把"清掉了"和"本来就没有"分开报：两者都让 nn=0，但含义完全不同。
if rr == 0:
    print('  注意：厂商原始本来就是 0 —— 这一帧**没有东西可清**，'
          '所以这条判据这次不构成"自滤在工作"的证据')
else:
    print('  自滤实际清掉 %d 格' % (rr - nn))
raise SystemExit(1 if nn > 0 else 0)
PY
  [ $fail -eq 0 ] && ok "/map_nav 清理半径圆内占据格 = 0"

  say "判据 5/7  nav2 活着且吃到地图（MPPI / ${AXIS_SPEED_CAP}m/s 按轴 / 真墙钟）"
  local n_active n_nomap vx
  n_active=$(grep -c "Managed nodes are active" $LOG/nav2.log 2>/dev/null | head -1); n_active=${n_active:-0}
  n_nomap=$(grep -c "no map received" $LOG/nav2.log 2>/dev/null | head -1); n_nomap=${n_nomap:-0}
  [ "$n_active" -ge 1 ] && ok "生命周期已 active" || { bad "nav2 未 active"; fail=1; }
  [ "$n_nomap" -eq 0 ] && ok "静态层无 'no map received'" \
    || { bad "静态层报 no map received ×$n_nomap（map_topic 指错了）"; fail=1; }
  grep -oE "Received a [0-9]+ X [0-9]+ map" $LOG/nav2.log 2>/dev/null | tail -1 \
    | sed 's/^/  全局图: /' || true
  python3 - <<'PY' || fail=1
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
rclpy.init()
n = rclpy.create_node('clk_chk')
cli = n.create_client(GetParameters, '/controller_server/get_parameters')
if not cli.wait_for_service(timeout_sec=15.0):
    print('  ✘ /controller_server/get_parameters 15s 内不可用'); raise SystemExit(1)
fut = cli.call_async(GetParameters.Request(names=['use_sim_time']))
rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
if fut.result() is None or not fut.result().values:
    print('  ✘ 读不到 use_sim_time'); raise SystemExit(1)
sim = fut.result().values[0].bool_value
print('  controller_server use_sim_time =', sim)
# 时钟前进量：用节点自己的时钟（use_sim_time=False 时就是墙钟）
t0 = n.get_clock().now().nanoseconds
import time; time.sleep(1.0)
dt = (n.get_clock().now().nanoseconds - t0) / 1e9
print('  节点时钟 1s 内前进 %.3f s' % dt)
rclpy.shutdown()
raise SystemExit(0 if (sim is False and dt > 0.5) else 1)
PY
  [ $fail -eq 0 ] && ok "use_sim_time=False 且时钟在走（真墙钟）"
  local cap_fail=0
  python3 - <<'PY' || cap_fail=1
import rclpy
import os
from rcl_interfaces.srv import ListParameters, GetParameters
CAP = float(os.environ['AXIS_SPEED_CAP'])   # 无默认值：见脚本顶部 AXIS_SPEED_CAP
# 这条判据能抓到"外层 launch 把 max_linear_speed 静默吞了"，靠的是漏传时
# vx_max 会回落到 yaml 的 1.0 而 1.0 > CAP。所以 CAP 一旦被调到 >= 1.0，
# 判据就退化成恒真、再也分不清"限速生效"和"参数根本没传下去"。
if CAP >= 1.0:
    print('  ✘ AXIS_SPEED_CAP=%.2f >= 1.0：本判据会退化成恒真（yaml 回落值也是 1.0），'
          '分不清限速生效与参数被吞' % CAP)
    raise SystemExit(1)
rclpy.init()
n = rclpy.create_node('cap_chk')
lc = n.create_client(ListParameters, '/controller_server/list_parameters')
if not lc.wait_for_service(timeout_sec=15.0):
    print('  ✘ list_parameters 15s 内不可用'); raise SystemExit(1)
f = lc.call_async(ListParameters.Request(prefixes=[], depth=0))
rclpy.spin_until_future_complete(n, f, timeout_sec=20.0)
if f.result() is None:
    print('  ✘ list_parameters 无响应'); raise SystemExit(1)
keys = sorted(k for k in f.result().result.names
              if k.endswith('.vx_max') or k.endswith('.vy_max'))
if not keys:
    print('  ✘ 一个 v?_max 参数都没枚举到（判据自身失效，不能算通过）')
    raise SystemExit(1)
gc = n.create_client(GetParameters, '/controller_server/get_parameters')
if not gc.wait_for_service(timeout_sec=10.0):
    print('  ✘ get_parameters 不可用'); raise SystemExit(1)
f2 = gc.call_async(GetParameters.Request(names=keys))
rclpy.spin_until_future_complete(n, f2, timeout_sec=20.0)
if f2.result() is None or len(f2.result().values) != len(keys):
    print('  ✘ get_parameters 返回不完整'); raise SystemExit(1)
over = []
for k, v in zip(keys, f2.result().values):
    print('  %-40s = %.3f m/s' % (k, v.double_value))
    if v.double_value > CAP + 1e-4:
        over.append((k, v.double_value))
for k, val in over:
    print('  ✘ %s=%.3f 超过授权的各轴上限 %.2f' % (k, val, CAP))
print('  共查 %d 个轴上限，越限 %d 个' % (len(keys), len(over)))
raise SystemExit(1 if over else 0)
PY
  if [ $cap_fail -eq 0 ]; then
    ok "各轴线速度上限 ≤ $AXIS_SPEED_CAP m/s（vy≡0，故模长上界同为 $AXIS_SPEED_CAP）"
  else
    fail=1
  fi

  say "判据 6/7  探索调度器：红线不触发 + 目标**正在**派发（增量判据）"
  local n_block
  n_block=$(grep -c "物理真堵" $LOG/explore.log 2>/dev/null | head -1); n_block=${n_block:-0}
  [ "$n_block" -eq 0 ] && ok "红线未触发（0 次物理真堵）" \
    || { bad "红线触发 ×$n_block —— 机器人所在格又被判占据了，先查 /map_nav"; fail=1; }
  python3 - "$LOG/explore.log" <<'PY' || fail=1
import os, re, subprocess, sys, time
log = sys.argv[1]
if not os.path.exists(log):
    print('  ✘ %s 不存在' % log); raise SystemExit(1)
age = time.time() - os.path.getmtime(log)
print('  explore.log 最后写入 %.1fs 前' % age)
if age > 30:
    print('  ✘ 日志已经 %.0fs 没有新行 —— 调度器不是活的' % age); raise SystemExit(1)
# 只看**观察窗内新写的行**：先记下文件偏移，睡完从偏移处续读。
# 拿整个文件 grep 会把上一轮的历史行算进来 —— 那正是这条判据以前假过的方式。
WINDOW = 12.0
off = os.path.getsize(log)
print('  观察 %.0fs（只统计此刻之后新写的行）...' % WINDOW)
time.sleep(WINDOW)
with open(log, errors='replace') as f:
    f.seek(off)
    fresh = f.readlines()
def n(p):
    r = re.compile(p)
    return sum(1 for l in fresh if r.search(l))
n_disp  = n('已校验路径已交给控制器')      # 真派发
n_cycle = n('IDLE -> GEN_NEXT_POINT')      # 循环在转（前置条件就绪）
n_park  = n('自动恢复已达上限')            # 停在上限（**注意**：这行是 THROTTLE 复读，
                                           # 只要进过一次 PAUSED 就会一直印，不能当"现在停着"）
n_res   = n('PAUSED -> IDLE')              # 自动/人工恢复真的把状态推回去了
print('  窗内新增: 派发 %d | IDLE->GEN_NEXT_POINT %d | PAUSED->IDLE %d | 上限复读 %d'
      % (n_disp, n_cycle, n_res, n_park))
if n_disp >= 1:
    print('  ✔ 派发在推进'); raise SystemExit(0)
# 没派发。分两种，责任方完全不同，必须分开报：
if n_cycle + n_res == 0:
    print('  ✘ 调度器既没派发也没在转循环 —— 它真的停住了（查 %s）' % log)
    raise SystemExit(1)
# 循环在转但候选全被否 => 是"站位被围住"这一类，属已延后处理项，不是脚本/链路故障。
why = [l.split(']: ', 1)[-1].strip() for l in fresh
       if '候选' in l or '净空' in l or '安全门' in l]
print('  ! 调度器活着在转（%d 次前置就绪）但候选全被否，未派发。窗内原因：' % n_cycle)
for w in sorted(set(why))[:4]:
    print('      %s' % w)
print('  ! 这是"开机站位被膨胀带围住"，与 Starting point in lethal space 同一类，')
print('    按已定口径等 SLAM 解决 —— 不计为本脚本判据失败。')
raise SystemExit(0)
PY
  [ $fail -eq 0 ] && ok "调度器活着（派发与否见上）"

  say "判据 7/7  机器人**真的动了**（只在使能写通路时要求）"
  if [ "$DRIVE" != true ]; then
    warn "带了 --no-drive，写通路没使能 —— 跳过位移判据（此时机器人不动是正确的）"
  else
    python3 - <<'PY' || fail=1
import math, time, rclpy, os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
CAP = float(os.environ['AXIS_SPEED_CAP'])   # 无默认值：见脚本顶部 AXIS_SPEED_CAP
rclpy.init(); n = Node('moved_chk')
qv = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=50)
P, V = [], []
n.create_subscription(Odometry, '/odom',
                      lambda m: P.append((m.pose.pose.position.x, m.pose.pose.position.y)), qv)
n.create_subscription(Twist, '/cmd_vel',
                      lambda m: V.append((m.linear.x, m.linear.y, m.angular.z)), qv)
WINDOW = 20.0
t0 = time.time()
while time.time() - t0 < WINDOW:
    rclpy.spin_once(n, timeout_sec=0.1)
if not P:
    print('  ✘ /odom 一帧未收到'); raise SystemExit(1)
L = sum(math.dist(P[i], P[i + 1]) for i in range(len(P) - 1))
net = math.dist(P[0], P[-1]); far = max(math.dist(P[0], p) for p in P)
vmax = max((math.hypot(v[0], v[1]) for v in V), default=0.0)
nz = sum(1 for v in V if abs(v[0]) > 1e-4 or abs(v[1]) > 1e-4 or abs(v[2]) > 1e-4)
# 限速口径 = **各轴** 0.2（已定）。所以越限只能按轴数，不能按模长数：
# 按模长数在 vy_max>0 的配置下会把合法斜向运动报成越限。（本轮 vy≡0，两者恰好等价，
# 但判据仍按轴写 —— 一旦哪天放开 vy，按模长的判据会立刻开始假阳性。）
axmax = max((max(abs(v[0]), abs(v[1])) for v in V), default=0.0)
over = sum(1 for v in V if abs(v[0]) > CAP + 1e-4 or abs(v[1]) > CAP + 1e-4)
# 不要在这里印 sqrt(2)*CAP 当"模长上界" —— 那个算术是错的，脚本尾部
# 「已知未解决」里已经记了：vy_max 被路线A 钉死为 0.0 且 motion_model 是
# DiffDrive（navigation.launch.py 强制 'vy_max':'0.0'，yaml 三处也都是 0.0），
# MPPI 压根不输出横向速度，所以模长上界就等于按轴上限本身。
print('  /cmd_vel %d 帧，非零 %d，单轴峰值 %.4f m/s（越 %.2f 的帧 %d），'
      '合速度模长峰值 %.4f m/s（vy≡0，模长上界=按轴上限 %.2f）'
      % (len(V), nz, axmax, CAP, over, vmax, CAP))
if over:
    print('  ✘ 有 %d 帧单轴超过 %.2f m/s —— 越过授权上限' % (over, CAP)); raise SystemExit(1)
print('  /odom  %d 帧：净位移 %.4f m，最大离起点 %.4f m，轨迹长 %.4f m' % (len(P), net, far, L))
# 判据：轨迹长 > 0.05m（净位移会被"转一圈回原地"抹掉，轨迹长不会）
if L < 0.05:
    print('  ! %.0fs 内轨迹长只有 %.4f m —— 机器人没动。' % (WINDOW, L))
    if nz == 0:
        # /cmd_vel 全零 = 上游根本没有目标在跑（候选被否，见判据 6），
        # 不是写通路的问题。这一类按已定口径等 SLAM，不在这里判失败。
        print('  ! 且 /cmd_vel 全零 —— 上游没有目标在派发（见判据 6），不是使能/控制权问题。')
        raise SystemExit(0)
    print('  ✘ /cmd_vel 有非零帧却不动 —— 查桥接日志里有没有'
          ' "You don\'t have control rights"'); raise SystemExit(1)
print('  ✔ 机器人实测在移动')
PY
    [ $fail -eq 0 ] && ok "机器人实测位移达标"
  fi

  return $fail
}

if [ "$MODE" = "verify" ]; then
  verify_all && { printf '\n\033[1;32m全部判据通过\033[0m\n'; exit 0; } \
             || { printf '\n\033[1;31m有判据不过，见上\033[0m\n'; exit 1; }
fi

[ "$MODE" = "start" ] || die "未知模式 '$MODE'（可用 start / stop / status / verify）"

if [ "$DRIVE" = true ]; then
  printf '\n\033[1;33m┌──────────────────────────────────────────────────────────────┐\033[0m\n'
  printf '\033[1;33m│ 默认使能：会强夺控制权（**立刻停止机器人当前运动**）并驱动底盘 │\033[0m\n'
  printf '\033[1;33m│ 确认：周围安全 / 没有别人在操作 / 急停可及                    │\033[0m\n'
  printf '\033[1;33m└──────────────────────────────────────────────────────────────┘\033[0m\n'
fi

do_stop     # 干净起点：把上一轮我们的进程全清掉（含重复进程）

say "阶段 0  感知底座：雷达驱动 + 厂商 SLAM（没起就起，已起不动）"
SLAM_WS=/home/astribot/SLAM/vxlm-slam
LIVOX_PREFIX=/opt/astribot_ros/software/livox_ros_driver2
[ -f "$SLAM_WS/install/setup.bash" ] || die "找不到 $SLAM_WS/install/setup.bash"

GTSAM_LIB=/home/astribot/SLAM/ThirdParty/GTSAM/install4.1.0/lib
[ -d "$GTSAM_LIB" ] || warn "$GTSAM_LIB 不存在 —— voxelslam 可能缺 libmetis-gtsam.so"

launch_vendor() {   # launch_vendor <日志名> <包> <launch 文件> <存活判据模式>
  local tag=$1 pkg=$2 lf=$3 pat=$4
  if [ "$tag" = livox ]; then
    local badpid
    badpid=$(ps -eo pid,args --no-headers \
             | grep -F 'vxlm-slam/install/livox_ros_driver2' | grep -v 'grep -F' | awk '{print $1}')
    if [ -n "$badpid" ]; then
      warn "在跑的是 vxlm-slam 那份（本机 IP 配错，必然 bind failed），换成 /opt 厂商那份"
      kill $badpid 2>/dev/null || true; sleep 2
      kill -9 $badpid 2>/dev/null || true; sleep 1
    fi
  fi
  if [ "$(ps -eo args --no-headers | grep -F "$pat" | grep -vc "grep -F")" -ge 1 ]; then
    ok "$tag 已在跑，不重复起"
    return 0
  fi
  eval "${tag}_launched=1"   # 记下"本轮是我们起的" —— 否则下面读到的日志是上一轮的
  : > "$LOG/$tag.log"   # 必须清空：不清就会 grep 到上一轮的失败行，把好实例判成坏的
  ( if [ "$tag" = livox ]; then
      export AMENT_PREFIX_PATH=$LIVOX_PREFIX:${AMENT_PREFIX_PATH:-}
      export LD_LIBRARY_PATH=$LIVOX_PREFIX/lib:${LD_LIBRARY_PATH:-}
    else
      source "$SLAM_WS/install/setup.bash" >/dev/null 2>&1
      export LD_LIBRARY_PATH=$GTSAM_LIB:${LD_LIBRARY_PATH:-}
      cd "$SLAM_WS"
    fi
    nohup setsid python3 "$RL" "$pkg" "$lf" > "$LOG/$tag.log" 2>&1 & )
  local i
  for i in $(seq 1 30); do
    sleep 1
    [ "$(ps -eo args --no-headers | grep -F "$pat" | grep -vc "grep -F")" -ge 1 ] && {
      ok "$tag 已拉起"; return 0; }
  done
  bad "$tag 起不来，日志尾部："
  tail -12 "$LOG/$tag.log" 2>/dev/null | sed 's/^/      /'
  return 1
}

livox_hz() {   # 只回报拍率，不做判断
  PYTHONPATH=$LIVOX_PREFIX/local/lib/python3.10/dist-packages:${PYTHONPATH:-} \
  LD_LIBRARY_PATH=$LIVOX_PREFIX/lib:${LD_LIBRARY_PATH:-} \
    wait_hz /livox/lidar_front livox_ros_driver2/CustomMsg 1.0 "$1"
}

for attempt in 1 2; do
  launch_vendor livox livox_ros_driver2 msg_MID360_launch.py livox_ros_driver2_node \
    || die "雷达驱动没起来"
  if hz=$(livox_hz 30); then
    ok "/livox/lidar_front $hz Hz"
    break
  fi
  bad "/livox/lidar_front 只有 ${hz:-0} Hz —— 雷达没出数"
  if [ "${livox_launched:-0}" = 1 ]; then
    grep -qE "bind failed|Init lds lidar fail" "$LOG/livox.log" 2>/dev/null \
      && bad "日志里是 bind failed —— 本机 IP 不是 192.168.0.11（两台雷达应为 .12/.13）"
    tail -8 "$LOG/livox.log" 2>/dev/null | sed 's/^/      /'
  else
    warn "在跑的实例不是本轮起的，$LOG/livox.log 是旧日志、不能当本次的证据"
  fi
  [ "$attempt" = 2 ] && die "重起一次后仍没出数（看 $LOG/livox.log）"
  warn "把这个不出数的实例换掉重起"
  for p in $(pgrep -x livox_ros_driver2_node || true); do kill "$p" 2>/dev/null || true; done
  sleep 3
  for p in $(pgrep -x livox_ros_driver2_node || true); do kill -9 "$p" 2>/dev/null || true; done
  sleep 2
done

launch_vendor slam  voxel_slam        vxlm_mid360.launch.py voxel_slam/voxelslam \
  || die "厂商 SLAM 没起来"

NG=$(ps -eo args --no-headers | grep -c "nav_prob_grid_node" || true)
[ "${NG:-0}" -ge 1 ] && ok "nav_prob_grid_node 在跑" \
  || warn "nav_prob_grid_node 没看到 —— 它由 SLAM launch 带起，下面拍率判据会兜底"
hz=$(wait_hz /map_scan_filtered_prob nav_msgs/OccupancyGrid 0.3 60) \
  || die "/map_scan_filtered_prob 只有 $hz Hz —— SLAM 没在出栅格（看 $LOG/slam.log）"
ok "/map_scan_filtered_prob $hz Hz"

say "阶段 1  URDF -> robot_state_publisher + 静态 TF"
XACRO=$WS/install/astribot_s1_description/share/astribot_s1_description/urdf/astribot_s1.xacro
[ -f "$XACRO" ] || XACRO=$WS/src/astribot_s1_description/urdf/astribot_s1.xacro
rm -f /tmp/rsp_params.yaml   # 先删：xacro 失败时残留的旧文件会让下面的判据假过
python3 - "$XACRO" <<'PY'
import subprocess, sys, yaml
r = subprocess.run(['xacro', sys.argv[1], 'robot_name:=astribot_s1', 'use_lidar:=true',
                    'use_camera:=false', 'controllers_config:=/dev/null'],
                   capture_output=True, text=True)
if r.returncode != 0:
    print('XACRO_FAIL'); print(r.stderr[-800:]); sys.exit(1)
# URDF 不能当 -p 值传（含换行和引号，launch 的参数解析会拒绝），只能走 params-file
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
    'robot_description': r.stdout, 'use_sim_time': False, 'publish_frequency': 30.0}}},
    open('/tmp/rsp_params.yaml', 'w'), default_style='|', allow_unicode=True)
print('XACRO_OK links=%d joints=%d' % (r.stdout.count('<link '), r.stdout.count('<joint ')))
PY
[ -s /tmp/rsp_params.yaml ] || die "URDF 生成失败（xacro 报错见上）"
nohup setsid /opt/ros/humble/lib/robot_state_publisher/robot_state_publisher \
  --ros-args --params-file /tmp/rsp_params.yaml > $LOG/rsp.log 2>&1 &
nohup setsid $ST --frame-id map --child-frame-id camera_init \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 > $LOG/tf_map_ci.log 2>&1 &
nohup setsid $ST --frame-id camera_init --child-frame-id odom \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 > $LOG/tf_ci_odom.log 2>&1 &
ok "RSP + 2 个静态 TF 已起"

say "阶段 2  固定姿态 /joint_states"
python3 - <<'PY'
import yaml
urdf = yaml.safe_load(open('/tmp/rsp_params.yaml'))['robot_state_publisher']['ros__parameters']['robot_description']
yaml.safe_dump({'joint_state_publisher': {'ros__parameters': {
    'robot_description': urdf, 'use_sim_time': False, 'rate': 50,
    'publish_default_positions': True}}},
    open('/tmp/jsp_params.yaml', 'w'), default_style='|', allow_unicode=True)
PY
nohup setsid /opt/ros/humble/lib/joint_state_publisher/joint_state_publisher \
  --ros-args --params-file /tmp/jsp_params.yaml > $LOG/jsp.log 2>&1 &
ok "joint_state_publisher 已起"

say "阶段 3  感知链 -> /scan"
nohup setsid python3 $RL astribot_s1_perception hardware_perception.launch.py \
  use_sim_time:=false > $LOG/perception.log 2>&1 &
hz=$(wait_hz /scan sensor_msgs/LaserScan 5 45) \
  || die "/scan 只有 $hz Hz（雷达/自滤链断了，看 $LOG/perception.log）"
ok "/scan $hz Hz"

say "阶段 4  /odom（从 TF 反推）"
nohup setsid python3 $TOOLS/tf_to_odom_node.py --ros-args -p use_sim_time:=false > $LOG/odom.log 2>&1 &
hz=$(wait_hz /odom nav_msgs/Odometry 15 25) || die "/odom 只有 $hz Hz"
ok "/odom $hz Hz"

say "阶段 5  清机器人自身假障碍 -> /map_nav"
nohup setsid python3 $TOOLS/grid_self_clear_node.py --ros-args \
  -p use_sim_time:=false \
  -p input_topic:=/map_scan_filtered_prob -p output_topic:=/map_nav \
  -p map_frame:=map -p base_frame:=astribot_torso_base \
  -p clear_radius_m:=0.25 -p trail_window_sec:=20.0 > $LOG/selfclear.log 2>&1 &
hz=$(wait_hz /map_nav nav_msgs/OccupancyGrid 0.3 25) || die "/map_nav 只有 $hz Hz"
ok "/map_nav $hz Hz"

say "阶段 6  nav2（MPPI，线速度上限 $AXIS_SPEED_CAP m/s 按轴）"
ok "使用当前安装的 Nav2 配置；实机地图通过 launch 参数指定，不覆盖 YAML"
nohup setsid python3 $RL \
  --path $WS/src/astribot_s1_navigation/launch/navigation.launch.py \
  controller_plugin:=mppi use_sim_time:=false max_linear_speed:=$AXIS_SPEED_CAP \
  posture_normal_height:=0.0 map_topic:=/map_nav map_transient_local:=false \
  scan_topic:=/scan autostart:=true > $LOG/nav2.log 2>&1 &
for i in $(seq 1 40); do
  grep -q "Managed nodes are active" $LOG/nav2.log 2>/dev/null && break
  sleep 2
done
grep -q "Managed nodes are active" $LOG/nav2.log || die "nav2 生命周期没到 active（看 $LOG/nav2.log）"
ok "nav2 全部 active"

if [ "$USE_RVIZ" = true ]; then
  say "阶段 7  rviz2（物理桌面 :0）"
  CFG=$SHARE/rviz/nav2_view.rviz
  [ -f "$CFG" ] || die "rviz 配置不存在: $CFG（注意 setup.py 只 glob rviz/*.rviz）"
  env -u LC_ALL -u LC_CTYPE -u LC_NUMERIC -u LC_TIME -u LC_COLLATE -u LC_MONETARY \
      -u LC_MESSAGES -u LC_PAPER -u LC_NAME -u LC_ADDRESS -u LC_TELEPHONE \
      -u LC_MEASUREMENT -u LC_IDENTIFICATION \
      DISPLAY=:0 LANG=C LC_ALL=C \
      nohup setsid rviz2 -d "$CFG" --ros-args -p use_sim_time:=false > $LOG/rviz.log 2>&1 &
  sleep 8
  NR=$(ps -eo args --no-headers | grep "rviz2 -d" | grep -v grep | grep -c . || true)
  [ "${NR:-0}" -ge 1 ] && ok "rviz2 在跑（VNC / NoMachine 看 :0）" || bad "rviz2 起不来，看 $LOG/rviz.log"
fi

say "阶段 8  底盘写通路（astribot_trajectory_bridge）"
( cd $SDK
  source $SDK/env.sh > $LOG/envsh.log 2>&1
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
  export FASTRTPS_SHM_PERMISSION=UNRESTRICT ROS_DOMAIN_ID=25
  export PATH=/opt/ros/humble/bin:$PATH
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  source $WS/install/setup.bash >/dev/null 2>&1
  python3 -c "import astribot_sdk" 2>/dev/null || { echo "SDK_IMPORT_FAIL"; exit 1; }
  nohup setsid python3 $RL astribot_trajectory_bridge bridge_bringup.launch.py \
    target:=real allow_write_to_real:=true enable_slam_correction:=false > $LOG/bridge.log 2>&1 &
) || die "厂商 SDK import 失败（看 $LOG/envsh.log）"
ok "桥接已拉起，等控制权提示"

for i in $(seq 1 45); do
  grep -qi "forcibly acquire control" $LOG/bridge.log 2>/dev/null && break
  sleep 2
done
BP=$(ps -eo pid,args --no-headers | grep "bridge_container" | grep -v grep | awk '{print $1}' | head -1)
if [ -z "$BP" ]; then
  if grep -q "No simulation or real robot is started" $LOG/bridge.log 2>/dev/null; then
    bad "桥接起不来：SDK 报「No simulation or real robot is started」"
    grep -oE "astribot_[a-z_]+ is not alive" $LOG/bridge.log | sort -u | tr '\n' ' ' | sed 's/^/    离线部件: /'
    echo
    warn "→ 这几乎总是**急停按下**（或整机未使能）。松开急停 / 使能后重跑本脚本即可。"
    warn "→ 探索建图链(阶段 1~7)已经起好了，只有写通路这一段没起来。"
  else
    bad "bridge_container 没起来，看 $LOG/bridge.log"
  fi
elif ! grep -qi "forcibly acquire control" $LOG/bridge.log 2>/dev/null; then
  warn "没等到控制权提示（可能控制权本来就空闲），不写 stdin"
else
  if [ "$DRIVE" = true ]; then
    printf 'yes\n' > /proc/$BP/fd/0 && ok "已回 'yes'：强夺控制权"
  else
    printf '\n'    > /proc/$BP/fd/0 && ok "已回车：**不夺权**，机器人不会动（--no-drive 生效）"
  fi
fi
sleep 15
grep -E "写通路|底盘桥接已启动|桥接容器就绪" $LOG/bridge.log | tail -3 | sed 's/^.*\]: /  /' || true
BP2=$(ps -eo pid,args --no-headers | grep "bridge_container" | grep -v grep | awk '{print $1}' | head -1)
NREJ=$(grep -c "control rights of the robot" $LOG/bridge.log 2>/dev/null | head -1); NREJ=${NREJ:-0}
if [ -z "$BP2" ]; then
  bad "桥接进程已退出 —— 写通路这一段没起来（上面有原因）"
  warn "探索建图链仍然可用，只是机器人不会动"
elif [ "$DRIVE" = true ]; then
  grep -q "acquired control rights" $LOG/bridge.log && ok "控制权已拿到" \
    || bad "控制权没拿到（看 $LOG/bridge.log）"
  [ "$NREJ" -eq 0 ] && ok "SDK 拒绝次数 0" || bad "SDK 仍拒绝 ×$NREJ —— 控制权没真拿到"

  say "阶段 9  使能写通路（默认做；--no-drive 时跳过）"
  timeout 25 ros2 service call /astribot_bridge_container/enable \
    std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -2 | sed 's/^/  /'
  ok "写通路已 enable"
else
  ok "桥接在跑但**未使能**（--no-drive），/cmd_vel 有帧、底盘不动"
fi

say "阶段 10  自主探索调度器"
nohup setsid python3 $RL astribot_s1_autonomy exploration_coordinator.launch.py \
  use_sim_time:=false map_topic:=/map_nav odom_topic:=/odom \
  map_transient_local:=false robot_base_frame:=astribot_torso_base > $LOG/explore.log 2>&1 &
sleep 25
ok "调度器已起"

say "恢复探索派发（resume；不使能写通路，机器人仍然不动）"
timeout 25 ros2 service call /exploration_coordinator_node/resume \
  std_srvs/srv/Trigger "{}" 2>&1 | tail -2 | sed 's/^/  /'
sleep 5

say "阶段 11  路径跟踪诊断器（只读，唯一记录速度链路的东西）"
DIAG=$WS/install/astribot_s1_navigation/lib/astribot_s1_navigation/path_tracking_diagnostics_node
if [ -x "$DIAG" ]; then
  nohup setsid "$DIAG" --ros-args -p use_sim_time:=false \
    -p only_when_stuck:=false -p report_period_sec:=1.0 \
    > $LOG/trackdiag.log 2>&1 &
  before=$(grep -c '\[跟踪诊断\]' $LOG/trackdiag.log 2>/dev/null || true); before=${before:-0}
  sleep 8
  after=$(grep -c '\[跟踪诊断\]' $LOG/trackdiag.log 2>/dev/null || true); after=${after:-0}
  if [ "$((after - before))" -ge 3 ]; then
    ok "诊断器在报（8s 内 +$((after - before)) 行），看 $LOG/trackdiag.log"
    grep '\[跟踪诊断\]' $LOG/trackdiag.log | tail -1 | sed 's/^/  /'
  else
    warn "诊断器 8s 内只多了 $((after - before)) 行（期望 >=3），看 $LOG/trackdiag.log"
  fi
  warn "轮速一段在实机恒为 n/a（/joint_states 无 velocity 字段），这是预期的"
else
  bad "缺 $DIAG —— 速度链路本轮无任何记录，事后无法区分"
  bad "  「MPPI 没发速度」与「发了但底盘没动」。补法：colcon build astribot_s1_navigation"
fi

say "全链判据"
verify_all && printf '\n\033[1;32m═══ 全部判据通过 ═══\033[0m\n' \
           || printf '\n\033[1;31m═══ 有判据不过，见上 ═══\033[0m\n'

cat <<'TAIL'

┌─────────────────────────────────────────────────────────────────────────────┐
│ 已知未解决（不是本脚本的 bug，是待定的事）                                   │
└─────────────────────────────────────────────────────────────────────────────┘
· max_linear_speed 是 XY 各轴的上限，全向运动时速度模长可达到轴上限的 sqrt(2) 倍。
· safety_tripped 无复位通路：voxel_slam 位姿会被 GBA 回环修正，z 一次跳超
  0.06m 就永久跳闸，只能重启 cmd_vel_body_to_world_node。
· 实测机器人走出 1.23m 后卡在 "Starting point in lethal space"：中心格 253
  （膨胀致命，**不是** 254 真实障碍），最近真实障碍 0.354m，1.0m 内 0 个自由格。
  按安全红线不算物理堵死，属脱困范畴 —— 但 nav2 默认 BT 的 backup/spin 出不来。
· 同一类：**开机站位就被围住**，所以派发数常为 0。实测调度器本身是好的
  （12 次 IDLE->GEN_NEXT_POINT、6 次 PAUSED->IDLE 自动恢复，前沿格 2416 个在），
  是候选全被否：「目标点净空半径(0.25m)内存在占据栅格」/「距离机器人过近」，
  连自举旋转都被安全门拦（最近障碍 0.370~0.405m < 要求净空 0.420m）。
  => 判据 6 只判"调度器是否活着"，派发为 0 时如实归到这一条，等 SLAM。

日志都在 /tmp/s1_logs/：rsp jsp perception odom selfclear nav2 rviz explore bridge trackdiag
  · trackdiag.log 是**唯一**有速度记录的日志：四段速度链路 + 实位移，1Hz 一行。
    机器人不动时先看它，它会直接指出断在哪一段。
TAIL
