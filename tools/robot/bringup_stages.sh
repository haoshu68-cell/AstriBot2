#!/usr/bin/env bash
set -uo pipefail

WS=/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot
SDK_ROOT=/home/astribot/Downloads/astribot_sdk_aarch64   # `import astribot_sdk` 要它在 PYTHONPATH 上
SLAM_WS=/home/astribot/SLAM/vxlm-slam   # voxel_slam 独立工作区，不在 WS 下
GTSAM_LIB=/home/astribot/SLAM/ThirdParty/GTSAM/install4.1.0/lib
ENVF=/tmp/robot_env.sh
LOGDIR=/tmp/bringup
DISCOVER_SEC=40          # DDS 发现窗口，别调小
mkdir -p "$LOGDIR"

C_OK=$'\033[32m'; C_ER=$'\033[31m'; C_WA=$'\033[33m'; C_N=$'\033[0m'
ok()   { echo "${C_OK}[OK]${C_N} $*"; }
err()  { echo "${C_ER}[ERR]${C_N} $*"; }
warn() { echo "${C_WA}[WARN]${C_N} $*"; }
die()  { err "$*"; exit 1; }

build_env() {
    local p
    p=$(pgrep -f 'all_node.launch' | head -1)
    [ -n "$p" ] || die "厂商 all_node.launch 没在跑 —— 先启动本体驱动。"
    python3 - "$p" "$ENVF" <<'PYENV'
import os
import re
import shlex
import sys
from pathlib import Path
lines = []
for entry in Path('/proc', sys.argv[1], 'environ').read_bytes().split(b'\0'):
    key, sep, value = entry.partition(b'=')
    name = os.fsdecode(key)
    if sep and re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name) and name not in {'_', 'PWD', 'OLDPWD', 'SHLVL'}:
        lines.append('export ' + name + '=' + shlex.quote(os.fsdecode(value)))
path = Path(sys.argv[2])
with open(path, 'w', encoding='utf-8', errors='surrogateescape') as stream:
    os.fchmod(stream.fileno(), 0o600)
    stream.write('\n'.join(lines) + '\n')
PYENV
    grep -q 'ROS_DOMAIN_ID' "$ENVF" || die "导出的环境里没有 ROS_DOMAIN_ID"
    ok "环境取自 pid=$p（$(wc -l < "$ENVF") 个变量，domain=$(sed -n 's/^export ROS_DOMAIN_ID=//p' "$ENVF")）"
}

load_env() {
    set +u
    # shellcheck disable=SC1090
    source "$ENVF"
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash" 2>/dev/null || true
    source "$SLAM_WS/install/setup.bash" 2>/dev/null || true
    export LD_LIBRARY_PATH="$GTSAM_LIB:${LD_LIBRARY_PATH:-}"

    export PYTHONPATH="$SDK_ROOT:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$SDK_ROOT/astribot_sdk/core/common/robotics_library_py"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_ROOT/astribot_sdk/core/common/whole_body_control/third_party"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_ROOT/third_party/drake/lib"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_ROOT/third_party/third_pkg"
    export ROBOT_TYPE="${ROBOT_TYPE:-S1}"
    export ASTRIBOT_SDK_ROOT="${ASTRIBOT_SDK_ROOT:-$SDK_ROOT}"
    set -u
}

check_ldd() {
    local bin=$1 name=$2 missing
    load_env
    [ -x "$bin" ] || { err "$name 的可执行文件不存在：$bin"; return 1; }
    missing=$(ldd "$bin" 2>/dev/null | grep 'not found' | awk '{print $1}' | sort -u)
    if [ -n "$missing" ]; then
        err "$name 缺动态库，起了必死（exit 127）："
        printf '      %s\n' $missing
        err "  别去查它的配置或数据源 —— 先把库路径补齐。"
        return 1
    fi
    ok "$name 动态库齐全"
}

# 那种写法下 heredoc 与 `-` 抢 stdin，argv 传不进去，表现是**一行都不输出**、
# 退出码却是 0 —— 于是判据静默通过，比报错难查。
PROBE=/tmp/.bringup_probe.py
write_probe() {
    cat > "$PROBE" <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node

win = float(sys.argv[1])
topics = sys.argv[2:]
rclpy.init()
n = Node('bringup_probe')
t0 = time.time()
while time.time() - t0 < win:
    rclpy.spin_once(n, timeout_sec=0.1)
known = {a for a, _ in n.get_topic_names_and_types()}
bad = []
for t in topics:
    c = n.count_publishers(t) if t in known else 0
    print('    %-42s pub=%d' % (t, c))
    if c == 0:
        bad.append(t)
n.destroy_node()
rclpy.shutdown()
sys.exit(1 if bad else 0)
PY
}

count_pubs() {
    load_env
    [ -f "$PROBE" ] || write_probe
    python3 "$PROBE" "$DISCOVER_SEC" "$@"
}

#: 进程数（模式从文件读，命令行不含关键词 —— 否则 grep 命中自身）
proc_count() {
    local pat=$1 f=/tmp/.bringup_ps
    ps -eo pid,args > "$f"
    printf '%s\n' "$pat" > /tmp/.bringup_pat
    grep -c -f /tmp/.bringup_pat "$f"
}

wait_for_topics() {
    local label=$1; shift
    echo "  等判据（窗口 ${DISCOVER_SEC}s）：$*"
    if count_pubs "$@"; then
        ok "$label 判据通过"
        return 0
    fi
    err "$label 判据未通过 —— 上面 pub=0 的就是没起来的"
    return 1
}

#: 判据升级版：不仅要有发布者，还要**真的在发消息**。
#
# !!! 对一条处理链，count_publishers 是个几乎无用的判据 !!!
# 链上每个节点一启动就把自己的发布者建好了，所以 pub=1 恒成立。
# 实测过一次彻底的假成功：⑦b 判据 `/livox/fused_points pub=1` 通过，
# 而那一刻整条链
#   left/cloud_filtered 0Hz → fused_points 0Hz → cloud_self_filtered 0Hz → /scan 0Hz
# 全都是 pub=1、0 消息。真因是 QoS 不兼容（BEST_EFFORT 发布者 + RELIABLE 订阅者），
# rclpy 只打一条 WARNING，进程全都活得好好的。
#
# 所以凡是"链路中段"的话题，一律用本函数按**拍率**验。
wait_for_rate() {
    local label=$1 min_hz=$2 msgtype=$3; shift 3
    echo "  等拍率判据（>= ${min_hz}Hz）：$*"
    load_env
    python3 - "$min_hz" "$msgtype" "$@" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan, JointState
from nav_msgs.msg import Odometry
TYPES = {'PointCloud2': PointCloud2, 'LaserScan': LaserScan,
         'JointState': JointState, 'Odometry': Odometry}
min_hz = float(sys.argv[1])
if sys.argv[2] not in TYPES:
    # 不认识的类型名不能当成"没数据"处理 —— 那会把脚本自己的笔误
    # 报成"节点没起来"，把排查引到完全错的方向。
    print('    未知消息类型 %r，可选：%s' % (sys.argv[2], ', '.join(sorted(TYPES))))
    sys.exit(2)
ty = TYPES[sys.argv[2]]
topics = sys.argv[3:]
WINDOW = 12.0
rclpy.init()
n = Node('rate_probe')
# BEST_EFFORT 订阅者能收 RELIABLE 发布者，反之不行 —— 探针用 BEST_EFFORT
# 才不会自己造出"收不到"的假象。
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)
cnt = {t: 0 for t in topics}


def mk(t):
    def cb(_m):
        cnt[t] += 1
    return cb


for t in topics:
    n.create_subscription(ty, t, mk(t), qos)
t0 = time.time()
while time.time() - t0 < WINDOW:
    rclpy.spin_once(n, timeout_sec=0.05)
bad = False
for t in topics:
    hz = cnt[t] / WINDOW
    pubs = n.count_publishers(t)
    flag = '' if hz >= min_hz else '   <-- 未达标'
    print('    %-34s pub=%d  %.1f Hz%s' % (t, pubs, hz, flag))
    if hz < min_hz:
        bad = True
        if pubs > 0:
            print('      有发布者却收不到消息 —— 优先查 QoS 兼容性'
                  '（BEST_EFFORT 发布者 + RELIABLE 订阅者 = 一帧都收不到，'
                  '只会打一条 WARNING）。看该节点日志里的 "incompatible QoS"。')
n.destroy_node()
rclpy.shutdown()
sys.exit(1 if bad else 0)
PY
    if [ $? -eq 0 ]; then ok "$label 拍率判据通过"; return 0; fi
    err "$label 拍率判据未通过"
    return 1
}

launch_bg() {
    local log=$1; shift
    load_env
    nohup "$@" > "$LOGDIR/$log" 2>&1 &
    echo "  已后台启动 -> $LOGDIR/$log"
}

# ---------------------------------------------------------------- 阶段
stage1() {
    echo "──── ① voxel_slam ────"
    # !!! 模式必须匹配**可执行文件路径**，不能只写 "voxelslam" !!!
    # 有人在构建 voxel_slam 时，gmake / c++ / cc1plus 的命令行里都含这个词，
    # proc_count 会数出 4 个，于是误报"已在跑、跳过" —— 而实际一个都没起。
    [ "$(proc_count 'lib/voxel_slam/voxelslam')" -gt 0 ] && { warn "已在跑，跳过"; return 0; }
    wait_for_topics "雷达数据源" /livox/lidar_front /livox/imu_front || {
        err "雷达没数据，SLAM 起了也没用"; return 1; }
    check_ldd "$SLAM_WS/install/voxel_slam/lib/voxel_slam/voxelslam" "voxelslam" || return 1
    launch_bg R1_slam.log ros2 launch voxel_slam vxlm_mid360.launch.py
    sleep 20
    wait_for_topics "① SLAM" /map_scan /map_scan_filtered
}

stage2() {
    echo "──── ② 感知（cloud_to_grid）────"
    # !!! map_odom_tf **不在本阶段** !!!
    # 它要同时拿到 camera_init→aft_mapped（阶段①）和 odom→base（阶段④）
    # 才算得出 map→odom，而它有个 source_timeout_sec=60 的看门狗：
    # 60 秒内一次都没算出来就**主动退出**（刻意设计，避免 nav2 无声地等 TF）。
    # 早先把它放在这里，等阶段③④起来早就超过 60s 了，于是它每次都自杀，
    # 日志里写得很清楚（"缺失次数：SLAM 侧 0，里程计侧 297"）但主日志上
    # 只看到 map→odom 一直不存在。它没做错，是我把阶段顺序排错了。
    # 现在挪到阶段④拿到 /odom 之后启动。
    if [ "$(proc_count cloud_to_grid_node)" -eq 0 ]; then
        launch_bg R2_grid.log ros2 run astribot_s1_perception cloud_to_grid_node \
            --ros-args --params-file "$WS/install/astribot_s1_perception/share/astribot_s1_perception/config/cloud_to_grid_params.yaml"
    else warn "cloud_to_grid 已在跑"; fi
    sleep 20
    wait_for_topics "② 感知" /map
}

#: 回答厂商 SDK 的控制权提示。
#
# ════════════════ 这个提示会静默卡死整条链 ════════════════
# 用到 SDK 的节点（state_bridge / chassis_odom / bridge_container）在 init 里会问：
#   "Another user currently controls the robot. ... Enter 'yes' to forcibly
#    acquire control, or press 'Enter' to continue without control rights:"
# 它**阻塞在 stdin 上**，init 走不完，于是话题一帧都不发。
# 但话题的**发布者已经建好了** —— 所以 `pub=1` 判据照样通过。
# 实测代价：状态桥从 17:43 卡到 20:35（近 3 小时），/joint_states 恒 0Hz，
# 于是 RSP 收不到关节角、36 条连杆的动态 TF 全不存在、点云自滤"剔除 0 个点"、
# /scan 一帧不出 —— 而每一层看着都活得好好的。
#
# !!! 只发回车，绝不发 yes !!!
# 回车 = 继续但**不取控制权**（只读，这正是这几个节点该有的状态）。
# yes  = 强行夺取控制权，并会**立刻停止机器人当前运动**。
# 后者属于"使能/写通路"性质的动作，必须人工逐次确认，脚本不许代劳。
answer_sdk_prompt() {
    local pat=$1 label=$2 p
    p=$(ps -eo pid,args | grep -- "$pat" | grep -v grep | awk '{print $1}' | head -1)
    [ -n "$p" ] || return 0
    if printf '\n' > "/proc/$p/fd/0" 2>/dev/null; then
        echo "  已向 $label (pid=$p) 发回车：继续但**不取控制权**"
        sleep 8
    else
        warn "$label (pid=$p) 的 stdin 写不进去；若它卡在控制权提示上，"
        warn "  需要人工在机器人终端执行： printf '\\n' > /proc/$p/fd/0"
    fi
}

stage3() {
    echo "──── ③ 状态桥 + robot_state_publisher ────"
    [ "$(proc_count state_bridge_node)" -gt 0 ] && { warn "已在跑，跳过"; return 0; }
    local rsp=true
    [ "$(proc_count robot_state_publisher)" -gt 0 ] && {
        warn "已有 robot_state_publisher，本阶段不再起第二个（重复 RSP 会两份 /tf_static）"
        rsp=false; }
    launch_bg R3_state.log ros2 launch astribot_trajectory_bridge state_bridge.launch.py \
        "use_robot_state_publisher:=$rsp"
    sleep 30
    # 阶段⑥早就有这段应答逻辑，阶段③④漏了 —— 这就是那次三小时静默卡死的原因。
    answer_sdk_prompt 'lib/astribot_trajectory_bridge/state_bridge_node' '状态桥'
    # 判据按**拍率**验：pub=1 只说明发布者建好了，卡在 stdin 上时它也是 1。
    wait_for_rate "③ 状态桥" 10 JointState /joint_states
}

stage4() {
    echo "──── ④ 底盘里程计（只读）+ map_odom_tf ────"
    if [ "$(proc_count chassis_odom_node)" -gt 0 ]; then
        warn "chassis_odom 已在跑"
    else
        launch_bg R4_odom.log ros2 run astribot_trajectory_bridge chassis_odom_node
        sleep 30
    fi
    # SDK 会话可能弹控制权提示等 stdin；chassis_odom 是只读的，回车（不取控制权）即可。
    # 不应答的后果与阶段③一样：/odom 恒 0Hz 而发布者存在，判据看着过了。
    answer_sdk_prompt 'lib/astribot_trajectory_bridge/chassis_odom_node' '底盘里程计'
    wait_for_rate "④ 里程计" 10 Odometry /odom || return 1

    # map_odom_tf 必须**在 /odom 就绪之后**才启动（原因见阶段②的注释）。
    # 它同时依赖阶段① 的 camera_init→aft_mapped，所以这是最早的可行时机。
    if [ "$(proc_count map_odom_tf_node)" -eq 0 ]; then
        launch_bg R4_maptf.log ros2 run astribot_s1_perception map_odom_tf_node
        sleep 15
    else warn "map_odom_tf 已在跑"; fi
    # 判据是 map→odom 这条边真的出来了，不是进程活着。
    # /tf 一直有别的边在发，所以不能只看 /tf 的 pub 数 —— 那永远 >0。
    check_tf_edge map odom
}

#: 查一条 TF 边是否真的存在（不是查 /tf 有没有发布者）。
check_tf_edge() {
    local parent=$1 child=$2
    load_env
    echo "  等 TF 边：$parent → $child"
    python3 - "$parent" "$child" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from tf2_msgs.msg import TFMessage
parent, child = sys.argv[1], sys.argv[2]
rclpy.init(); n = Node('edge_probe')
seen = set()
def cb(msg):
    for t in msg.transforms:
        seen.add((t.header.frame_id, t.child_frame_id))
q = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST)
qs = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(TFMessage, '/tf', cb, q)
n.create_subscription(TFMessage, '/tf_static', cb, qs)
t0 = time.time()
while time.time() - t0 < 30.0:
    rclpy.spin_once(n, timeout_sec=0.1)
    if (parent, child) in seen:
        print('    %s -> %s  存在' % (parent, child)); rclpy.shutdown(); sys.exit(0)
print('    %s -> %s  **不存在**（这 30s 内共见到 %d 条边）' % (parent, child, len(seen)))
for p, c in sorted(seen):
    print('      %s -> %s' % (p, c))
rclpy.shutdown(); sys.exit(1)
PY
    if [ $? -eq 0 ]; then ok "TF 边 $parent→$child 判据通过"; return 0; fi
    err "TF 边 $parent→$child 不存在 —— 查 map_odom_tf 的日志，它会说是哪一侧缺"
    return 1
}

stage5() {
    echo "──── ⑤ nav2 ────"
    [ "$(proc_count nav2_controller)" -gt 0 ] && { warn "已在跑，跳过"; return 0; }
    # !!! use_sim_time:=false 必须显式传 !!!
    # navigation.launch.py 的 DeclareLaunchArgument 默认是 'true'，而
    # nav2_params_mppi.yaml 里也硬写了 11 处 use_sim_time: True。
    # 不传的后果：实机上六个 nav2 节点全部 use_sim_time=True，而 /clock
    # 发布者数为 0 —— 它们的时钟**恒为 0、永不前进**（实测）。
    # 症状极其误导：costmap 照常发布、进程都活着、判据也过，
    # 但所有带时间戳的 TF 查询与控制周期都建立在一个停住的时钟上。
    launch_bg R5_nav.log ros2 launch astribot_s1_navigation navigation.launch.py \
        use_sim_time:=false
    sleep 35
    wait_for_topics "⑤ nav2" /global_costmap/costmap /local_costmap/costmap || return 1

    # !!! costmap 有发布者**不等于**能定位 !!!
    # costmap 节点起来就会发，与 TF 能不能解无关。本项目已因此得到一次假成功：
    # 六阶段全 [OK]、打印"全部阶段完成"，而那一刻 map→odom 和 map→camera_init
    # 都不存在，nav2 根本无法定位。所以这里必须再验两件事。
    check_tf_edge map odom || return 1
    check_nav_clock
}

#: 查 nav2 的时钟是不是真的在走。use_sim_time=True 且无 /clock 时它恒为 0。
check_nav_clock() {
    load_env
    echo "  查 nav2 时钟"
    python3 - <<'PY'
import subprocess, sys
bad = []
nodes = ['/controller_server', '/planner_server', '/bt_navigator',
         '/local_costmap/local_costmap', '/global_costmap/global_costmap']
for n in nodes:
    try:
        r = subprocess.run(['ros2', 'param', 'get', n, 'use_sim_time'],
                           capture_output=True, text=True, timeout=25)
    except subprocess.TimeoutExpired:
        print('    %-34s 查询超时' % n); bad.append(n); continue
    txt = (r.stdout or '').strip()
    print('    %-34s %s' % (n, txt or (r.stderr or '').strip()[:50]))
    if 'True' in txt:
        bad.append(n)
if bad:
    print('  !! 以下节点 use_sim_time=True：%s' % ', '.join(bad))
    print('     实机没有 /clock 发布者，它们的时钟会恒为 0 且永不前进。')
    print('     启动时传 use_sim_time:=false。')
    sys.exit(1)
sys.exit(0)
PY
    if [ $? -eq 0 ]; then ok "nav2 时钟判据通过（use_sim_time=false）"; return 0; fi
    err "nav2 跑在停住的仿真时钟上 —— costmap 照发但定位与控制都不可信"
    return 1
}

stage6() {
    echo "──── ⑥ 控制桥接（写通路**关**）────"
    [ "$(proc_count bridge_container)" -gt 0 ] && { warn "已在跑，跳过"; return 0; }
    # !!! allow_write_to_real=false 是本阶段的安全前提，不要改成 true !!!
    # 要开写通路必须人工确认后单独重启，见脚本头注释第 4 点。
    launch_bg R6_bridge.log ros2 launch astribot_trajectory_bridge bridge_bringup.launch.py \
        target:=real allow_write_to_real:=false enable_slam_correction:=false
    sleep 40
    # !!! 这里改成只发回车，不再发 yes !!!
    # 那个提示的原文是：
    #   "Acquiring control will **immediately stop the robot's current motion**.
    #    Enter 'yes' to forcibly acquire control, or press 'Enter' to continue
    #    without control rights"
    # 发 yes = 夺取控制权 + 立刻停止机器人当前运动。这属于"使能/写通路"性质的
    # 动作，与本脚本头注释第 4 点"绝不碰运动、使能不在本脚本职责内"直接冲突。
    # 本阶段的目的只是把桥接**以拒绝写入的状态**拉起来，不需要控制权。
    # 若某天确实需要控制权，必须人工逐次确认后单独操作。
    answer_sdk_prompt 'lib/astribot_trajectory_bridge/bridge_container' '控制桥接'
    sleep 20
    if grep -q '写通路=被拒绝' "$LOGDIR/R6_bridge.log" 2>/dev/null; then
        ok "⑥ 控制桥接就绪，且写通路**被拒绝**（预期如此）"
    elif grep -q '写通路=允许' "$LOGDIR/R6_bridge.log" 2>/dev/null; then
        err "⑥ 写通路竟然是"允许" —— 与本脚本的安全前提不符，请立刻检查"
        return 1
    else
        err "⑥ 日志里找不到写通路结论，见 $LOGDIR/R6_bridge.log"
        return 1
    fi
}

stage7() {
    echo "──── ⑦ 动态避障链（CustomMsg 转换 + 感知链 + 自滤切片）────"
    # ════════════════ 这一阶段解决什么 ════════════════
    # nav2 两个 costmap 的 obstacle_layer **唯一**数据源是 /scan。
    # 在本阶段之前 /scan 发布者数为 0 —— 也就是完全没有动态避障，
    # 而 costmap 照常发布、判据照过，是个静默缺失。
    #
    # 链路（缺任何一环都是 /scan 一帧不出，且各进程看着都正常）：
    #   厂商驱动 CustomMsg
    #     → livox_custom_to_pc2_node        /livox/lidar_{front,back}_pc2
    #     → livox_preprocess ×2             /livox/{left,right}/cloud_filtered
    #     → livox_fusion_node               /livox/fused_points
    #     → pointcloud_slice_scan_node      /livox/cloud_self_filtered（含本体自滤）
    #     → pointcloud_to_laserscan         /scan
    if [ "$(proc_count livox_custom_to_pc2_node)" -eq 0 ]; then
        launch_bg R7_convert.log ros2 launch astribot_s1_autonomy \
            livox_custom_to_pc2.launch.py use_sim_time:=false
        sleep 12
    else warn "转换节点已在跑"; fi
    wait_for_rate "⑦a CustomMsg 转换" 8 PointCloud2 \
        /livox/lidar_front_pc2 /livox/lidar_back_pc2 || return 1

    if [ "$(proc_count livox_preprocess_node)" -eq 0 ]; then
        # 输入话题必须是转换后的 _pc2；实机没有 /livox/lidar_{left,right}。
        launch_bg R7_percep.log ros2 launch astribot_s1_perception \
            hardware_perception.launch.py use_sim_time:=false
        sleep 15
    else warn "感知链已在跑"; fi
    # 中段一律按拍率验：pub=1 在这条链上恒成立，验不出任何东西。
    wait_for_rate "⑦b 预处理" 5 PointCloud2 \
        /livox/left/cloud_filtered /livox/right/cloud_filtered || return 1
    wait_for_rate "⑦b 融合" 5 PointCloud2 /livox/fused_points || return 1

    if [ "$(proc_count pointcloud_slice_scan_node)" -eq 0 ]; then
        launch_bg R7_slice.log ros2 run astribot_s1_autonomy pointcloud_slice_scan_node \
            --ros-args --params-file \
            "$WS/install/astribot_s1_autonomy/share/astribot_s1_autonomy/config/pointcloud_slice_scan_params.yaml" \
            -p use_sim_time:=false
        sleep 15
    else warn "切片节点已在跑"; fi
    # 自滤输出是 pointcloud_to_laserscan 的输入，缺它 /scan 静默不出
    wait_for_rate "⑦c 自滤切片" 5 PointCloud2 /livox/cloud_self_filtered || return 1

    # 最终判据：/scan 真的在出帧。这是本阶段唯一算数的结论。
    wait_for_rate "⑦ 动态避障（/scan）" 5 LaserScan /scan
}

status() {
    echo "════════ 进程 ════════"
    for kv in "all_node.launch:厂商all_node" "livox_ros_driver2_node:厂商雷达" \
              "lib/voxel_slam/voxelslam:SLAM" "cloud_to_grid_node:点云转栅格" \
              "map_odom_tf_node:map_odom_TF" "state_bridge_node:状态桥" \
              "chassis_odom_node:底盘里程计" "bridge_container:控制桥接" \
              "nav2_controller:nav2" "robot_state_publisher:RSP" \
              "livox_custom_to_pc2_node:CustomMsg转换" \
              "livox_preprocess_node:点云预处理" "livox_fusion_node:双雷达融合" \
              "pointcloud_slice_scan_node:自滤切片" \
              "pointcloud_to_laserscan_node:投影2D"; do
        printf '  %-18s %s\n' "${kv#*:}" "$(proc_count "${kv%%:*}")"
    done
    echo "════════ 关键话题（pub>0 才算存在）════════"
    # /scan 单独列在最后：它是 nav2 obstacle_layer 的唯一数据源，
    # 为 0 就等于没有动态避障，而这一点不会有任何报错。
    count_pubs /livox/lidar_front /joint_states /odom /map /tf /tf_static \
               /global_costmap/costmap /cmd_vel \
               /livox/lidar_front_pc2 /livox/fused_points \
               /livox/cloud_self_filtered /scan || true
}

# ---------------------------------------------------------------- main
build_env
write_probe

if [ "${1:-}" = "--status" ]; then status; exit 0; fi

STAGES=("$@")
# 默认跑到⑦。阶段⑦是动态避障链 —— 少了它 /scan 恒为 0 发布者，
# nav2 有 costmap 但没有任何障碍数据，而这一点不会有任何报错。
[ ${#STAGES[@]} -eq 0 ] && STAGES=(1 2 3 4 5 6 7)

for s in "${STAGES[@]}"; do
    if ! "stage$s"; then
        err "阶段 $s 失败，**停止**后续阶段（依赖没满足，硬往下走只会得出假结论）"
        exit 1
    fi
    echo
done

ok "全部阶段完成"
echo
status
echo
cat <<'EOF'
════════════════════════════════════════════════════════════
本脚本**没有**使能桥接，也**没有**发任何速度：
  · 写通路 = 被拒绝（WriteGate 主动拦）
  · 桥接 start_disabled
要使能或发速度，必须先与人确认朝向和距离，再单独操作。
════════════════════════════════════════════════════════════
EOF
