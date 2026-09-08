#!/usr/bin/env bash
# =============================================================================
# Astribot S1 实机自主探索建图 —— 一键启停【RPP 跟踪版】
#
#   启动 + 使能（默认，底盘会动）：  s1_hardware_bringup_rpp.sh start
#   启动但不使能（底盘不动）：       s1_hardware_bringup_rpp.sh start --no-drive
#   停止：            s1_hardware_bringup_rpp.sh stop
#   看状态：          s1_hardware_bringup_rpp.sh status
#   只跑判据：        s1_hardware_bringup_rpp.sh verify
#
#   附加开关：--no-rviz          不起 rviz2
#             --keep-nav2-yaml   不从快照复原 nav2 参数（调参时用）
#
# 口径：局部规划器 **RPP**（Regulated Pure Pursuit），线速度上限见 AXIS_SPEED_CAP
# （当前 0.5 m/s，按轴），use_sim_time=false，ROS_DOMAIN_ID=25。
#
# ┌───────────────────────────────────────────────────────────────────────────┐
# │ 与 s1_hardware_bringup.sh（MPPI 版）的关系                                │
# └───────────────────────────────────────────────────────────────────────────┘
# 本文件是那份脚本的**派生副本**，派生时源 md5 = 9c7c1b698a9173f9d71231097f2b9157。
# 只改了 5 处，其余逐字一致：
#   ① SELF_TAG（否则 stop 会把自己杀掉）
#   ② 阶段 6 的参数快照：nav2_params_rpp_hw.yaml -> nav2_params_rpp.yaml
#   ③ 阶段 6 的 controller_plugin:=rpp
#   ④ 判据 5/7 的键名：MPPI 的 vx_max/vy_max -> RPP 的 desired_linear_vel
#      （RPP **没有** v?_max 这两个键，照抄会撞上"一个都没枚举到"的守卫而必然失败）
#   ⑤ 本段说明 + 一条派生源漂移告警
# MPPI 那份脚本**未做任何修改**。改动共同部分时两份都要改 —— start 时会比对
# 派生源 md5 并在漂移时告警（只告警不中止：漂移原因可能与本文件无关）。
#
# ⚠️ 已知分叉，未替用户决定（2026-09-08 发现）：实机 $TOOLS 下那份 MPPI 脚本
#    比仓库/git HEAD **新 1h16m**，且差异是功能性的 ——
#    "/home/astribot/SLAM/vxlm-slam/" 被从 VENDOR **移到了 OURS**，
#    即那份脚本的 stop 会**杀掉厂商 SLAM（地图随之丢失）**。
#    而它自己阶段 0 的注释仍写着"stop **不动**它们…SLAM 一杀地图就没了"，
#    两处互相矛盾，无从判断哪一侧是当时的本意。
#    本文件按 git HEAD 的语义走：vxlm-slam 留在 VENDOR，**stop 不杀厂商 SLAM**。
#    理由是两条里只有这一条与文件内仍在的设计说明自洽，且另一条的失败模式是
#    破坏性的（地图没了不可逆）。要改成杀 SLAM，把那一行搬到 OURS 即可，
#    但请同时改掉阶段 0 那段注释，别再留一份自相矛盾的文件。
#
# ┌───────────────────────────────────────────────────────────────────────────┐
# │ 切到 RPP 的行为差异（如实标注，不是遗漏）                                  │
# └───────────────────────────────────────────────────────────────────────────┘
#   · FollowPathExplore.approach_enabled: false —— 三段式的接近段限速在 rpp 路径上
#     是关掉的。2026-09-08 急停前那次"‖v‖ 被压到 0.05 而 wz 保持满权限、转弯半径
#     0.177m < 足迹内切 0.310m"的机制在这条路径上不存在。
#   · 但 RPP 自己的原地旋转**更快**：rotate_to_heading_angular_vel 1.0 rad/s，
#     高于三段式的 align_max_vel 0.6，触发阈值 rotate_to_heading_min_angle 0.785(45°)。
#     角速度上限全链路无人压（launch 刻意不压、桥接默认 2.0）—— 这是已知的、
#     与刚诊断出的那个轴同向的回归风险，第一次跑必须盯着看。
#   · 失去 CurvatureSpeedLimitCritic 与 MPPI 的 consider_footprint: true，
#     窄通道行为与 MPPI 路径不同。
#   · 失去 vy 不是真变化：MPPI 那份的 motion_model 本来就是 DiffDrive 且 vy_max=0。
#
# ┌───────────────────────────────────────────────────────────────────────────┐
# │ 使能（默认开）做了什么                                                     │
# └───────────────────────────────────────────────────────────────────────────┘
# 默认（无 --no-drive）：向控制权提示回 'yes'。**这会立刻停止机器人当前的运动**
#   （厂商 SDK 的行为，不是我们的），然后 enable 桥接 + resume 探索，机器人开始走。
#   跑之前必须确认：周围安全、没有别人在操作、急停可及。
# 带 --no-drive：底盘桥接**进程照起**（它是非厂商插件，要一并起来），但
#   ① 控制权提示只回车，不夺权 ② 不调 ~/enable。桥接本身"启动即停用"，
#   于是 /cmd_vel 有帧、底盘不动。这一档可以随便跑。
#
# 不夺权的后果实测过：内环 214.8Hz 照跑、每拍都调 set_joints_position，而 SDK
# 全部拒掉 —— 日志 1627 条 "You don't have control rights of the robot."，
# /cmd_vel 非零帧 205/1351、|v| 峰值 0.2085m/s，而机器人净位移 0.0003m（纯噪声）。
# 也就是说"指令对"完全不等于"机器人动了"：判据必须压在 /odom 位移上。
#
# 每一阶段都带判据，判据不过就停在那一阶段并打印实测数字 ——
# 不做"起完就算成功"。历史上这条链的每一环都出过"进程活着但一帧不流"的假成功：
#   · QoS 不兼容(BEST_EFFORT 发 / RELIABLE 收)：一帧不到，只有一行 WARN
#   · use_sim_time=true 而实机无 /clock：节点时钟恒 0，costmap 照发
#   · 静态层 map_topic 指错：全局图恒 100x100 全未知，只有一行 "no map received"
#   · 判据数的是日志里的**历史累计值**：数字一轮一轮逐字相同却报 ✔（真实事故）
# 所以判据一律压在**拍率 / 格数 / 实测位移 / 计数增量**上，不看进程存活。
# =============================================================================
set -e   # 刻意不用 -u：source ROS 的 setup.bash 在 set -u 下会静默退出，一个字都不打印

# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------
SDK=/home/astribot/Downloads/astribot_sdk_aarch64
WS=$SDK/ws_robot
TOOLS=/home/astribot/s1_tools
LOG=/tmp/s1_logs
SHARE=$WS/install/astribot_s1_navigation/share/astribot_s1_navigation
SELF_TAG=s1_hardware_bringup_rpp  # 用来把自己从"要杀的进程"里排除掉，见 ours_pids
                                  # ⚠ 必须跟着文件名改：沿用 MPPI 那份的 tag 时
                                  #   本脚本的命令行虽然含该子串（能自排除），但反过来
                                  #   MPPI 版的 stop 会误把本脚本也当成自己而漏杀。

# ---------------------------------------------------------------------------
# 环境：全部集中在这里，别处不再 export
# ---------------------------------------------------------------------------
export ROS_DOMAIN_ID=25          # 实机是 25，不是 42；查询侧不带就是"节点全都 Node not found"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 厂商用「多播路由 + iptables」做双网卡隔离，不是 interfaceWhiteList。
# 别再叠一层白名单：叠了会把雷达那张 192.168.0.x 网卡切掉。
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
export FASTRTPS_SHM_PERMISSION=UNRESTRICT
# ROS_LOCALHOST_ONLY 必须不设：雷达在 192.168.0.11 那张网卡上，设 1 就收不到点云。
# （仿真侧结论相反 —— 那边必须是 1，别把两边搬混。）
unset ROS_LOCALHOST_ONLY
export PATH=/opt/ros/humble/bin:$PATH   # 厂商裁剪版 ros2 只有 7 个子命令，必须压下去

# ---- 统一日志时间戳 -------------------------------------------------------
# 问题：同一次运行的日志里并存五种时间戳，跨模块对齐一个事件要靠人脑换算。
#   ① ros2 launch 自己的行  [INFO] [launch]: ...            —— 压根没有时间戳
#   ② 我们的 ROS 节点       [INFO] [1788840911.748911701] [x]: —— rcutils 默认，epoch 纳秒
#   ③ 厂商 SDK (Python)     [2026-09-08 09:53:47.960][astribot_client.py:44] [INFO]
#   ④ 厂商 SDK (C++)        [2026-09-08 09:53:48.330023][info][18507][ast_communicator.cpp:121]
#   ⑤ Voxel-SLAM            2026-09-08 09:53:19.488 [info] [LIDAR] ...
# 能动的只有 ②（③④⑤ 是厂商自己的 logger，不走 rcutils；① 走 launch.logging）。
# 于是把 ② 对齐到 ③④⑤ 的形状 —— 这是让最多的行互相可比的唯一改法。
#
# 代价：丢掉 epoch 纳秒。可接受 —— ms 分辨率足以区分 250Hz 的相邻拍（周期 4ms），
# 而真要纳秒对齐消息头 stamp 时，把下面这行注释掉即可回到默认。
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{date_time_with_ms}] [{severity}] [{name}]: {message}'

# ---- 线速度上限：全脚本唯一一处 -------------------------------------------
# 这个数**同时**是 nav2 的 max_linear_speed 和判据 5/7 的越限阈值。原来它以字面量
# 出现在 9 处（launch 参数 1 处 + 两个判据的 Python 里 3 处 + 说明文字 5 处）——
# 那种形状下"调速度"必然只改到一部分：改了 launch 没改判据，机器人合法地跑 0.5，
# 判据却仍按 0.2 判，于是报 "✘ 越过授权上限" 并让整轮验证失败。
# 判据和被判对象共用同一个变量，才不可能漂开。
#
# 口径未变（用户 2026-09-03 决定）：**按轴**独立上限，不是合速度模长上限。
# 但**不要**据此写"模长上界 = sqrt(2)*cap"：RPP 是为非全向载体设计的控制器，
# 结构上只输出 (vx, wz)，横向分量恒为 0，所以模长上界就等于 cap 本身。
# （在 MPPI 那份脚本里这个结论同样成立，但理由不同 —— 那边靠 vy_max=0.0 +
#   motion_model: DiffDrive；rpp yaml 里根本没有 vy_max 这个键，launch 侧那条
#   'vy_max':'0.0' 改写是无害 no-op。结论一致、依据不同，别混着引用。）
# （脚本尾部「已知未解决」里记着上一版这里的算术是错的，别再犯一次。）
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
RESTORE_YAML=true
for a in "$@"; do
  case "$a" in
    --no-rviz)         USE_RVIZ=false ;;
    --no-drive)        DRIVE=false ;;
    --drive)           DRIVE=true ;;   # 兼容旧写法，现在是默认值
    --keep-nav2-yaml)  RESTORE_YAML=false ;;
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

# =============================================================================
# 「哪些进程是我们的」—— stop 的全部依据
# =============================================================================
# 判别按**可执行文件路径**，不按节点名。实测整棵进程树（86 个 ROS 进程）后确认
# 这两组路径完全不重叠，比匹配节点名可靠得多：节点名会被 remap，而且厂商和我们
# 都有叫 robot_state_publisher / static_transform_publisher 的进程。
#
# 顺序很重要：**厂商护栏先判且一票否决**。这样即使我们的某条模式意外命中了厂商的
# 命令行，厂商进程也不会被杀。
#
# ⚠️ 三个已经踩过的坑，下面逐条防住了：
#   ① 绝不用 pkill -f。命令行里含有那个模式时它会连自己的 shell 一起杀掉，
#      表现是循环从中间静默断掉、日志文件不存在，被误读成"启动失败"。
#   ② 本脚本自己就装在 $TOOLS 下，而 $TOOLS 是"我们的"判据之一 —— 不排除自己
#      就是自杀。所以按 PID + 脚本名双重排除自己、父进程和 python 子进程。
#   ③ pgrep -x / ps comm 只有 15 字符，长可执行名永不匹配；一律走 ps + args 全串。
# ---------------------------------------------------------------------------
ours_pids() {   # 打印 "<pid> <cmdline>"，每行一个，只含我们的进程
  ps -eo pid,args --no-headers > /tmp/s1_ps_snap.txt
  SELF_PID=$$ SELF_PPID=$PPID SELF_TAG=$SELF_TAG python3 - <<'PY'
import os, re

VENDOR = (                       # 一票否决，先判
    "/opt/astribot_ros/",                 # 厂商整套栈
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
    "/home/astribot/SLAM/vxlm-slam/",     # 厂商 SLAM(voxelslam / nav_prob_grid_node)    
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

# =============================================================================
# stop —— 按 PID 停掉所有非厂商插件
# =============================================================================
do_stop() {
  say "停止所有非厂商插件（厂商 SLAM / 雷达驱动 / 远程桌面不动）"

  # ── 先让底盘停止接收指令，再杀进程 ────────────────────────────────────
  # 顺序反过来的后果：桥接被 KILL 时内环可能刚下发完一帧位置指令，而 SDK 的
  # control_way='filter' 还会继续收敛一段（停车距离分两段，积分器那段只占一半）。
  # 先 disable 让它主动归零再杀，停得干净。
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

    # 第 1 轮 SIGINT：ros2 launch 收到 INT 会**逐个优雅关闭子节点**。
    # 直接 KILL 父进程反而留下一堆孤儿节点 —— 孤儿 costmap / bridge 会重连、
    # 时间回跳清空 TF buffer，表现成"planner 反复 abort 而各模块看着都正常"。
    printf '%s\n' "$snap" | awk '{print $1}' | while read -r p; do
      [ -n "$p" ] && kill -INT "$p" 2>/dev/null || true
    done
    sleep 5

    # 第 2 / 3 轮：重新扫（不复用第 1 轮的快照，PID 可能已经没了），
    # 逐级升信号，每一级都打印实际发出去的 PID。
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

  # ── 残留判据：必须真的是 0，不能"发完信号就算停了" ──────────────────
  local left nleft
  left=$(ours_pids || true)
  nleft=$(printf '%s' "$left" | grep -c . || true)
  if [ "${nleft:-0}" -gt 0 ]; then
    bad "还有 $nleft 个残留，请手工处理："
    printf '%s\n' "$left" | cut -c1-140 | sed 's/^/    /'
  else
    ok "残留 0 个"
  fi

  # ── 共享内存 ──────────────────────────────────────────────────────────
  # 陈旧 shm + 陈旧 daemon 会伪装成"栈没起来"（实测话题数 2 vs 80）。
  # ⚠️ 清理模式必须同时含 sem.fastrtps_* 前缀：只删 fastrtps_* 时实测还剩 73 个。
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

# =============================================================================
# 判据集合（verify 与 start 共用同一份，避免"探针与真跑不同判据"）
# =============================================================================
# 历史事故：两轮外部探针白扫，因为探针的判据比真跑少一项 —— 少的那一项上
# 系统性假阳性。所以这里只有一份 verify_all，start 结尾直接调它。
# ---------------------------------------------------------------------------
verify_all() {
  local fail=0 hz

  say "判据 1/7  厂商 SLAM 在出栅格"
  hz=$(wait_hz /map_scan_filtered_prob nav_msgs/OccupancyGrid 0.3 20) \
    && ok "/map_scan_filtered_prob $hz Hz" \
    || { bad "/map_scan_filtered_prob 只有 $hz Hz（厂商标称 1.0）"; fail=1; }

  say "判据 2/7  /scan 干净（自滤没滤错体积）"
  # 阈值 = 足迹**外接**半径 0.42m，不是 0.6m。
  # 这一条判的是"自滤有没有把机器人自己漏进来"，而机器人自己只可能落在足迹以内 ——
  # 0.42m 以外的回波按定义不可能是自体回波，只能是真障碍。
  # 原来写 0.6m 的后果实测过：机器人停在窄处时真墙就在 0.447~0.479m（同一时刻
  # /map_nav 在 0.447m 处确有占据格），于是 111 束"越界"，判据把**真墙**报成自滤失效。
  # 判据超出自己声称要测的几何 = 在那一项上系统性假阳性。
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
  # 半径必须等于节点的 clear_radius_m(0.25)，而且必须是**圆**不是方框。
  # 老版本两处都错，叠起来虚报 12 格：
  #   ① 循环 ±ceil(0.45/res) 的方框却不做距离判定 —— 数的是边长 0.9m 的正方形，
  #      四角伸到 0.45*sqrt(2)=0.636m，远超足迹。
  #   ② 0.45 > 节点承诺清理的 0.25 —— 0.25~0.45 那圈**按设计**永远清不掉，
  #      于是只要附近有真墙判据就必败。
  # 按真实口径重测同一时刻：0~0.25m 占据 0 格，0.25~0.45m 只有 0.447m 处 1 格（真墙）。
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

  say "判据 5/7  nav2 活着且吃到地图（RPP / ${AXIS_SPEED_CAP}m/s 按轴 / 真墙钟）"
  local n_active n_nomap vx
  # ⚠️ 不能写 `grep -c ... || echo 0`：grep -c 没命中时**既打印 0 又返回 1**，
  #    于是变量拿到 "0\n0"，后面的 [ -eq ] 直接 "integer expression expected"，
  #    判据看起来像失败而其实是 0 次。
  n_active=$(grep -c "Managed nodes are active" $LOG/nav2.log 2>/dev/null | head -1); n_active=${n_active:-0}
  n_nomap=$(grep -c "no map received" $LOG/nav2.log 2>/dev/null | head -1); n_nomap=${n_nomap:-0}
  [ "$n_active" -ge 1 ] && ok "生命周期已 active" || { bad "nav2 未 active"; fail=1; }
  [ "$n_nomap" -eq 0 ] && ok "静态层无 'no map received'" \
    || { bad "静态层报 no map received ×$n_nomap（map_topic 指错了）"; fail=1; }
  grep -oE "Received a [0-9]+ X [0-9]+ map" $LOG/nav2.log 2>/dev/null | tail -1 \
    | sed 's/^/  全局图: /' || true
  # 时钟必须真的在走：实机无 /clock 发布者，use_sim_time=true 会让六个节点时钟
  # 恒为 0 永不前进，而 costmap 照发、上面几条判据全过。
  # 走 rcl_interfaces 服务读，**不用 `ros2 param get`**：实测同一条命令在 bash 里能读到
  # vx_max，放进 python subprocess 却回空字符串（判据于是印出 "?" 等于没验）。
  # 除了读参数还要**验时钟真的在走**：参数对但 /clock 有别的发布者时仍可能冻住。
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
  # 口径已定（用户 2026-09-03 决定）：AXIS_SPEED_CAP 是**各轴独立**上限，不是模长上限。
  #   => vx_max/vy_max 各 <=cap 就是正确配置。注意路线A 把 vy_max 钉死为 0.0，
  #      实测峰值 0.2085（1351 帧里 178 帧模长 >0.2）不是越限，不再报 warn。
  #   注意别和 [[越界判据必须用旋转不变的模长]] 搞混：那条讲的是"判哪一层在开车"，
  #   要用模长；这里讲的是"限速口径"，按轴。两者不冲突，各自的边界不同。
  # 按轴口径 => 必须**两轴都查**，只查 vx_max 会漏掉 vy_max 被调高。
  #
  # ⚠️ RPP 路径的键名跟 MPPI 完全不同：**没有** vx_max/vy_max，线速度上限叫
  #    desired_linear_vel（launch 侧 param_substitutions 两个键都写，各在对应
  #    yaml 上生效、在另一份上是无害 no-op）。照抄 MPPI 版会枚举到 0 个键，
  #    直接撞上下面"一个都没枚举到就算失败"的守卫 —— 那条守卫是对的，
  #    是键名该换。
  # ⚠️ 不要硬编码 `FollowPath.desired_linear_vel`：FollowPath 是三段式控制器时
  #    参数下移到 `FollowPath.inner.desired_linear_vel`，老键根本不存在。
  #    一律**枚举** controller_server 里所有以 .desired_linear_vel 结尾的参数。
  local cap_fail=0
  # RPP_YAML 必须显式传：heredoc 是 <<'PY'（不做 shell 展开），
  # 且判据要拿 yaml 回落值来判断自己是否退化。
  RPP_YAML=$SHARE/config/nav2_params_rpp.yaml \
  python3 - <<'PY' || cap_fail=1
import rclpy
import os
from rcl_interfaces.srv import ListParameters, GetParameters
CAP = float(os.environ['AXIS_SPEED_CAP'])   # 无默认值：见脚本顶部 AXIS_SPEED_CAP
# 这条判据能抓到"外层 launch 把 max_linear_speed 静默吞了"，靠的是漏传时
# desired_linear_vel 会回落到 **yaml 里的值**，而那个值 > CAP。
# ⚠️ 所以判据的分辨力取决于 yaml 回落值：CAP >= 回落值时它退化成恒真，
#    再也分不清"限速生效"和"参数根本没传下去"。
#    rpp 那份 yaml 的 desired_linear_vel 是 **0.5**，而用户 2026-09-08 定的
#    CAP 也是 0.5 —— 即当前配置下这一条**确实是退化的**，如实印出来，
#    不假装它还在验"参数传下去了"。它仍然是有效的**安全上界**判据。
#    回落值直接从磁盘上的 yaml 读，不硬编码，免得两处漂开。
import re
YAML = os.environ['RPP_YAML']
try:
    txt = open(YAML, encoding='utf-8').read()
    fb = max(float(m) for m in re.findall(r'^\s*desired_linear_vel:\s*([0-9.]+)',
                                          txt, re.M))
except Exception as exc:
    print('  ✘ 读不到 %s 的 desired_linear_vel 回落值（%s）——'
          '无法判断本判据是否退化' % (YAML, exc))
    raise SystemExit(1)
print('  yaml 回落值 desired_linear_vel = %.3f，授权上限 CAP = %.3f' % (fb, CAP))
if CAP >= fb:
    print('  ⚠ 本判据在 CAP(%.2f) >= yaml 回落值(%.2f) 时退化成恒真：'
          '它只验"不超上界"，**验不出 max_linear_speed 是否被静默吞掉**。'
          % (CAP, fb))
    print('    要验参数真传下去，只能看运行时实测幅值（桥接日志的指令均速）。')
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
              if k.endswith('.desired_linear_vel'))
if not keys:
    print('  ✘ 一个 desired_linear_vel 参数都没枚举到（判据自身失效，不能算通过）')
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
    print('  ✘ %s=%.3f 超过授权的线速度上限 %.2f' % (k, val, CAP))
print('  共查 %d 个线速度上限，越限 %d 个' % (len(keys), len(over)))
raise SystemExit(1 if over else 0)
PY
  if [ $cap_fail -eq 0 ]; then
    ok "RPP desired_linear_vel ≤ $AXIS_SPEED_CAP m/s（RPP 不输出 vy，故模长上界同为 $AXIS_SPEED_CAP）"
  else
    fail=1
  fi

  say "判据 6/7  探索调度器：红线不触发 + 目标**正在**派发（增量判据）"
  local n_block
  n_block=$(grep -c "物理真堵" $LOG/explore.log 2>/dev/null | head -1); n_block=${n_block:-0}
  [ "$n_block" -eq 0 ] && ok "红线未触发（0 次物理真堵）" \
    || { bad "红线触发 ×$n_block —— 机器人所在格又被判占据了，先查 /map_nav"; fail=1; }
  # ⚠️ 这一条曾经是假过的：老版本 grep 日志里的**历史累计**派发次数，
  #    于是连续两轮都报 "已派发 ×12" ✔，而调度器其实卡在
  #    "自动恢复已达上限 3 次，停止重试，等待人工调用 ~/resume"，每 3.5s 复读一次。
  #    冻结的读数被当成当前值 —— 修法只有一个：读数自带龄期，且看**增量**。
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
    # ⚠️ 判据必须压在 /odom 位移上，不能只比 /cmd_vel。
    #    实测过：/cmd_vel 非零 205/1351 帧、|v| 峰值 0.2085m/s 全都对，
    #    而 SDK 因为没有控制权把每一拍都拒掉，机器人净位移 0.0003m。
    #    "指令对"和"机器人动了"是两件事，e2e 只验前者等于只验证了自己。
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
# 「已知未解决」里已经记了：RPP 结构上只输出 (vx, wz)，横向分量恒为 0，
# 所以模长上界就等于按轴上限本身。
# ⚠️ 但 vy≡0 在 rpp 路径上是**控制器结构**保证的，不是限速层保证的：
#    rpp yaml 里没有 vy_max 这个键，launch 侧的 'vy_max':'0.0' 是 no-op。
#    所以下面这个越限判据仍然两轴都查 —— 若哪天真出现非零 vy，
#    那说明发速度的不是 RPP，而这正是需要被抓到的事。
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

# =============================================================================
# start
# =============================================================================
[ "$MODE" = "start" ] || die "未知模式 '$MODE'（可用 start / stop / status / verify）"

if [ "$DRIVE" = true ]; then
  printf '\n\033[1;33m┌──────────────────────────────────────────────────────────────┐\033[0m\n'
  printf '\033[1;33m│ 默认使能：会强夺控制权（**立刻停止机器人当前运动**）并驱动底盘 │\033[0m\n'
  printf '\033[1;33m│ 确认：周围安全 / 没有别人在操作 / 急停可及                    │\033[0m\n'
  printf '\033[1;33m└──────────────────────────────────────────────────────────────┘\033[0m\n'
fi

# ── 派生源漂移告警 ──────────────────────────────────────────────────────────
# 本脚本是 s1_hardware_bringup.sh 的派生副本（只差文件头列的 5 处）。
# 共同部分若只改了 MPPI 那份，这里的 RPP 版就是**陈旧**的，而症状会是
# "同一台机器上两个跟踪器行为差异莫名其妙" —— 很难反推到脚本没同步。
# 比的是**归一化** md5（剥掉注释行与空行）：实机那份与仓库那份差一行注释的
# 位置，用裸 md5 会必然误报。只告警不中止 —— 漂移原因可能与本文件无关，
# 而中止会挡掉一次正常启动。
DERIVED_FROM_NORM_MD5=88ba2948708e2ba258f8da2f24394486   # = 实机 2026-09-08 14:31 那份
_mppi_sh=$TOOLS/s1_hardware_bringup.sh
if [ -f "$_mppi_sh" ]; then
  _now=$(grep -vE '^[[:space:]]*#' "$_mppi_sh" | grep -vE '^[[:space:]]*$' \
         | md5sum | cut -d' ' -f1)
  if [ "$_now" != "$DERIVED_FROM_NORM_MD5" ]; then
    warn "派生源 s1_hardware_bringup.sh 已改动（归一化 md5 $_now != $DERIVED_FROM_NORM_MD5）"
    warn "  本 RPP 版可能缺了那边的修改，逐条核对后更新 DERIVED_FROM_NORM_MD5"
  fi
else
  warn "找不到 $_mppi_sh，跳过派生源漂移检查"
fi

do_stop     # 干净起点：把上一轮我们的进程全清掉（含重复进程）
            # 必须在阶段 0 **之前**：阶段 0 会拉起厂商 SLAM，而 do_stop 若排在它后面，
            # 就会在下一拍把刚拉起的 launcher 一起带走。

say "阶段 0  感知底座：雷达驱动 + 厂商 SLAM（没起就起，已起不动）"
# 用户 2026-09-04 要求把这两个也纳入脚本。它们仍归类为**厂商侧**：
#   => start 会起它们，stop **不动**它们。这个不对称是刻意的 ——
#      SLAM 一杀地图就没了，而 stop 的用途是重启我们这一侧的插件。
#
# 磁盘实名（与口头命令有出入，按磁盘为准，两处都实证过）：
#   · 包名是 voxel_slam（下划线）—— package.xml <name> 与 ament_index 双证，不是 voxel-slam
#   · livox launch 实名 msg_MID360_launch.py（下划线）—— 不是 msg_MID360.launch.py
#   · 树上有**两份** livox_ros_driver2，必须用厂商 /opt 那份（见 LIVOX_PREFIX）；
#     vxlm-slam 自带那份的配置里本机 IP 写着 192.168.1.5，而这台机器是 192.168.0.11
#     => 驱动起来就 "bind failed / Init lds lidar fail!"。这不是端口占用
#        （实测 ss -ulnp 查 5610x/5620x/5630x 全空），是本机 IP 在这台机上不存在。
#   · "无需 source" 只在交互 shell 成立(~/.bashrc 自动 source 了 /opt/ros/humble 与
#     厂商 robot_env.sh)；非交互 ssh 里连 `ros2` 都不在 PATH，且 ros2launch 包没装
#     (`ros2 launch --help` 实测 NO) —— 所以只能走 $RL 这个自带 launcher。
SLAM_WS=/home/astribot/SLAM/vxlm-slam
# 厂商 livox 前缀：配置里本机 IP=192.168.0.11、两台雷达 192.168.0.12(front)/.13(back)，
# 实测 ping 双通、驱动起来直接出 /livox/lidar_front 与 /livox/lidar_back。
LIVOX_PREFIX=/opt/astribot_ros/software/livox_ros_driver2
[ -f "$SLAM_WS/install/setup.bash" ] || die "找不到 $SLAM_WS/install/setup.bash"

# voxelslam 链接了 GTSAM，而 GTSAM 装在 ThirdParty 下、**既不在 ldconfig 里也不在
# vxlm-slam 的 setup.bash 里** —— 所以非交互 shell 里必然缺库：
#   voxelslam: error while loading shared libraries: libmetis-gtsam.so
# 实测加上这一条后 ldd 的 not-found 从 10 条降到 0，二进制能起来。
GTSAM_LIB=/home/astribot/SLAM/ThirdParty/GTSAM/install4.1.0/lib
[ -d "$GTSAM_LIB" ] || warn "$GTSAM_LIB 不存在 —— voxelslam 可能缺 libmetis-gtsam.so"

# 起在子 shell 里：SLAM 工作区的 setup.bash 会改 AMENT_PREFIX_PATH，
# 不能污染我们自己的环境（我们的包和它同名包一旦串了很难查）。
launch_vendor() {   # launch_vendor <日志名> <包> <launch 文件> <存活判据模式>
  local tag=$1 pkg=$2 lf=$3 pat=$4
  if [ "$tag" = livox ]; then
    # 跑着的可能是 vxlm-slam 那份**配置 IP 写错**的构建（192.168.1.5，本机是 .11）：
    # 它 bind 失败也不退出，于是"已在跑"恒为真而一帧不出。按可执行文件路径认出来就换掉。
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
                        # （实测踩过：日志 09:55、脚本 10:19 在读，见记忆"陈旧读数"）
  ( if [ "$tag" = livox ]; then
      # 只叠厂商 livox 前缀，**不** source vxlm-slam —— 否则那份错 IP 的同名包
      # 会按 AMENT_PREFIX_PATH 顺序抢先命中，症状就是 bind failed。
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
  # CustomMsg 的 python 支持要**两条**路径：PYTHONPATH 找到 _custom_msg.py，
  # LD_LIBRARY_PATH 找到 liblivox_ros_driver2__rosidl_generator_py.so（只给前者会崩在
  # UnsupportedTypeSupport，看着像雷达没数据，其实是探针自己起不来 —— 实测踩过）。
  PYTHONPATH=$LIVOX_PREFIX/local/lib/python3.10/dist-packages:${PYTHONPATH:-} \
  LD_LIBRARY_PATH=$LIVOX_PREFIX/lib:${LD_LIBRARY_PATH:-} \
    wait_hz /livox/lidar_front livox_ros_driver2/CustomMsg 1.0 "$1"
}

# 雷达判据只能是**拍率**，"进程在跑"至少有两种假阳性，两种都实测到过：
#   1) 配置里本机 IP 写错(192.168.1.5) -> bind failed 但进程不退出
#   2) 被 SIGTERM 打成半死 -> 进程在、pub=1、一帧不出（实测 PID 16127 活了 12 分钟 0 Hz）
# 所以验不过就**换掉重起一次**，而不是直接 die —— 只重起一次，避免无限循环。
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
# 拍率才是判据：进程在≠话题在流（实测过 pub=1 但 0Hz）。SLAM 冷启动要收敛，给足 60s。
hz=$(wait_hz /map_scan_filtered_prob nav_msgs/OccupancyGrid 0.3 60) \
  || die "/map_scan_filtered_prob 只有 $hz Hz —— SLAM 没在出栅格（看 $LOG/slam.log）"
ok "/map_scan_filtered_prob $hz Hz"

say "阶段 1  URDF -> robot_state_publisher + 静态 TF"
XACRO=$WS/install/astribot_s1_description/share/astribot_s1_description/urdf/astribot_s1.xacro
[ -f "$XACRO" ] || XACRO=$WS/src/astribot_s1_description/urdf/astribot_s1.xacro
# xacro 缺任一替换参数就是 Undefined substitution argument
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
# TF 树： map -> camera_init -> aft_mapped(SLAM 动态) -> astribot_torso_base -> 各 link
#         map -> odom 恒等别名，让 local_costmap 的 global_frame: odom 可解
# ⚠️ 根 frame 是 astribot_torso_base，**没有 base_link**。
# ⚠️ odom 走别名而不是 REP-105 分解：这一版不引入 SDK 里程计，
#    代价是回环跳变会直接打到 local_costmap 上。已知取舍，不是遗漏。
nohup setsid $ST --frame-id map --child-frame-id camera_init \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 > $LOG/tf_map_ci.log 2>&1 &
nohup setsid $ST --frame-id camera_init --child-frame-id odom \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 > $LOG/tf_ci_odom.log 2>&1 &
ok "RSP + 2 个静态 TF 已起"

say "阶段 2  固定姿态 /joint_states"
# 主线只要"连杆 TF 存在"，探索期间手臂不动，所以不接 SDK 关节状态 ——
# 绕开整条 astribot_msgs / middleware / 控制器存活的依赖。
# 代价：姿态若与实机实际不符，自滤会滤错体积 -> /scan 冒近距假障碍。
# 所以判据 2 必须看 /scan 的 <0.6m 束数。
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
# aft_mapped -> astribot_torso_base 由 hardware_perception 内部发。
# 这一层里有 CustomMsg -> PointCloud2 的 C++ 转换节点：Python 反序列化两万点
# 跟不上 10Hz，会静默丢 60~80% 的帧，同一系统能测出 6%/41%/100% 三个配对率。
nohup setsid python3 $RL astribot_s1_perception hardware_perception.launch.py \
  use_sim_time:=false > $LOG/perception.log 2>&1 &
hz=$(wait_hz /scan sensor_msgs/LaserScan 5 45) \
  || die "/scan 只有 $hz Hz（雷达/自滤链断了，看 $LOG/perception.log）"
ok "/scan $hz Hz"

say "阶段 4  /odom（从 TF 反推）"
# 不用 astribot_trajectory_bridge 的 chassis_odom_node：那个跑在 SDK 会话里，
# 而 SDK init 会停在"控制权"交互提示上，/odom 一帧都不发。
# 探索链里 /odom 只用于停稳判断与 BT 速度反馈，差分 SLAM 位姿足够。
nohup setsid python3 $TOOLS/tf_to_odom_node.py --ros-args -p use_sim_time:=false > $LOG/odom.log 2>&1 &
hz=$(wait_hz /odom nav_msgs/Odometry 15 25) || die "/odom 只有 $hz Hz"
ok "/odom $hz Hz"

say "阶段 5  清机器人自身假障碍 -> /map_nav"
# 厂商 nav_prob_grid_node 用原始关键帧点云合成栅格、**无任何自滤**，
# 机器人躯干回波(实测 5 点，水平 0.12~0.20m / z 0.51~0.55m)会把自己所在格投成占据：
#   -> /map_scan_filtered_prob 该格 100，探索调度器红线"物理真堵"当场触发、无限 PAUSED
#   -> 静态层继承后该格原始代价 254，规划器必报 Starting point in lethal space
# 且它跟着机器人走(hit_delta=6/miss_delta=1，每帧在脚下重投)，不是开机残留。
# nav2 自带的 obstacle_layer.footprint_clearing_enabled 实测已是 True 但救不了：
# ObstacleLayer::updateCosts 用 updateWithMax 合并，静态层的 254 恒胜。
nohup setsid python3 $TOOLS/grid_self_clear_node.py --ros-args \
  -p use_sim_time:=false \
  -p input_topic:=/map_scan_filtered_prob -p output_topic:=/map_nav \
  -p map_frame:=map -p base_frame:=astribot_torso_base \
  -p clear_radius_m:=0.25 -p trail_window_sec:=20.0 > $LOG/selfclear.log 2>&1 &
hz=$(wait_hz /map_nav nav_msgs/OccupancyGrid 0.3 25) || die "/map_nav 只有 $hz Hz"
ok "/map_nav $hz Hz"

say "阶段 6  nav2（**RPP**，线速度上限 $AXIS_SPEED_CAP m/s 按轴）"
# 从快照复原实机参数。快照 $TOOLS/nav2_params_rpp_hw.yaml 相对仓库版本含 5 处
# 实机差异：planner tolerance=0.5、velocity_smoother 上下限 ±0.5、静态层
# map_topic=/map_nav、transient_local=False，外加 11 处 use_sim_time=False。
# ⚠️ 这台机器上 $SHARE/config 这条路径 install->build->src **三层都是符号链接**
#    （已逐层实证；MPPI 那份脚本此处的注释写着"是真实拷贝"，那句话在这台机器上
#    是错的）。后果有两条，都要知道：
#      · 改 yaml **不需要** colcon build —— 写进去就是运行时读到的；
#      · 这个 cp 会**覆盖仓库工作区里的** config/nav2_params_rpp.yaml。
if [ "$RESTORE_YAML" = true ]; then
  if [ -f $TOOLS/nav2_params_rpp_hw.yaml ]; then
    cp $TOOLS/nav2_params_rpp_hw.yaml $SHARE/config/nav2_params_rpp.yaml
    ok "已从快照复原实机 nav2 参数（rpp）"
  else
    bad "缺 $TOOLS/nav2_params_rpp_hw.yaml —— 静态层可能指向厂商原始栅格"
  fi
else
  warn "--keep-nav2-yaml：不复原，用 install 里当前的 nav2_params_rpp.yaml"
fi
# 行为树里 FollowPath 的 controller_id 写死是 FollowPathExplore
# （behavior_trees/navigate_to_pose_explore_three_phase.xml），
# 所以 yaml 的 controller_plugins 必须含这个实例名，否则每个 follow_path 目标
# 都会失败。实机上曾有一份 2026-09-02 的陈旧 rpp yaml 只有 ["FollowPath"]。
grep -q 'FollowPathExplore' $SHARE/config/nav2_params_rpp.yaml \
  || die "$SHARE/config/nav2_params_rpp.yaml 里没有 FollowPathExplore —— 行为树会每个目标都失败"
# use_sim_time:=false —— 实机无 /clock 发布者，true 会让六个节点时钟恒 0 且永不前进，
#                        而 costmap 照发、判据照过（nav2 的默认值是 true，最容易踩）
# posture_normal_height:=0.0 —— 姿态监控**保持开启**，只把基准高度换成实机这个
#   /odom 源的正确值。0.134 是 Gazebo 的基准（那里 world z=0 不是地面），而实机
#   /odom 由 tf_to_odom_node 从 SLAM 的 map->astribot_torso_base 导出，z=0 是开机
#   位姿。给 0.134 的后果实测过：第一帧 |-0.0011-0.1340|=0.1351 > 0.06 -> 止损 ->
#   /cmd_vel 30s 内 1668 帧全零，而日志只说"请检查 Gazebo 画面"，nav2 侧表现为
#   Failed to make progress，看起来像局部规划器不行。
#   ⚠️ 这不是"把安全件关掉"：给对基准后监控是真的在判定的 —— 实测 30s 内
#      z 峰峰值 0.0034m、roll/pitch < 0.006rad，离 0.06m/0.12rad 还有 17 倍余量。
#      要关它得显式 enable_posture_monitor:=false，本脚本不这么做。
#   ⚠️ 残留风险：safety_tripped 目前**没有复位通路**，而 voxel_slam 的位姿会被
#      GBA 回环修正。z 一次跳超 0.06m 就永久跳闸，只能重启该节点。未修。
# ⚠️ 走 --path 直指 src 下的 launch 文件，不经 nav2_full_bringup：
#    launch_arguments 是白名单，那一层曾同时漏掉 max_linear_speed 与
#    enable_posture_monitor，漏项不报错、子 launch 静默用自己的默认值。
nohup setsid python3 $RL \
  --path $WS/src/astribot_s1_navigation/launch/navigation.launch.py \
  controller_plugin:=rpp use_sim_time:=false max_linear_speed:=$AXIS_SPEED_CAP \
  posture_normal_height:=0.0 \
  scan_topic:=/scan autostart:=true > $LOG/nav2.log 2>&1 &
for i in $(seq 1 40); do
  grep -q "Managed nodes are active" $LOG/nav2.log 2>/dev/null && break
  sleep 2
done
grep -q "Managed nodes are active" $LOG/nav2.log || die "nav2 生命周期没到 active（看 $LOG/nav2.log）"
ok "nav2 全部 active"

if [ "$USE_RVIZ" = true ]; then
  say "阶段 7  rviz2（物理桌面 :0）"
  # ⚠️ 三个坑：① 必须直接 GLX，ssh -X 转发的 display 建不出 GL 窗口
  #              （Ogre 重试 100 次后 core dump）；实机只能走 NoMachine/VNC 连 :0
  #            ② ssh 会透传本机 zh_CN 的 LC_*，实机没生成该 locale，
  #               rviz2 会在 DISPLAY/OpenGL 自检**全部通过之后**才崩在 std::locale
  #            ③ setup.py 只 glob rviz/*.rviz，.rviz 放 config/ 下永不安装
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
# ┌─ 这一段是整条链唯一"能让机器人动"的地方，四个坑逐条防住 ─────────────┐
# ① SDK import：必须 source $SDK/env.sh，它提供 PYTHONPATH / LD_LIBRARY_PATH /
#    ROBOT_TYPE / ASTRIBOT_SDK_ROOT。少 ASTRIBOT_SDK_ROOT 时报的是
#    "TypeError: str expected, not NoneType"，**报错里不含变量名**。
# ② env.sh 会**覆盖** FASTRTPS_DEFAULT_PROFILES_FILE：它只在本机 192.168.0.x
#    地址恰好等于 192.168.0.10 时才跳过覆盖，而这台机器人是 .11，所以它会生成一份
#    interfaceWhiteList profile 顶掉厂商的。→ source 之后必须**重新钉回**厂商 profile。
#    （旧结论"不能 source env.sh"过宽：source 是必须的，只是要补钉一下。）
# ③ SDK 多处用 cwd 相对路径 → 必须 cd $SDK 再起。
# ④ 控制权提示会阻塞在 stdin：不回就一直卡着（实测卡了 3 小时），话题 pub=1 但一帧不发。
#    回车 = 不夺权（内环照跑但 SDK 每拍都拒，1627 条 ERROR，机器人不动）；
#    'yes' = 强夺，**立刻停止机器人当前运动**。由 --drive 决定回哪个。
# └───────────────────────────────────────────────────────────────────────┘
( cd $SDK
  source $SDK/env.sh > $LOG/envsh.log 2>&1
  # 坑 ② 的补钉，顺序必须在 source 之后
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
  # 最常见的原因是**急停按下**：SDK 看不到任何在线部件，报
  #   "[ERROR] astribot_chassis is not alive"（七个部件各一条）
  #   "[ERROR] No simulation or real robot is started."
  # 然后 bridge_container exit 1。这不是脚本的问题 —— 急停生效时写通路本来就该起不来。
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
# ⚠️ 重新取一次 PID，不复用上面那个：桥接可能在这 15s 里退出了。
# 老版本在这里无条件打 "桥接在跑但未使能 ✔"，而桥接其实已经 exit 1 —— 假成功。
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
# ⚠️ 这一阶段**必须排在写通路使能之后**，2026-09-04 实机吃过这个亏。
#    老顺序是「调度器(8) -> 桥接(9) -> 使能(10)」，于是调度器在写通路活起来之前
#    就开始派发目标：实测 nav2 六节点 643.4s active、调度器随即开跑，而内环直到
#    **789.3s** 才真正有帧落地 —— 中间约 130s 里 nav2 一直在对一台聋了的底盘发速度。
#    机器人不动 -> 控制器判"原地转不动"抛超时 -> Controller patience exceeded ->
#    Aborting handle -> 上游连败 -> PAUSED -> 自动恢复预算(3 次)烧光 -> 永久 parked。
#    末尾那次 resume 只能救回一次，救不回"预算在链路还没成型时就被烧掉"这件事。
#    新顺序让调度器的**第一次**派发就落在一条通的链上。
# map_topic 必须是 /map_nav，与 nav2 静态层同源 ——
# 换数据源时判据口径也要跟着换，历史上在这上面栽过（0.42 重复计足迹 / 0.0 导航超时）。
# map_transient_local:=false —— /map_nav 是 VOLATILE，
# TRANSIENT_LOCAL 订阅 VOLATILE 发布是不兼容的：一帧都收不到，只有一行 QoS WARN。
nohup setsid python3 $RL astribot_s1_autonomy exploration_coordinator.launch.py \
  use_sim_time:=false map_topic:=/map_nav odom_topic:=/odom \
  map_transient_local:=false robot_base_frame:=astribot_torso_base > $LOG/explore.log 2>&1 &
sleep 25
ok "调度器已起"

# resume 与 enable 必须**分开**：
#   enable = 底盘写通路，会让机器人真动 —— 默认做，--no-drive 时跳过。
#   resume = 让调度器继续派发目标，只影响"有没有目标在跑"，不碰底盘。
# 两者原来绑在同一个 --drive 分支里，于是不带 --drive 时调度器永远停在
# "自动恢复已达上限"，判据 6（派发增量）必然是 0 —— 那是脚本的分层错误，不是链路问题。
# 不带 --drive 时 resume 是安全的：写通路停用，nav2 照样规划、/cmd_vel 照发，底盘不动。
say "恢复探索派发（resume；不使能写通路，机器人仍然不动）"
timeout 25 ros2 service call /exploration_coordinator_node/resume \
  std_srvs/srv/Trigger "{}" 2>&1 | tail -2 | sed 's/^/  /'
sleep 5

say "阶段 11  路径跟踪诊断器（只读，唯一记录速度链路的东西）"
# 为什么必须起它：整条速度链上**没有任何一处**把速度落盘。实测扫过一遍 ——
#   · 每拍会打的只有 three_phase_controller.cpp:603 的"接近段限速 ‖v‖ x -> y"，
#     而它只在 D=1.50m 接近段内、且限速真的咬住时才打（上一轮共 24 条）。
#   · 底盘桥接与 cmd_vel_body_to_world_node 一个速度都不打。
# 于是"控制器根本没发速度"和"发了但底盘没动"在日志上长得一模一样，
# 上一轮就是卡在这个区分上。这个节点把四段速度 + 实位移一行打全，正好补这个缺口。
#
# 只订阅、不发布任何指令 —— 起它不会改变被诊断系统的行为，所以放在使能之后也安全。
# 走绝对路径而不是 `ros2 run`：阶段 8 在子 shell 里 source 过 env.sh，
# 别依赖 PATH 里此刻是哪个 ros2。这条路径本身命中 stop 的 OURS 判据，会被一并清掉。
DIAG=$WS/install/astribot_s1_navigation/lib/astribot_s1_navigation/path_tracking_diagnostics_node
if [ -x "$DIAG" ]; then
  # only_when_stuck=false：全量打。正常跑时也要有速度记录，否则事后只有故障期
  # 的数字、没有基线可比 —— 上一轮复盘就缺这个基线。
  nohup setsid "$DIAG" --ros-args -p use_sim_time:=false \
    -p only_when_stuck:=false -p report_period_sec:=1.0 \
    > $LOG/trackdiag.log 2>&1 &
  # 判据压在**报告行增量**上，不看进程存活：这个节点即使一个话题都收不到也照样活着，
  # 而"活着但没在报"与"没起来"的处置完全不同。
  # ⚠️ 计数一律走 `|| true` + 默认值，不用 `|| echo 0`：grep -c 在"文件存在但零命中"
  #    时会**既打印 0 又返回 1**，那种写法会让变量变成两行 "0\n0"，$(( )) 直接报错。
  before=$(grep -c '\[跟踪诊断\]' $LOG/trackdiag.log 2>/dev/null || true); before=${before:-0}
  sleep 8
  after=$(grep -c '\[跟踪诊断\]' $LOG/trackdiag.log 2>/dev/null || true); after=${after:-0}
  if [ "$((after - before))" -ge 3 ]; then
    ok "诊断器在报（8s 内 +$((after - before)) 行），看 $LOG/trackdiag.log"
    grep '\[跟踪诊断\]' $LOG/trackdiag.log | tail -1 | sed 's/^/  /'
  else
    # 不 die：诊断器是观测手段，它不出数不影响探索链本身。
    warn "诊断器 8s 内只多了 $((after - before)) 行（期望 >=3），看 $LOG/trackdiag.log"
  fi
  # 轮速这一段在实机上没有数据源：阶段 2 的 /joint_states 是固定姿态，
  # velocity 数组为空。节点会把它报成 "n/a" 而不是 0.000 —— 别把 n/a 读成"轮子没转"。
  warn "轮速一段在实机恒为 n/a（/joint_states 无 velocity 字段），这是预期的"
else
  bad "缺 $DIAG —— 速度链路本轮无任何记录，事后无法区分"
  bad "  「RPP 没发速度」与「发了但底盘没动」。补法：colcon build astribot_s1_navigation"
fi

say "全链判据"
verify_all && printf '\n\033[1;32m═══ 全部判据通过 ═══\033[0m\n' \
           || printf '\n\033[1;31m═══ 有判据不过，见上 ═══\033[0m\n'

cat <<'TAIL'

┌─────────────────────────────────────────────────────────────────────────────┐
│ 已知未解决（不是本脚本的 bug，是待定的事）                                   │
└─────────────────────────────────────────────────────────────────────────────┘
· 按轴口径的模长上界：**上一版的算术是错的**，写的是"各轴独立、模长上界
  sqrt(cap²+cap²)"。RPP 结构上只输出 (vx, wz)，横向分量恒为 0，
  所以模长上界就等于 cap 本身。MPPI 那条路径上（0.2 档）实测峰值 0.2085、
  1351 帧里 178 帧 > 0.2：在只有 vx 的前提下这些帧是**真的越了上限**，未查。
  rpp 路径上还没有对应实测。
· 判据 5/7 在 CAP=0.5 时是**退化的**：rpp yaml 的 desired_linear_vel 回落值
  也是 0.5，所以它验不出 max_linear_speed 被外层 launch 静默吞掉的情形，
  只能验"不超上界"。判据自己会把这一点印出来。要真验参数传下去，得看
  桥接日志的指令均速（阶段 11 的诊断器会打）。
· velocity_smoother 两份快照不一致：本脚本用的 rpp 快照是 ±0.5（用户
  2026-09-08 决定），而 nav2_params_mppi_hw.yaml 仍是 ±0.2。切回 MPPI 那份
  脚本时第二层限速会静默降到 0.2 —— 已如实报告给用户，等其决定是否对齐。
· RPP 的原地旋转比三段式更快（rotate_to_heading_angular_vel 1.0 rad/s vs
  align_max_vel 0.6），而**全链路没有任何一层压角速度**。2026-09-08 急停前
  那次异常正是"线速度被压死、角速度满权限"，方向同源。第一次跑 rpp 必须盯着
  桥接日志的 dθ 指令看，别只看线速度。
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
