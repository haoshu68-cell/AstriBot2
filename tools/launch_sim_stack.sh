#!/usr/bin/env bash
# 一键起仿真全链路，并**分层验证**它真的活了。
#
# 用法：
#   tools/launch_sim_stack.sh --mode explore
#   tools/launch_sim_stack.sh --mode localize --map maps/demo_warehouse
#   tools/launch_sim_stack.sh --mode explore --tracker rpp --max-linear-speed 0.6
#   tools/launch_sim_stack.sh --clean-only
#
# ════════ 这个脚本刻意**不**做的三件事，以及为什么 ════════
#
# 1) 不自己实现清理。清理一律调 tools/clean_sim_stack.sh。
#    那个脚本里积累的判据（comm 截断 15 字符让 pgrep -x 永不匹配、
#    sem.fastrtps_* 前缀、shm 必须在验证段之后再清一遍、宽口径残留自检抓
#    NAMES 表自己的遗漏）一条都不能少，重写一份等于把那些坑重踩一遍。
#
# 2) 不用「进程数」或「发布者数」当"起来了"的判据。
#    实测过 9 个进程全活而整套 nav2 从未 activate；也实测过 pub=1 而 0Hz
#    （BEST_EFFORT 发布者 + RELIABLE 订阅者，一帧都收不到、只有一条 WARNING）。
#    判据逐级往上走：Gazebo 步进 → /clock 实收帧 → TF → 生命周期 active
#    → 每条话题实测拍率 → 跟踪器参数回读 → 一次真实导航到位。
#    前一层不过就不查下一层（省时间，也避免把上游故障读成下游缺陷）。
#
# 3) 不假装能配置任意跟踪器参数。
#    FollowPath.align_kp 这类键只能从 nav2_params_*.yaml 给，而那份 yaml 的
#    路径是 navigation.launch.py 内部算出来的（见该文件 71-74 行），
#    没有任何 launch 入口能覆盖它。所以本脚本只暴露**现有**入口：
#      --tracker            -> controller_plugin，决定用哪份 yaml、内层是 MPPI 还是 RPP
#      --max-linear-speed   -> 经 RewrittenYaml 压住 vx_max/vx_min/desired_linear_vel
#      --scan-source        -> 同时切 costmap 数据源与感知节点
#    其余 FollowPath.* 只**回读展示**，不覆盖。
#    ⚠️ 为什么不用 `ros2 param set` 补上这个能力：ThreePhaseController 用
#    declare_parameter_if_not_declared 在 configure() 里一次性读进成员变量，
#    没有 on_set_parameters 回调（three_phase_controller.cpp:101-126）。
#    param set 之后 param get 会如实返回新值而**行为不变** ——
#    那是一条能自我验证通过的假验证，比没有这个功能更坏。
#
# ════════ 「存在就杀、不存在就起」是怎么落地的 ════════
# 无论进程列表里有没有东西，都跑一遍 clean_sim_stack.sh，并把它报的
# 「命中进程 N 个」原样打出来 —— N=0 就是"本来不存在"。
# 之所以不先自己数一遍再决定要不要清：陈旧的 fastrtps 共享内存段和陈旧的
# ros2 daemon **不是进程**，进程数为 0 时它们照样在，而它们会让下一轮
# `ros2 topic list` 只看到 2 个话题（实测 2 vs 80），表现为"栈没起来"。
# 清理器同时管这两样，让它无条件跑一遍才是干净的起点。
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---- 默认值 ----------------------------------------------------------------
MODE=""                 # explore | localize，必填
MAP=""                  # --mode localize 必填，不带扩展名的基础文件名
TRACKER="mppi"          # rpp | mppi
MAXV="1.0"              # 必须与 nav2_params_*.yaml 的 vx_max 一致，见下面的校验
SCAN="slice_scan"       # slice_scan | laserscan
HEADLESS="false"
USE_RVIZ="true"
GOAL=""                 # localize 档的验证目标 "x,y"；留空则自动挑一个可达点
SKIP_NAV="false"
RETRIES=2               # 基础设施闸门不过时重启几次
GATE_SEC=200
NAV_TIMEOUT=120
LOGDIR=""
CLEAN_ONLY="false"
FORCE="false"

usage() {
  sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

选项：
  --mode explore|localize   必填。explore=建图+探索协调器自主选点；
                            localize=加载已存地图纯定位，不起探索协调器
  --map <基础文件名>         --mode localize 必填。不带扩展名，需存在
                            <名>.posegraph 与 <名>.data 两个文件。
                            **刻意不设默认值**：猜一张图会让"定位跑通了"变成
                            在一张不是你想验的图上跑通了
  --tracker rpp|mppi        默认 mppi
  --max-linear-speed <m/s>  默认 1.0
  --scan-source <src>       slice_scan(默认) | laserscan
  --goal x,y                localize 档的验证目标（map 系）。留空则自动挑一个
                            **先经 planner 证明可达**的点
  --skip-nav-goal           跳过第 6 层（真实导航到位）验证
  --headless                Gazebo 只起 server 不起 GUI
  --no-rviz                 不起 rviz
  --retries N               默认 2
  --gate-timeout SEC        单道基础设施闸门的上限，默认 200
                            （闸门A=Gazebo 步进；闸门B=感知/TF 链出数据）
  --nav-timeout SEC         单次导航目标的上限，默认 120
  --log-dir DIR             默认 /tmp/simstack/<时间戳>
  --clean-only              只清理，不启动
  --force                   跳过"这台机器上有别人的栈"守卫
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --mode)              MODE="${2:-}"; shift 2 ;;
    --map)               MAP="${2:-}"; shift 2 ;;
    --tracker)           TRACKER="${2:-}"; shift 2 ;;
    --max-linear-speed)  MAXV="${2:-}"; shift 2 ;;
    --scan-source)       SCAN="${2:-}"; shift 2 ;;
    --goal)              GOAL="${2:-}"; shift 2 ;;
    --skip-nav-goal)     SKIP_NAV="true"; shift ;;
    --headless)          HEADLESS="true"; shift ;;
    --no-rviz)           USE_RVIZ="false"; shift ;;
    --retries)           RETRIES="${2:-}"; shift 2 ;;
    --gate-timeout)      GATE_SEC="${2:-}"; shift 2 ;;
    --nav-timeout)       NAV_TIMEOUT="${2:-}"; shift 2 ;;
    --log-dir)           LOGDIR="${2:-}"; shift 2 ;;
    --clean-only)        CLEAN_ONLY="true"; shift ;;
    --force)             FORCE="true"; shift ;;
    -h|--help)           usage; exit 0 ;;
    *) echo "🔴 未知选项: $1"; echo; usage; exit 2 ;;
  esac
done

die() { echo "🔴 $*" >&2; exit 2; }

# ---- 入参校验。全部在启动**之前**做完 --------------------------------------
# 理由：这些错误里的每一个，放到启动之后才发现的表现都是"机器人不动"，
# 而那个表象有二十种成因。在这里挡掉，报错就指向真正的那一条。
if [ "$CLEAN_ONLY" != "true" ]; then
  case "$MODE" in
    explore|localize) ;;
    "") die "必须给 --mode explore|localize（用 --help 看说明）" ;;
    *) die "--mode 只能是 explore 或 localize，收到 '$MODE'" ;;
  esac
fi
case "$TRACKER" in
  rpp|mppi) ;;
  *) die "--tracker 只能是 rpp 或 mppi，收到 '$TRACKER'" ;;
esac
case "$SCAN" in
  slice_scan|laserscan) ;;
  *) die "--scan-source 只能是 slice_scan 或 laserscan，收到 '$SCAN'" ;;
esac

MAP_ABS=""
if [ "$MODE" = "localize" ]; then
  [ -n "$MAP" ] || die "--mode localize 必须显式给 --map <基础文件名>（不带扩展名）。
   仓库里现有的图：$(cd "$REPO" && ls maps/*.posegraph ws_robot/maps/*.posegraph 2>/dev/null \
      | sed 's/\.posegraph$//' | tr '\n' ' ')"
  case "$MAP" in
    /*) MAP_ABS="$MAP" ;;
    *)  MAP_ABS="$REPO/$MAP" ;;
  esac
  # slam_toolbox 的序列化地图是**两个**文件，缺一个它会启动后报错而不是启动失败，
  # 表象又是"机器人不动"。所以两个都在这里查。
  for ext in posegraph data; do
    [ -f "${MAP_ABS}.${ext}" ] || die "地图文件不存在: ${MAP_ABS}.${ext}
   （--map 要给**不带扩展名**的基础文件名）"
  done
fi

# --max-linear-speed 与 yaml 里 vx_max 的一致性。
# 不一致时**只告警不拦**：真要现场限速就是要给一个比 yaml 小的值。
# 拦的是反向：给了个比 yaml 大的值，那等于用一个"默认"静默抬高了限速。
YAML="$REPO/ws_robot/src/astribot_s1_navigation/config/nav2_params_${TRACKER}.yaml"
[ -f "$YAML" ] || die "找不到跟踪器参数文件: $YAML"

if [ -z "$LOGDIR" ]; then
  LOGDIR="/tmp/simstack/$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$LOGDIR" || die "建不出日志目录 $LOGDIR"
STACK_LOG="$LOGDIR/stack.log"

# ---- ROS 环境。set +u 是必须的：ROS 的 setup.bash 引用了未定义变量，
#      在 set -u 下会让整个脚本**一个字都不打印**地退出（本仓库踩过）。
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || die "source /opt/ros/humble/setup.bash 失败"
source "$REPO/ws_robot/install/setup.bash" >/dev/null 2>&1 \
  || die "source ws_robot/install/setup.bash 失败 —— 先 colcon build"
set -u

# domain 25 是被 warehouse_sim.launch.py 用 SetEnvironmentVariable 钉死的，
# 不是由我这个 shell 决定的。这里设成同一个值只是为了让**查询侧**对得上；
# 启动之后还会从活着的进程里反读一次做核对（见 assert_domain_matches）。
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1
export DISPLAY="${DISPLAY:-:1}"

echo "════════════════════════════════════════════════════════════"
echo " 仿真全链路启动"
echo "   模式      : ${MODE:-（只清理）}"
[ "$MODE" = "localize" ] && echo "   地图      : ${MAP_ABS}.{posegraph,data}"
echo "   跟踪器    : $TRACKER   线速度上限: $MAXV m/s   scan 源: $SCAN"
echo "   rviz      : $USE_RVIZ   headless: $HEADLESS   DISPLAY=$DISPLAY"
echo "   日志      : $LOGDIR"
echo "════════════════════════════════════════════════════════════"

# ════════════════════════════════════════════════════════════════════════════
# 第 0 步 · 清理前先确认这台机器上的栈是谁起的
#
# 为什么必须有这一步：实测过同机另一个会话在跑同一套栈（共用 domain 25、
# 共用同一份 ws_robot/install/、进程模式完全重叠）。它跑了自己的清理，把我
# 4 分钟前起的实验杀了，我读到的"卡在 6 行"其实是它被杀那一刻的状态 ——
# 据此写下的根因整个是错的。反向同样成立：我清一次就毁掉它的实验。
#
# 判据用「launch 进程的 stdout 指向哪个文件」，不用进程名 —— 两边进程名一样。
# 规则：指向 /tmp/simstack/** 的算本脚本起的（"存在就杀"针对的就是它）；
#       指向别处（别的脚本、某个终端的 tty）的算别人的，停下来问，除非 --force。
# ════════════════════════════════════════════════════════════════════════════
list_launch_pids() {
  # awk 自己的命令行里含有 'ros2 launch' 这个模式串，会匹配到自己，凭空多出一个
  # 已经不存在的 pid ——  `$0 !~ /awk/` 不是多余的。这与 pkill -f 命中自身同类。
  ps -eo pid,args --no-headers | awk '$0 ~ /ros2 launch/ && $0 !~ /awk/ {print $1}'
}

foreign=0
mine=0
while read -r pid; do
  [ -n "$pid" ] || continue
  log="$(readlink "/proc/$pid/fd/1" 2>/dev/null || echo '?')"
  start="$(ps -o lstart= -p "$pid" 2>/dev/null | sed 's/^ *//')"
  case "$log" in
    /tmp/simstack/*) mine=$((mine+1)); tag="（本脚本先前起的，会被清掉）" ;;
    *)               foreign=$((foreign+1)); tag="🔴 **不是本脚本起的**" ;;
  esac
  echo "  发现 ros2 launch pid=$pid  起于 $start"
  echo "     stdout -> $log   $tag"
done < <(list_launch_pids)
[ "$mine" -eq 0 ] && [ "$foreign" -eq 0 ] && echo "  没有 ros2 launch 在跑"

if [ "$foreign" -gt 0 ] && [ "$FORCE" != "true" ]; then
  cat >&2 <<EOF
🔴 这台机器上有 $foreign 个不是本脚本起的 ros2 launch。清理会把它们杀掉。
   如果那是别人（或你自己在另一个终端）正在跑的实验，杀掉它会同时毁掉那份数据，
   而且被杀那一刻的读数极容易被误读成稳定状态下的故障特征。
   确认无主之后再加 --force 重跑。
EOF
  exit 3
fi

# ════════════════════════════════════════════════════════════════════════════
# 第 1 步 · 清理（存在就杀；不存在时它也会清 shm 与陈旧 daemon）
# ════════════════════════════════════════════════════════════════════════════
run_cleanup() {   # $1=日志文件
  if bash "$REPO/tools/clean_sim_stack.sh" > "$1" 2>&1; then
    grep -E '命中进程|清理后残留|/clock 发布者数|残留 fastrtps|^结果' "$1" | sed 's/^/    /'
    return 0
  fi
  echo "  🔴 清理器判定**未清干净**，全文如下（在这个状态下跑测量数据不可信）:"
  sed 's/^/    /' "$1"
  return 1
}

echo
echo "──── 第 1 步 · 清理 ────"
run_cleanup "$LOGDIR/clean_pre.log" || exit 4

if [ "$CLEAN_ONLY" = "true" ]; then
  echo
  echo "✅ --clean-only：已清理完毕，不启动。日志 $LOGDIR/clean_pre.log"
  exit 0
fi

# ════════════════════════════════════════════════════════════════════════════
# 第 2 步 · 按顺序起：先 Gazebo（连带感知/SLAM/nav2），等它**真的在步进**，再起 rviz
#
# 为什么要拆成两段而不是让 nav2_full_bringup 顺手把 rviz 也起了：
# Gazebo 有一种加载期死锁（gz_ros2_control 在 Configure() 里同步等
# robot_description，主循环卡在插件里，物理一步不走）。死锁时进程全在、
# rviz 也在，看起来什么都对。先等步进判据过了再起 rviz，rviz 的存在就不会
# 参与掩盖这个故障；而且死锁重启时不用连带重开一个 GUI。
#
# ⚠️ headless 与这个死锁**没有稳定因果**：实测 headless:=false 一样卡死过
# （连续 2 次、停在同一行）。所以这里的对策是"检测 + 重启"，不是"换个开关"。
# ════════════════════════════════════════════════════════════════════════════
launch_args=(
  "env:=sim"
  "launch_gazebo:=true"
  "headless:=$HEADLESS"
  "controller_plugin:=$TRACKER"
  "max_linear_speed:=$MAXV"
  "scan_source:=$SCAN"
  "use_rviz:=false"          # rviz 由本脚本在步进判据通过之后单独起
)
if [ "$MODE" = "explore" ]; then
  launch_args+=("mode:=mapping" "exploration:=true")
else
  launch_args+=("mode:=localization" "exploration:=false" "map_file_name:=$MAP_ABS")
fi

echo
echo "──── 第 2 步 · 启动 Gazebo + 感知/SLAM + nav2 ────"
echo "  ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py ${launch_args[*]}"

STACK_PID=""
infra_ok="false"
fail_kind=""

# ---- 闸门不过时留现场。**先留证据再杀**，否则下一轮日志把现场覆盖掉。
save_deadlock_evidence() {   # $1=attempt
  cp "$STACK_LOG" "$LOGDIR/stack_deadlock_attempt${1}.log" 2>/dev/null || true
  {
    echo "== 加载期死锁现场（第 $(($1+1)) 次）=="
    echo "-- [ign gazebo-1] 日志行数（个位数 = 卡在插件加载）:"
    grep -c 'ign gazebo-1' "$STACK_LOG" 2>/dev/null || echo 0
    echo "-- 最后一条 ign gazebo 行:"
    grep 'ign gazebo-1' "$STACK_LOG" 2>/dev/null | tail -1 | cut -c1-200
    echo "-- ign topic -l 条数（正常 ~21）:"
    timeout 15 ign topic -l 2>/dev/null | wc -l
    echo "-- gazebo server 自己的 CPU（正常 ~196%）:"
    ps -eo pcpu,args --no-headers | awk '$0 !~ /awk/ && /gazebo server/ {print "   "$1"%"}'
  } > "$LOGDIR/deadlock_evidence_${1}.txt" 2>&1
}

save_chain_evidence() {      # $1=attempt
  cp "$STACK_LOG" "$LOGDIR/stack_chainfail_attempt${1}.log" 2>/dev/null || true
  {
    echo "== 感知/TF 链闸门不过的现场（第 $(($1+1)) 次）=="
    echo "-- 嫌疑进程的**累计 CPU vs 存活秒数**（这是这类故障的特征读数）:"
    echo "   实测过的坏样本: robot_state_publisher 1200s 只烧 3.7s(0.3%)，"
    echo "   livox_fusion_node 1200s 只烧 10s(0.8%)，而健康的预处理是 28%。"
    printf '   %-8s %-8s %-9s %-6s %s\n' PID 存活s 累计CPU '%CPU' 命令
    ps -eo pid,etimes,time,pcpu,args --no-headers \
      | awk '$0 !~ /awk/ && /robot_state_publisher|livox_(preprocess|fusion)|slice_scan|slam_toolbox/ {
              printf "   %-8s %-8s %-9s %-6s %s\n", $1, $2, $3, $4, substr($5,1,60) }'
    # 注意 grep -c 的坑：没命中时它**已经打印了 0** 而退出码是 1，
    # 所以这里只能 `|| true`，写成 `|| echo 0` 会打印两个 0。
    echo '-- 日志里的下游症状计数（都是「上游没数据」的转述，不是各自的缺陷）:'
    for pat in '从未收到任何点云' 'observation buffer has not been updated' \
               '尚未收到任何激光数据' 'ConnectivityException'; do
      printf '   %-46s %s\n' "$pat" \
        "$({ grep -cF "$pat" "$STACK_LOG" || true; } 2>/dev/null)"
    done
    echo '-- 有没有进程真的死了（有 -> 不是本类故障，按日志里的报错查）:'
    printf '   %s\n' "$({ grep -c 'process has died' "$STACK_LOG" || true; } 2>/dev/null)"
  } > "$LOGDIR/chain_evidence_${1}.txt" 2>&1
}

for attempt in $(seq 0 "$RETRIES"); do
  [ "$attempt" -gt 0 ] && echo "  ---- 第 $((attempt+1)) 次启动 ----"
  cd "$REPO"
  nohup ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
    "${launch_args[@]}" > "$STACK_LOG" 2>&1 &
  STACK_PID=$!
  echo "  launch pid=$STACK_PID  日志 $STACK_LOG"
  echo "$STACK_PID" > "$LOGDIR/stack.pid"

  # ──── 闸门 A · 物理在不在步进 ────
  # 判据是 /clock 的**实收帧数**，不是发布者数：死锁时 /clock 发布者=1（桥活着）
  # 而 6 秒收 0 帧，两者表现完全不同。
  # timeout 是第二道保险：脚本内部已有 --timeout，包一层是防它在收尾路径上挂住
  # （rclpy 拆除挂死已修，但 call site 有上界才不会把整轮启动无限期拖住）。
  if ! timeout --signal=KILL "$((GATE_SEC + 60))" \
        python3 "$REPO/tools/wait_sim_stepping.py" --timeout "$GATE_SEC" 2>&1 | sed 's/^/  /'; then
    fail_kind="deadlock"
    save_deadlock_evidence "$attempt"
    echo "  ⚠️ 第 $((attempt+1)) 次是加载期死锁，现场留在 deadlock_evidence_${attempt}.txt"
    sed 's/^/     /' "$LOGDIR/deadlock_evidence_${attempt}.txt"
    # 只在**还有下一次**时清理。最后一次不清 —— 下面的失败出口承诺"栈仍在运行、
    # 现场留着才能查"，清掉就让那句话变成假的，而现场一去不回。
    if [ "$attempt" -lt "$RETRIES" ]; then
      run_cleanup "$LOGDIR/clean_retry_${attempt}.log" || exit 4
    fi
    continue
  fi

  # ──── 闸门 B · 感知/TF 链在不在出数据 ────
  # !!! 这道闸门是 2026-09-08 用 22 分钟换来的，不能省 !!!
  # 那一轮闸门 A **通过了**（3s 收 20 帧 /clock、仿真时间前进 0.02s），
  # 而 robot_state_publisher 与 livox_fusion_node 虽然进程活着、日志正常，
  # 却整个不在 DDS 图里：机器人 TF 0 条边、/livox/fused_points 0 帧、
  # scan 双 0Hz。于是 rviz 里 Fixed Frame=map 根本不存在（用户报的"无坐标"），
  # 而脚本浑然不觉地进了验证段。闸门 A 只证明 Gazebo 在走，
  # 对 RSP 与感知链**没有任何覆盖** —— 少这一道就是把基础设施故障
  # 当成导航缺陷去查。判据与失败归因都在 wait_perception_chain.py 里。
  if ! timeout --signal=KILL "$((GATE_SEC + 60))" \
        python3 "$REPO/tools/wait_perception_chain.py" \
        --scan-source "$SCAN" --timeout "$GATE_SEC" 2>&1 | sed 's/^/  /'; then
    fail_kind="chain"
    save_chain_evidence "$attempt"
    echo "  ⚠️ 第 $((attempt+1)) 次是感知/TF 链故障，现场留在 chain_evidence_${attempt}.txt"
    sed 's/^/     /' "$LOGDIR/chain_evidence_${attempt}.txt"
    # 只在**还有下一次**时清理。最后一次不清 —— 下面的失败出口承诺"栈仍在运行、
    # 现场留着才能查"，清掉就让那句话变成假的，而现场一去不回。
    if [ "$attempt" -lt "$RETRIES" ]; then
      run_cleanup "$LOGDIR/clean_retry_${attempt}.log" || exit 4
    fi
    continue
  fi

  infra_ok="true"
  break
done

if [ "$infra_ok" != "true" ]; then
  echo
  if [ "$fail_kind" = "deadlock" ]; then
    echo "🔴 $((RETRIES+1)) 次启动全部卡在加载期死锁，物理一步没走。"
    echo "   这是已知的 gz_ros2_control 取 robot_description 死锁，**根因未定**，"
    echo "   现有对策只有重启。证据在 $LOGDIR/deadlock_evidence_*.txt"
  else
    echo "🔴 $((RETRIES+1)) 次启动全部没能让感知/TF 链出数据（物理是在步进的）。"
    echo "   这是已知的启动期偶发故障：进程活着、日志正常，但 participant 在 DDS 图里"
    echo "   一次都不出现（同类记录见 tools/run_five_round_exploration.sh 头部）。"
    echo "   **根因未定**，现有对策只有重启。证据在 $LOGDIR/chain_evidence_*.txt"
  fi
  echo "   栈**仍在运行**，没有替你清掉 —— 现场留着才能查。"
  exit 5
fi

# ---- 查询侧 domain 必须与栈**实际**用的对上。从活着的进程反读，不从自己的 shell 猜。
#      对不上的表现是所有话题 pub=0、节点一个都看不见 —— 极像"系统没起来"。
stack_domain=""
while read -r pid; do
  [ -n "$pid" ] || continue
  stack_domain="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
    | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)"
  [ -n "$stack_domain" ] && break
done < <(ps -eo pid,args --no-headers | awk '$0 !~ /awk/ {
    k = split($2, p, "/")
    if (p[k] == "controller_server" || p[k] == "robot_state_publisher") print $1 }')
if [ -n "$stack_domain" ] && [ "$stack_domain" != "${ROS_DOMAIN_ID}" ]; then
  die "查询侧 ROS_DOMAIN_ID=$ROS_DOMAIN_ID，而栈进程实际用的是 $stack_domain。
   在这个状态下每一条查询都会读到空，「栈没起来」的结论会是假的。"
fi
echo "  查询侧 domain 已与栈进程核对一致: ROS_DOMAIN_ID=${stack_domain:-$ROS_DOMAIN_ID}"

# ════════════════════════════════════════════════════════════════════════════
# 第 3 步 · 起 rviz（步进判据已过之后）
# ════════════════════════════════════════════════════════════════════════════
count_rviz() {
  # 按 args 第一个字段的 basename 精确比，不用 pgrep -x：Linux 的 comm 字段被
  # 截断到 15 字符，pgrep -x 对长名字永不匹配；也不用 pgrep -f，那会把本脚本
  # 自己和并发的其它工具 shell 一起数进来。
  ps -eo args --no-headers | awk '$0 !~ /awk/ {
      k = split($1, p, "/"); if (p[k] == "rviz2") n++ } END { print n+0 }'
}

RVIZ_CFG="$REPO/ws_robot/install/astribot_s1_navigation/share/astribot_s1_navigation/rviz/nav2_view.rviz"
if [ "$USE_RVIZ" = "true" ]; then
  echo
  echo "──── 第 3 步 · 启动 rviz ────"
  [ -f "$RVIZ_CFG" ] || die "rviz 配置不在安装目录里: $RVIZ_CFG
   （setup.py 只 glob rviz/*.rviz —— .rviz 放到 config/ 下永远不会被安装）"
  nohup ros2 run rviz2 rviz2 -d "$RVIZ_CFG" --ros-args \
    -p use_sim_time:=true > "$LOGDIR/rviz.log" 2>&1 &
  echo "  rviz pid=$!  日志 $LOGDIR/rviz.log"
  # 起没起来只能等一下再数。rviz 冷启动在这台机器上要几秒。
  for _ in $(seq 1 20); do
    [ "$(count_rviz)" -ge 1 ] && break
    sleep 1
  done
  n_rviz="$(count_rviz)"
  if [ "$n_rviz" -lt 1 ]; then
    echo "  🔴 rviz2 进程数=0。日志尾部:"
    tail -15 "$LOGDIR/rviz.log" 2>/dev/null | sed 's/^/     /'
    die "rviz 没起来。DISPLAY=$DISPLAY 对不对？（本机可用的是 :1）"
  fi
  echo "  ✅ rviz2 进程数=$n_rviz"
else
  echo
  echo "──── 第 3 步 · 跳过 rviz（--no-rviz）────"
fi

# ════════════════════════════════════════════════════════════════════════════
# 第 4 步 · 分层验证。逐级往上，上一层不过就不查下一层。
#   L2 TF          map -> astribot_torso_base 能查到且龄期小
#   L3 生命周期     7 个 nav2 节点 get_state == active
#   L4 实测拍率     scan / map / costmap_raw / odom 在窗口内真的出帧
#   L5 跟踪器参数   从活着的 controller_server 回读，与本次入参逐项比对
#   L6 真实导航     explore 档看协调器派发+实测位移；localize 档发一个
#                   **先经 planner 证明可达**的目标并用实测位姿判到位
# ════════════════════════════════════════════════════════════════════════════
echo
echo "──── 第 4 步 · 分层验证 ────"
verify_args=(
  --mode "$MODE"
  --tracker "$TRACKER"
  --max-linear-speed "$MAXV"
  --scan-source "$SCAN"
  --nav-timeout "$NAV_TIMEOUT"
  --report "$LOGDIR/verify.tsv"
)
[ "$SKIP_NAV" = "true" ] && verify_args+=(--skip-nav-goal)
[ -n "$GOAL" ] && verify_args+=(--goal "$GOAL")

set +e
# ⚠️ 必须包 timeout（2026-09-08 实测教训）：验证脚本挂死过一次，
# 22 分钟只产出一行表头（`verify.log` 只有 17 字节），进程 1792s 烧掉 0.7s CPU。
# 根因是 rclpy 的 destroy_subscription 拆除路径挂死（已修），但**call site 也得有上界**：
# 挂死的成因不止一个，而没有上界时它会把整轮跑无限期拖住，
# 表象还极像"栈没起来"。上界给足导航超时 + 240s 各层余量。
VERIFY_TIMEOUT=$(( NAV_TIMEOUT + 240 ))
timeout --signal=KILL "$VERIFY_TIMEOUT" \
  python3 "$REPO/tools/verify_sim_stack.py" "${verify_args[@]}" 2>&1 | tee "$LOGDIR/verify.log"
verify_rc=${PIPESTATUS[0]}
set -e
if [ "$verify_rc" -eq 137 ]; then
  echo "🔴 验证脚本被 timeout 强杀（超过 ${VERIFY_TIMEOUT}s）—— 这本身是个缺陷，"
  echo "   不是栈的结论。verify.log 的最后一行就是它卡住的那一层。"
fi

echo
echo "════════════════════════════════════════════════════════════"
echo " 日志目录 : $LOGDIR"
echo "   stack.log     栈全部输出（死锁现场也在这里）"
echo "   verify.log    分层验证全文"
echo "   verify.tsv    逐层判据与实测数字（可直接贴报告）"
echo " 关栈     : bash $REPO/tools/clean_sim_stack.sh"
echo " 看导航   : bash $REPO/tools/watch_nav_live.sh"
echo "════════════════════════════════════════════════════════════"

if [ "$verify_rc" -ne 0 ]; then
  echo "🔴 验证未全过（退出码 $verify_rc）。栈**仍在运行**，没有替你清掉 ——"
  echo "   现场留着才能查。查完手动跑 clean_sim_stack.sh。"
  exit "$verify_rc"
fi
echo "✅ 全链路已起，且逐层判据全部通过。"
