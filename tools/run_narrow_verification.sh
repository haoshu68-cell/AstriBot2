#!/usr/bin/env bash
# 窄通道贴边通行 + 膨胀带脱困的**长时间跑机验证**。
#
# 循环：清理 -> 起栈 -> 门控 -> 观察到停滞 -> 收数 -> 杀栈 -> 下一轮。
#
# ============ 这个脚本的每一条纪律都来自一次实测踩坑 ============
#
# 1) **起栈前必须清干净，且判据是 /clock 发布者数。**
#    陈旧的 parameter_bridge 会重连新 Gazebo ⇒ /clock 两个发布者 ⇒
#    仿真时间 1ms 级回跳 ⇒ tf2 清空整个 TF buffer ⇒ planner 反复 abort ⇒
#    0 次到位，而 Gazebo/SLAM 看着都正常。上一轮的 cycle_2/3 就死在这里。
#
# 2) **本轮日志只认 T0 之后新建的。**
#    用 `ls -t | head -1` 曾经取到**上一轮**被杀掉的协调器日志，
#    把它的 10 次到位当成本轮成绩报出来 —— 这是最恶劣的一类错误：数据造假。
#    这里除了 -newermt 过滤，还加一条自检：日志的 mtime 必须 >= 轮次起点。
#
# 3) **grep -c 不能直接用。**
#    `n=$(grep -c pat f || echo 0)` 在无匹配时会输出 "0\n0"（grep 打印 0
#    且退出码 1），后续 [ ] 比较直接报"需要整数表达式"。
#
# 4) **绝不用 pkill -f / pgrep -f 模式匹配。**
#    命令行里含有那个模式，会命中脚本自己，循环从中间静默断掉。
#    清理统一交给 tools/clean_sim_stack.sh（它按 argv[0] 的 basename 精确匹配）。
#
# 5) **headless 保持 false。** headless:=true 是 gz_ros2_control 加载期死锁的
#    **触发条件**，不是规避手段（与直觉相反，已实测）。

REPO=/home/yjh/WorkSpace/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
source "$REPO/ws_robot/install/setup.bash"
set -u

export ROS_DOMAIN_ID=25          # 与 env.sh 一致；查询侧不带就是"节点全都 Node not found"
export ROS_LOCALHOST_ONLY=1

CYCLES="${CYCLES:-6}"            # 轮数
STALL_SEC="${STALL_SEC:-420}"    # 多久没有新的"目标收敛完成"算停滞
MAX_CYCLE_SEC="${MAX_CYCLE_SEC:-2700}"
GATE_SEC="${GATE_SEC:-180}"      # 起栈门控上限。DDS 发现实测要 30~50s 收敛

OUT="${OUT:-/tmp/narrow_verify}"
mkdir -p "$OUT"
RESULTS="$OUT/results.csv"
HEADER="cycle,arm,arm_verified,gate_ok,verdict,end_reason,dur_s,arrive,narrow_engage,narrow_through,narrow_yawhold,narrow_saturated,red_blocked,center_lethal,misfire,no_heading,escape,plan_abort,lethal_start,tf_jump,clock_pubs,prealign_trig,prealign_ok,prealign_timeout,prealign_sweep_blocked,prealign_capped,prealign_blocked_fav,sq_switch,sq_verify_fail,sq_revert_total,sq_revert_clear,sq_revert_timeout,sq_realign,sq_watchdog,sq_wd_blind,sq_dwell_med_s,sq_dwell_min_s,sq_sup_dwell,sq_sup_cooldown,sq_exit_degraded,sq_applicable,sq_switch_ceiling,fp_end_vertices"

if [ ! -f "$RESULTS" ]; then
  echo "$HEADER" > "$RESULTS"
elif [ "$(head -1 "$RESULTS")" != "$HEADER" ]; then
  # 🔴 绝不静默追加：旧文件是 21 列、新行是 27 列，追加进去每一列都会错位，
  #    而 CSV 不会报任何错 —— 事后所有读数都是错的且看不出来。
  echo "!! $RESULTS 的表头与当前版本不一致（列数/列名变了）。" >&2
  echo "   旧: $(head -1 "$RESULTS")" >&2
  echo "   新: $HEADER" >&2
  echo "   请换一个 OUT 目录，或先把旧结果改名归档。拒绝追加以免列错位。" >&2
  exit 1
fi


# ---------------------------------------------------------------- A/B 开关
#
# 只证明"接管会发生"是不够的 —— 那不回答「策略是否可行」。
# 可行性要靠对照：narrow_enabled 开/关 交替跑，比到位数。
#
# 切换方式是直接改 yaml 里的 narrow_enabled。因为：
#   · nav2 的插件参数是嵌套的，命令行 -p 覆盖不到；
#   · 参数在 configure() 里只读一次，运行期 ros2 param set 无效；
#   · 另起一份 params_file 会踩「params_file 共享上下文泄漏」那个坑
#     （第一个 include 抢走名字，后面的节点静默加载错的 yaml）。
#
# !!! 必须改**安装态**那份 !!!
# install/ 下的 config 是 colcon 复制出来的**真实文件**，不是符号链接
# （本仓没有用 --symlink-install）。launch 读的是安装态那份。
# 第一版只 sed 了源码态，于是"关臂"那轮实际仍然开着：
#   日志反证 arm=on、且发生了 57 次接管。整轮数据作废。
# 两份都改，但**判据取安装态**。
NAV_YAML_SRC="$REPO/ws_robot/src/astribot_s1_navigation/config/nav2_params_mppi.yaml"
NAV_YAML_INST="$REPO/ws_robot/install/astribot_s1_navigation/share/astribot_s1_navigation/config/nav2_params_mppi.yaml"
# 缩足迹的 A/B 要同时切协调器侧的看门狗 —— 两者必须同开同关
# （配置自洽性测试 TestNarrowSquareFootprint 会强制这一条）。
COORD_YAML_SRC="$REPO/ws_robot/src/astribot_s1_autonomy/config/exploration_coordinator_params.yaml"
COORD_YAML_INST="$REPO/ws_robot/install/astribot_s1_autonomy/share/astribot_s1_autonomy/config/exploration_coordinator_params.yaml"
YAML_BACKUP_SRC="$OUT/nav2_params_mppi.yaml.src.orig"
YAML_BACKUP_INST="$OUT/nav2_params_mppi.yaml.inst.orig"
[ -f "$YAML_BACKUP_SRC" ]  || cp "$NAV_YAML_SRC"  "$YAML_BACKUP_SRC"
[ -f "$YAML_BACKUP_INST" ] || cp "$NAV_YAML_INST" "$YAML_BACKUP_INST"
restore_yaml() {
  cp "$YAML_BACKUP_SRC"  "$NAV_YAML_SRC"
  cp "$YAML_BACKUP_INST" "$NAV_YAML_INST"
}
trap restore_yaml EXIT

set_arm() {
  local want
  want=$([ "$1" = on ] && echo true || echo false)
  local f
  for f in "$NAV_YAML_SRC" "$NAV_YAML_INST"; do
    sed -i "s/^\(      narrow_enabled: \).*/\1$want/" "$f"
  done
  # 判据只看安装态 —— 那才是 launch 真正读的文件。
  local n
  n=$(grep -c "^      narrow_enabled: $want\$" "$NAV_YAML_INST")
  # 两个控制器实例都必须切到位。切了一个漏一个，两个场景跑不同的策略。
  [ "$n" = "2" ] || { say "!! set_arm $1 在**安装态**只改到 $n 处(应为 2)"; return 1; }
  say "  set_arm $1: 安装态 narrow_enabled=$want x$n"
  return 0
}

# 切「进入前朝向预对齐」这一臂。与 set_arm 同一套纪律：
# 改源码态与安装态两份，但**判据只看安装态**（那才是 launch 真正读的文件）。
# 上一版就是因为只改了源码态，导致整轮 A/B 实际上跑的是同一个臂而毫无察觉。
set_prealign() {
  local want
  want=$([ "$1" = on ] && echo true || echo false)
  local f
  for f in "$NAV_YAML_SRC" "$NAV_YAML_INST"; do
    sed -i "s/^\(      narrow_prealign_enabled: \).*/\1$want/" "$f"
  done
  local n
  n=$(grep -c "^      narrow_prealign_enabled: $want\$" "$NAV_YAML_INST")
  # 两个控制器实例(FollowPathThreePhase / FollowPathExplore)都必须切到位。
  [ "$n" = "2" ] || { say "!! set_prealign $1 在**安装态**只改到 $n 处(应为 2)"; return 1; }
  say "  set_prealign $1: 安装态 narrow_prealign_enabled=$want x$n"
  return 0
}

# 切「窄通道内缩足迹」这一臂。必须同时切**控制器**与**协调器看门狗** ——
# 只开前者等于没有防锁存兜底，而那正是本功能最危险的失效模式。
set_square() {
  local want
  want=$([ "$1" = on ] && echo true || echo false)
  local f
  for f in "$NAV_YAML_SRC" "$NAV_YAML_INST"; do
    sed -i "s/^\(      narrow_square_enabled: \).*/\1$want/" "$f"
  done
  # 看门狗那一项缩进 6 格且在 footprint_watchdog: 段下，键名就叫 enabled，
  # 所以必须限定在该段之内改 —— 全局改 "enabled:" 会误伤别的段。
  for f in "$COORD_YAML_SRC" "$COORD_YAML_INST"; do
    [ -f "$f" ] || { say "!! 找不到协调器参数文件 $f"; return 1; }
    python3 - "$f" "$want" <<'PY'
import re, sys
path, want = sys.argv[1], sys.argv[2]
src = open(path, encoding='utf-8').read()
# 只替换 footprint_watchdog: 段里第一个 enabled:
m = re.search(r'(footprint_watchdog:\s*\n(?:.*\n)*?\s*enabled:\s*)(true|false)', src)
if not m:
    sys.exit('在 %s 里找不到 footprint_watchdog.enabled' % path)
src = src[:m.start(2)] + want + src[m.end(2):]
open(path, 'w', encoding='utf-8').write(src)
PY
  done
  # 判据只看安装态 —— 那才是 launch 真正读的文件。
  local n
  n=$(grep -c "^      narrow_square_enabled: $want\$" "$NAV_YAML_INST")
  [ "$n" = "2" ] || { say "!! set_square $1 在**安装态**只改到 $n 处(应为 2)"; return 1; }
  local w
  w=$(python3 - "$COORD_YAML_INST" <<'PY'
import re, sys
src = open(sys.argv[1], encoding='utf-8').read()
m = re.search(r'footprint_watchdog:\s*\n(?:.*\n)*?\s*enabled:\s*(true|false)', src)
print(m.group(1) if m else 'MISSING')
PY
)
  [ "$w" = "$want" ] || { say "!! set_square $1 看门狗安装态是 '$w'(应为 $want)"; return 1; }
  say "  set_square $1: 安装态 narrow_square_enabled=$want x$n, 看门狗=$w"
  return 0
}

# 安全的计数：无匹配返回 0，绝不返回 "0\n0"。
count() {
  local c
  c=$(grep -c -- "$1" "$2" 2>/dev/null | head -1)
  case "$c" in ''|*[!0-9]*) c=0 ;; esac
  echo "$c"
}

clock_pubs() {
  local n
  n=$(timeout 25 ros2 topic info /clock 2>/dev/null | awk '/Publisher count:/ {print $3}')
  case "$n" in ''|*[!0-9]*) n=-1 ;; esac
  echo "$n"
}

# 取一次仿真时刻(秒，浮点)。取不到回 -1。
clock_now() {
  local s
  s=$(timeout 15 ros2 topic echo /clock --once 2>/dev/null \
      | awk '/sec:/{sec=$2} /nanosec:/{printf "%.3f", sec+$2/1e9; exit}')
  case "$s" in ''|*[!0-9.]*) s=-1 ;; esac
  echo "$s"
}

# 仿真时间**是否在推进**。
#
# !!! 为什么不能只看 /clock 发布者数 !!!
# 实测过一次：gz_ros2_control 在「asking for robot_description」处死锁，
# Gazebo 物理一步没走、进程后来还死了，而 `ros2 topic info /clock` 依旧报
# Publisher count: 1 —— 因为报数的是 parameter_bridge，它自己活着，
# 只是桥接的 Gazebo 没了。门控于是一直"通过"这一项。
#
# 三件事必须分开验：有发布者 != 有消息 != 时间在推进。
clock_advancing() {
  local a b
  a=$(clock_now)
  [ "$a" = "-1" ] && { echo no; return; }
  sleep 3
  b=$(clock_now)
  [ "$b" = "-1" ] && { echo no; return; }
  awk -v a="$a" -v b="$b" 'BEGIN{ print (b > a + 0.05) ? "yes" : "no" }'
}

# Gazebo 加载期死锁的判据：`ign gazebo-1` 的日志行数。
#
# 正常启动几十秒内就有几百行（实测健康轮 274 行）；死锁时永远停在个位数
# （实测 9 行，最后一行是 "asking for robot_description"，之后 157s 无输出）。
# 这个判据比"进程是否活着"强 —— 死锁时进程活得好好的。
gazebo_stuck() {
  local log="$1" lines
  lines=$(grep -c "ign gazebo-1" "$log" 2>/dev/null)
  case "$lines" in ''|*[!0-9]*) lines=0 ;; esac
  if [ "$lines" -lt 30 ]; then echo "yes:$lines"; else echo "no:$lines"; fi
}

say() { echo "[$(date +%H:%M:%S)] $*"; }

# 用 while 而不是 `for c in $(seq ...)`：Gazebo 死锁轮要**重跑同一个轮号**，
# 而 for..in 的取值早已展开，循环体里改 c 完全无效（会静默地继续下一个值）。
c=0
gz_retry=0
while [ "$c" -lt "$CYCLES" ]; do
  c=$((c + 1))
  # 奇数轮开、偶数轮关。交替而不是「先全开再全关」：
  # 仿真长跑会退化（实测读数会漂出 MuJoCo 自己的硬限位），
  # 分块跑会把退化整块算到后一个臂头上。
  # ARM_FIRST 决定第一轮跑哪个臂。默认 on；本次修复后先跑 off,
  # 因为 off 是唯一一个从未真正跑过的臂（上一版 sed 改错了文件）。
  # ARM_FIXED=on 时全部轮次都开臂（用于「连续 N 轮无问题」的验收，
  # 而不是 A/B 对照 —— 对照会把一半轮次花在关臂上）。
  if [ -n "${ARM_FIXED:-}" ]; then
    ARM="$ARM_FIXED"
  elif [ $((c % 2)) -eq 1 ]; then
    ARM="${ARM_FIRST:-on}"
  else
    ARM=$([ "${ARM_FIRST:-on}" = on ] && echo off || echo on)
  fi
  say "=========== cycle $c / $CYCLES  (arm=$ARM) ==========="
  # AB_KNOB 决定这一臂切的是哪个开关：
  #   narrow   (默认) 切 narrow_enabled —— 窄通道贴边通行整层的 A/B
  #   prealign        切 narrow_prealign_enabled，narrow_enabled 恒为 true
  #                   —— 只测「进入前预对齐」这一步的增量
  if [ "${AB_KNOB:-narrow}" = "prealign" ]; then
    set_arm on || { say "cycle $c: 切臂失败，跳过"; continue; }
    set_prealign "$ARM" || { say "cycle $c: 切预对齐失败，跳过"; continue; }
  elif [ "${AB_KNOB:-narrow}" = "square" ]; then
    # 缩足迹的增量：两臂都开 narrow + 预对齐，只切缩足迹本身。
    set_arm on || { say "cycle $c: 切臂失败，跳过"; continue; }
    set_prealign on || { say "cycle $c: 切预对齐失败，跳过"; continue; }
    set_square "$ARM" || { say "cycle $c: 切缩足迹失败，跳过"; continue; }
  else
    set_arm "$ARM" || { say "cycle $c: 切臂失败，跳过"; continue; }
  fi

  # ---- 1) 清理，判据是 /clock 发布者数为 0 ----
  if ! bash "$REPO/tools/clean_sim_stack.sh" > "$OUT/clean_$c.log" 2>&1; then
    say "cycle $c: 清理未通过，跳过本轮（在脏状态下的数据不可信）"
    tail -4 "$OUT/clean_$c.log"
    # 列数必须与表头严格一致（34 列）。旧版这里只写了 18 个字段，
    # 追加进 21 列的表头里，后面每一列都是错位的且 CSV 不会报错。
    echo "$c,$ARM,?,0,SKIP,清理未通过,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1,0,0,0,0,-1,?" >> "$RESULTS"

    continue
  fi
  sleep 3

  T0=$(date +%s)
  LAUNCH_LOG="$OUT/launch_$c.log"

  # exploration:=true 起协调器；controller_plugin:=mppi 才有 ThreePhaseController
  #（rpp 那份参数里根本没有这个插件实例，默认值恰好是 rpp）。
  # USE_RVIZ 默认 false：长跑不需要它占资源。设 true 可边跑边看。
  #
  # 开 rviz 的前提是配置里的 Tools 段**不含 SetGoal** —— 默认工具集含它，
  # 会往 /goal_pose 发目标、与探索协调器抢目标。已核对
  # astribot_s1_autonomy/rviz/autonomy_debug.rviz 的 Tools 只有
  # MoveCamera / Select / FocusCamera / Measure，可以安全开启。
  nohup ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
    exploration:=true controller_plugin:=mppi use_rviz:=${USE_RVIZ:-false} headless:=false \
    > "$LAUNCH_LOG" 2>&1 &
  LAUNCH_PID=$!
  say "cycle $c: launch pid=$LAUNCH_PID (mppi + exploration)"

  # ---- 2) 门控：时钟唯一 + 仿真时间在推进 + 真的收到地图 ----
  gate_ok=0
  gz_deadlock=0
  for i in $(seq 1 "$GATE_SEC"); do
    sleep 1
    [ $((i % 15)) -eq 0 ] || continue

    # (a) Gazebo 加载期死锁：早判早重启，不要干等满 GATE_SEC。
    #     实测死锁一次白等 180s 门控 + 白占一个 FAIL 名额，
    #     而它是**竞争**不是确定性死锁（上一轮 cycle 1 死锁、cycle 2 自愈），
    #     重启就能过。
    if [ "$i" -ge 60 ]; then
      st=$(gazebo_stuck "$LAUNCH_LOG")
      if [ "${st%%:*}" = "yes" ]; then
        say "  !! Gazebo 加载期死锁：ign gazebo-1 日志仅 ${st##*:} 行(<30)。"
        say "     特征是停在 gz_ros2_control 'asking for robot_description'。本轮重启。"
        gz_deadlock=1
        break
      fi
    fi

    cp_now=$(clock_pubs)
    # !!! 判据必须是「真的收到一帧 /map」，不是「/map 有发布者」!!!
    # 实测过一轮：门控报 /map 发布者=1 而协调器整轮都在
    # 「等待地图就绪: 尚未收到占据栅格地图」，那轮 0 到位、0 失败，
    # 与被测策略毫无关系。有发布者 != 有消息。
    got_map=no
    timeout 12 ros2 topic echo /map --once > /dev/null 2>&1 && got_map=yes
    # !!! 而且「有消息」也不等于「时间在推进」!!!
    # Gazebo 死锁/死亡后 parameter_bridge 仍活着、/clock 发布者数仍是 1。
    # 只有比较两次采样才能证明仿真真的在跑。
    adv=$(clock_advancing)
    say "  门控 ${i}s: /clock 发布者=$cp_now 时间在推进=$adv 收到/map=$got_map"
    rviz_ok=yes
    if [ "${USE_RVIZ:-false}" = "true" ]; then
      # 起了 rviz 就必须确认它真的在，否则「带界面重跑」是个空承诺。
      #
      # 🔴 判据是**恰好 1 个**，不是 >=1。
      #    上一版写 `-ge 1`，于是 4 个 rviz2 也算通过 —— 而那 4 个里 3 个是
      #    上一轮残留的（clean_sim_stack.sh 的 NAMES 表当时没有 rviz2，
      #    /clock 判据也抓不到它）。那一轮 686s 内 0 次到位、足迹回读=-1，
      #    整轮数据作废，而唯一能提前发现的那道校验被我写成了恒真。
      #    多于 1 个 = 上一轮没清干净 = 这一轮数据不可信，必须判 gate 失败。
      n_rviz=$(ps -eo args --no-headers | awk '$0 ~ /rviz2/ && $0 !~ /awk/' | wc -l)
      if [ "$n_rviz" -ne 1 ]; then
        rviz_ok=no
        say "    🔴 rviz2 进程=$n_rviz（期望恰好 1）—— 多于 1 说明上一轮残留，本轮数据不可信"
      fi
      say "    rviz2 进程=$n_rviz"
    fi
    if [ "$cp_now" = "1" ] && [ "$adv" = "yes" ] && [ "$got_map" = "yes" ] &&
      [ "$rviz_ok" = "yes" ]
    then
      gate_ok=1; break
    fi
    if [ "$cp_now" -gt 1 ] 2>/dev/null; then
      say "  !! /clock 有 $cp_now 个发布者 —— 仿真时间会非单调，本轮数据不可信"
      break
    fi
  done

  # 死锁轮不占 PASS/FAIL 名额：它测的是启动时序竞争，不是被测策略。
  # 直接重跑同一个 cycle 编号，最多重试 GZ_RETRY 次。
  if [ "$gz_deadlock" = "1" ]; then
    kill "$LAUNCH_PID" 2>/dev/null
    sleep 5
    gz_retry=$((gz_retry + 1))
    if [ "$gz_retry" -le "${GZ_RETRY:-3}" ]; then
      say "cycle $c: Gazebo 死锁，第 $gz_retry 次重试本轮（不计入轮次判定）"
      c=$((c - 1))      # 退回一格，while 顶部会重新自增成同一个轮号
      continue
    fi
    say "cycle $c: Gazebo 死锁重试 $gz_retry 次仍失败，按 FAIL 记录"
  fi

  [ "$gate_ok" = "1" ] && gz_retry=0
  CLOCK_PUBS=$(clock_pubs)
  if [ "$gate_ok" != "1" ]; then
    say "cycle $c: 门控未过(/clock=$CLOCK_PUBS)，收尾本轮"
  fi

  # ---- 3) 观察：直到停滞或超上限 ----
  # 本轮日志只认 T0 之后新建的（见纪律 2）。
  find_log() {
    find "$HOME/.ros/log" -maxdepth 2 -name "$1*" -newermt "@$T0" -printf '%T@ %p\n' \
      2>/dev/null | sort -rn | head -1 | cut -d' ' -f2
  }

  last_arrive=0
  cycle_end_reason=timeup
  last_seen=$(date +%s)
  while true; do
    sleep 20
    NOW=$(date +%s)
    DUR=$((NOW - T0))
    [ "$DUR" -ge "$MAX_CYCLE_SEC" ] && { say "  达本轮上限 ${MAX_CYCLE_SEC}s"; break; }
    [ "$gate_ok" = "1" ] || { [ "$DUR" -ge 90 ] && break; continue; }

    CL=$(find_log exploration_coordinator)
    [ -n "$CL" ] || { say "  ${DUR}s: 还没有本轮的协调器日志"; continue; }

    # 自检：日志必须是本轮的。命中旧日志就立刻停，不允许把旧成绩算进来。
    lmt=$(stat -c %Y "$CL" 2>/dev/null || echo 0)
    if [ "$lmt" -lt "$T0" ]; then
      say "  !! 命中的协调器日志比本轮起点还旧($lmt < $T0)，拒绝计数"
      break
    fi

    arrive=$(count "目标收敛完成" "$CL")
    CTRL_NOW=$(find_log controller_server)
    ne=0
    # !!! 接管日志在 controller_server，不在协调器 !!!
    # 起初这里 grep 的是 $CL，于是进度行全程显示"接管=0"，而收数阶段
    # 从 $CTRL 读出来是 27 —— 进度行在说谎。查错日志比不查更糟。
    [ -n "$CTRL_NOW" ] && ne=$(count "窄通道接管启动" "$CTRL_NOW")
    say "  ${DUR}s: 到位=$arrive 窄通道接管=$ne"

    # PAUSED(连续多轮无合法候选) = 探索收尾，正常终止。
    # 实测特征：前沿格从 6661 降到 105、单块增益=1、自动恢复 3 次全失败、
    # 读数逐位冻结。此时再等 STALL_SEC 只是空转。
    if grep -q "自动恢复已达上限" "$CL" 2>/dev/null; then
      say "  探索收尾(自动恢复已用尽, PAUSED)，结束本轮"
      cycle_end_reason=finished
      break
    fi
    if [ "$arrive" -gt "$last_arrive" ]; then
      last_arrive=$arrive
      last_seen=$NOW
    elif [ $((NOW - last_seen)) -ge "$STALL_SEC" ]; then
      say "  停滞 ${STALL_SEC}s 无新到位，结束本轮"
      cycle_end_reason=stalled
      break
    fi
  done

  # ---- 4) 收数 ----
  CL=$(find_log exploration_coordinator)
  CTRL=$(find_log controller_server)
  PLAN=$(find_log planner_server)
  DUR=$(( $(date +%s) - T0 ))

  arrive=0; escape=0
  if [ -n "$CL" ]; then
    arrive=$(count "目标收敛完成" "$CL")
    escape=$(count "膨胀带脱困" "$CL")
  fi
  ne=0; nthru=0; nyaw=0; nsat=0; nred=0; ncl=0
  if [ -n "$CTRL" ]; then
    ne=$(count "窄通道接管启动" "$CTRL")
    nthru=$(count "接管退出：足迹已连续脱离致命带" "$CTRL")
    nyaw=$(count "闸门关" "$CTRL")
    nsat=$(count "饱和:放弃横向寻优" "$CTRL")
    nred=$(count "禁止贴边通行" "$CTRL")
    ncl=$(count "应由协调器 ESCAPE 挪车" "$CTRL")
  fi
  pa=0; ls_=0; tj=0
  if [ -n "$PLAN" ]; then
    pa=$(count "Aborting handle" "$PLAN")
    ls_=$(count "Starting point in lethal space" "$PLAN")
    tj=$(count "Detected jump back in time" "$PLAN")
  fi

  # ---- 反证真正跑的是哪个臂 ----
  # 判据取自 configure() 打的那行启动日志，不取自我们 sed 的意图。
  #
  # ⚠️ AB_KNOB=prealign 时必须反证**预对齐**那一行，不能反证 narrow_enabled ——
  #    后者两臂恒为"已启用"，反证会永远通过，等于没有反证。
  arm_verified="?"
  if [ -n "$CTRL" ]; then
    if [ "${AB_KNOB:-narrow}" = "prealign" ]; then
      on_n=$(count "进入前朝向预对齐已启用" "$CTRL")
      off_n=$(count "进入前朝向预对齐已\*\*禁用\*\*" "$CTRL")
    elif [ "${AB_KNOB:-narrow}" = "square" ]; then
      # 反证缩足迹那一行。两臂的 narrow_enabled 与预对齐恒为开，
      # 反证它们会永远通过 —— 等于没有反证。
      on_n=$(count "窄通道缩足迹已启用" "$CTRL")
      # 关臂时控制器根本不打这一行，所以"没有这一行"就是 off 的证据。
      off_n=0
      if [ "$on_n" -eq 0 ]; then off_n=1; fi
    else
      on_n=$(count "窄通道贴边通行已启用" "$CTRL")
      off_n=$(count "窄通道贴边通行已\*\*禁用\*\*" "$CTRL")
    fi
    if [ "$on_n" -gt 0 ] && [ "$off_n" -eq 0 ]; then arm_verified="on"
    elif [ "$off_n" -gt 0 ] && [ "$on_n" -eq 0 ]; then arm_verified="off"
    elif [ "$on_n" -gt 0 ] && [ "$off_n" -gt 0 ]; then arm_verified="MIXED"
    fi
  fi
  if [ "$arm_verified" != "$ARM" ]; then
    say "  !! 臂不符：打算跑 $ARM，日志反证是 $arm_verified —— 本行数据按不可信处理"
  fi
  # 关臂时接管次数必须为 0。不为 0 说明开关根本没起作用。
  #
  # ⚠️ 只在 AB_KNOB=narrow 下成立。AB_KNOB=prealign 时 narrow_enabled 恒为 true，
  #    关臂(预对齐关)照样会接管 —— 那是**预期行为**，不是开关失效。
  if [ "${AB_KNOB:-narrow}" != "prealign" ] && [ "$ARM" = "off" ] && [ "$ne" -gt 0 ]; then
    say "  !! 关臂却发生了 $ne 次接管 —— narrow_enabled=false 没有生效"
    arm_verified="BROKEN"
  fi

  # ---- 预对齐的逐项计数 ----
  # 逐项而非总数：总数分不出「从不触发」与「一路开阔」，而这两者一个是故障一个不是。
  pre_trig=0; pre_ok=0; pre_to=0; pre_sweep=0; pre_cap=0; pre_blocked=0
  if [ -n "$CTRL" ]; then
    # ⚠️ 每个模式都必须匹配**只出现在该事件行**的串。
    #    第一版这里用 "转正也过不去" 数出 12，而真实事件只有 6 次 ——
    #    因为那个词组同时出现在「预对齐完成」INFO 的累计尾巴里，一个事件被数两遍。
    #    这与之前 escape=1 那个假计数是同一类错误（匹配到了非事件行）。
    #
    # ⚠️ 而且两类计数**方式不同，别混用**：
    #   · 启动/完成/超时/上限 —— 每次事件都打一行且**不节流**，数行即是数事件。
    #   · 前视无余量/扫掠被拒 —— 日志**被节流**(5s/3s)，数行会系统性少算
    #     （实测内部累计 14 而只打了 6 行）。这两项只能读日志里那个
    #     **累计值的最后一次**，不能数行数。同 LOOP_OVERRUN 那个截尾样本坑。
    pre_trig=$(count "进入前预对齐启动" "$CTRL")
    pre_ok=$(count "进入前预对齐完成" "$CTRL")
    pre_to=$(count "进入前预对齐超时" "$CTRL")
    pre_cap=$(count "进入前预对齐已达本目标上限" "$CTRL")
    pre_blocked=$(grep -o "前视发现窄处.*累计=[0-9]\+" "$CTRL" 2>/dev/null \
                  | tail -1 | grep -o "累计=[0-9]\+" | grep -o "[0-9]\+")
    case "$pre_blocked" in ''|*[!0-9]*) pre_blocked=0 ;; esac
    pre_sweep=$(grep -o "累计扫掠被拒=[0-9]\+" "$CTRL" 2>/dev/null \
                | tail -1 | grep -o "[0-9]\+")
    case "$pre_sweep" in ''|*[!0-9]*) pre_sweep=0 ;; esac
  fi
  # 预对齐关闭时触发数必须为 0，否则开关没生效。
  if [ "${AB_KNOB:-narrow}" = "prealign" ] && [ "$ARM" = "off" ] && [ "$pre_trig" -gt 0 ]; then
    say "  !! 预对齐已关却触发了 $pre_trig 次 —— narrow_prealign_enabled=false 没有生效"
    arm_verified="BROKEN"
  fi

  # ---- 缩足迹的逐项计数 ----
  #
  # 🔴 第一轮 A/B 的教训：主判据(起点致命 1.08/min -> 0)看起来极好，但**不可采信**，
  #    因为小足迹只生效 5.7% 的时长、切换 123 次、单次驻留中位 0.15s。
  #    所以本轮的第一等度量不是"起点致命降了多少"，而是**驻留时长分布** ——
  #    它决定了主判据能不能归因到这个机制。下面 sq_dwell_med 就是那个数。
  sq_switch=0; sq_verify_fail=0; sq_rev_total=0; sq_rev_clear=0; sq_rev_timeout=0; sq_realign=0
  sq_wd=0; sq_wd_blind=0
  sq_sup_dwell=0; sq_sup_cool=0; sq_degraded=0
  sq_dwell_med=-1; sq_dwell_min=-1; sq_ceiling=-1; sq_applicable=0
  if [ -n "$CTRL" ]; then
    sq_switch=$(count "已回读确认生效" "$CTRL")
    sq_verify_fail=$(count "小足迹切换\*\*回读失败\*\*" "$CTRL")
    # ⚠️ 复原原因的日志串在加迟滞时改过。旧串 "默认足迹已连续脱离致命带" 现在
    #    一次都不会出现 —— 若不同步改这里，这一列会静默恒为 0，
    #    而"复原 0 次"读起来像"从不复原"（那是完全相反的结论）。
    # 总复原数：包含"换了新目标"/deactivate 那些**安全通路**的复原。
    # 少了它就分不清"从没复原(可能锁存)"与"只是没有装得下型复原"。
    sq_rev_total=$(count "窄通道足迹已复原为默认" "$CTRL")
    sq_rev_clear=$(count "大足迹已连续装得下" "$CTRL")
    sq_rev_timeout=$(count "小足迹硬超时" "$CTRL")
    sq_realign=$(count "小足迹生效期间失去对齐" "$CTRL")

    # 被节流的三项(INFO_THROTTLE 2s)只能读**累计值的最后一次**，不能数行数。
    # 同 LOOP_OVERRUN 那个截尾样本坑：节流日志的行数系统性少于事件数。
    # ⚠️ 「可用拍」与「转正也过不去」都是**逐拍**计数器，可直接相比 ——
    #    这个比值决定缩足迹在本图上到底有没有用武之地。
    #    第一轮我只有 kBlocked=790，而"需缩足迹"那行是节流 INFO 且无累计值，
    #    只能数到 1 行；拿 1 去比 790 是错的（节流日志不能当事件计数器）。
    for kv in "驻留压住:sq_sup_dwell" "冷却挡掉:sq_sup_cool" "切出退化:sq_degraded" "可用拍:sq_applicable"; do
      k=${kv%%:*}
      v=$(grep -o "$k=[0-9]\+" "$CTRL" 2>/dev/null | tail -1 | grep -o "[0-9]\+")
      case "$v" in ''|*[!0-9]*) v=0 ;; esac
      case "$kv" in
        *sq_sup_dwell) sq_sup_dwell=$v ;;
        *sq_sup_cool)  sq_sup_cool=$v ;;
        *sq_degraded)  sq_degraded=$v ;;
        *sq_applicable) sq_applicable=$v ;;
      esac
    done

    # ---- 驻留时长分布：本轮最重要的一个数 ----
    # 复原日志里每次都打 "本次小足迹生效 %.1fs"，逐次取出算中位与最小。
    # 中位必须 >= narrow_square_min_dwell(2.0)，否则最短驻留根本没起作用。
    #
    # ⚠️ 只取**"装得下"那类复原**的驻留，别把所有复原混在一起：
    #   · 回读失败的复原发生在 pending 态，驻留恒为 0.0（会把中位拖到 0）
    #   · 换新目标 / deactivate 的复原是**安全通路，本来就不受最短驻留约束**，
    #     混进来会让"中位 < 最短驻留"这条断言在完全正常的跑法上误报。
    #   判据只能压在它要管的那条路径上（同"重试上限要数连续失败"的教训）。
    dwells=$(grep "大足迹已连续装得下" "$CTRL" 2>/dev/null \
             | grep -o "本次小足迹生效 [0-9.]\+s" \
             | grep -o "[0-9.]\+" | awk '$1>0' | sort -g)
    if [ -n "$dwells" ]; then
      sq_dwell_med=$(echo "$dwells" | awk '{a[NR]=$1} END{
        if(NR%2) printf "%.2f", a[(NR+1)/2]; else printf "%.2f", (a[NR/2]+a[NR/2+1])/2}')
      sq_dwell_min=$(echo "$dwells" | head -1)
    fi
  fi
  if [ -n "$CL" ]; then
    sq_wd=$(count "足迹锁存看门狗介入" "$CL")
    # 看门狗"瞎眼"必须单独计数：它瞎眼时**不报故障、只是安静地不动手**，
    # 于是"看门狗没介入"会被读成"一切正常"，而真相是根本没有兜底。
    sq_wd_blind=$(count "足迹看门狗\*\*瞎眼\*\*" "$CL")
  fi

  # ---- 🔴 切换次数上限断言（算术，不是感觉）----
  # 单次切换周期下限 = min_dwell + cooldown（从安装态 yaml 读，不写死）。
  # 上限 = 本轮时长 / 周期下限。实测超过它 ⇒ 迟滞逻辑有 bug，不是参数问题。
  if [ "${AB_KNOB:-narrow}" = "square" ] && [ "$ARM" = "on" ]; then
    md=$(grep -m1 "^      narrow_square_min_dwell:" "$NAV_YAML_INST" | awk '{print $2}')
    cd_=$(grep -m1 "^      narrow_square_cooldown:" "$NAV_YAML_INST" | awk '{print $2}')
    case "$md" in ''|*[!0-9.]*) md=0 ;; esac
    case "$cd_" in ''|*[!0-9.]*) cd_=0 ;; esac
    sq_ceiling=$(awk -v d="$DUR" -v m="$md" -v c="$cd_" \
      'BEGIN{p=m+c; if(p<=0){print -1}else{printf "%d", d/p + 1}}')
    if [ "$sq_ceiling" -gt 0 ] && [ "$sq_switch" -gt "$sq_ceiling" ]; then
      say "  🔴 切换 $sq_switch 次 > 算术上限 $sq_ceiling 次(时长 ${DUR}s / (驻留 ${md}s + 冷却 ${cd_}s))"
      say "     ⇒ 最短驻留或冷却期**没有生效**，是 bug 不是参数问题"
    fi
    if [ "$sq_dwell_med" != "-1" ]; then
      lo=$(awk -v a="$sq_dwell_med" -v m="$md" 'BEGIN{print (a+0.001 < m)?1:0}')
      if [ "$lo" = "1" ]; then
        say "  🔴 驻留中位 ${sq_dwell_med}s < 最短驻留 ${md}s ⇒ 最短驻留没生效"
      fi
    fi
    # 🔴 判据必须能区分三种情形，否则会像 cycle 1 那样冤枉自己"锁存"：
    #   (a) 一次都没复原        -> 真的可能锁存（但 fp_end_vertices 才是铁证）
    #   (b) 复原过但没有装得下型 -> 切出判据从未成立，是**能力问题不是 bug**
    #   (c) 有装得下型复原      -> 正常
    if [ "$sq_switch" -gt 0 ] && [ "$sq_rev_total" -eq 0 ]; then
      say "  🔴 切换了 $sq_switch 次却**一次都没复原** ⇒ 查是否锁存(看 fp_end_vertices)"
    elif [ "$sq_switch" -gt 0 ] && [ "$sq_rev_clear" -eq 0 ]; then
      say "  !! 切换 $sq_switch 次、复原 $sq_rev_total 次，但**装得下型复原 0 次** ——"
      say "     切出判据从未成立：缩了足迹也没脱离致命带。这是能力问题，不是切换 bug。"
    fi
  fi
  # 缩足迹关闭时切换数必须为 0。
  if [ "${AB_KNOB:-narrow}" = "square" ] && [ "$ARM" = "off" ] && [ "$sq_switch" -gt 0 ]; then
    say "  !! 缩足迹已关却切换了 $sq_switch 次 —— narrow_square_enabled=false 没有生效"
    arm_verified="BROKEN"
  fi

  # ---- 🔴 防锁存断言：本轮结束时代价地图必须回到默认(8 顶点)足迹 ----
  # 判据取自**代价地图自己发布的回读话题**，不是日志说了什么。
  # 停栈前必须查 —— 停了就查不到了。
  fp_end="?"
  if [ "${AB_KNOB:-narrow}" = "square" ]; then
    fp_end=$(timeout 20 ros2 topic echo /global_costmap/published_footprint --once 2>/dev/null \
             | grep -c "^  - x:" || echo 0)
    case "$fp_end" in ''|*[!0-9]*) fp_end=-1 ;; esac
    if [ "$fp_end" != "8" ]; then
      say "  🔴 收尾时代价地图足迹是 $fp_end 顶点(应为 8) —— **足迹被锁存**，这是本功能最危险的失效"
    fi
  fi

  # ---- 误杀合计：只算「本层抛异常、杀掉一条路径」的那几类 ----
  #
  # 这四类都会 throw -> FollowPath abort，是真正的误杀，
  # 每一类都对应一个已修的实测缺陷，任一 > 0 就说明修复回退了。
  mis_giveup=0; mis_stall=0; mis_hard=0; mis_dev=0; mis_heading=0
  if [ -n "$CTRL" ]; then
    mis_giveup=$(count "均未穿过，判定该路径不可行" "$CTRL")
    mis_stall=$(count "判定原地蹭" "$CTRL")
    mis_hard=$(count "触及绝对上限" "$CTRL")
    mis_dev=$(count "偏离参考路径" "$CTRL")
    # 「估不出通道方向」**不算误杀**，只作观测项。理由（cycle 1 实测取证）：
    #   · 它不抛异常、不计失败次数、不 abort，只是本拍不接管，MPPI 继续开；
    #     日志原文即 "本层不接管。这一项不计入放弃上限"
    #   · 那一轮它出现 2 次，而窄通道层 4 类真异常抛出 0 次；
    #     2 次 Aborting handle 的紧邻上文是 MPPI 自己的 Failed to make progress
    #   · 其后连续 5 次接管全部以「足迹已连续脱离致命带」退出，跟踪未被中断
    # 行为上等价于「这里不适用窄通道」。把它算成误杀会让判据比真实缺陷更严 ——
    # 而它恰恰是修复 1 想要的行为（把"没能开始"与"没穿过去"分开）。
    # 它偏高说明全局路径在机器人附近退化，值得看，但不构成不合格。
    mis_heading=$(count "估不出通道方向" "$CTRL")
  fi
  misfire=$((mis_giveup + mis_stall + mis_hard + mis_dev))

  # ---- 本轮判定。「没问题」必须是可检查的判据，不是我看一眼觉得还行 ----
  #   PASS 要求全部满足：
  #     门控过 / 臂反证一致 / 时钟唯一 / 无 TF 回跳 / 无起点落致命区
  #     / 误杀为 0 / 至少发生过 1 次接管(否则这一轮没测到本层)
  #     / 至少到位 1 次(否则栈根本没工作)
  verdict=PASS
  reasons=""
  [ "$gate_ok" = "1" ] || { verdict=FAIL; reasons="$reasons 门控未过"; }
  [ "$arm_verified" = "$ARM" ] || { verdict=FAIL; reasons="$reasons 臂不符($arm_verified)"; }
  [ "$CLOCK_PUBS" = "1" ] || { verdict=FAIL; reasons="$reasons clock=$CLOCK_PUBS"; }
  [ "$tj" -eq 0 ] || { verdict=FAIL; reasons="$reasons TF回跳=$tj"; }
  [ "$ls_" -eq 0 ] || { verdict=FAIL; reasons="$reasons 起点致命=$ls_"; }
  [ "$misfire" -eq 0 ] || { verdict=FAIL; reasons="$reasons 误杀=$misfire"; }
  # 「这一轮到底测到了没有」的判据随 AB_KNOB 走：
  #   narrow   模式：至少接管 1 次
  #   prealign 模式：on 臂至少触发 1 次预对齐（off 臂本来就该是 0，不能拿它当失败）
  if [ "${AB_KNOB:-narrow}" = "prealign" ]; then
    if [ "$ARM" = "on" ] && [ "$pre_trig" -lt 1 ]; then
      verdict=FAIL; reasons="$reasons 本轮未触发预对齐(没测到本层)"
    fi
  elif [ "${AB_KNOB:-narrow}" = "square" ]; then
    if [ "$ARM" = "on" ] && [ "$sq_switch" -lt 1 ]; then
      verdict=FAIL; reasons="$reasons 本轮未发生缩足迹(没测到本层)"
    fi
    # 🔴 收尾必须回到 8 顶点。锁存比"过不去"严重得多，必须判 FAIL。
    if [ "$fp_end" != "8" ] && [ "$fp_end" != "?" ]; then
      verdict=FAIL; reasons="$reasons 足迹锁存(收尾$fp_end顶点)"
    fi
    # 回读失败说明跨进程写入路径不可靠，数据不可信。
    if [ "$sq_verify_fail" -gt 0 ]; then
      verdict=FAIL; reasons="$reasons 缩足迹回读失败=$sq_verify_fail"
    fi
  else
    [ "$ne" -ge 1 ] || { verdict=FAIL; reasons="$reasons 本轮未触发接管(没测到本层)"; }
  fi
  [ "$arrive" -ge 1 ] || { verdict=FAIL; reasons="$reasons 零到位"; }
  [ "$verdict" = "PASS" ] || say "  判定 FAIL:$reasons"

  say "cycle $c 结果[arm=$ARM 反证=$arm_verified 判定=$verdict 结束=$cycle_end_reason]: 时长=${DUR}s 到位=$arrive | 窄通道 接管=$ne 正常穿越=$nthru 闸门关=$nyaw 饱和=$nsat 红线拦=$nred 交ESCAPE=$ncl 估不出方向=$mis_heading(观测) | 预对齐 触发=$pre_trig 成功=$pre_ok 超时=$pre_to 扫掠被拒=$pre_sweep 上限=$pre_cap 无余量=$pre_blocked | 缩足迹 切换=$sq_switch 回读失败=$sq_verify_fail 复原(装得下)=$sq_rev_clear 复原(超时)=$sq_rev_timeout 重新对齐=$sq_realign 看门狗介入=$sq_wd 收尾顶点=$fp_end | 脱困=$escape | planner abort=$pa 起点致命=$ls_ TF回跳=$tj | /clock=$CLOCK_PUBS"
  echo "$c,$ARM,$arm_verified,$gate_ok,$verdict,$cycle_end_reason,$DUR,$arrive,$ne,$nthru,$nyaw,$nsat,$nred,$ncl,$misfire,$mis_heading,$escape,$pa,$ls_,$tj,$CLOCK_PUBS,$pre_trig,$pre_ok,$pre_to,$pre_sweep,$pre_cap,$pre_blocked,$sq_switch,$sq_verify_fail,$sq_rev_total,$sq_rev_clear,$sq_rev_timeout,$sq_realign,$sq_wd,$sq_wd_blind,$sq_dwell_med,$sq_dwell_min,$sq_sup_dwell,$sq_sup_cool,$sq_degraded,$sq_applicable,$sq_ceiling,$fp_end" >> "$RESULTS"

  # 留存本轮日志路径，便于事后逐条核对（不复制内容，避免磁盘暴涨）。
  { echo "coordinator=$CL"; echo "controller=$CTRL"; echo "planner=$PLAN"; } > "$OUT/logs_$c.txt"

  kill "$LAUNCH_PID" 2>/dev/null
  sleep 5
done

say "全部轮次结束。汇总："
PASS_N=$(awk -F, 'NR>1 && $5=="PASS"' "$RESULTS" | wc -l)
TOTAL_N=$(awk -F, 'NR>1' "$RESULTS" | wc -l)
say "PASS $PASS_N / $TOTAL_N 轮"
if [ "$PASS_N" -ge 5 ]; then
  say ">>> 已达成「五轮无问题」，可以提交"
else
  say ">>> 未达成五轮无问题，**不要提交**"
fi
column -s, -t < "$RESULTS"
bash "$REPO/tools/clean_sim_stack.sh" > "$OUT/clean_final.log" 2>&1
