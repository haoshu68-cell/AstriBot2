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
[ -f "$RESULTS" ] || echo "cycle,arm,arm_verified,gate_ok,verdict,end_reason,dur_s,arrive,narrow_engage,narrow_through,narrow_yawhold,narrow_saturated,red_blocked,center_lethal,misfire,no_heading,escape,plan_abort,lethal_start,tf_jump,clock_pubs" > "$RESULTS"

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
  set_arm "$ARM" || { say "cycle $c: 切臂失败，跳过"; continue; }

  # ---- 1) 清理，判据是 /clock 发布者数为 0 ----
  if ! bash "$REPO/tools/clean_sim_stack.sh" > "$OUT/clean_$c.log" 2>&1; then
    say "cycle $c: 清理未通过，跳过本轮（在脏状态下的数据不可信）"
    tail -4 "$OUT/clean_$c.log"
    echo "$c,$ARM,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,清理未通过" >> "$RESULTS"
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
      n_rviz=$(ps -eo args --no-headers | awk '$0 ~ /rviz2/ && $0 !~ /awk/' | wc -l)
      [ "$n_rviz" -ge 1 ] || rviz_ok=no
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
  arm_verified="?"
  if [ -n "$CTRL" ]; then
    on_n=$(count "窄通道贴边通行已启用" "$CTRL")
    off_n=$(count "窄通道贴边通行已\*\*禁用\*\*" "$CTRL")
    if [ "$on_n" -gt 0 ] && [ "$off_n" -eq 0 ]; then arm_verified="on"
    elif [ "$off_n" -gt 0 ] && [ "$on_n" -eq 0 ]; then arm_verified="off"
    elif [ "$on_n" -gt 0 ] && [ "$off_n" -gt 0 ]; then arm_verified="MIXED"
    fi
  fi
  if [ "$arm_verified" != "$ARM" ]; then
    say "  !! 臂不符：打算跑 $ARM，日志反证是 $arm_verified —— 本行数据按不可信处理"
  fi
  # 关臂时接管次数必须为 0。不为 0 说明开关根本没起作用。
  if [ "$ARM" = "off" ] && [ "$ne" -gt 0 ]; then
    say "  !! 关臂却发生了 $ne 次接管 —— narrow_enabled=false 没有生效"
    arm_verified="BROKEN"
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
  [ "$ne" -ge 1 ] || { verdict=FAIL; reasons="$reasons 本轮未触发接管(没测到本层)"; }
  [ "$arrive" -ge 1 ] || { verdict=FAIL; reasons="$reasons 零到位"; }
  [ "$verdict" = "PASS" ] || say "  判定 FAIL:$reasons"

  say "cycle $c 结果[arm=$ARM 反证=$arm_verified 判定=$verdict 结束=$cycle_end_reason]: 时长=${DUR}s 到位=$arrive | 窄通道 接管=$ne 正常穿越=$nthru 闸门关=$nyaw 饱和=$nsat 红线拦=$nred 交ESCAPE=$ncl 估不出方向=$mis_heading(观测) | 脱困=$escape | planner abort=$pa 起点致命=$ls_ TF回跳=$tj | /clock=$CLOCK_PUBS"
  echo "$c,$ARM,$arm_verified,$gate_ok,$verdict,$cycle_end_reason,$DUR,$arrive,$ne,$nthru,$nyaw,$nsat,$nred,$ncl,$misfire,$mis_heading,$escape,$pa,$ls_,$tj,$CLOCK_PUBS" >> "$RESULTS"

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
