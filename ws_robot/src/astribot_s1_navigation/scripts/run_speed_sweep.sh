#!/usr/bin/env bash
# Copyright 2026 Astribot.
#
# MPPI 限速扫描驱动：逐档位重启 nav2 + 跟随探索录制到位精度。
#
# ==================================================================
# 安全约束（不是建议，是硬要求）
# ==================================================================
#   · 默认 **OBSERVE_ONLY=true**：只录制，不使能写通路、不动机器人。
#   · 使能写通路必须显式 ENABLE_WRITE=true，并且**每个档位都会停下来
#     向操作者确认速度朝向与距离**。技术前置条件（cmd_vel 无帧、电量足）
#     不构成授权。
#   · 急停按钮在操作者手里。本脚本不代替急停，也不假设有人在看屏幕。
#
# ==================================================================
# 起停判据
# ==================================================================
#   · nav2 就绪 = 生命周期状态 ACTIVE，**不数进程数**。
#     实测过 9 个进程全活而 lifecycle_manager 报 "Aborting bringup"，
#     整套 nav2 从未 activate，而"进程数 >= 5"的判据照过。
#   · 起档位后先校验**实测** vx_max == 请求值。不一致立即停、不采数：
#     nav2_full_bringup 曾漏转发 max_linear_speed，不报错不告警，
#     vx_max 仍是 1.0，整轮扫描产出若干档位数字完全相同的表。
#
# ==================================================================
# 用法
# ==================================================================
#   CAPS="0.1 0.2 0.4" MINUTES_PER_CAP=10 ./run_speed_sweep.sh
#   ENABLE_WRITE=true CAPS="0.1 0.2" ./run_speed_sweep.sh     # 会逐档确认
#
# 环境变量见下面的默认值段。
set -uo pipefail

# ------------------------------------------------------------------ 可配置项
CAPS="${CAPS:-0.1 0.2 0.4}"
MINUTES_PER_CAP="${MINUTES_PER_CAP:-10}"
ROUNDS_PER_CAP="${ROUNDS_PER_CAP:-0}"          # >0 时按轮数收工，优先于时长
OUT_ROOT="${OUT_ROOT:-/tmp/explore_metrics_sweep}"
OBSERVE_ONLY="${OBSERVE_ONLY:-true}"
ENABLE_WRITE="${ENABLE_WRITE:-false}"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-}"                       # 留空时自动找 ws_robot/install
DOMAIN_ID="${ROS_DOMAIN_ID:-25}"               # 实机实测是 25，不是 42
LIFECYCLE_NODES="${LIFECYCLE_NODES:-controller_server planner_server \
behavior_server bt_navigator velocity_smoother global_costmap/global_costmap \
local_costmap/local_costmap}"
NAV2_READY_TIMEOUT="${NAV2_READY_TIMEOUT:-180}"
# DDS 发现实测要 30~50s 收敛。短于这个去判"话题不存在"是仪器问题。
DISCOVERY_WAIT="${DISCOVERY_WAIT:-50}"
LOW_OBSTACLE_TRUTH="${LOW_OBSTACLE_TRUTH:-}"
COLLISIONS_MANUAL="${COLLISIONS_MANUAL:--1}"

# ROS2 CLI 必须走 /opt/ros/humble：厂商的 /opt/astribot_ros/middle_ware/bin
# 在 PATH 里排在前面，但它的 ros2 是**子集**，缺 lifecycle/param 等子命令。
ROS2_BIN="${ROS2_BIN:-/opt/ros/humble/bin/ros2}"

log()  { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }
warn() { printf '[%s] 警告: %s\n' "$(date '+%H:%M:%S')" "$*" >&2; }
die()  { printf '[%s] 失败: %s\n' "$(date '+%H:%M:%S')" "$*" >&2; exit 1; }

# ------------------------------------------------------------------ 环境
load_env() {
  # `set -u` 下 source ROS 的 setup.bash 会因为里面引用未定义变量而**静默退出** ——
  # 一个字都不打印，看起来像后面的探针坏了。所以先关 -u，source 完再开回来。
  set +u
  # shellcheck disable=SC1090
  [ -f "$ROS_DISTRO_SETUP" ] && source "$ROS_DISTRO_SETUP"
  if [ -z "$WS_SETUP" ]; then
    for cand in "$PWD/ws_robot/install/setup.bash" \
                "$PWD/../install/setup.bash" \
                "$HOME/WorkSpace/astribot_sdk_ros2/ws_robot/install/setup.bash"; do
      [ -f "$cand" ] && WS_SETUP="$cand" && break
    done
  fi
  # shellcheck disable=SC1090
  [ -n "$WS_SETUP" ] && [ -f "$WS_SETUP" ] && source "$WS_SETUP"
  set -u
  export ROS_DOMAIN_ID="$DOMAIN_ID"
  # 只走本机：实测只有 ROS_LOCALHOST_ONLY 有效，
  # Fast DDS 的 interfaceWhiteList XML 会静默把它抵消掉。
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
  log "环境: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
  log "  ws setup: ${WS_SETUP:-<未找到，将只用 /opt/ros>}"
  [ -x "$ROS2_BIN" ] || die "找不到 $ROS2_BIN（厂商的 ros2 是子集，不能用）"
}

# ------------------------------------------------------------------ 清理
cleanup_stack() {
  log '清理残留进程与共享内存'
  # 三件事都必须做，只做前两件会留下伪装成"栈没起来"的残留：
  #   · 工作空间路径匹配不到 /opt/ros/humble 下的二进制（nav2_*、parameter_bridge）
  #   · 陈旧 daemon 会让话题数从 80 掉到 2
  #   · sem.fastrtps_* 前缀实测还会剩几十个
  # 注意 pkill 的模式不能出现在本脚本自己的命令行里，否则会杀掉自己的 shell。
  local pats='nav2_|controller_server|planner_server|bt_navigator|behavior_server'
  pats="$pats|velocity_smoother|lifecycle_manager|explore_metrics_recorder"
  pkill -f "$pats" 2>/dev/null || true
  sleep 2
  pkill -9 -f "$pats" 2>/dev/null || true
  "$ROS2_BIN" daemon stop >/dev/null 2>&1 || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  sleep 1
}

# ------------------------------------------------------------------ 就绪判据
lifecycle_active() {
  # 返回 0 表示全部 ACTIVE。逐个查 get_state，不数进程。
  local node st
  for node in $LIFECYCLE_NODES; do
    st="$("$ROS2_BIN" service call "/$node/get_state" \
          lifecycle_msgs/srv/GetState '{}' 2>/dev/null \
          | grep -o "label='[a-z]*'" | head -1)"
    case "$st" in
      *active*) ;;
      *) printf '%s=%s ' "$node" "${st:-无响应}"; return 1 ;;
    esac
  done
  return 0
}

wait_nav2_active() {
  local deadline=$((SECONDS + NAV2_READY_TIMEOUT)) missing
  while [ "$SECONDS" -lt "$deadline" ]; do
    missing="$(lifecycle_active)" && { log 'nav2 全部 ACTIVE'; return 0; }
    log "  等待 ACTIVE ... 未就绪: $missing"
    sleep 5
  done
  warn "nav2 在 ${NAV2_READY_TIMEOUT}s 内没有全部 ACTIVE：$missing"
  warn '  注意：进程可能全都活着 —— lifecycle_manager 报 Aborting bringup 时'
  warn '  9 个进程照样在，所以绝不能用进程数当就绪判据。'
  return 1
}

# 限速实参核对：**枚举**所有 *.vx_max 逐个查，不硬编码键名。
#
# 曾硬编码 `FollowPath.vx_max`。2026-09-07 起 FollowPath 是三段式控制器，
# MPPI 的限速键下移到了 `<实例>.inner.vx_max`（three_phase_controller.cpp:246
# 用 name_ + ".inner" 配置内层），老键**根本不存在** —— 判据于是印
# 「查不到 FollowPath.vx_max」，看着像 controller_server 没就绪，
# 实际是判据在查一个不存在的键。
# 枚举法结构再变也不会误判，而且比只查一个键更强：三个控制器实例的 vx_max
# 是被 RewrittenYaml 同时改写的（navigation.launch.py 的 param_rewrites 按
# **键名**全树替换），任何一个没跟上就说明限速没全落地。
# 守卫必不可少：一个键都没枚举到时绝不能算通过 —— 0 个键的「全部一致」是空真。
list_vx_max_keys() {
  "$ROS2_BIN" param list /controller_server 2>/dev/null \
    | tr -d ' ' | grep -E '\.vx_max$'
}

verify_cap() {
  local want="$1" keys got bad=0 n=0
  keys="$(list_vx_max_keys)"
  [ -n "$keys" ] || {
    warn '一个 *.vx_max 参数都没枚举到（controller_server 未就绪？）'
    warn '  这一条不算通过：0 个键时"全部一致"恒成立，等于没有判据。'
    return 1
  }
  while read -r k; do
    [ -n "$k" ] || continue
    n=$((n + 1))
    got="$("$ROS2_BIN" param get /controller_server "$k" 2>/dev/null \
           | grep -oE '[-0-9]+\.?[0-9]*' | head -1)"
    if [ -z "$got" ]; then
      warn "  $k 查不到值"; bad=1; continue
    fi
    if awk -v a="$got" -v b="$want" 'BEGIN{exit (a-b<1e-6 && b-a<1e-6)?0:1}'; then
      log "  $k = $got == 请求 $want"
    else
      warn "  $k = $got 与请求 $want **不一致**"; bad=1
    fi
  done <<EOF
$keys
EOF
  [ "$bad" -eq 0 ] && { log "实测限速全部一致（$n 个 *.vx_max 均为 $want）"; return 0; }
  warn "本档位不采数：$n 个 *.vx_max 中存在不一致项"
  warn '  最常见原因：外层 launch 没有把 max_linear_speed 转发给'
  warn '  navigation.launch.py（launch_arguments 是白名单，漏项不报错不告警）'
  return 1
}

# ------------------------------------------------------------------ 授权确认
confirm_enable() {
  local cap="$1"
  if [ "$OBSERVE_ONLY" = 'true' ] || [ "$ENABLE_WRITE" != 'true' ]; then
    log '只观测模式：不使能写通路，机器人不会动。'
    log '  （要真跑请显式 ENABLE_WRITE=true OBSERVE_ONLY=false）'
    return 1
  fi
  cat <<EOF

============================================================
即将在限速档位 ${cap} m/s 下**使能写通路并让机器人自主移动**。

  · 运动由探索协调器自主决定，方向与距离在运行前不可预知
  · 线速度上限: ${cap} m/s（实测值已核对）
  · 请确认：机器人前方无人、急停按钮在手、周围留出余量

技术前置条件（无速度帧、电量充足）**不构成授权**。
============================================================
EOF
  printf '输入 YES 继续，其它任何输入都会跳过本档位: '
  local ans; read -r ans || true
  [ "$ans" = 'YES' ] || { warn "未获授权，跳过档位 $cap"; return 1; }
  log "已获授权：档位 $cap"
  return 0
}

# ------------------------------------------------------------------ 单档位
run_one_cap() {
  local cap="$1"
  local label; label="cap$(printf '%s' "$cap" | tr -d '.')"
  local out="$OUT_ROOT/$label"
  mkdir -p "$out"
  local logdir="$out/logs"; mkdir -p "$logdir"

  log "================ 档位 $cap m/s ================"
  cleanup_stack

  log '启动 nav2 + 感知/SLAM + 探索协调器'
  # max_linear_speed 必须显式传：它同时压住 vx_max/vy_max/vx_min/vy_min。
  # enable_posture_monitor 在 env:=real 下默认就是 false，这里再显式写一遍 ——
  # 它是"实机被永久归零 /cmd_vel"的开关，不能靠默认值的正确性。
  # exploration:=true 让协调器自主选点（本脚本不自己发目标 ——
  # 客户端周期性重发目标会自激出"重规划造失败、失败触发重试、重试再抢占"）。
  # 只用 use_rviz:=false，不传 nav2_rviz_flag：后者是本文件内部
  # SetLaunchConfiguration 出来的副本，命令行传进去也会被覆盖。
  nohup "$ROS2_BIN" launch astribot_s1_navigation nav2_full_bringup.launch.py \
      env:=real mode:=mapping controller_plugin:=mppi \
      exploration:=true use_rviz:=false \
      enable_posture_monitor:=false \
      max_linear_speed:="$cap" \
      > "$logdir/nav2.log" 2>&1 &
  local nav2_pid=$!
  log "  nav2 pid=$nav2_pid  日志 $logdir/nav2.log"

  log "等待 DDS 发现收敛（${DISCOVERY_WAIT}s）—— 实机实测要 30~50s"
  sleep "$DISCOVERY_WAIT"

  if ! wait_nav2_active; then
    warn "档位 $cap：nav2 未就绪，跳过。看 $logdir/nav2.log"
    kill "$nav2_pid" 2>/dev/null || true
    return 1
  fi
  if ! verify_cap "$cap"; then
    kill "$nav2_pid" 2>/dev/null || true
    return 1
  fi

  log '启动录制器（只读，零发布）'
  local rec_args=(
    --ros-args
    -p "output_dir:=$out"
    -p "run_label:=$label"
    -p "speed_cap_requested:=$cap"
    -p "collisions_manual:=$COLLISIONS_MANUAL"
    -p 'use_sim_time:=false'
  )
  [ -n "$LOW_OBSTACLE_TRUTH" ] && rec_args+=(-p "low_obstacle_truth_file:=$LOW_OBSTACLE_TRUTH")
  nohup "$ROS2_BIN" run astribot_s1_navigation explore_metrics_recorder_node \
      "${rec_args[@]}" > "$logdir/recorder.log" 2>&1 &
  local rec_pid=$!
  log "  录制器 pid=$rec_pid  日志 $logdir/recorder.log"

  if confirm_enable "$cap"; then
    log '写通路已获授权 —— 请按 runbook §10.6/§10.7 完成取控制权与使能，'
    log '完成后回到本终端按回车继续计时。'
    read -r _ || true
  fi

  # 收工条件：轮数达标 优先于 时长
  local deadline=$((SECONDS + MINUTES_PER_CAP * 60))
  local rounds=0
  while [ "$SECONDS" -lt "$deadline" ]; do
    sleep 15
    if [ -f "$out/${label}_rounds.csv" ]; then
      rounds=$(( $(wc -l < "$out/${label}_rounds.csv") - 1 ))
      [ "$rounds" -lt 0 ] && rounds=0
    fi
    log "  档位 $cap：已录 $rounds 轮，剩余 $(( (deadline - SECONDS) / 60 )) 分钟"
    if [ "$ROUNDS_PER_CAP" -gt 0 ] && [ "$rounds" -ge "$ROUNDS_PER_CAP" ]; then
      log "  已达目标轮数 $ROUNDS_PER_CAP，提前收工"
      break
    fi
  done

  # SIGINT 让录制器走 finish()：把在途的那一轮也写出去
  log '停止录制器（SIGINT，让它把在途轮次落盘）'
  kill -INT "$rec_pid" 2>/dev/null || true
  sleep 5
  kill -9 "$rec_pid" 2>/dev/null || true
  kill -INT "$nav2_pid" 2>/dev/null || true
  sleep 5
  cleanup_stack

  if [ ! -f "$out/${label}_rounds.csv" ]; then
    warn "档位 $cap 一轮都没录到。先看 $logdir/recorder.log 的自检段："
    warn '  它会打印每个话题的实测消息数与上游真实 QoS。'
    warn '  零消息 + 有发布者 = QoS 单向不兼容（这种情形只有一条 WARNING）。'
    return 1
  fi
  log "档位 $cap 完成：$rounds 轮 -> $out/${label}_rounds.csv"
  return 0
}

# ------------------------------------------------------------------ main
main() {
  load_env
  mkdir -p "$OUT_ROOT"
  log "扫描档位: $CAPS"
  log "输出根目录: $OUT_ROOT"
  if [ "$OBSERVE_ONLY" = 'true' ]; then
    log '模式: **只观测**（不使能写通路，机器人不动）。'
    log '  这个模式仍然有用：它验证整条录制管线通不通，'
    log '  且能测出各话题的实测拍率与 QoS 匹配情况。'
  else
    warn '模式: 允许使能写通路 —— 每个档位都会停下来要求你确认。'
  fi

  local ok=0 fail=0 dirs=()
  for cap in $CAPS; do
    if run_one_cap "$cap"; then
      ok=$((ok + 1))
      dirs+=("$OUT_ROOT/cap$(printf '%s' "$cap" | tr -d '.')")
    else
      fail=$((fail + 1))
    fi
  done

  log "================ 汇总 ================"
  log "成功 $ok 档，失败/跳过 $fail 档"
  if [ "${#dirs[@]}" -eq 0 ]; then
    die '没有任何档位产出数据 —— 不出表（空表会被当成"测过了、没差异"）'
  fi

  local agg
  agg="$(dirname "$0")/aggregate_speed_sweep.py"
  [ -f "$agg" ] || agg="$(command -v aggregate_speed_sweep.py || true)"
  if [ -n "$agg" ] && [ -f "$agg" ]; then
    python3 "$agg" "${dirs[@]}" --out-prefix "$OUT_ROOT/sweep"
    local rc=$?
    log "汇总退出码 $rc（1 = 有 UNUSABLE 档位，2 = 一个档位都读不到）"
    log "看这两个文件: $OUT_ROOT/sweep.md  $OUT_ROOT/sweep.csv"
    return $rc
  fi
  warn "找不到 aggregate_speed_sweep.py，请手动汇总: ${dirs[*]}"
  return 0
}

main "$@"
