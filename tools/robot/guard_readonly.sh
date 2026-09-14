#!/usr/bin/env bash
set -uo pipefail

WS=/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot
LOGDIR="${ASTRIBOT_LOG_DIR:-${ROS_LOG_DIR:-${HOME}/.ros/log/astribot/hardware}}"
export ASTRIBOT_LOG_DIR="$LOGDIR" ROS_LOG_DIR="$LOGDIR"
GUARD_LOG="$LOGDIR/guard.log"
STAGES=/tmp/bringup_stages.sh

INTERVAL="${GUARD_INTERVAL:-10}"

MAX_RESTARTS="${GUARD_MAX_RESTARTS:-5}"
WINDOW="${GUARD_WINDOW:-600}"

GUARDED=(
  "lib/astribot_trajectory_bridge/state_bridge_node|3|状态桥(只读)"
  "lib/astribot_trajectory_bridge/chassis_odom_node|4|底盘里程计(只读)"
)

FORBIDDEN=("bridge_container" "arm_traj_bridge" "chassis_cmd_bridge" "gripper")
for entry in "${GUARDED[@]}"; do
    pat="${entry%%|*}"
    for bad in "${FORBIDDEN[@]}"; do
        if [[ "$pat" == *"$bad"* ]]; then
            echo "[guard][FATAL] 白名单里出现写通路进程：$pat" >&2
            echo "[guard][FATAL] 写通路在急停后自动重启会绕过人按急停的意图。" >&2
            echo "[guard][FATAL] 恢复写通路请人工确认后单独执行阶段⑥。" >&2
            exit 2
        fi
    done
done

mkdir -p "$LOGDIR"

log() { echo "[$(date '+%F %T')] $*" | tee -a "$GUARD_LOG"; }

SELF_NAME="$(basename "${BASH_SOURCE[0]}")"
proc_count() {
    local pat=$1 snap n
    snap=$(mktemp)
    ps -eo pid,args > "$snap" 2>/dev/null
    n=$(grep -- "$pat" "$snap" 2>/dev/null \
        | awk -v me="$$" -v parent="$PPID" '$1 != me && $1 != parent' \
        | grep -v -- "$SELF_NAME" \
        | grep -v -E '(^|[[:space:]])(grep|ps)([[:space:]]|$)' \
        | wc -l)
    rm -f "$snap"
    echo "${n:-0}"
}

driver_alive() {
    python3 - <<'PY' 2>/dev/null
import sys, time
try:
    import rclpy
    from rclpy.node import Node
except Exception:
    sys.exit(3)          # 环境不对，当作"未知"，不阻止重启
rclpy.init()
n = Node('guard_driver_probe')
# 实机 DDS 发现要 30~50s 收敛；这里给 35s。
t0 = time.time()
alive = False
while time.time() - t0 < 35.0:
    if len(n.get_publishers_info_by_topic(
            '/astribot_error_code/control_driver')) > 0:
        alive = True
        break
    time.sleep(1.0)
rclpy.shutdown()
sys.exit(0 if alive else 1)
PY
}

declare -A RESTART_COUNT RESTART_WINDOW_START GAVE_UP

log "守护启动：巡检 ${INTERVAL}s，窗口 ${WINDOW}s 内最多重启 ${MAX_RESTARTS} 次"
log "守护对象（**不含写通路**）："
for entry in "${GUARDED[@]}"; do
    log "    - ${entry##*|}"
done

while true; do
    for entry in "${GUARDED[@]}"; do
        pat="${entry%%|*}"
        rest="${entry#*|}"
        stage="${rest%%|*}"
        name="${rest#*|}"

        [ "$(proc_count "$pat")" -gt 0 ] && continue
        [ "${GAVE_UP[$pat]:-0}" = "1" ] && {
            log "[$name] 仍未运行，但已放弃自动重启（需人工介入）"
            continue
        }

        now=$(date +%s)
        start=${RESTART_WINDOW_START[$pat]:-0}
        if [ $((now - start)) -gt "$WINDOW" ]; then
            RESTART_WINDOW_START[$pat]=$now
            RESTART_COUNT[$pat]=0
        fi

        cnt=${RESTART_COUNT[$pat]:-0}
        if [ "$cnt" -ge "$MAX_RESTARTS" ]; then
            GAVE_UP[$pat]=1
            log "[$name] ${WINDOW}s 内已重启 ${cnt} 次仍未稳定，放弃。"
            log "[$name] 请先确认厂商控制驱动：/astribot_error_code/control_driver 发布者是否 >0"
            continue
        fi

        log "[$name] 未运行。先确认厂商控制驱动是否已恢复..."
        if ! driver_alive; then
            log "[$name] 控制驱动仍未恢复（control_driver 发布者=0），本轮不重启。"
            log "         最可能原因：物理急停仍处于按下状态 —— 这需要人来松开。"
            continue
        fi

        RESTART_COUNT[$pat]=$((cnt + 1))
        log "[$name] 控制驱动已恢复，执行阶段 ${stage} 重启（第 ${RESTART_COUNT[$pat]} 次）"
        if [ -x "$STAGES" ] || [ -f "$STAGES" ]; then
            bash "$STAGES" "$stage" >> "$LOGDIR/guard_stage${stage}.log" 2>&1 \
                && log "[$name] 阶段 ${stage} 返回成功" \
                || log "[$name] 阶段 ${stage} 返回失败，详见 guard_stage${stage}.log"
        else
            log "[$name] 找不到 $STAGES，无法重启"
        fi
    done
    sleep "$INTERVAL"
done
