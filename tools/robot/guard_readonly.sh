#!/usr/bin/env bash
# =====================================================================
# 只读 SDK 进程的守护（**非对称**：写通路绝不在此列）
#
# 为什么需要它 —— 2026-09-01 21:26 实测事件
# ========================================
# 按下物理急停 → 厂商控制驱动停 → 我们 3 个持有 SDK 会话的进程在**同一秒**
# 全部退出，死因一致：
#     Driver heartbeat timeout detected! Count: 5/5
#     Driver crashes. Reached timeout threshold (5). Exiting...
# 三者引用同一个 Last error_code_timestamp，即同一个上游事件。
#
# 这三个进程退出本身是**正确**的（fail-fast）。缺陷是**没有一个会自己回来**：
# 急停是正常操作事件，而它每次都要求人工恢复整栈。
#
# 连带后果（逐环实测）：
#   /joint_states 0Hz → robot_state_publisher 停发连杆 TF
#     → 自滤逐连杆等满超时 → /scan 掉到 0.56Hz、龄期 2.03s
#       → nav2 照常规划、照常发速度，零告警
#
# ═══════════ 为什么写通路（bridge_container）不在守护列表里 ═══════════
# 急停之后让写通路自动回来，等于**绕过人按急停这个动作的意图**。
# 按急停是一个明确的"停止执行"指令；一个自动重启的写通路会在人松开急停时
# 悄悄恢复下发能力，而人并不知道。
#
# 所以本脚本在**结构上**排除它：白名单里根本没有它，而且下面还有一道
# 显式断言 —— 万一将来有人往 GUARDED 里加了写通路进程，脚本直接拒绝启动，
# 不靠注释提醒，也不靠人记得。
#
# 恢复写通路的正确方式：人工确认后单独执行阶段⑥（见 bringup_stages.sh）。
# =====================================================================
set -uo pipefail

WS=/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot
LOGDIR=/tmp/bringup
GUARD_LOG="$LOGDIR/guard.log"
STAGES=/tmp/bringup_stages.sh

# 巡检周期(s)。取 10s 的理由：SDK 看门狗从心跳丢失到退出约 1.2s，
# 而厂商控制驱动恢复本身要数十秒；巡检快于恢复速度只会白重启。
INTERVAL="${GUARD_INTERVAL:-10}"

# 单个进程在 WINDOW 秒内最多重启 MAX_RESTARTS 次，超过就放弃并持续告警。
# 为什么要放弃：控制驱动没回来时重启必然在 1.2s 内再死，无限重启会把
# 真实故障刷成一片重启日志，反而更难查。
MAX_RESTARTS="${GUARD_MAX_RESTARTS:-5}"
WINDOW="${GUARD_WINDOW:-600}"

# ─────────────── 白名单：只读进程 ───────────────
# 格式：<进程匹配模式>|<阶段号>|<人类可读名>
# 阶段号复用 bringup_stages.sh，避免这里再抄一份启动命令 ——
# 抄一份就会与那边漂移，而漂移的那份是"恢复"路径，最不该出错。
GUARDED=(
  "lib/astribot_trajectory_bridge/state_bridge_node|3|状态桥(只读)"
  "lib/astribot_trajectory_bridge/chassis_odom_node|4|底盘里程计(只读)"
)

# ─────────── 结构性防呆：写通路绝不允许进白名单 ───────────
# 这不是注释提醒，是**启动期硬断言**。
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

# 进程计数。
#
# !!! 这个函数有两个独立的自匹配来源，都实测踩过 !!!
#   ① 同一条命令里既写模式又读 ps → `ps -eo args` 抓到模式串本身
#   ② **调用方**的命令行里含模式串 → 快照里有调用方那一行
# ②比①更隐蔽：即使先落快照再读模式，只要外层 shell 的 argv 含模式就照样中招。
# 实测：`proc_count 'zzz_definitely_not_running_zzz'` 对一个根本不存在的进程
# 返回 **1** —— 而且偏差方向是"永远只会高"，正好让守护误判"进程还在"而不去
# 重启。这是最坏的方向。
#
# 所以这里显式排除：本进程、父进程、命令行含本脚本名的进程、以及 grep/ps 自身。
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

# 厂商控制驱动是否活着。判据是**话题发布者数**，不是进程在不在 ——
# 实测过"进程活着而话题 pub 恒 0"（Fast DDS endpoint 级发现失效，已记录未解决）。
# 控制驱动没回来时重启我们的进程毫无意义：它们会在 1.2s 内以同样原因再退出。
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

        # !!! 计"窗口内重启次数"，不是"总次数" !!!
        # 用总次数的话，机器人跑一整天累计到上限就永久放弃，之后每次真实
        # 故障都不再自愈。本项目在重试上限那条上已经栽过同一个错
        # （把成功也计入，误杀了完全走得通的长路径）。
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
