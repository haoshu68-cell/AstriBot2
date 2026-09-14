#!/usr/bin/env bash
set -euo pipefail

TOOLKIT="/home/astribot/chassis_tests/toolkit_20260914_parking_v3"
CONFIG_DIR="/home/astribot/chassis_tests/configs"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_ROOT=$(mktemp -d "/home/astribot/chassis_tests/runs/response_braking_suite_${STAMP}_XXXXXX")
LOG_FILE="${RUN_ROOT}/suite.log"
MISSING=()

if [ ! -x "${TOOLKIT}/tools/robot/chassis_characterization.py" ]; then
  echo "[ERR] 找不到执行脚本: ${TOOLKIT}/tools/robot/chassis_characterization.py" >&2
  exit 2
fi

mkdir -p "$RUN_ROOT"

echo "[INFO] 套件目录: $RUN_ROOT" | tee "$LOG_FILE"

echo "[INFO] 开始加载环境" | tee -a "$LOG_FILE"
set +u
source "$TOOLKIT/env_test.sh"
set -u

run_case() {
  local mode="$1"
  local direction="$2"
  local cfg="$3"

  if [ ! -f "$cfg" ]; then
    echo "[WARN] 配置不存在: $cfg" | tee -a "$LOG_FILE"
    MISSING+=("$cfg")
    return 0
  fi

  local dir="$RUN_ROOT/${mode}_${direction}"
  rm -rf "$dir"
  echo "\n[INFO] ===== ${mode} ${direction} =====" | tee -a "$LOG_FILE"
  echo "[INFO] 配置: $cfg" | tee -a "$LOG_FILE"
  echo "[INFO] 输出: $dir" | tee -a "$LOG_FILE"
  echo "[INFO] 停车文件: $dir/STOP" | tee -a "$LOG_FILE"

  local start_ts
  start_ts=$(date +%Y%m%d_%H%M%S)
  set +e
  python3 "$TOOLKIT/tools/robot/chassis_characterization.py" \
    --config "$cfg" \
    --output "$dir" \
    --execute \
    2>&1 | tee "$dir/run.log"
  local ec=${PIPESTATUS[0]}
  set -e

  local status
  status="FAILED"
  [ "$ec" -eq 0 ] && status="COMPLETED"

  {
    echo "{\"case\": \"${mode}/${direction}\", \"status\": \"$status\", \"start\": \"$start_ts\", \"return_code\": ${ec}}"
  } >> "$RUN_ROOT/suite_status.jsonl"

  echo "[INFO] ${mode}/${direction} return_code=$ec (status=$status)" | tee -a "$LOG_FILE"
  return 0
}

# 统一六向顺序：x+ x- y+ y- yaw+ yaw-
for mode in response braking; do
  for direction in x_pos x_neg y_pos y_neg yaw_pos yaw_neg; do
    run_case "$mode" "$direction" "$CONFIG_DIR/chassis_characterization_${mode}_${direction}.json"
  done
done

echo "\n[INFO] 全部执行完成。" | tee -a "$LOG_FILE"
echo "[INFO] 套件目录: $RUN_ROOT" | tee -a "$LOG_FILE"

echo "\n==== 全量状态汇总 ====" | tee -a "$LOG_FILE"
cat "$RUN_ROOT/suite_status.jsonl" | tee -a "$LOG_FILE"

if [ ${#MISSING[@]} -gt 0 ]; then
  echo "\n[WARN] 缺失配置：" | tee -a "$LOG_FILE"
  printf '%s\n' "${MISSING[@]}" | tee -a "$LOG_FILE"
  exit 3
fi

exit 0
