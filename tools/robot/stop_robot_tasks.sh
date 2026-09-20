#!/usr/bin/env bash
set -eo pipefail
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
if [[ "${1:-}" == --help ]]; then
    echo "用法: bash $0 [--dry-run] [--session /绝对路径/会话目录]"
    echo '先停车并关闭导航/探索，再关闭 SLAM、雷达及关联感知；保留厂家本体驱动和机器人模型。'
    exit 0
fi
source "$PROJECT/tools/robot/env_deployed.sh"
exec python3 "$PROJECT/tools/robot/robot_task_control.py" "$@"
