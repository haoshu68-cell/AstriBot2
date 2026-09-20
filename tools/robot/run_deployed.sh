#!/usr/bin/env bash
set -eo pipefail
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
ROLE="${1:-precision}"
if [ "$#" -gt 0 ]; then shift; fi
case "$ROLE" in
    start|restart) ROLE=precision ;;
    precision|explore|sensors|check|navigation|bridge|rviz) ;;
    stop) exec bash "$PROJECT/tools/robot/stop_robot_tasks.sh" "$@" ;;
    --help|-h)
        echo "用法: bash $0 [precision|explore|start|restart|sensors|stop|check|navigation|bridge|rviz]"
        echo '默认 precision：重启感知和导航、使能底盘后等待人工目标；explore 自动探索；sensors 只重启感知链。'
        echo 'precision/explore/sensors --dry-run 仅打印计划。'
        exit 0 ;;
    *) echo "未知模式: $ROLE（使用 --help 查看）" >&2; exit 2 ;;
esac
if [[ "$ROLE" != precision && "$ROLE" != explore && "$ROLE" != sensors ]] && [ "$#" -gt 0 ]; then
    echo "模式 $ROLE 不接受额外参数；precision/explore/sensors 支持 --dry-run。" >&2; exit 2
fi
if [ "$ROLE" = sensors ]; then set -- --sensors-only "$@"; fi
if [ "$ROLE" = precision ]; then set -- --navigation-only "$@"; fi
if [[ "$ROLE" = precision || "$ROLE" = explore || "$ROLE" = sensors ]] && [[ " $* " = *' --dry-run '* ]]; then
    exec python3 "$PROJECT/tools/robot/hardware_exploration.py" "$@"
fi
source "$PROJECT/tools/robot/env_deployed.sh"
case "$ROLE" in
    precision|explore|sensors) exec python3 "$PROJECT/tools/robot/hardware_exploration.py" "$@" ;;
    check)
        exec python3 "$PROJECT/tools/robot/hardware_readiness.py" ;;
    navigation)
        python3 "$PROJECT/tools/robot/hardware_readiness.py" --require-idle-navigation
        COMMAND=(ros2 launch astribot_s1_navigation navigation.launch.py
            use_sim_time:=false arrival_precision_profile:=hardware
            navigation_policy_stage:=off controller_plugin:=mppi
            map_topic:=/map map_transient_local:=true scan_topic:=/scan_from_cloud
            enable_posture_monitor:=false enable_arm_chassis_coupling:=true
            max_linear_speed:=0.35 autostart:=true) ;;
    bridge)
        python3 "$PROJECT/tools/robot/hardware_readiness.py" --require-idle-bridge
        # Keep the two node names distinct so chassis and arm authorization stay separate.
        COMMAND=(ros2 run astribot_trajectory_bridge bridge_container --ros-args
            --params-file "$PROJECT/tools/robot/config/deployed_chassis.yaml") ;;
    rviz)
        CONFIG="$PROJECT/ws_robot/install/astribot_s1_navigation/share/astribot_s1_navigation/rviz/nav2_view.rviz"
        if [ ! -f "$CONFIG" ]; then
            echo "RViz 配置不存在: $CONFIG" >&2; exit 1
        fi
        COMMAND=(rviz2 -d "$CONFIG" --ros-args -p use_sim_time:=false) ;;
esac
mkdir -p "$HOME/.ros/log/astribot/hardware"
SESSION="$(mktemp -d "$HOME/.ros/log/astribot/hardware/${ROLE}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
export ASTRIBOT_LOG_DIR="$SESSION" ROS_LOG_DIR="$SESSION" ASTRIBOT_LOG_CAPTURE=1
ln -sfn "$SESSION" "$HOME/.ros/log/astribot/latest_hardware_${ROLE}"
printf '角色: %s\n日志: %s/session.log\n' "$ROLE" "$SESSION"
printf '%s\n' "$PROJECT" > "$SESSION/project.txt"
"${COMMAND[@]}" 2>&1 | tee "$SESSION/session.log"
