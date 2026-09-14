# Shared by the colcon overlay and SDK env.sh. No processes or ROS contexts started.
export ASTRIBOT_LOG_DIR="${ASTRIBOT_LOG_DIR:-${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log/astribot}}"
export ROS_LOG_DIR="$ASTRIBOT_LOG_DIR"
export ASTRIBOT_LOG_LEVEL="${ASTRIBOT_LOG_LEVEL:-info}"
export ASTRIBOT_LOG_MAX_BYTES="${ASTRIBOT_LOG_MAX_BYTES:-10485760}"
export ASTRIBOT_LOG_BACKUP_COUNT="${ASTRIBOT_LOG_BACKUP_COUNT:-5}"
# {time} is supported by stock Humble; date_time_with_ms is not portable to it.
if [ -z "${RCUTILS_CONSOLE_OUTPUT_FORMAT:-}" ]; then
  export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'
fi
export RCUTILS_LOGGING_USE_STDOUT="${RCUTILS_LOGGING_USE_STDOUT:-0}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-0}"
