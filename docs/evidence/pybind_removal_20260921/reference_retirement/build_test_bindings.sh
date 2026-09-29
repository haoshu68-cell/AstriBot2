#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/local_setup.bash
TASK_DIR=/tmp/codex_reference_retirement_20260921
ROOT=/home/yjh/WorkSpace/astribot_sdk_ros2
cmake -S "$ROOT/ws_robot/src/astribot_s1_robot_geometry" -B "$TASK_DIR/geometry_build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$TASK_DIR/install" -DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=ON -DBUILD_TESTING=ON
cmake --build "$TASK_DIR/geometry_build" -j2
cmake --install "$TASK_DIR/geometry_build"
source "$TASK_DIR/install/share/astribot_s1_robot_geometry/local_setup.bash"
for spec in 'astribot_s1_navigation_policy_native:_navigation_math_native' 'astribot_trajectory_bridge_native:_chassis_math_native'; do
  PACKAGE="${spec%%:*}"
  TARGET="${spec#*:}"
  cmake -S "$ROOT/ws_robot/src/$PACKAGE" -B "$TASK_DIR/$PACKAGE" -DCMAKE_BUILD_TYPE=Release -DASTRIBOT_BUILD_PYBIND=ON -DBUILD_TESTING=ON
  cmake --build "$TASK_DIR/$PACKAGE" --target "$TARGET" -j2
  mkdir -p "$TASK_DIR/bindings/$PACKAGE"
  cp "$ROOT/ws_robot/src/$PACKAGE/$PACKAGE/__init__.py" "$TASK_DIR/bindings/$PACKAGE/"
  cp "$TASK_DIR/$PACKAGE"/"$TARGET"*.so "$TASK_DIR/bindings/$PACKAGE/"
done
