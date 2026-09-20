#!/usr/bin/env bash
# Source after ROS Humble. Local dependencies; MTC core builds against installed MoveIt.
_astribot_mtc_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_astribot_mtc_root="$_astribot_mtc_repo/ws_robot/deps/mtc_humble"
if [[ ! -f "$_astribot_mtc_root/opt/ros/humble/share/moveit_task_constructor_msgs/package.xml" ]]; then
    mkdir -p "$_astribot_mtc_root/downloads"
    (set -e; cd "$_astribot_mtc_root/downloads"
     apt download ros-humble-moveit-task-constructor-msgs=0.1.3-1jammy.20260908.071527 \
       ros-humble-rviz-marker-tools=0.1.3-1jammy.20260908.012414 \
       ros-humble-py-binding-tools=2.0.1-1jammy.20260907.235014
     for package in ./ros-humble-moveit-task-constructor-msgs_*.deb ./ros-humble-rviz-marker-tools_*.deb ./ros-humble-py-binding-tools_*.deb; do
         dpkg-deb -x "$package" "$_astribot_mtc_root"
     done) || return 1
fi
_astribot_mtc_prefix="$_astribot_mtc_root/opt/ros/humble"
export AMENT_PREFIX_PATH="$_astribot_mtc_prefix:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$_astribot_mtc_prefix:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$_astribot_mtc_prefix/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$_astribot_mtc_prefix/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
if [[ ! -f "$_astribot_mtc_repo/ws_robot/deps/mtc_install/moveit_task_constructor_core/lib/libmoveit_task_constructor_core.so" ]]; then
    (
      set -e
      cd "$_astribot_mtc_repo"
      if [[ ! -d ws_robot/deps/mtc_source/.git ]]; then
          git clone --branch humble https://github.com/moveit/moveit_task_constructor.git ws_robot/deps/mtc_source
          git -C ws_robot/deps/mtc_source checkout 756634951326ae17ae099882f7110c6f1d0a98c0
      fi
      CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4 colcon build --base-paths ws_robot/deps/mtc_source/core \
        --build-base ws_robot/deps/mtc_build --install-base ws_robot/deps/mtc_install \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
    ) || return 1
fi
# Generated colcon hooks reference optional variables and do not support nounset.
_astribot_mtc_restore_nounset=0
[[ $- == *u* ]] && _astribot_mtc_restore_nounset=1
set +u
source "$_astribot_mtc_repo/ws_robot/deps/mtc_install/local_setup.bash"
_astribot_mtc_setup_status=$?
if [[ $_astribot_mtc_restore_nounset == 1 ]]; then set -u; fi
if [[ $_astribot_mtc_setup_status != 0 ]]; then return "$_astribot_mtc_setup_status"; fi
unset _astribot_mtc_restore_nounset _astribot_mtc_setup_status
_astribot_mtc_built="$_astribot_mtc_repo/ws_robot/deps/mtc_install/moveit_task_constructor_core"
export AMENT_PREFIX_PATH="$_astribot_mtc_built:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$_astribot_mtc_built:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$_astribot_mtc_built/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$_astribot_mtc_built/local/lib/python3.10/dist-packages:$PYTHONPATH"
unset _astribot_mtc_built
unset _astribot_mtc_repo _astribot_mtc_root _astribot_mtc_prefix
