# Bounded Gazebo model request, simulation candidate

The upstream gz_ros2_control 0.7.20 getURDF method calls future.wait() without a
request deadline. A lost first parameter response leaves Gazebo Configure blocked
indefinitely, even when a later independent reader can retrieve the model.

This C++ helper uses a 10-second steady-clock total deadline and 1-second attempts.
Timed-out requests are removed; late replies are not recycled. Missing/empty/wrong
type parameters reject setup. It only reads the model; control update/read/write,
joint limits, controller configuration and execution ownership are unchanged.

Reproduction archive: runs/task_chain_20260922/dependencies/gz_ros2_control-0.7.20.tar.gz
Upstream source: https://github.com/ros-controls/gz_ros2_control/tree/0.7.20
Apply tools/vision/patches/gz_ros2_control_0_7_20_bounded_urdf.patch with -p1 at the
extracted repository root, and copy bounded_urdf.hpp to gz_ros2_control/src/.
Build that single package with colcon --base-paths pointing to gz_ros2_control,
--cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo, and isolated
build/install/log roots. Never install this candidate over /opt/ros/humble.
The recorded source archive, helper and patch are all required to reproduce it.

The standalone CMake project in this directory tests delayed reply/retry, missing
service, wrong type, and normal reply using real ROS clients/services. It does not
command a robot. Scope and real Gazebo loaded-library evidence are recorded in
docs/evidence/task_chain_20260922. Simulation validation does not promote this
candidate to hardware use or certify arbitrary library combinations.
