# Trajectory bridge native cores

`bridge_runtime` is an ordinary C++17 static library. It contains the chassis
controller, arm trajectory executor, waypoint dispatcher and gripper controller.
Their configuration, events, results, clock, pose and session interfaces contain
no Python types. `chassis_math` remains the shared numerical library.

The exported headers are `runtime_ports.hpp`, `chassis_controller.hpp`,
`arm_executor.hpp` and `gripper_controller.hpp`. A caller must keep its ports
alive longer than the controllers. The host retains write admission, the single
SDK session and control rights, and serializes each chassis/arm controller's
callbacks with the existing control/goal locks. The gripper retains its internal
per-effector busy gate. The cores do not open a device, claim control rights or
publish hardware commands by themselves.

The three `*_pybind.cpp` adapters currently expose the existing Python API by
converting inputs and outputs and forwarding port calls. They contain no second
copy of the control state machines. `ASTRIBOT_BUILD_PYBIND=ON` remains the migration
default so existing Python ROS consumers continue using the same implementation.
This extraction does **not** claim the production ROS/SDK chain is Python-free.
Do not turn the option off in a production deployment before migrating those
consumers: their existing Python fallback is outside this package's extraction.

## Isolated verification

With ROS Humble development packages available, configure outside the shared
workspace:

```sh
cmake -S ws_robot/src/astribot_trajectory_bridge_native \
  -B /tmp/astribot-bridge-no-python \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DASTRIBOT_BUILD_PYBIND=OFF -DBUILD_TESTING=ON
cmake --build /tmp/astribot-bridge-no-python -j4
ctest --test-dir /tmp/astribot-bridge-no-python --output-on-failure
```

The three independent C++ probes link `bridge_runtime`; the Python tests act only
as an offline oracle and compare each tick's results/events and device-command
metadata. They do not import the native extension, create ROS nodes or load the
vendor SDK. CTest additionally checks numerical kernels, direct C++ configuration
safety gates and native-thread gripper busy/waypoint contracts. Test execution
needs the Python interpreter and pytest; the runtime library does not.

The compatibility adapter can be tested with a separate `ASTRIBOT_BUILD_PYBIND=ON`
build and the existing differential/replay/fault tests. Do not reuse installed
extensions from a shared overlay as evidence for a changed candidate.

## Preserved behavior and remaining boundary

The extraction retains the chassis SDK leash, command timeout, pose/scan age and
grace periods, manual leash reset, pose-relative integration and diagnostics;
arm cancel continues through the existing hold-and-observe state machine before
terminal reporting. Existing failure/timeout terminal results remain unchanged.
The existing SDK call flags are preserved, including chassis `filter,false,true`
and the arm/gripper defaults `direct,false,false`. Parameter checks formerly
performed by Python configuration constructors also run at direct C++ entry.

`JointSessionPort`, `ArmSessionPort` and `GripperSessionPort` are abstract ports,
not a newly implemented manufacturer SDK. A production C++ adapter still needs
the vendor's supported native headers/API or a confirmed ROS write protocol,
including control ownership, driver heartbeat, filter/limits and stop/restart
semantics. No ABI is inferred from distributed library symbols. Offline replay
is not closed-loop simulation or hardware acceptance.
