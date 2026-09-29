# Cmd velocity adapter runtime parameters

The source audit found that the Python adapter reads six settings during its
callbacks, while the C++ adapter copied them into its core only at startup.
The ROS parameter service could therefore report success without changing
command conversion, timeout or posture behavior.

The C++ adapter now reads committed values before odometry, command and watchdog
callbacks. Parameter validation has no control-state side effects: a rejected
atomic update cannot apply another valid member to the core. Updating settings
preserves odometry/yaw, source timestamps, command age, the posture sample window
and an already latched stop.

| Parameter | Runtime update contract |
|---|---|
| `enable_body_to_world` | Boolean; the next callback observes the new value |
| `normal_height` | Finite double; the next posture evaluation uses it |
| `max_height_deviation`, `max_tilt_rad` | Finite, nonnegative doubles |
| `odom_timeout_sec`, `cmd_timeout_sec` | Finite, positive doubles |
| `input_topic`, `output_topic`, `odom_topic`, `enable_posture_monitor` | Startup only; descriptor is explicitly read-only |

This is **not literal equivalence for invalid or startup-only setters**. The old
Python adapter accepted some invalid values, and its startup-only settings could
be changed in the parameter store without changing runtime behavior. The new
C++ interface rejects these requests with a reason. Valid dynamic updates retain
the existing control, freshness, sample-window and irreversible stop-latch rules.
`use_sim_time` remains the standard ROS clock parameter. No extra runtime topic,
QoS or posture-monitor enable/disable feature was added.
Constructor/runtime exceptions now return exit status 1 after shutdown; normal
SIGINT shutdown returns 0. Previously the C++ exception handler logged FATAL but
returned 0, hiding invalid startup configurations from launch supervision.

Evidence and reproduction:

- `cmd_parameters_red.log`: old C++ binary, 16 failed / 8 passed / 12 skipped.
  All six valid dynamic cases pass with the Python reference and fail with the
  old C++ implementation. Wrong-type rejection already worked in the old C++
  parameter framework; the extra failures expose invalid-value acceptance and
  startup-only setters claiming success.
- `cmd_parameters_initial_green.log`: repaired binary, 24 passed / 12 skipped.
  Six valid cases run against each implementation; eight invalid atomic updates
  and four read-only checks run against C++ only. The 12 skips explicitly exclude
  the old Python adapter from the stronger rejection contract.
- `cmd_parameters_startup_red.log`: four invalid startup cases logged FATAL but
  returned 0 before the exit-status correction; all four tests failed as expected.
- `cmd_parameters_green.log`: final test and binary, **30 passed / 12 skipped**.
  This includes the six additional startup/shutdown checks and the corrected
  actual child-environment assertion. Four invalid startup configurations return
  nonzero; Python and C++ normal shutdown both return 0.
- `cmd_parameters_startup_environment_check_failed.log`: a subsequent test-only
  environment assertion ran immediately after process creation and observed a
  transiently empty `/proc/<pid>/environ`. It failed before the protocol cases.
  The harness now reads the child environment after ROS readiness, while still
  requiring the actual domain and localhost flag to match.
- `cmd_parameters_ctest.log`: existing C++ core test passes, extended to verify
  that toggling parameters without new odometry retains yaw and source age.
  The three protocol threshold cases already check preservation of the sample
  window and the existing stop latch after thresholds are relaxed.
- Build logs, compiler flags, ELF dependencies and source/test/binary hashes are
  archived beside this file. The existing `navigation_math.cpp:303` mixed
  `&&`/`||` warning is unchanged. New runtime dependencies are explicitly declared
  through `rcl_interfaces` in the package manifest and CMake target.

The validation build compiles the repository's node, core and navigation math
sources in an isolated component CMake project. It does not establish a complete
package install or a successful robot deployment. Reproduce from the repository
root after checking that domain 124 is free:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /tmp/astribot_cpp_entry_audit_20260921/cmd_source
cp docs/evidence/pybind_removal_20260921/entry_cleanup/cmd_parameters_isolated_CMakeLists.txt \
  /tmp/astribot_cpp_entry_audit_20260921/cmd_source/CMakeLists.txt
# If the checkout moved, update NATIVE_SOURCE in this copied CMakeLists.txt.
cmake -S /tmp/astribot_cpp_entry_audit_20260921/cmd_source \
  -B /tmp/astribot_cpp_entry_audit_20260921/cmd_build \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/astribot_cpp_entry_audit_20260921/cmd_build -j 4
ctest --test-dir /tmp/astribot_cpp_entry_audit_20260921/cmd_build --output-on-failure
CMD_VEL_CPP_BINARY=/tmp/astribot_cpp_entry_audit_20260921/cmd_build/cmd_vel_body_to_world_cpp \
ROS_LOG_DIR=/tmp/astribot_cpp_entry_audit_20260921/cmd_ros_logs \
  python3 -m pytest -q \
  ws_robot/src/astribot_s1_navigation_policy_native/test/test_cmd_vel_runtime_parameters.py
```

Every protocol process uses domain 124 with localhost communication and unique
per-test node/topic names. It publishes only to its own test topics and stops
only its own child. No Gazebo, shared robot stack, hardware or closed-loop robot
motion was involved; the shared `ws_robot/install` was not modified.
