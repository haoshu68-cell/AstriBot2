# Wheel drive C++ entry cleanup

2026-09-21, source checkout `/home/yjh/WorkSpace/astribot_sdk_ros2`.

The wheel launch selects only `astribot_s1_chassis_effort_drive_native/omni_effort_drive_cpp`.
The Python console entry and module `main`/`__main__` entry are removed. The Python
class and math remain importable as validation references; the A/B harness owns
the reference node's startup. Wheel radius, topics, YAML path, simulation clock,
respawn and controller-exit/enable ordering are checked by parsing real launch
actions without executing them. The warehouse launch already had no
`effort_drive_node_impl` declaration, variable or forwarded argument when this
audit began; this audit did not edit its camera changes.

Results:

- Wheel package checks: 10 passed, including the retired Python startup check.
- Native package Python math checks: 4 passed.
- Fresh isolated Release CMake build succeeded; CTest `wheel_math`: 1 passed.
- The A/B replay exited 0. C++ and Python first active efforts were both
  `[-0.45355, 0.45355, -0.45355, 0.45355]`; the harness checks all four output
  values, active nonzero output and zero output after stale feedback. The first
  eight active rows printed in the raw log match. This is not a claim that all
  asynchronously sampled rows were aligned or compared.
- The executable is an ELF and its resolved dependencies contain no `libpython`.

`wheel_ctest.log` is the unmodified CTest `Testing/Temporary/LastTest.log`.
`wheel_ab.log` and `wheel_ldd.log` are the raw replay and dependency outputs.
`wheel_entry_pytest.log` and `wheel_math_pytest.log` preserve the two pytest runs.
Compiler flags are preserved; the Release C++ test includes `-UNDEBUG`.
`wheel_source_manifest.json` records the source/test/config hashes and ELF hash.
The commit alone cannot reproduce the uncommitted checkout; use the manifest.

Reproduction from the repository root:

```bash
source /opt/ros/humble/setup.bash
cmake -S ws_robot/src/astribot_s1_chassis_effort_drive_native \
  -B /tmp/astribot_cpp_entry_audit_20260921/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX=/tmp/astribot_cpp_entry_audit_20260921/install \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/astribot_cpp_entry_audit_20260921/build -j 4
ctest --test-dir /tmp/astribot_cpp_entry_audit_20260921/build --output-on-failure

# Read package-share metadata from the existing installation; do not overwrite it.
source ws_robot/install/local_setup.bash
export PYTHONPATH="$PWD/ws_robot/src/astribot_s1_chassis_effort_drive:$PWD/ws_robot/src/astribot_logging:$PYTHONPATH"
python3 -m pytest -q ws_robot/src/astribot_s1_chassis_effort_drive/test --disable-warnings
python3 -m pytest -q ws_robot/src/astribot_s1_chassis_effort_drive_native/test --disable-warnings

# Run only after verifying domains 92 and 93 have no other owner.
export ROS_LOG_DIR=/tmp/astribot_cpp_entry_audit_20260921/ros_logs
python3 ws_robot/src/astribot_s1_chassis_effort_drive_native/test/test_omni_effort_drive_ab.py \
  /tmp/astribot_cpp_entry_audit_20260921/build/omni_effort_drive_cpp
```

The A/B replay sets and uses domains 92/93 with `ROS_LOCALHOST_ONLY=1`; before
this run, reading `/proc/*/environ` found no processes using either domain. It
starts and stops only its own nodes. No Gazebo, ros2_control integration,
shared stack, hardware or deployment acceptance was performed. The shared
`ws_robot/install` directory was not modified. `/tmp` binaries are temporary
validation artifacts and must be rebuilt when missing.
