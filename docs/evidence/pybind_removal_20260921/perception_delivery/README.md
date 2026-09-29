# Perception migration integration checkpoint

The clean native package provides `map_odom_tf_node` and `map_domain_relay`.
Both old Python console registrations and all three production modules are
removed. The two launch files select only `astribot_s1_perception_native`;
existing launch conditions, executable/node names, parameters and exit handlers
are preserved. The unrelated `perception_slam_bringup.launch.py` change belongs
to another task and is excluded from this migration checkpoint.

## Final verification

- `final_ctest.log/xml` and `final_LastTest.log`: **4/4 CTest targets pass** in
  the root's independent whole-package build, 77.09 seconds. This includes the
  C++ core smoke, **34 pure differential, 64 TF ROS and 46 map ROS pytest cases**.
  Every final test uses the frozen explicit oracle or actual native executable.
- `entries_green.txt` in the sibling `map_relay` directory: 23 migration entry,
  reference packaging and inventory-alias checks pass.
- `install_audit.json`: a fresh Python installation exposes only the remaining
  patrol/session/start-cell executables. None of the three retired modules is
  importable. The independent native installation exposes exactly two ELF
  executables, resolves its linked libraries and has no libpython/pybind links
  or installed test/reference Python files.
- Installed map relay passes 4 additional ROS cases; installed map/odom passes
  4 additional ROS cases. Source and installed-ELF manifests are retained in
  both role evidence directories; the map/odom installed hash matches the root
  and implementation worker builds.
- Package discovery recognizes the original launch/remaining-adapter package
  as ament_python and the new runtime package as ament_cmake. Fresh configure
  defaults to Release; explicitly selected Debug remains Debug.
- `frozen_reference_check.json`: the three test references are byte-identical
  to Git `1b2c33a4`, and their old production paths are absent.
- `recorded_tree_entries.log`: 5 entry/inventory tests also pass from the
  extracted Git `80a70bde` tree. `source_manifest.json` covers 25 actual source
  files, whose Git blobs and working-tree hashes match; ignored pytest caches
  were excluded when checking the initial filesystem manifest against Git.

## Reproduction environment and retained failure

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/astribot_logging/share/astribot_logging/local_setup.bash
cmake -S ws_robot/src/astribot_s1_perception_native \
  -B /tmp/codex_map_relay_20260921/build -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX=/tmp/codex_map_relay_20260921/install
cmake --build /tmp/codex_map_relay_20260921/build -j2
ctest --test-dir /tmp/codex_map_relay_20260921/build --output-on-failure
cmake --install /tmp/codex_map_relay_20260921/build
/usr/bin/python3 tools/migration/verify_perception_install.py \
  --native-prefix /tmp/codex_map_relay_20260921/install \
  --output /tmp/codex_perception_install_audit
```

The logging hook is a read-only dependency of the frozen Python oracle's error
reporting; C++ runtime does not need it. The first root integration run sourced
only `/opt/ros/humble`, so seven Python rejection/overflow cases raised
PackageNotFoundError inside error logging and exited1 instead of the expected2.
`missing_logging_*` preserves that failed run and its raw files. Loading the
documented logging dependency fixed the environment; production and oracle code
were not changed to hide those failures. `final_raw.tar.gz` contains the separate
successful whole-package run.

Only owned temporary build/install prefixes and isolated loopback ROS domains
were used. No shared install, robot domain, Gazebo, real SLAM/Nav2, deployment or
hardware was changed. Historical Python busy-SIGINT failures, native regression
fixes, the small-map C++ maximum-latency outlier and short-run stability limits
remain explicit in the [TF report](../map_odom/README.md) and
[map relay report](../map_relay/README.md). This checkpoint does not establish
whole-stack or long-duration/hardware acceptance, nor whole-project pybind removal.
