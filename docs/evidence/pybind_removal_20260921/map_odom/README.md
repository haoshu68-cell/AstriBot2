# map→odom native migration evidence — 2026-09-21

The native `map_odom_tf_node` replaces only the existing TF decomposition role.
It has no command/control authority. The frozen Python sources, interface,
numerical boundaries, clock behavior and inherited policy are recorded in
[CONTRACT.md](CONTRACT.md). Parent integration owns the launch/entry removal;
this evidence covers the native role and isolated ROS replay, not full SLAM,
Gazebo, navigation closed-loop behavior or hardware acceptance.

## Implementation and reference

Production code is `ws_robot/src/astribot_s1_perception_native/src/map_odom_tf_node.cpp`
and `include/astribot_s1_perception_native/map_odom_core.hpp`.
`cmake/map_odom.cmake` registers the executable, standalone pure-function probe,
and the pure/ROS differential CTests. C++ is built in Release with floating-point
contraction disabled to preserve Python's operation order.

The two original Python modules were frozen verbatim before migration under
`test/reference/astribot_s1_perception/`; each oracle process asserts the exact
module path. They are validation inputs and are not installed as runtime roles.
The original production sources/entries are removed by the parent integration.
No shared install, hardware or Gazebo process was changed by this subtask.

## Isolated evidence

Final run: **3/3 CTests passed**, containing **34 pure and 64 ROS pytest cases**
plus the C++ core smoke. The installed native executable passed **4/4 additional
ROS smoke cases** (composition/height, ROS rollback/cache, default timeout and
watchdog). Final CTest elapsed50.59 s; installed smoke4.63 s. These are isolated
ROS2 process results, not simulator or hardware results.


- Pure differential suite: 34 tests, including 600 paired random compositions,
  exact jump/age/50 ms future boundaries, ±pi/signed zero, no quaternion
  normalization, NaN/Inf and extreme finite arithmetic.
- ROS replay compares both implementations on the same scenarios: sequential
  source lookup and 0.2 s missing timeout, composition/height/tilt warning,
  missing/stale source recovery, ROS pause/rollback/cache recovery, static zero
  stamps and disabled/NaN age limits, startup-only watchdog, typed runtime
  parameter snapshot behavior, zero-nanosecond timers, fatal parameters,
  nonfinite wire rejection and arithmetic-overflow exit.
- Native dependency/build/install evidence and all initial failures are retained.
  Normal test cleanup checks exit0; watchdog and decomposition cases explicitly
  check exit1 and exit2. SIGKILL is a test failure, not successful cleanup.

The initial full CTest run had 63/64 ROS cases passing: the remaining case
completed its functional assertions but its busy Python process exited1 on
SIGINT. The raw failure and the follow-up classification are retained rather
than excluded from the record.

### Busy shutdown observation

The original default-timeout fixture published again after a single `spin_once`.
Its own TF subscription made that return immediately, accidentally producing
roughly 2,159–2,555 source messages in 0.4 s for Python. After the first matching
output, immediate SIGINT reproduced the failure in 1/4 Python probes; the other
3/4 and all 4/4 C++ probes exited0. The exact Python failure was:

```text
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py:400
msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
RuntimeError: Unable to convert call argument to Python object (compile in debug mode for details)
```

This is an observed historical rclpy busy-shutdown failure, not a decomposition
mismatch or a proven fault in message readiness. The pybind conversion failure
coincides with SIGINT while TF is being consumed; the deeper middleware cause
is not established by this experiment. The frozen oracle was not modified.
The probe retained every return code and traceback. No arbitrary Python exit1
is accepted by the normal tests.

The timeout/recovery test now explicitly limits input to 100 Hz instead of
accidentally flooding. Paired classification at that cadence gave Python4/4 and
C++4/4 exit0, with the same output and timeout assertions. However, the following
full CTest run reproduced the identical Python shutdown exception again. Both
full-run failures are archived. Limiting the source rate is not a fix for this
historical rclpy race.

Only this default-timeout functional test now explicitly quiesces before normal
shutdown: after all wall-clock timeout/recovery assertions, it sets the standard
`use_sim_time=true`, sends no TF or clock updates, and drains delivery for100 ms.
This pauses the publication timer and separates idle SIGINT0 from the separately
observed busy shutdown. It does not alter the oracle or any business assertion.
`Runtime.close` remains strict and unchanged, and the benchmark's input loop,
fixture and native binary are unchanged by this test-only cleanup.

## Paired performance

Four AB/BA pairs, each with a fresh subprocess, 8 warmups and 200 measured TF
inputs; 50 Hz input, 100 Hz publication timer, system time, default 0.2 s lookup
timeout. Both input edges are in one TF message with a position sequence marker;
latency is publisher monotonic timestamp to the first received matching
`map→odom` output, including the timer phase, DDS and probe scheduling. It is not
pure math time, SLAM accuracy, or end-to-end navigation latency.

| Metric | Frozen Python | C++ |
| --- | ---: | ---: |
| Matching measured inputs | 800/800 | 800/800 |
| P50 latency (ms) | 5.735264 | 5.271405 |
| P95 latency (ms) | 10.190713 | 9.736047 |
| Maximum latency (ms) | 10.892938 | 10.869976 |
| Mean process CPU per input (ms) | 1.5750 | 0.2125 |
| Mean ending RSS (MiB) | 57.2500 | 23.7988 |
| Per-trial P50 range (ms) | 5.4905–6.0566 | 5.2566–5.2899 |

All 8 performance subprocesses exited0. CPU is the child process's `/proc`
user+system CPU time at the host's tick resolution; RSS is the ending resident
set, not peak memory. Warmups excluded from statistics but retained in raw TF
traces. No measured sample was discarded. Latency is dominated by the 10 ms
publication phase; the more pronounced gains are process CPU and resident memory.
The parent paused builds and ROS experiments during this window. Host: i7-14700K,
28 logical CPUs, Ubuntu22.04, GCC11.4, ROS2 Humble, rclcpp16.0.19/rclpy3.3.21,
tf2 0.25.22, Fast DDS RMW6.2.10. Per-trial load averages (about 3.4–4.2) are retained in the raw results.

## Reproduction

Run from the repository root. Source only the ROS installation and the logging
package needed by the frozen Python oracle. The tests also explicitly prepend
the logging source package; no production perception Python import is used.

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/astribot_logging/share/astribot_logging/local_setup.bash
cmake -S ws_robot/src/astribot_s1_perception_native \
  -B /tmp/codex_map_odom_20260921/build -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/codex_map_odom_20260921/install
cmake --build /tmp/codex_map_odom_20260921/build -j2
cmake --install /tmp/codex_map_odom_20260921/build
ctest --test-dir /tmp/codex_map_odom_20260921/build -R '^map_odom_' --output-on-failure
MAP_ODOM_DOMAIN=161 ROS_LOCALHOST_ONLY=1 \
  MAP_ODOM_CPP=/tmp/codex_map_odom_20260921/build/map_odom_tf_node \
  python3 ws_robot/src/astribot_s1_perception_native/test/map_odom_benchmark.py \
  --output /tmp/codex_map_odom_20260921/benchmark --pairs 4 --count 200 \
  --rate 100 --input-rate 50
```

ROS tests use domain160, performance uses161, with localhost-only DDS, unique
TF/clock remaps and actual child `/proc` environment assertions. Cleanup addresses
only each fixture's owned `Popen` PID. Coordinate an otherwise quiet performance
window before reproducing timing measurements.

## Inherited limitations

Tilt excess warns but still publishes, z is unchecked, nonpositive maximum age
bypasses all stamp gates, NaN maximum age disables expiry comparison, and source
lookups are sequential latest samples rather than time synchronized. One valid
update permanently disables the startup watchdog. The assumption that both
source bases represent the same physical point remains a deployment precondition.
These behaviors are preserved and are not evidence of physical calibration,
safety acceptance or a complete SLAM integration.

## Files and provenance

- [raw_evidence.tar.gz](raw_evidence.tar.gz): original RED missing-header failure,
  initial compile correction, initial Python fixture timeout failure, both full
  busy-shutdown failures, classification script and all16 observations, final
  CTest/pytest logs, TF samples, installation smoke, all performance samples,
  compiler configuration, library dependencies and environment versions.
- [benchmark_summary.json](benchmark_summary.json): pooled timing and process
  resource summary; per-trial values and every measured input remain in the archive.
- [manifest.json](manifest.json): SHA256 for native/header/probe/test/reference
  sources and build/installed map→odom ELFs. Build and installed ELF hashes differ
  because installation removes the build RPATH. The installed native ELF hash is
  `2b95a785b2ad173a2645a6966fe7b7f1d24b173a7be18ae8d64760884148e2ca`.
- [SHA256SUMS](SHA256SUMS): evidence artifact checksums. Shared relay binaries in
  this temporary installation are outside this subtask's validation claim.

The initial Python recovery fixture used a0.5 s window while missing lookups
blocked for the default0.2 s; that fixture failed before native integration.
General simulated-clock cases subsequently use0.02 s lookup timeouts, while the
separate wall-clock case verifies the actual default0.2 s blocking behavior.
This earlier failure, the initial compiler diagnostic, and both shutdown failures
are preserved. None are silently replaced by the final passing results.

Performance was measured before the final test-only default-timeout cleanup
changes. Native code, compiled ELF, `Runtime` and the benchmark loop were unchanged;
the measured semantics and sample population therefore match the final native
artifact. No runtime policy was changed to make the oracle tests pass.
