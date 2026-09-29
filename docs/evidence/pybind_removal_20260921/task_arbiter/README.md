# Navigation task arbiter C++ migration evidence

Scope: `TaskArbiter` navigation admission and Nav2 action ownership. The runtime
is now `astribot_s1_task_arbiter_native/task_arbiter_cpp`; production Python module
and console entry are removed. The independent package uses C++17, defaults to
Release for ordinary builds, and links no Python/pybind library. No shared install,
Gazebo, hardware, or shared ROS stack was changed by these experiments.

The frozen original is `ws_robot/src/astribot_s1_navigation_policy/test/reference/task_arbiter_node.py`;
its SHA256 and full action/status/clock/parameter/ownership contract are in
[CONTRACT.md](CONTRACT.md). `test/reference` has no package initializer and is not
installed by `find_packages()`; the executable reference is validation-only.

## Results and limitations

- Final installed-ELF replay: **77/77 passed** (74 identical-input ROS2 protocol
  cases including 4 shutdown cases, plus 3 static entry checks).
  Both action types, all three sources, full goal/feedback/result forwarding,
  success/abort/backend cancellation, lower-priority rejection, equal/higher
  replacement, pending busy, user cancel, cancel rejection, late goal response,
  late result after pending timeout, unavailable/recovered backend, wall timeout
  with paused ROS clock, ROS clock rollback, transient status replay, startup
  parameter boundaries and startup-only runtime parameter consumption are covered.
- 4 shutdown cases passed while backend goal response or terminal result remained
  unresolved. Both implementations exited with status 0 within 2 seconds. Native
  has no detached worker, blocked future wait, or join waiting for backend result.
- 3 static entry/oracle/install-contract checks passed; pure C++ ownership CTest
  passed 1/1, including pending cancellation and old-generation release guards.
- Protocol tests assert that cancel ACK/rejection and pending handover timeout do
  not release the old owner or dispatch its replacement. Backend result is the
  existing release barrier; this is not measured physical stopping.
- No whole-robot task bus, physical stop confirmation, restart recovery, retry,
  real Nav2 integration, Gazebo closed loop, hardware, controller timing, or
  navigation quality acceptance is claimed. Lost backend responses retain the
  owner indefinitely, while the asynchronous executor remains responsive.

Startup parameter remains statically DOUBLE. An initial hypothesis that Python's
`float(value)` accepted integer/bool/string ROS overrides was disproved by running
the frozen oracle: Humble rejects them during declaration. Both implementations
reject those types and NaN/infinities/out-of-range doubles. Successful runtime
DOUBLE updates change the parameter store but not the startup timeout snapshot,
matching the old consumption timing; this is deliberately not a new dynamic knob.

## Alternating performance and stability experiment

5 pairs, alternating order Python→C++ then C++→Python, 100 measured serial goals
plus 8 warmups per process, 500 measured goals per implementation (540 including
warmup), zero rejected/failed goals. Fake Nav2 immediately reports success; timing
is client submission to frontend terminal result, measured with monotonic clock
and event-driven future completion. This includes ROS/DDS and fake-backend overhead.
All tasks use identical pose/frame/tree fields. No latency assertion substitutes
for the separate cancellation/ownership protocol checks.

| Metric, 500 measured tasks each | Frozen Python | C++ |
| --- | ---: | ---: |
| Pooled median submit→terminal | 23.583 ms | 3.957 ms |
| Pooled p95 | 24.886 ms | 6.245 ms |
| Maximum | 25.873 ms | 7.520 ms |
| Mean process CPU per goal | 5.28 ms | 0.40 ms |
| Mean process RSS after trial | 64.12 MiB | 26.62 MiB |

Each C++ trial's median is 3.714–4.392 ms; Python is 23.515–23.628 ms. CPU is sampled
from `/proc/PID/stat` at 100 Hz resolution; RSS from `/proc/PID/statm`, including
libraries and standard action result caches. The Python implementation's 10 ms
polling contributes to its latency. This is a proxy-overhead observation on this
host, not a promise for real navigation workloads or hard realtime execution.

Environment: ROS2 Humble, Fast DDS, localhost-only, domains 150/152 for protocol
and 153 for performance; each fixture has unique remapped underlying action
services/topics. Actual child environment was checked through `/proc/PID/environ`
after readiness. i7-14700K, 28 available CPUs; the final performance load sample is preserved in
`final_benchmark_environment.json`; the earlier run sampled 2.08/1.69 one-minute
load during/end. Other migration agents paused
build/ROS work during the paired window. No load-related outlier was observed;
raw per-goal timestamps, all trial summaries, and sample times are retained.
The table uses only `final_benchmark/` after the fixture teardown fix; an earlier
5-pair run is also archived but not pooled into the final statistics.

## Reproduction

From repository root (only the custom message install is read as a dependency):

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/astribot_navigation_msgs/share/astribot_navigation_msgs/local_setup.bash
cmake -S ws_robot/src/astribot_s1_task_arbiter_native \
  -B /tmp/astribot_task_arbiter_20260921/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -Dastribot_navigation_msgs_DIR="$PWD/ws_robot/install/astribot_navigation_msgs/share/astribot_navigation_msgs/cmake" \
  -DCMAKE_INSTALL_PREFIX=/tmp/astribot_task_arbiter_20260921/install
cmake --build /tmp/astribot_task_arbiter_20260921/build -j2
cmake --install /tmp/astribot_task_arbiter_20260921/build
ctest --test-dir /tmp/astribot_task_arbiter_20260921/build --output-on-failure
export ARBITER_CPP=/tmp/astribot_task_arbiter_20260921/build/task_arbiter_cpp
export ARBITER_DOMAIN=150
/usr/bin/python3 -m pytest -q ws_robot/src/astribot_s1_task_arbiter_native/test/
export ARBITER_DOMAIN=153
/usr/bin/python3 ws_robot/src/astribot_s1_task_arbiter_native/test/benchmark_arbiter.py \
  --output /tmp/astribot_task_arbiter_20260921/benchmark --pairs 5 --count 100
```

Reserve the selected isolated domains before replay; do not run this against a
shared robot graph. Native can alternatively be built with normal scoped colcon
including `astribot_navigation_msgs`. No shared install was written in this run.

## Raw evidence and iteration notes

Protocol and performance raw child logs and status traces are compressed in
`raw_runs.tar.gz`; summary/build/CTest/ldd logs, frozen/source/ELF hashes and runtime
versions are stored beside this report. Actual experiment root was
`/tmp/astribot_task_arbiter_20260921`; no simulation session.log exists for these
isolated action subprocesses. Logs are evidence, not physical trajectory data.

Earlier failed fixture iterations are retained. The initial shared-name fixture
suffered short discovery/stale endpoint effects; final fixtures use unique names.
Humble base action-name remapping alone did not remap underlying endpoints in
this setup, so final remaps explicitly cover send_goal/get_result/cancel_goal and
feedback/status. A single /clock sample at discovery was dropped; final replay
uses repeated finite clock samples and checks observed status timestamps. Initial
numeric-conversion expectations were corrected from the oracle's actual startup
failure. An initial shutdown fake-server teardown emitted an unawaited coroutine
warning; the fixture now drains released fake-server callbacks before executor
shutdown; the final complete replay and paired run have no such warning. None of these fixture corrections changes production control semantics.
