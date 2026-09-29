# Standalone fake-port core benchmark — 2026-09-21

This measures the existing Python core and the extracted C++ core with deterministic fake ports. It creates no ROS nodes, opens no vendor SDK session and sends no hardware commands. Results describe this machine and workload only.

Host: Intel(R) Core(TM) i7-14700K; both persistent workers pinned to logical CPU 0. Python 3.10.12; GCC 11.4.0; C++ Release `-O3 -DNDEBUG`, no pybind or Python interpreter linked into the native worker.

Each core used 5 alternating Python/C++ pairs of 50,000 operations, after 5,000 warmup operations. Core order rotates by round. Before timing, full device-write traces were compared for 300 gripper requests, 300 arm steps and 700 chassis ticks; every measured pair also matched its state, final position, write/open/close counts, event/feedback counts and checksum. Numeric comparison uses 3e-12 absolute / 3e-10 relative tolerance.

| Core / operation | Python wall median (µs) | C++ wall median (µs) | Python CPU median (µs) | C++ CPU median (µs) | Wall ratio Python/C++ |
|---|---:|---:|---:|---:|---:|
| Gripper request | 4.018 | 0.734 | 4.018 | 0.734 | 5.48× |
| Arm streaming step | 4.157 | 0.219 | 4.157 | 0.219 | 18.96× |
| Chassis control tick | 29.220 | 0.385 | 29.219 | 0.385 | 75.95× |

Gripper commands cycle through 0/25/50/75/100 with immediate fake following: intermediate openings need one write and endpoint calls return immediately. Arm uses one long smooth trajectory and remains STREAMING during measurement; events and feedback are drained each step. Chassis includes command submission, the control tick and event drain. Its original Python locking and timing diagnostics remain in the measurement; C++ uses one host mutex around the tick. These workload differences in existing bookkeeping are part of the measured implementation, so the large chassis ratio is not an isolated arithmetic-kernel speedup.

Timing uses process CPU time and monotonic wall time around a whole batch. Object construction, source hashing, RSS collection, JSON encoding/decoding and pipe transport are outside the timing interval. The worker is not relaunched per operation or per sample.

Startup-to-ready was recorded separately: Python 24.199 ms and C++ 1.305 ms. This is a single startup observation, not a repeated startup benchmark.

Median warmed worker RSS was 19,320 KiB for Python and 4,528 KiB for C++. These are whole persistent worker footprints including interpreter/import/allocator baselines, not memory attributed to each core. Per-batch RSS growth ranged 0–32 KiB for Python and 0–0 KiB for C++; the median was 0 and 0 KiB respectively. VmHWM records the post-exec address-space peak and remains cumulative across samples.

| Core | Python wall range (µs/op) | C++ wall range (µs/op) | Python loop baseline (ns/op) | C++ loop baseline (ns/op) |
|---|---:|---:|---:|---:|
| gripper | 3.936–4.025 | 0.725–0.750 | 71.55 | 0.39 |
| arm | 4.123–4.694 | 0.216–0.223 | 72.34 | 0.39 |
| chassis | 29.073–29.451 | 0.378–0.394 | 69.87 | 0.38 |

Loop baselines are reported separately and are not subtracted. Outside-loop RPC time includes setup/cleanup, RSS reads, JSON and IPC; these fields are preserved for every raw sample. CPU load from other activity was not controlled; five paired samples characterize this run, not a real-time latency guarantee.

[Raw JSON](standalone_fake_cores_20260921.json) contains all ordered samples, verification traces, warmups, baselines, startup figures, RSS values, compiler/link flags, binary SHA-256 and per-file source SHA-256.

```sh
cmake -S ws_robot/src/astribot_trajectory_bridge_native \
  -B /tmp/astribot-bridge-performance-release \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DASTRIBOT_BUILD_PYBIND=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/astribot-bridge-performance-release --target benchmark_standalone_driver -j4
python3 ws_robot/src/astribot_trajectory_bridge_native/test/benchmark_standalone.py \
  --driver /tmp/astribot-bridge-performance-release/benchmark_standalone_driver \
  --iterations 50000 --warmup 5000 --repeats 5 \
  --output /tmp/astribot-bridge-performance-20260921.json
```
