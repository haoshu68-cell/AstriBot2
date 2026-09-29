# Native cross-domain map relay evidence

Production role: `astribot_s1_perception_native/map_domain_relay`, a C++ ELF with
the same executable name and startup contract. Its former Python console and
production module are removed. The old implementation is frozen only under
`test/reference` and is not installed. See [CONTRACT.md](CONTRACT.md) for the
precise inherited policies and limitations, including first-map-only timeout.

## Correctness and lifecycle

`final_ctest.txt` / `final_LastTest.log`: **46/46 real ROS2 tests passed** against
the whole-package build target. Both original and native implementations receive
the same fixtures. Tests cover scoped parameter files, global node/namespace/
topic remapping, reliable/transient-local publication and late subscribers,
empty and mismatched map sizes, every metadata field and float bit pattern,
NaN/Inf origin, a 1,048,576-cell payload, repeated identical content with changing
headers, no reverse map or control forwarding, burst/latest-map convergence,
initial timeout and no later timeout, paused ROS clock, zero/negative/NaN/Inf
timeouts, invalid parameter types/domains/topics, SIGINT 0 and SIGTERM -15.

Depth-1 QoS deliberately permits losses during overload. The burst test checks
ordered, intact received outputs and eventual delivery of the latest map; it
does not invent an all-messages delivery guarantee. Normal paired measurements
use serialized request/output matching and have no missing or duplicate frames.
Field comparison excludes unspecified CDR padding and compares the actual
message values, float bit patterns and all cell bytes.

The final 46 children comprise 16 normal exits, 28 expected startup/timeout
failures, and 2 expected OS-default SIGTERM exits; each PID is waited and recorded.
Test cleanup fails if SIGKILL is needed. Performance adds 16 normally reaped
processes. This is repeated short-run stability, not a long-duration leak test.

Regression proof: `signal_red.txt` demonstrates the initial C++ SIGINT regression
(-2 instead of 0). Shutting down only the bootstrap Context preserves the signal
handler; the final same test passes. `mutation_red.txt` uses a temporary-only
native source copy that corrupts map height; field parity immediately fails.
The production source and installed candidate were never replaced by that mutant.

`interrupt_probe_red.json` records a separate near-deadline SIGINT race: all five
Python probes exited 0 while all five native probes exited 1. `deadline_red.txt`
then reproduces the same failure as a regression test. Checking context validity
immediately after the interrupted wait preserves Python’s skip of the timeout
branch; the final 46-case CTest passes. The earlier 44-case run remains archived
as a historical intermediate result.

## Paired performance

Four alternating pairs per map size, ten warmups per process, 100 measured
256×256 maps or 40 measured 1024×1024 maps per trial. Each implementation receives
400 small and 160 large measured maps: **1,120/1,120 measured outputs** across
both implementations, exact field/content/sequence matching, no duplicates.
The table reports the median of four per-run statistics; maximum is across all
individual samples for that implementation and size.

| Map / metric | Python | C++ |
|---|---:|---:|
| 256×256 p50 | 4.710 ms | 0.562 ms |
| 256×256 p95 | 4.969 ms | 0.652 ms |
| 256×256 worst observed | 6.161 ms | 10.436 ms |
| 256×256 CPU time per map | 4.50 ms | 0.30 ms |
| 256×256 RSS | 58.69 MiB | 25.84 MiB |
| 1024×1024 p50 | 65.959 ms | 1.973 ms |
| 1024×1024 p95 | 76.379 ms | 2.409 ms |
| 1024×1024 worst observed | 84.109 ms | 13.143 ms |
| 1024×1024 CPU time per map | 63.00 ms | 1.25 ms |
| 1024×1024 RSS | 64.72 MiB | 30.04 MiB |

The final small-map maximum increased (6.161→10.436ms), even though median/p95
improved. Keep that outlier; no all-tail improvement or hard realtime claim is
made. The earlier series did not show this maximum reversal; do not cherry-pick
that earlier series or pool it into the final table.

Latency is monotonic time before the driver publishes in the remote context to
its local-context output callback. It includes DDS and driver scheduling, not
just native forwarding. Remote driver spin is nonblocking; local driver waits
for callbacks, avoiding a fixed remote-poll delay that would hide the C++ result.
CPU comes from process `/proc/PID/stat`, including all threads, with 10ms ticks:
per-run resolution is 0.1ms/map for small and 0.25ms/map for large samples. RSS is
process resident memory, not total system footprint. No hard realtime bound,
whole-stack speedup or SLAM accuracy claim follows from these measurements.

ROS2 Humble/Fast DDS, loopback-only domains remote 162/local 163/bootstrap 164;
child environment and every thread CPU affinity are recorded. Nodes use CPU 26,
driver CPU 27. Other migration build/ROS work paused during the performance
window. Existing shared robot/simulation processes were not stopped or changed.
The final post-signal-fix raw CSVs and per-run JSONs are in `final_performance/`;
the earlier `performance/` series is retained separately and is not pooled. Node
logs and PID/exit records
are in `raw_runs.tar.gz`. These runs have per-child `node.log`, not a whole-stack
`session.log`, and do not create a `latest_sim` session.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
cmake -S ws_robot/src/astribot_s1_perception_native \
  -B /tmp/codex_map_relay_20260921/build -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX=/tmp/codex_map_relay_20260921/install
cmake --build /tmp/codex_map_relay_20260921/build -j2
ctest --test-dir /tmp/codex_map_relay_20260921/build -R map_relay_protocol --output-on-failure
cmake --install /tmp/codex_map_relay_20260921/build
export ROS_LOCALHOST_ONLY=1
export MAP_RELAY_CPP=/tmp/codex_map_relay_20260921/build/map_domain_relay
/usr/bin/python3 ws_robot/src/astribot_s1_perception_native/test/benchmark_map_relay.py \
  --output /tmp/codex_map_relay_20260921/performance --pairs 4
```

Use an owned build/install location and coordinate the isolated fixture domains
and performance window before rerunning. No shared overlay, robot network,
Gazebo, Nav2 lifecycle, physical maps/control or hardware were exercised.

## Iteration record

The initial standalone package configure occurred while the parallel map/odom
source did not yet exist and failed accordingly. Relay development used an
owned temporary CMake copy omitting only the unfinished map/odom include; the
final replay and performance used the real whole-package relay target.
The first Python shutdown fixture signaled after graph discovery but before
executor startup and observed KeyboardInterrupt during initialization. Final
readiness requires an actual local parameter-service reply before signaling.
Review also prevented CDR-padding comparisons, `/proc` pre-exec races and an
unrequested change to the original local-thread exception policy. Earlier
failures remain archived instead of being relabeled as passing runs.
