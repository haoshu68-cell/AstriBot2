# Native persistent fusion and immutable contracts

This evidence covers the standalone C++ contracts and `ConservativeFusion` core. It does not establish a complete observer/controller migration, binding retirement, ROS integration, closed-loop simulation, or hardware acceptance. No shared ROS processes, installs, production Python files, launch files, or Git state were changed by this subtask.

The byte-identical frozen test authorities are `contracts.py`, `ports.py`, and `fusion.py` under `test/reference/policy_fusion/`. `manifest.json` records their authoritative source paths and SHA-256 hashes. JSON is confined to the test executable; production contracts, state, callbacks, and math use typed C++ values.

## Implemented scope

- All immutable data values from the two contract files, with validated constructors, error codes/fields, temporal checks, covariance PSD checks, prediction/world invariants, and decision execution authority checks. Static C++ types enforce structural type constraints before runtime.
- Persistent fusion state: tracks and insertion order; source identities; provenance correlation; broad-phase cells; seen-measurement retention; last sensor capture stamps; unassociated observations and insertion order; monotonic identifiers; epoch reset; observation sequence; sensor order; version; prediction steps; velocity samples and fitting.
- Exact source ordering, same-sensor assignment limits, cross-sensor simultaneous conservative unions, measured velocity preservation, fitted speed cap, stationary/occupancy variance, spatial occupancy retention, fresh evidence resolution, translated whole-box clearing callbacks, region relevance, and memory-clamped forecasts.
- `set_envelope_bounds()` applies accepted effective half extents without resetting tracks/history/sequence. It rejects finite extents below the baseline.
- Existing `fusion_snapshot.hpp` remains the snapshot math owner. The only shared-header extension is an optional `SnapshotParams::norm2` callback; null preserves previous callers. The persistent core supplies the compensated norm helper. Projection, scan free-space coverage, and polygon geometry remain existing geometry-kernel responsibilities in the adapter.

## Verification and failures

The final logs are `green.txt` (default Python oracle), `green_native_snapshot.txt` (optional historical binding selected), and `sanitized.txt` (ASan + UBSan, including leak detection). Each final run exercises 47 tests, with 20 seeded stateful sequences of 65 ingests, 8,001 near-unit bearing samples, exact association/provenance/stationarity/relevance thresholds, and 3,004 exact-valued normal/subnormal norm cases. All pass; inspect the logs for the measured durations. This is offline evidence only.

Earlier failures are retained, rather than overwritten:

- `red.txt`: native persistent executable absent before implementation.
- `red_envelope.txt`: live envelope update unsupported before adding the state-preserving setter.
- `red_overflow.txt`: derived center overflow escaped as the kernel's generic error rather than the default contract exception.
- `red_bearing.txt`: standard-library three-dimensional norm rejected a direction the Python contract accepts at the original `1e-6` gate.
- `red_norm_gates.txt`: the previous kernel arithmetic selected variance `0.01` instead of `0.0004` at the exact stationary-distance gate.
- `red_subnormal_norm.json`: independent review found four subnormal rounding differences among 106,000 norm samples before the tiny-number branch was aligned with Python.

The compensated norm uses scaled product residuals, compensated addition, and square-root correction. The tiny-number branch preserves Python's ratio/sum rounding. The numerical techniques and Python's behavior were checked against [CPython 3.10.12 mathmodule.c](https://github.com/python/cpython/blob/v3.10.12/Modules/mathmodule.c#L2345-L2434). No policy threshold or tolerance was widened.

## Exceptional arithmetic and domain boundaries

The default Python implementation and its optional native snapshot historically disagree at some exact floating-point gates and exceptional magnitudes. Common-domain tests run against both; targeted known-discrepancy tests explicitly select the default Python authority.

Concrete preserved default behavior:

- A box centered at `x=1e308` with measured `vx=1e308`, snapshot at age 2 seconds with one-second memory, fails `INVALID_INPUT: x` before any world is returned.
- Measured velocity covariance diagonal `1e308` at age 2 seconds fails `INVALID_INPUT: covariance.value` when inflation overflows.
- Position covariance diagonal `1e308`, finite center `x=1e308`, and region `(0,0,0)` produce an infinite conservative radius. Default Python and the new core retain relevance. The historical optional binding raises `snapshot region overflow`; the test explicitly verifies that historical difference.
- The observer's region travel can become positive infinity from finite odometry (`vx=1e308` multiplied by the prediction horizon). The new core retains all tracks as relevant, matching the default Python behavior; it does not reject this reachable derived value.

Native stamp/epoch storage is signed 64-bit nonnegative, matching the native time boundary rather than Python's unbounded integers. Region origins are finite robot positions and travel is nonnegative; negative travel is outside the observer's derived call contract. External profile mutation beyond the existing live envelope half extents is not an exposed runtime API. No new track-count or observation-count cap was introduced.

## Reproduction

From `/home/yjh/WorkSpace/astribot_sdk_ros2`:

```sh
mkdir -p /tmp/codex_policy_fusion_20260921
g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Wpedantic \
  -Iws_robot/src/astribot_s1_navigation_policy_native/include \
  -Iws_robot/src/astribot_s1_robot_geometry/include \
  ws_robot/src/astribot_s1_navigation_policy_native/src/policy_contracts.cpp \
  ws_robot/src/astribot_s1_navigation_policy_native/src/policy_fusion.cpp \
  ws_robot/src/astribot_s1_navigation_policy_native/test/policy_fusion_probe.cpp \
  -o /tmp/codex_policy_fusion_20260921/policy_fusion_probe
python3 -m pytest -q ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_fusion.py
```

The first sanitizer attempt failed before main because the host audit library is globally preloaded; that output is retained in `sanitizer_preload_failure.txt`. The final run prepends ASan for the probe while retaining the audit hook. Copy `sanitizer_runner.sh` to `/tmp/codex_policy_fusion_20260921/policy_fusion_sanitized_runner` (and make it executable). Its runtime path records this machine’s GCC 11 installation.

For the sanitizer run, use `-O1 -g -fno-omit-frame-pointer -fno-pie -no-pie -fsanitize=address,undefined` instead of `-O2`, name the result `policy_fusion_probe_sanitized`, and run:

```sh
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
POLICY_FUSION_PROBE=/tmp/codex_policy_fusion_20260921/policy_fusion_sanitized_runner \
python3 -m pytest -q ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_fusion.py
```

The optional historical binding replay uses `POLICY_FUSION_BASELINE_GEOMETRY_BINDING=/tmp/codex_geometry_validation_20260921/baseline_build/_geometry_native.cpython-310-x86_64-linux-gnu.so`. Its source/binary hashes and the shared header's pre-extension hash are recorded in the manifest. This binding is test-only and is not required by the C++ runtime or the default test run.

## Archived independent review and shared-kernel compatibility

`independent_review/` contains the reviewer scripts, exact failing inputs and
before/after results, plus separate `source_hashes_before.json` and
`source_hashes_final.json`. `manifest.json` records every original absolute
script path and archive hash. To repeat the reviewer scripts unchanged, restore
them under their original `/tmp/review_policy_fusion` paths; they intentionally
refer to that scratch directory and this checkout. Compiled binaries were not
archived.

`snapshot_default_compatibility/` contains the standalone comparison source,
both complete 3,000-row outputs, and comparison metadata. Compile the same
source once with the recorded baseline include directory and once with the
current geometry include directory, using `g++ -std=c++17 -O2 -ffp-contract=off`;
compare the resulting standard output files. The SHA-256 values are identical.
The old baseline header and its hash remain distinct from the current header's
additive callback extension.
