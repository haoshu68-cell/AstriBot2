# Native sensor health and camera calibration registry evidence

Scope: the complete pure registry behavior in `sensor_health.py`, implemented in
`policy_health.hpp` / `policy_health.cpp`. The shared immutable contracts belong
to `policy_contracts.hpp` / `.cpp`; numerical scan cells, movement directions and
continuous angular coverage link directly to the existing `navigation_math`.
No production Python, binding, ROS entry point, package installation or running
stack was removed or changed by this subtask.

## Initial registry evidence (2026-09-21)

- Initial missing-native red: `red.log` (one expected assertion failure).
- Release same-input differential: **39 passed**, `green.log`.
- Undefined-behavior sanitizer: **39 passed**, no sanitizer diagnostics,
  `ubsan.log` and `ubsan_build.log`.
- Standalone inclusion/build of `cmake/policy_health.cmake`: its registered CTest
  `policy_health_differential` passed, `ctest.log`.
- A temporary mutant changed expiration `now > valid_until` to `>=`; the boundary
  test failed with STALE versus VALID, `expiry_mutant_red.log`. The mutation was
  confined to `/tmp/codex_policy_health_20260921`.
- `case_inventory.json` records each executed scenario's operation count and
  input/output SHA256. The initial suite compared 5,980 operations. Four fixed random seeds each
  generate 1,200 primary operations plus calibration updates/lookups.
  `source_hashes.json` is the initial historical source/binary manifest; the
  final manifest is `norm_boundary_source_hashes.json`.

The frozen oracle files are exact source copies, checked against their own
`test/reference/policy_health/manifest.json`. They import only the frozen test
package and explicitly load with native dispatch disabled. They are not a
production fallback or installation target.

## Final motion-boundary correction

Independent review found that libc `hypot` rounded
`(-0.006876729203900029, -0.007260206295707338)` one ULP below 0.01,
whereas Python returned exactly 0.01. The old native kernel returned no movement
headings and allowed empty depth coverage for this input. The default Python
requires movement headings and rejects empty coverage.

The authorized correction adds the shared header-only `policy_numeric.hpp` and
changes only the norm calls in `navigation_math::movement_directions` and
`navigation_math::coverage_allows_motion`. It preserves the original 0.01
threshold; no tolerance is widened and no other navigation-math function changes.

- The real failure and original native outputs are retained in
  `norm_boundary_red.log` and `norm_boundary_before.json`.
- Final Release and UBSan: **41 passed**, **10,079 operations**, including
  2,048 deterministic nextafter speed inputs. Logs:
  `norm_boundary_green.log`, `norm_boundary_ubsan.log`.
- Final CTest: **1/1 passed**, `ctest_norm_boundary.log`.
- `case_inventory.json` and `norm_boundary_source_hashes.json` describe this final
  health run; the earlier logs and manifest remain historical evidence.

## Behavior covered

Source capture timestamps and expiry equality; nanosecond integers beyond 2^53;
future/expired/mismatched-clock rejection before mutation; epoch change and
rollback; query-only clock mismatch preserving stored state; sample ordering and
calibration downgrade; sorted required/source union; unavailable/degraded/stale
records and reasons; empty coverage and missing depth; required-source gating;
continuous angular gaps between sampled headings; tolerance/velocity/rotation
boundaries and wraparound; finite/invalid/infinite scan beams; timeout integer
truncation; invalid replacement preserving old data; calibration idempotence,
strict version increase, missing/version mismatch lookup, validation/error order
and invalid new calibration preserving the previous version.

Booleans, strings, timestamps, epochs, sequence results and error messages are
compared exactly. Geometric floating outputs use absolute and relative tolerance
2e-14. The reference is independent Python numerical code; native does not invoke
Python or dynamically load a bridge.

## Reproduction

From any directory, run:

```sh
python3 /home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/pybind_removal_20260921/policy_health/reproduce.py
python3 /home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/pybind_removal_20260921/policy_health/reproduce.py --ubsan
```

Dependencies: C++17 compiler, nlohmann JSON headers, Python 3 with pytest. Builds
are isolated under `/tmp/codex_policy_health_20260921`; neither ROS nor a deployed
install is used. Integrate `cmake/policy_health.cmake` after the `policy_contracts`
and `navigation_math` targets.

## Evidence boundary and remaining acceptance

This is implementation plus isolated offline testing. ROS message conversion,
QoS, ownership and runtime integration, observer/controller startup, closed-loop
simulation, real sensors and physical safety acceptance are **not tested** here.
Both production policy consumers still need complete migration before shared
Python or bindings can retire. No robot, session, task action, TF frame or runtime
log is represented by these offline JSON/probe artifacts.

The native API uses signed 64-bit timestamps/epochs, `int` image dimensions and
fixed-size intrinsic arrays. Python arbitrary-size integers and wrong dynamic
types are outside that typed domain. Native timeout/expiry integer overflow is
explicitly rejected. Tests cover valid typed-domain calibration errors, including
the first failing field; wrong-size intrinsic arrays cannot be constructed by
this C++ API. The captured build has style warnings in shared `policy_contracts.cpp`
(misleading indentation) and existing `navigation_math.cpp` (parentheses). The
owned health core/probe produce no compiler warnings; the shared-source owners
may address their warnings separately.
