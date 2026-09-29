# Stage 6B: exact JSON integer contracts

This stage follows the root candidate/evidence commits `1b51e14e` / `51132768`. It preserves the frozen Python policy contracts for calibration epochs and image dimensions beyond signed or unsigned 64-bit ranges. It does not change Stamp, Version, metric geometry, risk rules, ROS messages or runtime entry selection. Previous stage manifests remain historical and were not rewritten.

## Implementation

- `policy_integer.hpp` defines `Integer = boost::multiprecision::cpp_int` and immutable `PixelScalar`, retaining integer versus double values, signed floating zero and exact mixed comparisons. The integer finite test matches Python's int-to-binary64 overflow boundary: magnitudes at or above `2**1024 - 2**970` fail.
- CameraCalibration, Observation and SensorHealth retain calibration epochs as Integer. CameraCalibration and ImageBox retain image dimensions as Integer; ImageBox retains all pixel coordinate kinds and validates them in the existing order. Constructors still reject negative epochs and nonpositive dimensions at the same validation slots.
- Health/calibration registry epoch comparisons are exact. `record` takes its epoch by value, preserving its lifetime when clock adoption clears previous records. Stale records retain the original epoch. Fusion requires no algorithm edits: replacement observations copy the widened epoch and image observations remain unassociated.
- The test probes use the independently owned `policy_integer_json.hpp` bridge. Its internal bigint category is binary; external JSON stays ordinary numeric tokens. No test oracle or Python compatibility path is installed in production.

## Evidence and limits

`red.txt` records 31 failures of the new valid tests against the preceding wire-width probes. Examples are rejection of calibration_epoch=2**63, wrap/truncation of image dimensions, and loss of exact float/integer bounds. `initial_harness_errors.txt` is an explicitly discarded first test wiring attempt: its ImageBox fixtures accidentally retained MetricBox fields, so those failures are not used as migration evidence. After correcting the fixture, all 31 new tests failed against the old probes for actual behavior differences. The initial 10 health failures were already valid before implementation. The corrected fusion fixtures were rerun against the old binary after the first implementation draft, so those 21 cases establish baseline differences but do not represent a correctly completed test-first cycle.

The new cases cover:

- calibration epochs 2**63, UINT64_MAX, UINT64_MAX+1, UINT64_MAX+2 and a 400-digit integer through metric/image observation retention, health, expiry equality, downgrade and duplicate rejection, and clock reset;
- calibration idempotence, changed-calibration rejection at the same epoch, and subsequent higher-epoch registration with very large dimensions;
- exact width UINT64_MAX versus float(2**64), versus integer UINT64_MAX; width 2**64 and UINT64_MAX+1; 400-digit dimensions; signed zero and stored coordinate numeric kinds;
- integer coordinate conversion overflow just below and at its exact threshold; negative/overflow input error ordering;
- 500 deterministic random image bounds near binary64/integer comparison edges, including strict lower bounds.

Validation is a private native build with frozen, test-owned Python references. Release and undefined-behavior sanitizer builds each pass 189 tests (fusion 89, health 56, risk 20, observer core 24); corresponding test/build logs record the results. The four suites include the preceding fusion, health, risk and observer-core regression range; root's newer observer-core boundary test is included when present. The retained NumPy overflow warnings are from risk-oracle tests which deliberately exercise overflow. The compiler emits an existing navigation_math mixed &&/|| parenthesis warning; this stage does not alter that code.

This evidence establishes isolated contracts/state behavior only. ROS health uint64 output conversion, observer integration, package acceptance, closed-loop simulation and hardware acceptance belong to the root integration work. Lone-surrogate handling is a separate implementation stage. Source timestamps remain the existing int64 domain; this change does not claim arbitrary Python integer parity for all unrelated fields.

## Reproduction

From repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 docs/evidence/pybind_removal_20260921/policy_json_integer_contracts/reproduce.py
PYTHONDONTWRITEBYTECODE=1 python3 docs/evidence/pybind_removal_20260921/policy_json_integer_contracts/reproduce.py --sanitizer
```

Builds stay under `/tmp/codex_policy_json_integer_contracts_20260921/{release,ubsan}`. There is no ROS initialization, shared install, process cleanup or Git operation. `source_hashes.json` distinguishes owned implementation/probes/tests, frozen authority, shared build inputs and the old red-test executables. `commands.json` copies record exact compilation commands. After the independently owned JSON header froze, `refresh_json_probes.py` rebuilt both affected probes for Release and UBSan and repeated all four suites. The final root manifest must recapture any shared sources subsequently modified by other tasks.
