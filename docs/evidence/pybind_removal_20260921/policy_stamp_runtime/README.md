# Stage 6B: arbitrary Stamp domains and original failure boundaries

**Selection status:** The user requested retaining the prior native version after
reviewing the native performance increase. This candidate is archived on a
separate Git branch; it is not the selected working version. The selected
pre-Stamp source checkpoint is `a309d9bfcfef13d9dac2aaf9432b4fdd6804a6ea`
(strings implementation `f4feccdf`). No production deployment occurred.

This checkpoint fixes the Stamp and failure-stage differences recorded in
`../policy_stamp_boundary_review`. It does not switch either production policy
entry, retire bindings, or complete the controller. Correctness evidence below
does not imply performance neutrality: the native-to-native cost is measured.

## Source and behavior

`source_manifest.json` identifies 200 isolated build/test inputs based on
`a309d9bfcfef13d9dac2aaf9432b4fdd6804a6ea`; `scoped_paths.json` lists 24 changes.
Concurrent navigation-zones CMake/package/final-protection edits are excluded.
Frozen Python authorities are unchanged. Geometry tests additionally use the
23 Git-derived support files and explicit pytest plugin identified by
`geometry_test_support_manifest.json`; these are test-only imports.

Stamp ns/epoch, health deadlines, packet metadata, prediction offsets and fusion
history now retain Python's integer domain. Subtraction happens before float
conversion. Conversion to binary64 implements ties-to-even and the original
overflow threshold; float-to-integer handles NaN/infinity at the original
consumer. Retention and timeout conversion preserve evaluation order and lazy
execution, including partial fusion mutations before an error.

Metric TF conversion still fails at the ROS Time boundary; image packets retain
the raw capture value until the original acquisition checks. Health deadlines
remain wide internally and fail only when serialized into bounded ROS time.
Pure Prediction offsets remain wide, while the NumPy-equivalent risk-row
boundary still rejects out-of-int64 offsets. `PredictionRow` remains the exact
base int64 layout. The geometry snapshot algorithm is shared via a timestamp
template and explicit conversion callback, without a second algorithm fallback.

Timer errors now log their cause before rethrowing; fatal propagation is
preserved. This matters because the prior C++ ROS exception unwind sometimes
printed only `terminate called without an active exception`, obscuring failure
order in the installed comparison.

## Validation and retained failures

- Initial Stamp RED: 22 failed / 8 passed; prediction RED: 2 failed; integer-to-
  float overflow RED: 1 failed. The intermediate widened risk-row candidate
  incorrectly accepted six out-of-range inputs; `risk_boundary_red.log` retains
  those failures before restoring the original row boundary.
- `ctest_9.log`: final native Release **14/14**. `geometry_ctest_3.log`: geometry
  **5/5**, including its owned ROS fixture. Earlier geometry runs failed due to
  absent retired-reference imports and then a cached plugin search path; those
  environment failures are retained separately.
- `installed_ros_verified.log`: final installed **120/120**, 60 scenarios on each
  implementation. New cases cover huge finite timeouts, wide image/metric capture
  stamps and original TF versus health output failure order. Raw logs and outputs
  are in `installed_ros_verified/`. Earlier `installed_ros.log` has 118 pass / 2
  failures because the fatal C++ log lost the error cause; the subsequent
  diagnostic fix and final rerun are preserved.
- `sanitizer_tests.log`: **275 passed** with ASan/UBSan/LSan; after restoring the
  final int64 row header, the affected risk library/probe were explicitly rebuilt
  and **26/26** rerun in `sanitizer_risk_final_2.log`. These 26 are a rerun, not
  additional unique tests. Six NumPy overflow warnings are expected from existing
  extreme-input cases. No sanitizer findings were reported. The compiler also
  reported an existing mixed `&&`/`||` parentheses warning in navigation_math.
- One intermediate health test incorrectly expected a huge negative timeout to
  yield valid health. Both implementations already agreed on ContractError;
  only the assertion was corrected (`core_green_1.log`).
- A frozen header resync initially retained the base file's old mtime, allowing
  Make to reuse stale objects. `build_9.log` and `sanitizer_rebuild_2.log` explicitly
  show risk consumers rebuilt after refreshing the mtime; final CTest, installed
  ROS and benchmarks use this rebuild. Earlier passing runs are historical, not
  proof of final source/binary correspondence.
- Native/native benchmark initially read an unused Python binding while writing
  its summary. `benchmark_binding_red.log` reproduces FileNotFoundError after
  measurements. Final `benchmark_native/summary.json` succeeds without a binding
  environment variable or binding metadata. Python comparison still records its
  oracle hash.
- `install_audit.json`: six installed ELF files, all dependencies resolved, no
  libpython/pybind dependency and no installed Python source in this selected
  prefix. This is not a whole-repository dependency audit.

The original Python SIGINT path can report a double-shutdown error; the fixture
requires bounded termination, not identical successful shutdown exit codes.

## Performance

Each comparison uses four interleaved AB/BA pairs per 8/64-track workload, 20
warmup and 100 measured frames per process: **1,600 frames per comparison**.
Every measured semantic output matches (excluding timing fields, floating point
rel=2e-12 / abs=3e-12). Values below are medians of four per-process summaries.
Driver and observer use separate CPUs on a shared host; the two comparisons ran
sequentially. Raw results include tail delays and RSS growth.

| Tracks | Metric | Original Python | Current C++ | Change |
|---:|---|---:|---:|---:|
| 8 | tick P95 ms | 1.771406 | 0.223785 | -87.37% |
| 64 | tick P95 ms | 3.836718 | 0.724768 | -81.11% |
| 8 | whole-process CPU ms/frame | 6.650000 | 0.800000 | -87.97% |
| 64 | whole-process CPU ms/frame | 13.750000 | 2.200000 | -84.00% |
| 8 | RSS MiB | 62.460938 | 27.970703 | -55.22% |
| 64 | RSS MiB | 64.050781 | 29.013672 | -54.70% |

| Tracks | Metric | Prior strings C++ | Current C++ | Change |
|---:|---|---:|---:|---:|
| 8 | tick P95 ms | 0.180140 | 0.212339 | +17.87% |
| 64 | tick P95 ms | 0.563720 | 0.744388 | +32.05% |
| 8 | whole-process CPU ms/frame | 0.700000 | 0.700000 | 0.00% |
| 64 | whole-process CPU ms/frame | 1.800000 | 2.250000 | +25.00% |
| 8 | RSS MiB | 27.750000 | 27.966797 | +0.78% |
| 64 | RSS MiB | 28.849609 | 29.056641 | +0.72% |

Thus the complete integer-domain fix has a measured native cost. It is not
described as performance-neutral or as satisfying an unspecified no-regression
threshold. At 64 tracks, median measured scan/fusion work rises about 125→221 µs
and snapshot work 147→212 µs; risk work is about 126→130 µs. These timings locate
follow-up optimization work, without attributing every difference causally on a
shared host. Any optimization must keep the new wide-domain and failure tests.

Tick timing excludes the preceding vision callback; whole-process CPU includes
callbacks during the measured window, excluding startup. External delay starts
at clock publication after preceding inputs are drained. RSS is a short-window
endpoint, not a leak/soak guarantee. No controller/Nav2/Gazebo closed-loop,
hardware, long-term stability or safety-stop acceptance is claimed.

## Reproduction

Use the recorded Git checkpoint and `reproduce.sh` with a NEW absolute directory.
Supply `NAVIGATION_MSGS_PREFIX`, `POLICY_CONFIG_PREFIX` and `CPP_BASELINE` (the
prior strings-checkpoint native observer). The wrapper composes the same verified
build/test commands and has been shell-syntax checked; the wrapper as a whole has
not been rerun after packaging. Test-only oracle bindings are never loaded by the
native observer. The shared workspace install and robot stack were not changed.
