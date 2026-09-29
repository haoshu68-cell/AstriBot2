# Native observation adapters: isolated Stage6B evidence

Scope: `VisionAdapter`, `PointCloudBoxAdapter`, the native extension factory/registry and their packet boundary. This is offline differential and sanitizer evidence. It is not a ROS subscription, closed-loop simulation, performance, or hardware acceptance claim. Parent task owns observer, top-level CMake and runtime selection.

## Authority and method

`test/reference/policy_observation_adapters/observation_adapters.py` and `contracts.py` are byte-identical to Git `965bf057eb8722d5e0b7aa0491863885ba2e56d0`. `point_cloud2.py` and `numpy_compat.py` are byte-identical to the installed Humble `sensor_msgs_py` files under `/opt/ros/humble/lib/python3.10/site-packages/`. `source_manifest.json` records all SHA-256 values and the Stage6B contract/header dependencies; it does not modify historical Stage6A manifests. The test suite checks the four frozen hashes.

Both implementations receive the same packets, fake capture/steady clock sequence and TF results. The frozen installed NumPy-based `read_points` executes, with test-only message classes replacing ROS generated message objects. The native executable uses a test-only JSON command protocol. Runtime vision input is its existing String JSON protocol; metric work and point cloud decoding use typed C++ values. The cloud adapter computes the source's generated String size but does not serialize and reparse cloud math.

Comparison covers accepted observations, geometry kind/values, exact discrete state, ordered callback traces, `last_packet` on success and failure, mutated cloud bytes, exception class and contract error code/field. Error text is not compared. Finite floating outputs use 2e-12 relative/absolute tolerance; zero signs and branch decisions are checked separately.

## API

`policy_observation_adapters.hpp` exposes raw PointCloud2-equivalent fields/bytes, String packets, typed TF callback with target/source/capture nanoseconds, capture and steady clocks, profile/options, calibration updates, message kind, parsed packet state and typed metadata. `NativeAdapterRegistry` provides an explicit C++ factory extension interface. Arbitrary Python `module:Class` plugins are rejected; Python embedding and fallback are absent. Root converts actual ROS messages and chooses subscriptions from the message-kind enum.

Missing frame metadata is accepted for an empty observation packet, as the Python normalizer reads frame only inside its loop. Resolved measurement IDs stay as an unvalidated transport value until the observer's original post-ingest transaction point. For malformed empty packets with a non-string sensor, typed metadata rejects the sensor; root has been told to preserve original observer-side partial effects via `last_packet()` where required.

## Tests and retained red evidence

`red.txt` records the missing implementation, `red_json_overflow.txt` records the JSON exponent-overflow mismatch, `red_validation_order.txt` records expanded ImageBox argument/validation ordering, and `red_numeric_strings.txt` records numeric-string coercion. Final `green.txt` and `sanitizer_green.txt` contain 35 passing tests, including 150 seeded arbitrary-quaternion metric packets and 600 seeded stateful mixed-geometry/fault operations. The suite also covers clock faults and ordering, capture-time TF, velocity/covariance pairs, image/bearing geometry, UINT32_MAX image dimensions, Unicode String budgets, empty packets, partial batch failure, calibration update, dense/non-dense NaN and Inf, mixed point datatypes, field expansion, unused malformed fields, endian repeated reads, row padding and buffer/point/string budget boundaries.

The build uses C++17 and `-ffp-contract=off`; `build.txt` and `sanitizer_build.txt` contain compiler diagnostics. Sanitizers are ASan + UBSan with leak detection. The child wrapper places ASan before the machine's audit preload and retains that preload; no host setting is changed.

## Deferred configuration follow-up

The initial typed-only integration could parse configured options at startup rather than at the source's packet-processing point. `red_deferred_options.txt` records four new failing tests before this correction. The API now retains an optional raw `AdapterJson` configuration through `set_raw_options`/`raw_options`, while preserving the typed constructor and `options()` API. Raw values receive Python truth/equality and int/float conversion only when used. `set_calibration_epoch` updates both the raw object and typed epoch. The parent owns passing raw node configuration and preserving delayed coverage/calibration processing.

Vision reads byte budget before parsing JSON, observation budget after observation length, and sensor identity after the first capture-clock call. Point cloud reads point budget before any cloud access, then decodes/swaps bytes, reads sensor, parses calibration epoch and variance, and finally enters the parent packet budget/schema path. Bad unused cloud-only options do not break Vision startup or normalization. Null, falsey and wrong-type configuration, absent/numeric/null/container sensor identities, bool-number/dict-order equality, post-read failures and calibration correction are covered. The prior 31-test logs and source manifest are preserved under `before_deferred_options_*`; the new final suite has 35 tests.

## Preserved inherited behavior and limits

* Installed `read_points` ignores `row_step` and flattens `width*height` records at `point_step` stride. Padding bytes can therefore become points, exactly as the source does.
* Opposite-endian input swaps only selected x/y/z fields in place. Reusing a packet swaps them again; overlapping selected fields can swap the same bytes more than once. Original field order, duplicate-name and unused-field validation are preserved.
* NaNs are skipped only for non-dense clouds. Infinity is retained by the reader. Dense NaN ordering follows Python min/max comparisons; later contract construction rejects nonfinite metric geometry. Empty/all-filtered/over-budget clouds provide no free-space evidence.
* `max_packet_bytes` counts Unicode characters for String input. Generated point cloud JSON uses Python default escaping, spacing and float representation for that budget.
* Both source `now()` calls occur in order; the second result is unused. Each observation gets its own monotonic receipt call. TF uses the capture timestamp, and arbitrary quaternion coefficients follow the source rotation formula.
* Native options and integer contracts use signed 64-bit values, and ROS cloud fields preserve their wire widths. Positive String image dimensions/calibration epochs above INT64_MAX, arbitrary-precision JSON integers, and Python object-only inputs beyond those typed domains are not claimed equivalent. UINT32_MAX image dimensions and ROS maximum timestamp are covered. Escaped lone-surrogate strings are another known difference: `surrogate_boundary_result.json` records measurement_id `\ud800`, accepted by Python and rejected as JSONDecodeError by the native Unicode parser before any clock call. Unicode scalar-value strings and UTF-8 budget cases are covered. No claim of unrestricted Python-object compatibility is made. `integer_boundary_results.json` records three concrete valid JSON differences: calibration epoch 9223372036854775808, image width 9223372036854775808, and capture nanoseconds 9223372036354775808 with the default 0.5 s timeout. Python accepts each; native throws OverflowError. `integer_boundary_audit.py` reproduces them. These are unresolved whole-contract migration gaps; this adapter evidence does not authorize retiring the Python entry point.

## Reproduce

From any directory run the evidence `reproduce.sh`. It builds only in `/tmp/codex_policy_observation_adapters_20260921` by default and runs the dedicated test module. Set `POLICY_ADAPTERS_SANITIZE=1` for the sanitizer build; `POLICY_ADAPTERS_BUILD_DIR` selects another private directory. Dependencies are g++, Python 3 with pytest/NumPy, nlohmann-json headers and the repository sources. No shared installation, runtime Python edit or robot process is used.
