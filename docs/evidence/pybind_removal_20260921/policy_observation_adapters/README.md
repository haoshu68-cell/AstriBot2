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

`red.txt` records the missing implementation, `red_json_overflow.txt` records the JSON exponent-overflow mismatch, `red_validation_order.txt` records expanded ImageBox argument/validation ordering, and `red_numeric_strings.txt` records numeric-string coercion. Final `green.txt` and `sanitizer_green.txt` contain 47 passing tests, including 150 seeded arbitrary-quaternion metric packets and 600 seeded stateful mixed-geometry/fault operations. The suite also covers clock faults and ordering, capture-time TF, velocity/covariance pairs, image/bearing geometry, UINT32_MAX image dimensions, Unicode String budgets, empty packets, partial batch failure, calibration update, dense/non-dense NaN and Inf, mixed point datatypes, field expansion, unused malformed fields, endian repeated reads, row padding and buffer/point/string budget boundaries.

The build uses C++17 and `-ffp-contract=off`; `build.txt` and `sanitizer_build.txt` contain compiler diagnostics. Sanitizers are ASan + UBSan with leak detection. The child wrapper places ASan before the machine's audit preload and retains that preload; no host setting is changed.

## Deferred configuration follow-up

The initial typed-only integration could parse configured options at startup rather than at the source's packet-processing point. `red_deferred_options.txt` records four new failing tests before this correction. The API now retains an optional raw `AdapterJson` configuration through `set_raw_options`/`raw_options`, while preserving the typed constructor and `options()` API. Raw values receive Python truth/equality and int/float conversion only when used. `set_calibration_epoch` updates both the raw object and typed epoch. The parent owns passing raw node configuration and preserving delayed coverage/calibration processing.

Vision reads byte budget before parsing JSON, observation budget after observation length, and sensor identity after the first capture-clock call. Point cloud reads point budget before any cloud access, then decodes/swaps bytes, reads sensor, parses calibration epoch and variance, and finally enters the parent packet budget/schema path. Bad unused cloud-only options do not break Vision startup or normalization. Null, falsey and wrong-type configuration, absent/numeric/null/container sensor identities, bool-number/dict-order equality, post-read failures and calibration correction are covered. The prior 31-test logs and source manifest are preserved under `before_deferred_options_*`; that intermediate suite had 35 tests.

## Stage A: lossless JSON integers

`before_lossless_integer/` preserves the preceding 35-test candidate's manifest, logs, README and known-boundary results without relabeling their hashes. `red_lossless_integer.txt` records five failing differential groups before the bigint correction.

The new header-only `policy_integer_json.hpp` defines `IntegerJson`, `parse_integer_json`, `dump_integer_json`, exact `json_is_integer`/`json_integer`/`integer_json`, and an Integer serializer. Before any ordinary parse can round a large integer, a token-aware scan preserves the literal. An injective temporary marker key is selected after decoding all original string literals; the real parser's value callback restores the exact integer at the original parse point. The returned tree stores large integers in nlohmann's dedicated binary scalar category with a reserved subtype. Input JSON cannot construct that category. Unknown user objects/strings, marker-like content, duplicate keys and keys that merely look like binary JSON remain ordinary input values. Integer output is an exact decimal number token, never an object or string tag. NaN/Infinity and overflowing floating literals retain their earlier behavior. The helper has no Python dependency or separate library target.

Adapter calibration epochs and budgets now use the shared `Integer=boost::multiprecision::cpp_int`; ImageBox uses the shared exact dimensions and integer-or-double `PixelScalar`. Width/height positivity, coordinate finite checks, all four validation slots and final mixed numeric bounds retain Python ordering. `adapter_integer` returns Integer; `adapter_int64` retains the explicitly bounded timestamp conversion. Metric Vec3/covariance, Stamp and Version representations are outside this patch. Parent owns the ROS health assignment-time uint64 check, including state changes before that output failure.

The final 47-test suite adds exact epochs at 2**63, UINT64_MAX, UINT64_MAX+1 and 400 decimal digits; exact int/int and mixed int/float image edges; big raw budgets; deferred epoch changes; unknown bigint retention; duplicate keys; marker collisions; parser syntax and error precedence; and 1,200 exact random integer/float round trips. The valid large-floating-token regression found during peer integration is retained explicitly. The probe itself uses lossless parse/dump, and convenience packet inputs are converted to identical Python String bytes before dispatch so budget checks compare the same wire text.

`python_integer_digit_limit()` reads and validates PYTHONINTMAXSTRDIGITS once: default/empty is 4300, 0 disables the limit, and valid nonzero values are 640..INT_MAX. Parse, int-from-string and decimal dump share that setting. Tests use independent Python/native subprocesses with limits 0, 640 and 4300, and reject malformed/too-small environments. `python_digit_limit_environment_authority.json` retains the installed Python's detailed acceptance results. Parent calls the validation at its main-entry boundary before ROS setup. Runtime Python sys.set_int_max_str_digits has no counterpart inside this native process.

## Preserved inherited behavior and limits

* Installed `read_points` ignores `row_step` and flattens `width*height` records at `point_step` stride. Padding bytes can therefore become points, exactly as the source does.
* Opposite-endian input swaps only selected x/y/z fields in place. Reusing a packet swaps them again; overlapping selected fields can swap the same bytes more than once. Original field order, duplicate-name and unused-field validation are preserved.
* NaNs are skipped only for non-dense clouds. Infinity is retained by the reader. Dense NaN ordering follows Python min/max comparisons; later contract construction rejects nonfinite metric geometry. Empty/all-filtered/over-budget clouds provide no free-space evidence.
* `max_packet_bytes` counts Unicode characters for String input. Generated point cloud JSON uses Python default escaping, spacing and float representation for that budget.
* Both source `now()` calls occur in order; the second result is unused. Each observation gets its own monotonic receipt call. TF uses the capture timestamp, and arbitrary quaternion coefficients follow the source rotation formula.
* The previously recorded >INT64_MAX calibration/image differences are closed for the Stage A fields. `integer_boundary_results.json` now shows those inputs accepted on both sides. The capture-nanosecond-plus-timeout overflow remains a separate Stamp domain difference; `integer_boundary_audit.py` reproduces it, and its previous results remain in `before_lossless_integer/`.
* Escaped lone-surrogate strings remain a separate Stage B task: `surrogate_boundary_result.json` records measurement_id `\ud800`, accepted by Python and rejected by this parser before clock calls. Ordinary Unicode scalar-value strings and UTF-8 budgets are covered. The integer patch does not authorize retiring the Python entry point or claim equivalence for every Python-object domain.


## Reproduce

From any directory run the evidence `reproduce.sh` with the baseline Python digit limit (4300); individual subprocess tests exercise alternate configured limits. It builds only in `/tmp/codex_policy_observation_adapters_20260921` by default and runs the dedicated test module. Set `POLICY_ADAPTERS_SANITIZE=1` for the sanitizer build; `POLICY_ADAPTERS_BUILD_DIR` selects another private directory. Dependencies are g++, Python 3 with pytest/NumPy, nlohmann-json and Boost headers, and the repository sources. No shared installation, runtime Python edit or robot process is used.
