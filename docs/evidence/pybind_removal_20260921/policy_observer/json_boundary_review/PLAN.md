# Remaining JSON equivalence: bigint first, lone surrogate separately

Scope: read-only review of current native policy contracts/health/fusion/observer/adapters and frozen Python observer authority recorded at Git 965bf057. All new artifacts are under `/tmp/policy_json_equivalence_review`. No repository edits, Git operations, ROS initialization, DDS graph, launch, or robot activity. This is offline source/message evidence, not a running observer acceptance test.

## 1. Observed final observer behavior

`observer_reference.py` executes literal frozen `accept_observations` and `tick` methods with real frozen policy cores, deterministic time, no robot/TF-dependent geometry, and a normal image-box packet. Publishers call real installed ROS `serialize_message` and record completed publications. Node/executor setup is bypassed. Frozen observation stays unassociated and therefore exercises the normal uncertainty path.

Every row below reaches `errors=0`, `vision_count=1`, one unassociated observation and one health record before tick.

| Changed packet value | Tick result | Completed health / observer publications |
|---|---|---:|
| calibration_epoch = 2**63 | succeeds | 1 / 1 |
| calibration_epoch = 2**64-1 | succeeds | 1 / 1 |
| calibration_epoch = 2**64 | AssertionError in generated SensorHealth.calibration_epoch setter | 0 / 0 |
| image width = 2**64 | succeeds | 1 / 1 |
| image width = 10**399 (400 decimal digits) | succeeds | 1 / 1 |
| unused field = lone escaped high surrogate | succeeds | 1 / 1 |
| measurement_id = ordinary prefix + lone high surrogate | succeeds | 1 / 1 |
| sensor_id = ordinary prefix + lone high surrogate | UnicodeEncodeError during real ROS serialization | 0 / 0 |

The too-large epoch is **not an ingress error**: fusion/health have already been updated, and no observer `errors` increment or warning occurs. `tick` assigns the epoch after capture/expiry conversions and before finishing the health message. Narrowing or rejecting it during normalization changes both transaction and error ordering.

Process consequence: frozen `main` catches only KeyboardInterrupt and always performs cleanup. Installed Humble MultiThreadedExecutor._spin_once_impl checks completed task.result(), which rethrows its exception. `exception_propagation.py` runs that actual installed exception-check code and the literal frozen main with inert init/node/executor setup; the AssertionError escapes main after all three cleanup steps. Thus ordinary observer execution has an unhandled callback failure, rather than recovering as an adapter error. No live executor/process failure was launched or claimed. The observed generated setter assumes normal, non-optimized Python (__debug__ true), as in the recorded runtime.

## 2. Field scope and native representation

| Field / ingress | Actual use | Minimal representation |
|---|---|---|
| NavigationExecutionStatus.sequence, RobotEnvelope.epoch, V2 envelope wire versions | Wire uint64, ordering and identity | Keep existing uint64 correction; do not widen merely because Python int exists |
| CameraInfo.width/height | Wire uint32, positivity validation; current original callback then rejects numpy.float64 calibration coefficients | uint32 bridge into contract integer; current int64 is sufficient for the actual wire input |
| Observation.calibration_epoch, SensorHealth.calibration_epoch, CameraCalibration.calibration_epoch | >=0 validation, equality, downgrade comparisons, copy into replacement observation/stale health; observer candidate epoch +1 | Signed arbitrary-precision integer (`boost::multiprecision::cpp_int`) internally, with nonnegative validation at original constructor slot |
| SensorHealthRegistry.record epoch, CameraCalibrationRegistry.calibration lookup epoch | `<`, `<=`, `!=` only | Same arbitrary-precision type end-to-end |
| AdapterOptions / raw options calibration_epoch, PacketMetadata.calibration_epoch, set_calibration_epoch | Python int(...) conversion, normalization, health preflight, cloud-generated JSON | Same arbitrary-precision type; retain original int conversion and exception ordering |
| HealthMessage.calibration_epoch output | Actual wire uint64 | Checked conversion only at current tick setter point; throw on >UINT64_MAX before publish, after preceding state changes/time checks |
| ImageBox.image_width_px/image_height_px | Exact int >0; coordinate bounds; retained as unassociated data | cpp_int, no uint64 cap; no metric geometry arithmetic depends on dimensions |
| CameraCalibration.image_width_px/image_height_px contract fields | Exact int >0 and full calibration equality | cpp_int for unrestricted native contract parity; ordinary CameraInfo origin still bounded uint32 |
| ImageBox xmin/ymin/xmax/ymax | finite/nonnegative validation, strict interval bounds against exact integer dimensions; retained/copy-only downstream | Preserve numeric kind using a local integer-or-double pixel scalar, with exact int/int and mixed int/float comparisons |
| max_packet_bytes/max_observations/max_points options | int(...) then compare to lengths/counts | cpp_int limits; compare widened count directly, never narrow a valid large positive limit |

The calibration epoch is distinct from Stamp.epoch and execution Version counters. Do not widen every timestamp, metric scalar, index or counter in this patch. Source Stamp storage and independent remaining arbitrary-Python domains need separate scope decisions; this report does not certify all such domains.

## 3. Exact image bounds are part of the bigint patch

`image_edges.py` against the frozen contract shows:

- width=UINT64_MAX, xmax=float(2**64): rejects `INVALID_INPUT: image.box.x`.
- width=2**64, same float xmax: accepts.
- width=UINT64_MAX, xmax=integer UINT64_MAX: accepts.
- 400-digit width with xmax=1.0: accepts.
- 400-digit width and a 399-digit xmax integer: rejects `INVALID_INPUT: xmax_px` because Python math.isfinite(int) conversion overflows.

Converting every pixel coordinate to double before validation destroys the third result. Converting dimensions to double destroys the first result. A scoped `PixelScalar = variant<cpp_int,double>` preserves these boundaries and signed floating zero without touching metric Vec3/covariance arithmetic. Preserve width then height errors, all four finite checks, then x and y ordering checks. Integer finite validation must mirror the Python int-to-float overflow threshold; cpp_int storage alone does not make every giant coordinate finite. No tolerance change is needed.

## 4. Stage A: lossless integers, with ordinary string parsing unchanged

1. Freeze focused red cases above against current native adapter/observer probe and frozen authority. Include epoch 2**63, UINT64_MAX, UINT64_MAX+1, and a downgrade/out-of-order operation after an accepted large epoch. Keep the original tick-time failure in the expectation.
2. Introduce one internal arbitrary-integer type and lossless JSON integer storage. Retain integer token lexemes **before** the early ordinary `J::parse` success path. Standard nlohmann parses some >uint64 integer literals as double successfully and loses low bits; 400-digit integers instead fail number overflow. A fallback invoked only after parse error misses the first case. nlohmann SAX also rejects nonfinite numeric results before its number_float callback, so that callback alone cannot fix oversized literals.
3. Use a token-aware integer preservation layer with typed, non-user-collidable storage in the decoded tree (a dedicated JSON scalar wrapper is sufficient). Do not encode bigint as an ordinary user-visible string/object, do not convert through double, and do not widen integers inside JSON strings. Preserve unknown fields, duplicate-key behavior, normal parser syntax errors and the existing NaN/Infinity/overflowed-float support. This stage does not require implementing lone-surrogate strings.
4. Extend int(...) conversion to cpp_int for booleans, integer values, finite float truncation, and currently supported Unicode decimal/underscore strings; keep invalid type/NaN/infinity exceptions in original order. Keep exact-integer-only ImageBox dimension checks distinct from int(...) conversion. Update JSON equality/truth/conversion/dump helpers for the new integer category; printing must emit a numeric decimal token. The observed Python default decimal digit limit is 4300; preserve the configured authority's policy instead of inventing a smaller cap.
5. Propagate calibration epochs and image dimensions through the exact fields listed above, retaining registry mutation and error order. Compare budgets against cpp_int directly. Introduce exact pixel scalar comparisons. No fusion association/risk algorithm changes are needed; those only retain the image box or copy its epoch.
6. Check the ROS health epoch range at its existing assignment stage. Preserve successful complete processing for huge image dimensions and accepted state before too-large epoch output failure. Then run existing contract/fusion/health/adapter/observer regressions plus focused boundary cases. Direct wire fields continue using their bounded native types.

## 5. Stage B: lone-surrogate string preservation, independent of bigint

Parent's proposed NUL-tag string-token preprocessing is a viable separate direction: scan JSON string escapes; replace only isolated high/low surrogate escapes by reversible unique tags; protect genuine NUL/tag text against collision; retain valid surrogate pairs; parse normally; recursively restore all object keys and values to canonical WTF-8 internal std::string. This requires an injective restoration mapping, not a tag chosen only from unescaped raw input. Never replace with U+FFFD, discard characters, or reject all strings containing a surrogate.

The corresponding dump helper must escape stored surrogate code points back to `\udxxx`, while emitting ordinary characters under existing JSON rules. Key/value traversal, duplicate-key resolution after restoration, ordering/equality, Unicode whitespace label rules, original NUL, marker collisions, adjacent high/low pairs, and original raw packet character-budget accounting all need targeted checks. Native byte ordering on canonical UTF-8/WTF-8 strings can preserve code-point ordering, but malformed byte strings must not be mistaken for canonical decoded strings.

Retain failure stage: unused surrogate field and measurement_id have successful final observer outputs. A sensor_id surrogate reaches stored state but fails during health serialization. An explicit UTF-8 check for native ROS output must occur where original serialization happened, after all health messages and their numeric/time checks have been constructed. An early per-field/ingress guard changes error precedence, e.g. a surrogate sensor plus an oversized calibration epoch must still fail first at the epoch setter. TF input strings are another string-to-ROS boundary to audit, rather than grounds for a global label rejection.

Minimal Stage B cases: key and value lone high/low surrogate; adjacent valid pair; mixed paired/lone sequence; original NUL; literal marker collision; ignored and measurement-id successful paths; health sensor/frame output failure; simultaneous surrogate plus invalid epoch; raw UTF-8 versus escaped form budget edge. Implement and validate separately from Stage A.

## Reproduction

For observer cases, source the ROS distribution and installed message-library paths only:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/astribot_navigation_msgs/local/lib/python3.10/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/astribot_navigation_msgs/lib:$LD_LIBRARY_PATH
PYTHONDONTWRITEBYTECODE=1 python3 /tmp/policy_json_equivalence_review/observer_reference.py epoch_uint64_plus
PYTHONDONTWRITEBYTECODE=1 python3 /tmp/policy_json_equivalence_review/observer_reference.py image_400_digits
PYTHONDONTWRITEBYTECODE=1 python3 /tmp/policy_json_equivalence_review/observer_reference.py measurement_surrogate
PYTHONDONTWRITEBYTECODE=1 python3 /tmp/policy_json_equivalence_review/exception_propagation.py
PYTHONDONTWRITEBYTECODE=1 python3 /tmp/policy_json_equivalence_review/image_edges.py
```

All eight observer results are saved in `observer_results.json`, image boundaries in `image_edges.json`, and exception cleanup propagation in `exception_propagation.json`. `review_sha256.txt` identifies reviewed authority/current source snapshots and installed generated-message/executor files; concurrent root changes after that snapshot are not included in this review.
