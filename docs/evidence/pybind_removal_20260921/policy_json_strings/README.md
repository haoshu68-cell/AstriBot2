# Stage 6B: JSON strings and original ROS conversion boundaries

This checkpoint closes the recorded lone-surrogate JSON difference. It does not retire a production entry or any binding. The remaining Stamp representation/failure-stage difference and full controller closure are still open; see ../policy_stamp_boundary_review and ../../../CPP_POLICY_MIGRATION_NEXT_20260921.md.

## Source identity and implementation

source_manifest.json identifies 200 isolated build/test inputs: Git b7a2240b plus the seven paths in scoped_paths.json. The same scoped sources were compiled, installed and used in the final 108-case ROS run and benchmark. Concurrent navigation-zones edits are excluded. The native geometry install and rebuilt Python geometry oracle are the Stage 6B integer recovery artifacts documented in ../policy_json_integer_runtime; the benchmark records their selected oracle hash. Old Python is used solely as validation authority in this experiment, not embedded in the C++ executable.

JSON preprocessing preserves original code points in values and keys, including isolated high/low surrogates, valid pairs, NUL, and strings that resemble internal escape markers. Internal strings use WTF-8 for the previously valid Python surrogate domain. JSON output re-escapes those code points. Generated cloud-packet byte budgets retain Python's ASCII escaping, spacing and representation rules. This avoids silently replacing characters or rejecting whole packets before their original consumers.

The original Humble boundaries are retained: TF conversion validates UTF-8 before checking embedded NUL; health output constructs every message and checks numeric/time fields before string serialization. At health serialization the entire string is UTF-8 validated before the generated Python converter's NUL truncation. Internal sensor identity is not truncated. Raw CDR assertions avoid Python deserialization hiding a wire difference. ros_string_authority.json records the actual installed Python converter and TF binding behavior; no ROS node was needed for that authority check.

## Validation and failures retained

* Initial core RED: 17 failed / 8 passed (25 then-existing tests), in red.log. Expanded final string suite: **28/28**, including all 2,048 surrogate code points and 1,000 seeded strings, duplicate keys, marker collisions, adapter field/call order and exact packet-budget edges.
* Existing adapter suite: **47/47**. Both suites also pass ASan/UBSan/LSan, **47 + 28**, with their compiler and test logs retained. Repeated runner results are not additional independent cases.
* Native-only configure/build/install succeeded; **14/14 CTest** passed. install_audit.json records six installed ELFs, resolved dependencies without libpython/pybind, and no installed Python source. This selected-install audit is not a whole-project binding gate.
* Final installed ROS comparison: **54 scenarios on each implementation, 108/108 pytest cases**. This includes 41 earlier scenarios, six string acceptance cases, four health failure-order cases, one raw NUL wire case, and two TF conversion-order cases. Raw per-process outputs/logs are in installed_ros_final/. The earlier 104-case pass log predates the two additional paired TF cases.
* Intermediate ROS RED is **4 failed / 1 passed**. Three failures are real: the codec-only candidate incorrectly published health containing lone surrogates. The fourth was a test assertion that accepted only an epoch-specific error label; the already-correct native numeric boundary reported `ROS unsigned integer out of range`. The fixture now recognizes that message while still requiring numeric failure before Unicode failure. The NUL wire test already passed before the explicit guard. pre_wire_guard_* identifies this intermediate candidate; ros_wire_red/ retains its logs. ros_python/ records the corresponding original-runtime authority run.
* Original Python SIGINT can exit 1 with a double-shutdown traceback. The fixture checks bounded termination; this is not a claim of matching successful shutdown exit codes.

## Paired performance

Four AB/BA pairs per workload; 20 warmup and 100 measured frames per process, **1,600 measured frames**. Semantic outputs match for every measured frame (timing fields excluded, float rel=2e-12 / abs=3e-12). Each table value is the median of four per-process summaries. The driver used CPU 0 and each observer CPU 1 on a shared host. Exact configuration, source paths and binary/oracle hashes are in benchmark/summary.json; all raw runs are retained.

| Tracks | Metric | Python | C++ | Reduction |
|---:|---|---:|---:|---:|
| 8 | tick P50 ms | 1.592295 | 0.156540 | 90.17% |
| 8 | tick P95 ms | 1.729231 | 0.187608 | 89.15% |
| 8 | whole-process CPU ms/frame | 6.750000 | 0.700000 | 89.63% |
| 8 | publish-to-result P95 ms | 5.792596 | 0.794102 | 86.29% |
| 8 | RSS MiB | 62.402344 | 27.873047 | 55.33% |
| 64 | tick P50 ms | 3.687973 | 0.508508 | 86.21% |
| 64 | tick P95 ms | 3.962826 | 0.572632 | 85.55% |
| 64 | whole-process CPU ms/frame | 14.150000 | 1.950000 | 86.22% |
| 64 | publish-to-result P95 ms | 8.008611 | 1.051520 | 86.87% |
| 64 | RSS MiB | 63.953125 | 28.869141 | 54.86% |

Tick timing excludes the preceding vision callback. Whole-process CPU includes ROS callbacks over the measured window and excludes startup. External delay starts at clock publication after the fixture drains preceding input. RSS is a short-window endpoint, not a leak or soak guarantee. Worst delays and RSS growth remain in raw summaries; not every individual frame is asserted faster. These are paired Python-to-C++ measurements, not a causal comparison against the earlier native integer checkpoint on a shared host.

No complete controller, Gazebo/Nav2 closed loop, hardware, long-term soak or safety-stop acceptance is claimed. Reproduce from the recorded commit using reproduce.sh and a new owned build/install directory; source/binary caches under runs are not production entries.
