# Stage6B — preserve complete message integer widths

This is implementation and isolated/offline regression evidence for the observer integration's native integer boundaries. It is a new Stage6B record; existing fusion, health, envelope, and Stage6A manifests remain historical and were not rewritten.

## Defect and preserved interfaces

ROS `NavigationExecutionStatus.sequence`, envelope epochs, and navigation version fields use uint64 values. The prior native context and `policy::Version` used signed int64 storage. Valid values at/above `2**63` were ignored, narrowed, or rejected. `CameraInfo` dimensions use uint32, while native image/calibration dimensions used signed `int`, rejecting valid dimensions above `2**31-1`.

The narrow correction changes:

- `navigation::ExecutionContext::task_wire(identifier,state,uint64_t)` consumes the complete sequence domain. Existing `task(...,int64_t)` still ignores every negative sequence and delegates nonnegative values to the same monotonic state machine.
- Context version counters and `version()` / `set_version()` use uint64. Goal changes, terminal ownership, equal/out-of-order suppression, path reset, map identity and localization calculations remain unchanged. In particular the parent's `policy::euclidean_norm` localization correction was retained.
- `policy::Version` stores uint64 counters. Its implicitly constructed integral `Version::Counter` inputs preserve signedness/type validity until the constructor's original goal/path/map/envelope/localization/clock validation order. Negative inputs and bool are rejected without silently wrapping; normal integer callers need no alternate runtime compatibility layer.
- `CameraCalibration` and `ImageBox` dimensions use int64, covering every legal uint32 dimension while retaining positive/negative validation and the original error order.
- The parent explicitly authorized the necessary downstream `Decision::committed_path_revision` correction: stored `optional<uint64_t>`, constructor input `optional<Version::Counter>`. FOLLOW/RETREAT checks validate the signedness/type flag before unsigned equality, at the original `VERSION_MISMATCH` check position. Existing optional signed and unsigned integer callers remain constructible; `api_compat.cpp` verifies that at compile time.

The downstream test reproduced a concrete error after widening Version but before widening committed revisions: `-1 == UINT64_MAX` incorrectly passed through implicit signed/unsigned equality. The corrected constructor rejects it, and accepts valid full-width committed revisions.

## Scope

Production edits are confined to the `ExecutionContext` declaration/implementation portion of `navigation_math.hpp/.cpp`, image/calibration dimensions plus Version and the explicitly approved committed revision portions of `policy_contracts.hpp/.cpp`. Test transport changes are limited to full-width version/committed parsing and task sequence dispatch in their corresponding probes. New tests extend existing fusion, health and observer suites.

No envelope files, production Python, shared installation, ROS stack, Git state, or historical evidence manifests were changed in this subtask. Root owns the observer callback's use of `task_wire`, uint64 envelope temporaries, camera dimension conversions, package integration, and full runtime checks.

## Tests and failures

- `red.txt`: **15 failed, 5 passed** against the previously built native probes, before source edits. Failures cover `2**63`, `UINT64_MAX`, negative validation order around full-width fields, and dimensions `2**31` / `UINT32_MAX`.
- `red_committed.txt`: **5 failed, 2 passed** before the downstream committed-revision correction: four rejected high-width FOLLOW/RETREAT inputs and the accepted `-1`/`UINT64_MAX` alias.
- `release_tests.txt`: **157 passed**.
- `ubsan_tests.txt`: **157 passed** with `undefined,float-cast-overflow`, no recovery, and halt-on-error enabled.
- `test_inventory.json`: fusion **68** (including all original 47), health **46** (all original 41), risk **20**, observer core **23**.
- `release_build.txt` / `ubsan_build.txt`: successful native rebuilds. The existing unrelated `scan_coverage` parentheses warning is retained; no new width warning remains. The six Python warnings during risk tests are expected overflow cases in the frozen oracle, not sanitizer diagnostics.
- `release_commands.json` / `ubsan_commands.json`: executed private compilation/link commands.
- `source_hashes.json`: final edited files, current shared dependencies, frozen references, and tested executable hashes.

Tests compare exact integer values and original contract code/field errors against the existing byte-frozen Python authorities under `test/reference/policy_fusion`, `policy_health`, and `policy_observer`. Coverage includes `2**63-1`, `2**63`, `UINT64_MAX`, `2**31-1`, `2**31`, `UINT32_MAX`, negative signed task inputs, terminal ownership, repeated/stale sequence values, version round trips through snapshots, full-width FOLLOW/RETREAT execution, negative/type validation precedence, and unchanged image/calibration metadata.

The separately labeled `test_wire_counter_overflow_is_explicit_and_leaves_context_unchanged` tests the native domain beyond `UINT64_MAX`. Path/map/localization increments throw before mutating counters or their supporting identity/transform state; after restoring representable versions, retry proves those supporting states were retained. This is a defined native wire boundary, **not** a claim to reproduce Python's arbitrary-size integer behavior outside the message domain.

`Stamp` time/epoch types remain int64 as instructed. Existing `PlanningSessionState` signed counters are outside this observer correction; this record does not establish full-width arbiter/planning-session migration.

## Reproduction

From the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 docs/evidence/pybind_removal_20260921/policy_wire_width/reproduce.py
PYTHONDONTWRITEBYTECODE=1 python3 docs/evidence/pybind_removal_20260921/policy_wire_width/reproduce.py --sanitizer
```

The script uses GCC, nlohmann JSON, OpenSSL, NumPy and pytest. It builds all affected pure cores and four probes under `/tmp/codex_policy_wire_width_20260921/{release,ubsan}` and tests the frozen references. `--build-dir` selects another private directory. The syntax-only API compatibility check is also included. No ROS launch, graph operation, installation or Git command is performed.

Evidence identity: task `Task6`; step `Stage6B-policy-wire-width`; test session `codex_policy_wire_width_20260921`; operator Codex subagent `/root/policy_health_core`; source node/topic not active (serialized pure-core replay); robot ID, pose source, map/scene physical revision, controller/SDK state and hardware calibration acceptance not applicable. Individual source/receipt timestamps, clock identity and versions remain specified in the deterministic input cases. No runtime `session.log` was created.

This record establishes code and offline behavior only. Full package integration, live callback transport/QoS, simulation and hardware acceptance remain separate work.
