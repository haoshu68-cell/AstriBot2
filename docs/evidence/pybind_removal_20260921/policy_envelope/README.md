# Task6 Stage6B — native policy envelope consumption

This is an **implementation + isolated/offline validation** record for `PolicyEnvelopeProfile`. It is additional to the earlier Stage6A core records at `f9790e29` / `965bf057`; those historical manifests were not rewritten. It is not an observer/node entry-point, live ROS, simulation, or hardware acceptance result.

## Scope and ownership

New native files are `policy_envelope.hpp`, `policy_envelope.cpp`, and `cmake/policy_envelope.cmake` in `ws_robot/src/astribot_s1_navigation_policy_native`. New test files are `test/policy_envelope_probe.cpp`, `test/test_policy_envelope.py`, and `test/reference/policy_envelope/`.

The parent additionally authorized two const map accessors in existing `policy_health.hpp`: `SensorHealthRegistry::records()` and `CameraCalibrationRegistry::records()`. These expose source records for adapter preflight/idempotence and do not mutate registry state. Their current header hash is recorded here; earlier health evidence remains historical.

No production Python, package entry point, launch file, shared install, shared ROS stack, Git index, or branch was changed by this subtask. Root owns top-level CMake and runtime integration.

## Implemented contract

`astribot::navigation::policy::PolicyEnvelopeProfile` accepts a baseline JSON already validated through the existing `load_policy_profile`, and a fixed/legacy mode. It exposes immutable `baseline`, numeric `value(key)`, `base_frame()`, `tracking_frame()`, optional canonical `polygon()`, optional native `envelope()`, typed legacy/V2 `accept`, `ready`, `confirms_applied`, and `stopping_distance`.

The runtime passes generated `RobotEnvelope` / `NavigationEnvelopeV2` values directly. Existing `validatePolygon`, `inflatePolygon`, `containsPolygon`, and `geometryHash` kernels are reused directly. No binding, Python fallback, duplicate profile loader, or final-protection approximation is involved.

Preserved behavior includes:

- Validation order and exact in-domain error strings; legacy rejection preserves state while malformed fixed V2 revokes applied identity, limits, and receipt state.
- Fixed V2 revocation intentionally retains its previous canonical footprint, matching Python.
- Lower epochs are ignored only after geometry validation; same-session epoch mutations and new-session behavior preserve the original ordering.
- Source and receipt freshness remain distinct, with exact integer nanoseconds, inclusive source/receipt lease equality, strict V2 deadline equality, clock identity, receipt rollback, and heartbeat source limits.
- Heartbeat confirmation only checks the already applied configuration; it never applies or refreshes the profile, and can succeed independently of current motion readiness as in Python.
- Configuration identity includes exact serialized Point32 XYZ, point order, nested limits metadata, and the complete original key; source timestamps, reason text, transport/navigation flags, and source state sequence remain excluded from that key.
- Geometry hashing retains signed-zero behavior; rounded hashes do not replace exact geometry identity. Nested limit values and negative-zero payload values retain their serialized values.
- Source nanoseconds are computed from legal ROS Time components with integer arithmetic. Comparisons avoid overflowing subtraction at the full native Stamp range.
- Stopping distance retains the envelope-specific formula, including its original handling of negative/nonfinite speeds and absent optional linear stop delay.

Typed mode mismatch calls are rejected explicitly (`fixed profile requires V2 envelope` / `legacy profile requires RobotEnvelope`). These are API misuse guards outside the selected ROS message subscription type; the Python wrong-object `AttributeError` text is not a transport contract. Accepted messages are copied as native snapshots; differential tests replay serialized messages, not Python object aliasing or post-accept mutation.

## Frozen reference and evidence

The reference files were copied byte-for-byte from Git `965bf057`, with original paths and SHA256 hashes in `test/reference/policy_envelope/manifest.json`. They load under an isolated test-only module name; geometry imports are temporarily redirected only while invoking the frozen oracle. No current production Python import is used. Every test run checks the frozen bytes.

- `red_missing_native.txt`: the initial failing legacy boundary test, executed before the native implementation/probe existed.
- `test_release.txt`: **99 passed**, 130 same-input cases, 2,336 state operations.
- `test_ubsan.txt`: **99 passed**, the same 130 cases / 2,336 operations, with `-fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all` and halt-on-error enabled.
- `stats_release.json` and `stats_ubsan.json`: actual replay counts.
- `build_release.txt`, `build_ubsan.txt`: successful warning-free private compilations.
- `compile_release.json`, `compile_ubsan.json`, `compiler.txt`: compiler/command evidence.
- `source_hashes.json`: current owned source, shared dependency, frozen reference, additive accessor, and tested binary hashes.

Each operation compares acceptance/error result plus readiness, active limit values, complete legacy message metadata, canonical footprint, and stopping distances. Floating values are compared exactly (including zero sign); no tolerance is applied to mask a decision or boundary mismatch.

Coverage includes malformed numeric/order cases, every configuration key field, excluded heartbeat metadata, same-epoch changes, session switches, receipt clock/epoch changes, nonfinite ignored Z, source/deadline equality, nanosecond values beyond `2**53`, legal ROS maximum timestamps, extreme native query timestamps, maximum uint64 wire identities, Point32 rounding neighbors, broad-phase and containment strict edges, signed-zero hashes, rounded-hash collisions, polygon validity/cardinality, four seeded state streams, and two seeded geometry streams.

Evidence identifiers: task `Task6`; step `Stage6B-policy-envelope`; test session `codex_policy_envelope_20260921`; operator Codex subagent `/root/policy_health_core`; environment Linux/GCC 11.4/Python 3.10; frame `astribot_torso_base`; clock `ros` with explicit test clock/epoch changes. `sample_stamp` / `receive_stamp` / `envelope_epoch` are per deterministic test input. Robot ID, map/scene revision, calibration, controller/SDK state, source topic/action, and motion pose source are not applicable to this pure message-core replay. No live `session.log` was created.

## Reproduction

From the repository root, with existing ROS Humble generated message headers, GCC, OpenSSL, nlohmann JSON, NumPy and pytest available:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 docs/evidence/pybind_removal_20260921/policy_envelope/reproduce.py
PYTHONDONTWRITEBYTECODE=1 python3 docs/evidence/pybind_removal_20260921/policy_envelope/reproduce.py --sanitizer
```

Both commands compile into owned `/tmp/codex_policy_envelope_20260921`, using the current source geometry header as an include overlay and existing installed ROS message headers read-only. `--build-dir` can select another private directory. There is no ROS launch, graph inspection, shared installation, or Git operation in this reproducer.

The CMake fragment registers `policy_envelope_differential`; its top-level integration, full-package CTest, callback synchronization, actual heartbeat transport/QoS, closed-loop timing, simulation, and hardware remain root integration / later acceptance work. This subtask alone does not establish removal of every runtime Python consumer.
