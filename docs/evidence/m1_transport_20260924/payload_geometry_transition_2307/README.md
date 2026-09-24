# scene13 ATTACH: distinguish unavailable geometry from a changed context

This correction follows the coordinator's observed scene13 failure and explicit authorization. Existing issue clocks, historical pauses and resource records remain authoritative. Only `src/hold_executor.cpp` and the existing `test/verify_full_manipulation.py` were changed. The coordinator is the sole build, installation, protocol-test, actual-scene and Git owner.

## Confirmed first cause and resource disposition

scene13 used task `fixed_transfer_dfeca583499e49559238c52c5633f08b`, PICK context `cf1b0542156b4ce49828e35f0ad35f1f_op_1_PICK`, and domain 78. The zero-preload simulation candidate passed `GRASP_CONFIRM` and submitted the physical attachment command. This repair does not modify that gripper setting or the MTC candidate.

The actual result's `geometry_records[552]`, received at ROS 106.641 s, is complete and attachment-confirmed. Record 553 at ROS 106.663 s is `ATTACHMENT_UNCONFIRMED`, incomplete, and has empty model/attachment revision fields. Record 565 at ROS 106.984 s is complete again, with the same source, clock and model, and the new attachment revision. The geometry publisher's `message()` and exception path construct the incomplete message with source, sequence, published time and frame only; its default model revision and clock epoch do not assert a new model or epoch.

The original domain journal orders `physical_submission` at line 68, `stop_requested` with `MTC_CONTEXT_CHANGED` at line 69, `physical_applied_scene_submission` at line 71, and quarantine at line 72. Lease `issued_at` values are renewal timestamps, not event timestamps. Observer line 151013 still reports `WAITING_FOR_PHYSICAL_APPLICATION` at ROS 106.639 s; line 151154 reports the latched first cause `MTC_CONTEXT_CHANGED` and subsequent cleanup reason `GEOMETRY_UNCONFIRMED` at ROS 106.678 s.

Thus the failure precedes a successful payload commit. The old `advance_plan` compared the incomplete message's empty `model_revision` with the confirmed reference before reaching its existing payload-transaction return. It incorrectly classified missing context evidence as changed context. The cleanup geometry reason is not substituted for the first cause.

The actual parent terminal status is 6 with `resources_released: false` and `RESOURCE_RECOVERY_REQUIRED:MTC_CONTEXT_CHANGED`; disposition is `UNRESOLVED`. Process-owner shutdown does not release those business resources. The original domain 78 journal is unchanged. `scene13_first_cause/` contains a copy, exact status-line excerpts, indexed geometry/result excerpts and hashes identifying the full original files.

## Minimal correction

- Publisher `source_id` is compared unconditionally because even an incomplete message carries it.
- With no payload transaction, all original clock/model/attachment-version comparisons remain in effect. During a payload transaction, clock/model fields are interpreted only from complete, attachment-confirmed geometry. A complete message with a genuinely different clock or model still stops with `MTC_CONTEXT_CHANGED`.
- Raw-observation checks, stopped-base checks and the original base frame/2 cm/0.02 rad motion check retain their order. The payload return remains before any trajectory dispatch; it was not moved to the top of `advance_plan` or `tick`.
- `advance_payload` still runs. Actual joints and claims, physical application, fresh authoritative ledger, matching geometry revision, original source/clock/model, actual scene readback, remaining-path revalidation and final readback remain required. Its original deadlines, including the 30 s transaction bound and 3 s fresh-inventory/frame waits, are unchanged.
- No unknown message updates `reference_geometry_`. Its attachment revision is still committed only at the existing successful payload transaction boundary before `next_stage`. No reason-string whitelist, fallback geometry or new abstraction was added.

## Exactly two targeted protocol cases

Both cases reuse the existing six-stage fixture with synthetic controller/MTC/physical endpoints and a real C++ ledger. They do not establish real Gazebo/MTC planning, full Transfer, hardware or performance acceptance.

1. `--mode delayed_ledger --operation PICK`, coordinator-reserved domain 41: incomplete geometry now matches the real publisher's unset version fields while the separate actual joint-state stream remains available. Assertions require at least one such observed publication, no later controller submission before payload revalidation, subsequent independent scene readback, the original 30 child Goals across five motion stages, final measured Hold and cancellation/release. `geometry_samples.capture_ns` retains the separately published joint capture used for independent settling checks; incomplete geometry itself has an unset header stamp and is excluded from settling evidence.
2. `--mode geometry_model_changed --operation PICK`, coordinator-reserved domain 42: once revision 2 is confirmed, publish complete geometry with the original source and epoch but a different model. Require the original `MTC_CONTEXT_CHANGED` first cause, exactly the preceding 18 child Goals, no LIFT, no payload revalidation or transaction commit, and an unresolved/quarantined result. It must not turn a real model change into a wait followed by success.

All existing UUID, endpoint, source/steady settling and first-cause assertions are retained. This window parsed the modified Python file with `ast.parse` only; it did not import ROS, run the fixture, build or launch processes. Runtime verification is pending the coordinator's two runs.

## Freeze and handoff

Parent source SHA256: `7dc3777c424f0b32a767f22bcf90f77a6e54ab29007aef0d7a319a1e38363a50`.

Existing fixture SHA256: `252b7c1e6e4f4c680151c3cdb94f8d1c56de43c6892a567638184023f7e89c32`.

Exact before/after copies, patch and manifests are retained here. The native build directory is `runs/m1_transport_20260924/continuous_build`; the relevant target is `trajectory_executor`. The existing physical fixture is that directory's `full_physical_fixture`. The ledger executable is `/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_20260923_065115/install/astribot_s1_payload_state/lib/astribot_s1_payload_state/payload_state`.

The coordinator received the two full invocation argument lists, using a fresh native install prefix and unique `m1_full_...` partitions for domains 41/42. Its candidate must preserve the frozen zero-preload MTC at `runs/mainline_20260924/rigid_grasp_2302/mtc_install`; that manually installed prefix has no `local_setup.bash`, so its libraries, package and Python paths require the coordinator's existing explicit overlay method. Neither this directory's timestamps nor the new build prefix reset a problem timer. Build/protocol/actual-scene results must be appended with their own source/binary bindings; the handoff manifest records the unverified-at-freeze state.

## Coordinator results, independently read back after freeze

The new native build, installation and dependency resolution passed. Executor SHA256 is `84b8c2d127607a14e3936672e6597f4ea6daf395e7f609717c778695a0999b99`. The recorded dependency listing resolves the native libraries from the new prefix and the scene-signature library from `rigid_grasp_2302/mtc_install`. Read-only hash checks matched all three native candidate artifacts, the rigid MTC executable (`cfe4b67ec5fe42def9c456b02eeb69ef50dab6adce1ba4f89cc48b3c07acd6e6`), its scene library, both frozen source files and each protocol's four recorded executable/script inputs.

| Actual targeted protocol | Verified raw result |
|---|---|
| Domain 41: delayed ledger and unavailable geometry | PASS. 30 controller Goals across five motion stages, including six coordinated controller Goals during LIFT; one payload transaction commit; measured final Hold followed by requested cancellation, terminal status 5, `TASK_CANCELED`, resources released, journal final phase 0. |
| Domain 42: complete geometry with a different model | PASS as a negative case. 18 controller Goals across the first three motion stages, zero LIFT Goals, zero payload revalidation/commit; first cause `MTC_CONTEXT_CHANGED`, terminal status 6, resources not released, journal final phase 5/quarantined. |

For domain 41, the raw fixture recorded **three incomplete geometry publications in total, one within the physical-command-to-payload-revalidation interval**. The other two belong outside that transaction interval and are not counted as additional transaction checks. The transaction-window publication has unset model/attachment revisions and clock epoch zero; there are zero later controller submissions between physical submission and payload revalidation. The subsequent independent scene readback and original source/steady settling assertions passed. Six LIFT Goals are six controller participants in one LIFT stage, not six lifts.

Both raw reports contain `assertions_passed`. Each lists three owned child processes exiting with code 0, six total across the two cases. The coordinator also reported both driver processes exiting with code 0. The expected domain 42 quarantine remains a rejection result, not a successful release. These outcomes do not change scene13's unresolved domain 78 disposition.

`verified_protocol/` retains 21 unmodified records: raw protocol reports/logs/ledger journals, both resource journals, candidate bindings, build/install/dependency logs, the exact coordinator launch scripts and overlay, and protocol summary. Its manifest records provenance, hashes and the independently checked counts. No protocol was repeated and no product/test code changed during this verification. The original freeze manifest remains the contemporaneous pending-validation record.

The coordinator is preparing actual scene14/domain77 with this native executor and the same rigid-object MTC. Full Transfer and actual-scene acceptance remain pending; the two synthetic protocol passes establish neither long-duration stability nor runtime performance or hardware acceptance.
