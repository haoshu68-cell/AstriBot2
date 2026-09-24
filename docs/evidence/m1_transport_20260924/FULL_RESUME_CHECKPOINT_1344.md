# Full candidate recovery checkpoint — 2026-09-24 13:44 +08

Original integration start 11:28, pause 12:28, explicit resume 12:44 remain unchanged. This checkpoint does not restart the problem clock. No protocol remains active. Full Gazebo/hardware acceptance and M3 integration are not performed.

## Candidate identity

- Installed trajectory_executor: `8fb2d6d475c8fd7bcfc5752677881e59e50a977316da94824595aef18fe4b42d`.
- Installed libpayload_scene.so: `40d1d72919231d53fd72ea16c7b2a2b738bf871e28971a508b354a036b8592f4`.
- The native late-raw observation invalidation repair has source review and 5/5 payload_scene helper tests. Products were not changed for controller startup readiness.
- Earlier normal PICK domain229 and PLACE domain226 passed with the preceding `bcf54ad9…` executor. These do not verify the repaired candidate's normal paths.

## Preserved failures and observed progress

Domain232 failed admission with CONTROLLER_CLAIMS_EXPIRED; the intended raw fault was never exercised. Controller reads occurred at monotonic 84254.562749587 and 84255.062749469 before goal rejection at 84255.507747182. They were approximately 500ms apart, and the driver recorded a response-send timeout. The earlier description that the first query followed the goal was incorrect. All three child processes exited 0. Its persistent records remain untouched.

The verification fixture now records response preparation, waits for two consecutive responses less than 200ms apart, a later native status, and a first-response age below 300ms. This observes normal query/response cadence rather than extending product freshness limits or repeatedly submitting goals. The exact prior fixture is archived as driver_before_controller_handshake.py. No new product readiness interface was added.

Domains233–237 were not started. ROS2 Humble's documented default DDS/UDP domain maximum is232. The coordinator withdrew those allocations; read-only process and journal/lock checks selected26–30, with a new graph preflight and unique Ignition partition per actual run. Only26 was started.

Domain26 established the handshake (49.9ms between responses, later native status), accepted the task, and exercised the late unpaired raw revision during final scene readback. It submitted18 JTC goals before the transition and **zero afterward**, never reported final Hold, and never released resources. The native journal records stop_requested/PAYLOAD_RAW_REVISION_CHANGED and ends quarantined, phase5, side_effects=true. This provides evidence of native consumption and rejection, unlike the old candidate's publication-only RED timing.

The driver nevertheless exited1 because it required PAYLOAD_RAW in the final action result reason; subsequent geometry invalidation changed that result to RESOURCE_RECOVERY_REQUIRED:GEOMETRY_UNCONFIRMED. Native status authority_reason and the original journal preserve PAYLOAD_RAW_REVISION_CHANGED. This assertion failure is retained verbatim and is not relabeled a fully passing test. All three child processes exited0. No retry has run.

## Recovery entry and remaining work

1. Independently derive the expected late-revision assertions from domain26's saved native journal/status, including no payload confirmation, no subsequent submissions, and quarantine; retain the original driver failure. Do not claim a whole successful run merely by changing the text assertion.
2. Make the fixture's error provenance check use the native stop record/status while preserving the final outcome and all safety assertions. No product reason/expiry change is required for this check.
3. Domains27 UNKNOWN,28 normal PICK,29 normal PLACE,30 legal150ms ledger delay remain unstarted. Resume only under the coordinator's timing/measurement-window decision; do not erase/reuse26 or232 journals/locks.
4. Full actual simulation, performance comparison and long-duration stability remain unverified. Existing frozen first-stage next_install and root runtime candidates stay unchanged.

Source records and exact hashes are in [full_resume_checkpoint_1344/manifest.json](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/m1_transport_20260924/full_resume_checkpoint_1344/manifest.json). Original run files remain under runs/m1_transport_20260924/full_six_stage. No Git stage/commit, old-artifact cleanup, shared installation update or memory write was performed here.

Domain reference: [ROS2 Humble domain documentation source](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Domain-ID.rst).

## Post-pause offline audit (no additional runtime or product edits)

The saved domain26 evidence passed the derived audit in `full_resume_checkpoint_1344/audit_domain26.py`; result `domain26_derived_audit.json`. It verifies18 total/0 subsequent JTC, six matching successful UUIDs per first-three motion stages, independent source/steady settling windows of approximately0.640/0.560/0.600s, native PAYLOAD_RAW stop, no payload transaction confirmation, no Hold, and final quarantine. The original driver exit1 is retained as a failed end-to-end assertion, separate from these passing offline checks.

A remaining diagnostic defect is confirmed by source and saved results: `stopping()` initially stores PAYLOAD_RAW_REVISION_CHANGED in `reason_`, but cleanup `measured_safe()` calls `geometry_fresh()` which overwrites that shared string with GEOMETRY_UNCONFIRMED. The final Action result and quarantine reason use the overwritten value. The original native stop record remains available. This is a first-cause return gap, not evidence of unsafe release; no product fix or assertion relaxation has been made after the pause.

`full_resume_checkpoint_1344/recovery_identity.json` records source/installed-product/fixture hashes and the three captured PID/start-ticks checks. None of those owned identities is live. Domains27–30 remain unstarted. The coordinator requested stopping new code and actual runs at this checkpoint; recovery items above are pending future work, not authorization to resume them now.

Root archival closeout: all nine original/copy hashes and thirteen current identity hashes matched. `full_resume_checkpoint_1344/unaccepted_native_source.tar.gz` preserves all 41 package source files, with each archive member verified against `source_snapshot_manifest.json`. This is a recoverable development snapshot, not accepted product code; the current candidate build/install and original failures remain in place. Root records this evidence in Git without staging the shared product working tree.
