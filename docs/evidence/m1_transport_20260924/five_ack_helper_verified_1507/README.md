# Five-ACK navigation helper: three isolated protocols passed

Recorded on 2026-09-24 after the coordinator's `navigation_helper_drained` run. The exact recording time, source and binary hashes are in `manifest.json`. This is helper acceptance only; the combined PICK→NAV→PLACE executor has not been built or accepted.

## Finding and minimal change

The first run passed the missing-controller case and failed the normal and owned-cancel cases while waiting for a navigation Goal. Added diagnostics isolated the failures to valid ACKs for individual consumers, rather than the stop window or envelope binding. The original fixture publishes five ACKs to one subscription each cycle but invokes `spin_some()` once. Humble's executor collects work once in that call; queued ACK batches were not drained, leaving individual consumers stale.

The fixture now uses bounded `spin_all(5ms)` before the helper tick. Publication cadence, source stamps, envelope and ACK lifetimes, required consumers, and the source/steady 600ms stop window were not relaxed. Production changes only expose the particular envelope/permission/ACK/stop waiting condition and replace stale six-ACK diagnostic labels with five-ACK labels. The timeout preserves the last waiting reason.

## Actual results

- `SameLeaseLegUsesCurrentMapTransformAndPostTerminalStop`: PASS.
- `LegacyProtectionCannotReplaceControllerAndCancellationStillRequiresStop`: PASS.
- `CancelTargetsOnlyOwnedGoalAndTerminalDoesNotProveStopped`: PASS.

The coordinator-owned run exited 0, with `still_alive: []`. The full original group was rerun because all three cases share the changed fixture. The positive tests publish only the five required consumers; the negative test supplies legacy protection while omitting controller and still cannot dispatch navigation. Goal identity, cancellation and terminal-versus-stopped checks remain.

`first_failure.log` and `detailed_failure.log` preserve the original failures. `missing_domain_setup.log` records a separate attempt rejected by the fixture's required isolated-domain precondition before any protocol ran; it is neither a product failure nor a passed test. `verified.log`, `verified_gtest.xml` and `verified_result.json` are unmodified copies of the accepted run. Copied evidence and source hashes were checked against their originals.

This fixture provides synthetic fixed-envelope and Nav2 endpoints. It does not prove complete parent execution, real navigation, Gazebo manipulation, throughput, long-duration stability or hardware operation. It also does not authorize removal of task ownership, Hold, ledger, geometry, scene, source-lifetime or cancellation barriers.

## Parent checkpoint

The combined parent retains its original 14:10 start and 15:10 one-hour checkpoint. It is paused without further source work, ROS or motion while the coordinator completes the new navigation boundary, including the separately discovered arm-coupling placement issue. The parent source still has SHA256 `bb09030246d5d100769d53d12b2bf1a85ea02baffb5b2e4064481590ed8f26e0` and remains uncompiled/unaccepted. This helper repair does not restart the parent clock.

Resume from the existing `continuous_architecture_pause_1430` snapshot and this helper snapshot only after the coordinator's navigation prerequisites are met. The separately verified standalone six-stage candidate in `full_first_cause_verified` and its frozen installed archive remain unchanged. Build the combined candidate separately and verify its own success, cancellation, scene readback and resource-release contracts; do not inherit helper or standalone acceptance as whole-task acceptance. Git staging, commit and shared runtime scheduling remain coordinator-owned.

## Read-only interface findings for resumption

M2's probe review found that the combined parent publishes its internal operation context in `FixedStationTransfer::Feedback.context_id`. Both the periodic feedback and the direct COMPLETE feedback currently use `input().context_id` (`hold_executor.cpp` lines 1220 and 416 in the frozen parent). That context is suffixed for PICK or PLACE; during PLACE_REVOKE the public phase already says PLACE while the internal context still belongs to PICK. The proposed public contract is to echo the parent Goal context in these two Transfer feedback paths, retaining child-operation contexts in internal planning/revalidation and `/transport/hold_executor/status`. This proposal was sent to the coordinator; no implementation or verification was performed after the pause.

The navigation helper is created only when a parent Goal is accepted. A zero-before/one-after navigation-client probe is valid for a fresh executor's first Goal, scoped to that executor node. The helper remains allocated after task completion, so a reused executor may already have one client before its next Goal. The probe must bind its assertion to the instance and first-Goal precondition instead of imposing a global unconditional zero count. M2 confirmed that neither proposed probe adjustment has been implemented and no new actual scene has started. These findings are recovery items, not a resumed task or a new task clock.
