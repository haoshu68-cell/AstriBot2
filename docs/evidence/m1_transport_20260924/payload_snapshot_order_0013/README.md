# scene22: physical snapshot receipt after caller time sample

The coordinator assigned a bounded correction of this confirmed direct mainline blocker. Original issue start remains 2026-09-24 23:30 Asia/Shanghai; unfinished investigation stops by 2026-09-25 00:30. The payload client source/header and its existing test are the only product files changed. The coordinator owns compilation, installation, tests and simulation. Future-drop, TF and GPU investigations remain paused.

## Actual failure evidence

`scene22_diagnostic.txt` is the complete diagnostic line from `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene22_world69/trajectory_executor.log`:

- Source capture, accepted receipt ROS time and caller ROS time all equal 111320000000 ns.
- Caller steady time is 21786366339340 ns; accepted receipt steady time is 21786367169203 ns, **829863 ns (0.829863 ms) later**.
- The failing condition is `poll_and_source_lifetime`; `poll_steady_before_receipt=true`, with zero source age and no ROS-before-receipt violation.
- Capture/deadline are still zero: the first physical observation check rejects this already received fresh sample before establishing its normal capture deadline.

The caller samples its tick clocks before `observation` locks and copies the asynchronous inbox. A callback arriving in between can therefore have a newer receipt than the caller's earlier steady sample. The stale error in this specific diagnostic is not a source timeout. Its older future-drop record does not change this: the accepted source capture itself is current. Other historical generic stale failures are not automatically assigned this cause.

The coordinator's scene22 first-cause record is `runs/mainline_20260924/payload_receipt_order_0013/scene22_first_cause.json`; the coordinator reports bounded shutdown and RELEASE_CONFIRMED. This patch does not modify any journal or resource disposition.

## Minimal correction

The client retains the existing ROS-clock callable. While holding the inbox lock, `observation` copies the state/receipt and then samples the check ROS/steady pair. This fixes the ordering rather than extending a deadline. The original caller pair remains an explicit lower bound: an actual check-clock rollback is rejected with the original `PAYLOAD_STATE_STALE` reason. Original receipt/capture ordering checks remain too. There is no max, clamp or default-success path.

The callback's future rejection, duplicate first-receipt rule, original 300 ms lifetime, receipt-based remaining lifetime and nonrenewable capture deadline are unchanged. `begin` is byte-for-byte unchanged. The physical command's original 2 s entry check remains, and the coordinator approved one final real-steady check immediately before `unresolved_` becomes false: waiting or validation cannot complete the command after the original deadline by relying on an earlier caller sample. `started_` is not reset. Public signatures and caller code are unchanged. Both caller and actual check clocks appear in the diagnostic; existing poll fields now denote the check snapshot's time.

The header adds a private clock callable, so all client users must be rebuilt with this header; an old caller must not load the changed-layout library. The coordinator was told to rebuild `trajectory_executor` and `payload_client_test` together.

## Targeted validation handoff

One deterministic ordering test is added to the existing five cases. It saves the caller pair before publishing on the fixture's private Ignition partition, then queries using that saved pair after an actual receipt: this ordering must return the fresh state without sending any physical command. It then rolls ROS time backward and requires rejection both against the original caller time and against the newer backward caller time. The old code rejects the first state once received; this is not a random scheduler stress test.

The existing repeated-capture test still requires expiry without receipt renewal and exactly one failure diagnostic; it now distinguishes the original caller steady time from the later actual check time. The existing no-readback timeout case now waits two real seconds instead of supplying a fabricated future steady time. It first checks timeout with no readback, then receives fresh complete physical input and invokes the completion path with the earlier saved caller/inventory receipt. It requires the original `PAYLOAD_APPLICATION_TIMEOUT` and unresolved state even with those complete inputs. Existing physical error propagation, command-counter and detach cases remain. The suite still has six cases, with about two additional seconds of fixture elapsed time, not a performance experiment.

Source review confirmed unchanged `begin`, original entry checks and deadline formulas, plus the explicitly approved final application-deadline check; whitespace checks passed. At the final 00:18:29 freeze, compilation, the six client tests and the ordinary mainline scene are pending coordinator verification. No performance or stability pass is claimed. `before_manifest.json`, all before/after files, `changes.patch` and `manifest.json` preserve exact source scope and the original issue deadline.

## Coordinator build and verified client-test result

The coordinator built the final source/header and rebuilt both `trajectory_executor` and `payload_client_test`. Its run at 2026-09-25 00:20:08 records **6/6 passes, zero failures, errors or disabled cases**, with fixture elapsed time 2.483 s. The real waiting in the timeout case accounts for about two seconds; this is test duration, not a runtime performance measurement.

The passing cases include the saved-caller/newer-receipt ordering, rejection of real ROS rollback against both caller/receipt bounds, actual 2 s application expiry even with fresh physical input and an older caller sample, repeated capture not renewing receipt freshness, physical errors, command ownership/counters and detach behavior. The `clock_regression` diagnostic in the raw log is intentionally produced by the rollback negative case, not an unhandled stale failure.

The recorded dependency listing resolves both payload-client and arm-hold libraries from `runs/m1_transport_20260924/continuous_build`. Read-only checks matched all three frozen source/header/test hashes. `verified_client_test/` retains unmodified build/test logs, dependency listing, XML and the coordinator's scene22 first-cause record, with original paths, copy hashes and four executable/library hashes checked after testing. The original handoff manifest remains the historical pending snapshot; the new manifest records this later verified result. No test was repeated here.

At this update the coordinator is preparing the new install and scene23; the complete ordinary Transfer has not been accepted. These isolated private-transport tests do not establish full simulation, long-duration stability, performance or hardware acceptance. The source remains frozen, and the original issue start/cutoff and paused unrelated investigations are unchanged.
