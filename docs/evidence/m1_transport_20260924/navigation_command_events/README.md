# Fixed-station navigation: command events and measured stop

This is a coordinator-authorized repair of a direct parent-task blocker, retaining the coordinator's existing issue clock and historical pauses. The recording/freeze times in the manifests do not start or reset a task timer. The coordinator reported actual scene02 cold-start evidence: the new output chain had no `/cmd_vel` event while odometry was stationary and the executor was idle. Its body-to-world output emits no inputless heartbeat and may emit only one final zero; the smoother also does not promise periodic zero messages. This window did not run that scene or independently reclassify its result.

The old helper required a command before collecting odometry samples and a fresh command throughout its stop window. A single real zero became unusable after 300ms even while fresh odometry continued proving a stop. That requirement conflicts with the output event contract. The fix distinguishes an absent observation from an actual zero; it never generates or publishes a velocity command.

## Minimal implementation

- Before this helper has submitted Nav, it relies on the caller's existing exclusive-owner/live-Hold contract and the original complete measured-odometry stop window. Command absence remains explicit. An actually observed nonzero still prevents admission until a real zero supersedes it.
- After Nav submission, planning may wait with fresh stationary odometry before the first command. Observed motion without a fresh command is rejected. A last nonzero command still expires at the original 300ms threshold even if the measured robot later slows down.
- An actual zero is retained as a command event, without requiring repeated zero heartbeats. If measured motion continues, a command observation must still be within 300ms; a fresh zero can accompany bounded physical deceleration. Zero-event silence alone does not prove a stop.
- After Nav submission, stop/cleanup additionally requires its actual terminal result and a real zero received after dispatch. A zero may precede delivery of the terminal result. The terminal still clears the samples and requires a new complete odometry window. A missing final zero or any later nonzero prevents completion.
- When a zero arrives after an expired nonzero, the original stale-command failure is retained, while the real zero is recorded for subsequent safe cleanup. The failure is not turned into successful navigation.

`Twist` is unstamped here: command timing is its actual receipt on the steady clock, not an invented source timestamp or proof that a particular controller produced it. Command absence is stored as `optional`, not as a fabricated zero sample. The helper continues to trust the parent for exclusive ownership and does not acquire a second lease or restore a final command gate.

The source/steady 600ms window, minimum 12 samples, 120ms source-gap bound, odometry freshness, velocity/angular/drift/fit limits, five ACKs and their lifetimes, actual map transform, arrival tolerances and task/cleanup timeouts are unchanged. Only this helper and its existing test file changed. The parent remains SHA256 `e259b34f4ef08643b4c4119309417668b0e2a8e2751246afecb7f64a466a3fe4`.

## Targeted verification handed to the coordinator

All cases use the same existing isolated ROS fixture, with synthetic fixed-envelope and Nav2 endpoints. The fixture can suppress command or odometry publication independently; it does not synthesize a zero while publication is disabled. Existing isolated-domain ownership checks and bounded callback draining remain.

| Requirement / risk | Existing or added case | Independent checks | Coordinator isolated ROS result |
|---|---|---|---|
| Cold start, real final zero without a heartbeat | Existing SameLeaseLegUsesCurrentMapTransformAndPostTerminalStop | Zero command messages before actual Goal submission; one later nonzero and one zero; current map transform; terminal followed by at least the original stop interval | PASS |
| Missing required ACK cannot authorize Nav | Existing LegacyProtectionCannotReplaceControllerAndCancellationStillRequiresStop | No Goal despite legacy protection; no command heartbeat; cancellation still requires the measured stop window | PASS |
| Owned cancellation and terminal do not prove stop | Existing CancelTargetsOnlyOwnedGoalAndTerminalDoesNotProveStopped | Cancel targets the actual owned handle; moving odometry prevents cleanup; one real zero plus measured stop later permits cleanup | PASS |
| Nonzero silence is not a final zero | Added NonzeroSilenceCannotBeMistakenForFinalZero | Last nonzero stops publishing while fresh odometry becomes stationary; stale-command failure and no cleanup; only actual late zero permits cleanup, with first cause preserved | PASS |
| Successful terminal cannot fabricate a zero | Added SuccessfulTerminalWithoutObservedZeroDoesNotProveStop | Terminal and continued fresh stationary odometry do not complete without an actual zero; observed zero subsequently permits the full stop check | PASS |
| A retained zero cannot replace live odometry | Added FinalZeroDoesNotReplaceFreshOdometry | Actual zero and terminal followed by odometry loss fail; no cleanup during loss; recovery only completes failure cleanup | PASS |

This is the original three cases plus three targeted cases, not a new framework or expanded full-stack matrix. Performance is NOT_MEASURED; actual parent/Gazebo and hardware acceptance are NOT_RUN by this window. Full-stack results must come from the coordinator's assigned actual scene.

## Frozen source and evidence

At the 18:27:45 handoff, helper SHA256 is `4cdc18cb3c7eb6b636fd5eb0b2e1099bed32a2dbe20983235c987b21b783ee34` and test SHA256 is `da67a93f54e1cdd96a7e9185b3ba8d32979fdd36a94682bb1f2bbcc3e469a986`. `before_manifest.json`, before/after source, `changes.patch` and `manifest.json` preserve the exact scope. Source-diff review confirmed the retained thresholds; no compilation, installation or ROS test was performed here. The coordinator owns the new build prefix and all six isolated test executions. Product source is frozen after this handoff; any subsequent correction requires a separately reported hash.

## Actual coordinator result, appended after handoff

The coordinator's new build and install completed at `/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_event_command_1814/native_install`, preserving the earlier `continuous_install`. The isolated helper run beginning at 2026-09-24 18:29:05 records 6/6 passing tests, zero failures/errors/disabled cases, process return code 0 and `still_alive: []`. It passed on the first run of this candidate; the original handoff manifest remains an unchanged NOT_RUN snapshot.

Read-only verification matched all seven candidate binary/library hashes and the helper, test and unchanged parent source hashes. The installed navigation helper is SHA256 `34a08ca131511f2f0e54d53014b59557b118ac21e6d6bf2c8f333f6652dc913d`. Both recorded test and executor dependency lists resolve this helper from the new install prefix. The executor ELF remains `ec42c1481080448960a45e4e4d5802c1a0d428ab833a06aaa43e44777c8f286b`; the implementation change is in its dynamically linked helper, so the executable hash alone does not identify this correction.

Original coordinator evidence is `/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/transfer_event_command_1814/helper_protocol/{stdout.log,gtest.xml,result.json}`, with `candidate_binding.json` in its parent directory. Unmodified copies and a hash manifest are retained in `verified_protocol/` here. This window did not rerun or expand the tests, build, install or launch ROS.

The coordinator has authorized M2 to proceed with actual scene03. That dispatch is not a completed parent-Action result, and the full simulation outcome remains pending. These six isolated passes support only the command-event, ACK, cancellation, terminal and measured-stop contracts exercised above; they do not establish full-parent, long-duration, performance or hardware acceptance. Product source remains frozen pending an actual reported mainline failure.
