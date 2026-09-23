# Final protection publication lease implementation

This is the bounded follow-up to the two R1 CONTINUE proposals published beyond their scan deadline. Runtime edits are limited to `ws_robot/src/astribot_s1_navigation_policy_native/src/final_protection_node.cpp`; shared pure header/binding edits are owned and built by a3_envelope_review. Source review and offline tests below are not ROS wire acceptance.

Changes:
- Scan adoption and odometry acceptance preserve the original integer nanosecond capture timestamp. Invalid message timestamps are rejected, and clock reset clears exact captures.
- All existing input/coverage/collision/envelope/zone/camera/clock and clear-confirmation checks remain. After expensive work, an actual publication anchor is sampled.
- The upstream proposal's original stamp plus conservatively converted lease creates an absolute deadline; malformed, nonpositive, excessive (>0.5s), unrepresentable and overflowing leases cannot authorize.
- The same independent scan/odom captures used by the collision decision contribute their original sensor-timeout deadlines. No replacement frame refreshes an old result.
- Existing ControlTime freshness is checked again at the anchor, including proposal freshness. Its time-domain semantics are preserved: simulated mode uses ROS capture time plus the existing separate stall watchdog; wall receive-age TTL applies in non-simulated mode. No new simulated wall-TTL threshold is introduced.
- The shared C++ publication gate checks epoch/rollback/interval/deadline and limits non-HOLD lease to remaining time. Failure immediately sets stop before the limiter, clears clear-confirmation state and clears motion flags/planning. HOLD retains the original configured constraint lease and cannot authorize motion; only positive authorization is shortened.
- Diagnostic mode ASTRIBOT_SCAN_TIMING_DIAGNOSTICS=1 emits each tick; default remains every fifth tick. Records join proposal epoch/sequence/stamp/deadline to final epoch/sequence/anchor/effective lease/hold/reason, source capture/deadlines and actual output/constraint publish-start times.

Evidence:
- `test_final_publication_wiring.py`: 3 source-wiring guards observed failing on old node, then passing. This is static regression coverage, not behavioral execution.
- `final_publication_lease_test.cpp`: six pure C++ forwarding/expiry/epoch/malformed/independent-source/caller-invalid cases; together with 14 shared-core tests, 20/20 passed under UBSan in `runs/joint_acceptance_20260923/publication_lease_20260923/chain_ubsan_final_cpp.log` (build owner execution; final log independently inspected).
- Independent publication_gate_review inspected current source; its source/odom-deadline and proposal final-freshness findings were applied. No new blocking source finding reported afterward.
- Final node build passed (`final_protection_hold_restored_build.log`, final source SHA256 `ce2bd87e794fab0feb11b3c1980e7b28b83de3c5b9eb780981293f9dae4f9bf3`); final source receipt independently matches. Fresh two-stage ROS wire acceptance remains owned by root/navigation. R1 pre-fix evidence remains historical and cannot validate this candidate.
