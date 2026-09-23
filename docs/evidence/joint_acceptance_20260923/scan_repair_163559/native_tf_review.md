# Independent native TF receiver review

Review scope: source and existing isolated ROS evidence only. Reviewer did not edit runtime code, build the candidate, start ROS, or perform a performance run.

## Disposition

No remaining blocking source finding found for the bounded opt-in same-clock candidate. Integrated navigation/performance remains NOT_RUN in this review; a successful TF contract fixture is not a repaired 90-degree return.

Frozen binding SHA256: `495852d20675cf01274a349da749307ca1be151aa0a3cb111907ff71742fae21`. Source hashes independently match candidate result.json at review time.

## Code inspection

- Private rclcpp context inherits process ROS domain/RMW environment; it does not install global signal handlers or own host logging. Node has no command/ACK publishers.
- TF dynamic/static reception uses reliable depth100; dynamic volatile, static transient-local. Cache is10 seconds. One private C++ executor handles TF and clock without Python callbacks.
- Queries call tf2 BufferCore with the exact supplied integer capture timestamp and no wait; only an explicit zero requests latest. Return message is standard geometry_msgs/TransformStamped. TF lookup/connectivity/extrapolation/invalid exceptions map to their Python tf2 classes.
- Actual negative ROS-time delta with NO_CHANGE latches CLOCK_ROLLBACK_RESTART_REQUIRED. Clock activation does not latch. Both captured-time and latest queries refuse, and clear cannot unlock. Late old TF therefore cannot revive query acceptance.
- This is a deliberately stricter fault behavior than the installed Python buffer. Whole strategy restart and fresh-source readmission are required; the implementation does not claim TFMessage has cross-epoch provenance. Reconstructing an object is not by itself full mission recovery.
- Lifetime shared/exclusive locking protects query versus close; receiving callbacks do not acquire the lifetime lock. Query/rollback serialization protects the latch. Reviewed call graph contains no identified join-lock cycle.
- close is idempotent, sets stop before cancellation, joins even if cancellation throws, and cleans before reporting later cleanup failures. 50ms is spin_once wait timeout, not an execution/closure deadline.
- Observer destruction now guards a not-yet-created tf member. C++ RAII handles successful native construction followed by later Python setup failure; actual full PolicyNode failed-initialization fixture is not covered.

## Findings resolved during review

1. clear-only rollback could accept a late old future TF or latest lookup. Replaced with durable restart-required latch; explicit late-old/clear/latest fixture added.
2. Swallowing close exception before joining could leave a joinable std::thread at destruction. Cancellation failure now cannot skip join.
3. Constructor/close concurrency and failed construction had no evidence. Dedicated fixtures added.
4. A Python busy loop did not guarantee sustained GIL ownership. Fixture now uses PyDLL sleep with GIL retained; source confirms no Python receiving callbacks. This still is not an end-to-end latency test.
5. Private contexts produced repeated global logging initialization; private logging ownership disabled.

## Isolated ROS evidence reviewed separately

`runs/joint_acceptance_20260923/native_tf_candidate_20260923/contract_delivery.log`:12 tests,12 pass,0.684s,owned domain94. Covers interpolation, exception categories, exact/latest semantics, cache pruning, static late join, frozen clock no-wait, rollback+late old TF+latch+reconstruction, reception while GIL held, idempotent close/thread join, concurrent query/close, failed construction, endpoint publisher/QoS checks, startup activation and positive-time→zero.

Source hash receipt/result and domain94_after.json inspected; final residual list empty. Endpoint depth0 means DDS discovery did not report depth; configured100 is established by source, not endpoint readback. Prior11-test report is superseded by12, not an additional12 tests.

## Remaining acceptance

Owned R1 deployment must confirm loaded binding hash and backend, maintain identical camera/world/controller/policy thresholds, and compare capture→receive→internal TF-ready→select→finish→constraint ages plus scan STALE, motion permits, outcome and cleanup. No claim that this optimization fixes every queue/compute delay or all return-90 failure causes.
