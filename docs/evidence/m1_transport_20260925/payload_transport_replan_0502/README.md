# Explicit actual-payload transport replanning candidate

Issue `MTC-PAYLOAD-REMAINING-PATH-01` retains its original 2026-09-25 **00:26:36** start and previous bounded-run record. The coordinator reports the user's explicit continuation; this resume interval is **05:02:36–06:02:36 Asia/Shanghai**. The only mainline objective is one ordinary single-box parent PICK → navigation → PLACE with recording. No new scene matrix or fixture parameter search is added.

This window owns only the listed MTC header/core/planner/test/README. `before_manifest.json` and `before/` preserve their actual pre-change bytes, including prior shared planner changes. The coordinator owns the service schema, native executor/plan consumer, builds, installation, ROS/Gazebo/GPU, resource cleanup and Git. This window has not built or run tests or ROS.

## Candidate behavior

Actual attachment geometry, scene invariants, context, transaction identity, octomap and geometry-format checks remain in place. Every original remaining stage is first checked with the actual payload and its original ACM. Only a first PICK transaction whose actual-body LIFT passes and whose following, unexecuted TRANSPORT_POSTURE fails specifically with `EXTERNAL_COLLISION` may invoke one replanner. PLACE, failed LIFT, non-collision errors and already-bound transactions still reject. No old scene/body is substituted to make the collision disappear.

The production callback uses the existing PipelinePlanner with `RRTConnectConfig`, the original joint-margin constraints, current actual-body scene/octomap/ACM, and a planning time bounded by the original remaining revalidation budget and existing 3 s pipeline timeout. Start is the already-validated original LIFT endpoint; target is the original compact endpoint. Local MTC source confirms `goal_joint_tolerance=1e-4`; the candidate binds returned endpoints back to the exact original boundary states before time parameterization and complete interpolation/collision/singularity/margin checking. This makes any connection to those exact boundaries part of the checked path. Non-group joints cannot move.

Only this new trajectory is time-parameterized using the original velocity, acceleration and time-scaling settings. Original LIFT and all paths that need no replanning retain serialized times, positions and derivatives. The production values remain .1 velocity/.1 acceleration and the configured 2.5 time scaling; the patch does not change parameter defaults or scene goals.

The coordinator's new service response carries the full two-stage suffix and `transport_replanned`. Stage IDs come directly from the original cached stages. Expected-start joint names/positions come from the original stage scene with the actual body; a separate scene clone supplies the replanner's LIFT-end start, avoiding modification of the expected-start joint serialization. All response construction and candidate validation finish before atomic cache/binding commit. Failure clears the response suffix and submits no candidate cache. The original cached creation time is never refreshed.

Same-transaction replay preserves the transaction's original replanned flag and validates/returns the cached suffix without another planning call or retiming. New occupancy is still checked and may reject. The existing single-call 10 s limit is also bounded by the original 120 s context expiry and checked again after response serialization. Native consumer adoption and final independent Scene readback are separate coordinator-owned prerequisites; service success does not itself authorize motion.

## Defined validation, pending coordinator execution

The existing 16 payload-transition cases remain; eight additional cases bring the fixture to 24. Its minimal model adds a second planning axis to create a real collision-free detour and a non-group joint to test ownership. Collision geometry uses actual MoveIt/FCL, while the planner callback supplies deterministic offline paths rather than invoking ROS or OMPL. The new cases cover:

- A larger actual attachment rejects the nominal path; only transport replans from the verified lift end, preserving actual body, ACM, exact endpoints and original lift/cache bytes.
- A clear original suffix never invokes the planner and retains original timing.
- Same-transaction replay preserves the true replanned flag and full trajectory messages; newly occupied geometry rejects without replanning.
- Failed actual LIFT, non-collision joint-limit errors and PLACE collisions never invoke transport replanning.
- Planner errors, no returned path and a still-colliding replacement leave the original cache/binding untouched.
- New-path 2.5 timing and derivative scaling, joint-margin boundary ±1e-6 and exact boundary acceptance/rejection.
- Non-group joint movement and planning completion beyond the original context deadline are rejected. The deadline case waits about one real second; this is fixture duration, not runtime performance data.

`manifest.json`, `after/` and `changes.patch` bind the 05:15:53 source freeze. CMake is unchanged; coordinator build targets are `mtc_planner` and `payload_transition_test`. Whitespace and caller-reference review passed, but at this snapshot compilation, 24-case execution and ordinary-scene acceptance are **pending**. These deterministic core cases, even if passing, do not prove real OMPL planning, full parent Transfer, controller-spline collision coverage, long-duration stability, performance or hardware acceptance.

## Coordinator build and verified isolated results

The coordinator subsequently built and installed the frozen MTC candidate. Its XML at **05:16:51** records **24/24 payload-transition cases and 6/6 trajectory-timing cases**, with no failures, errors or disabled cases. The two CTest groups took 1.20 s overall; the roughly one-second expiry fixture accounts for most of this time. This duration is not a runtime performance comparison. The coordinator's four native-consumer fixtures at **05:18:44** record 6 digest, 15 plan, 6 payload-scene and 5 scene-binding cases, all passing. They belong to the coordinator-owned interface/consumer change and are not tests run by this window.

Read-only checks matched all five frozen MTC source/document hashes, all six original XML hashes and seven candidate executable/library hashes. `verified_coordinator/` retains unchanged MTC configure/build/install/test logs, native test logs, the earlier `native_test_old_library_failure.log`, the coordinator's summary/binding, all six XML files and a provenance manifest. The earlier failed run remains visible; final passing evidence is separately timestamped. The original freeze manifest still represents its historical pending-validation state.

The installed MTC planner SHA-256 is `01abe4991268775f89da1d2cc51eed5237f84475bf78973fc1b3913bb9bb07a9`; its scene-signature library remains `b09745947a9af5daf3eb2d8dfa02910feddce8f1dd1ed39807f5de4184e262c2`. File hashes establish the candidate identity, not proof that a running process loaded it; the coordinator's ordinary-scene runner owns the pre-Goal runtime identity check.

Coordinator scene32/domain59 has begun with the unique frozen candidate and video recording. At this update its full ordinary Transfer is **running and unaccepted**. Product files remain frozen, original/resume clocks are unchanged, and no additional build, tests, ROS/GPU operation or new scenario was performed here.
