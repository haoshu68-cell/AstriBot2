# Task-chain progress

Plan: `docs/superpowers/plans/2026-09-21-remaining-work.md`.

## Time policy

User changed the limit to **90 minutes per unfinished task**. Count from the original task start, including retries. Pause at the limit, preserve evidence, and do not disguise a retry as a new task. Exact recorded deadlines: `task_timing.json`.

## Verified checkpoints

- W6 prerequisite: installed Nav2 VoxelLayer reproduces 32-configured/16-actual truncation. Both MPPI/RPP local/global profiles now use 16 × 0.15 m. Four profiles × 14 height cases pass marking/clearing/rejection. Capacity is 2.4 m; sensor height filters remain 2.0/1.8 m. This is not a Gazebo corridor coverage acceptance.
- W0: corrected default perception frame; C++ perception Action/input/worker tests 16 + 9 + 4 pass. Default versions remain zero (uninitialized).
- W0: 44 collision links and 58 joints match live Gazebo and MoveIt when attachment flags match. Six camera links / 11 camera collision primitives are contained in C++ full and layered envelopes across 24 camera-pose checks.
- W0: native navigation dependencies and message headers rebuilt in the isolated overlay; stale CMake dependency caches were corrected. Native policy CTest 14/14 passes. Perception CTest 8/8 passes (wrapper counts, not additional independent GTests).
- W0: missing MoveIt PointCloudOctomapUpdater identified and built from upstream 2.5.9 to match installed MoveIt. No system package overwrite. Missing plugin now blocks launch.
- W0: MoveIt shutdown faults reproduced. TEM cleanup alone moves the fault to the main node; both controller and capability-loader lifetime fixes give 3/3 clean planning-only lifecycles and one additional cold-session probe. Broader execution shutdown remains untested. Patches are simulation candidates, not hardware release approval.
- W0: verified owned camera-session stop with no remaining members of the launch process group; cold restart uses the final installed projection node and alias conditions.
- W1 prerequisite: actual C++ projection regression was 1/10 pass before repair, now 18/18 including invalid layout, wrong/stale frames, both arrival orders, units/endianness, duplicate prevention, clock rollback, frozen clock, and bounded pending cache.
- W1: four projection candidates over 600 s produce 17,747 unique clouds with no unmatched depth/info pairs. Maximum sampled receive age 103 ms. Compilation and brief planning diagnostics overlapped; no MPPI/SLAM/real-model inference concurrency acceptance.
- W1: optional wrist health/private pointcloud launch connected. At 5 Hz, right-wrist health reports STALE; independent 30 s trace confirms ages 254–284 ms against the unchanged 250 ms deadline. This checkpoint is NOT PASSED.
- W3 prerequisite: new reference-mount seven-scene dataset captured separately. Known-CAD C++ registration accepts 5/7, maximum accepted error 2.331 mm / 0.668 degrees. Tilted/close rejected for inadequate coverage/normal diversity. HSV fixture masks, not YOLO. Thresholds unchanged; truth only used by independent scoring.
- W5 diagnostic: actual READY_RIGHT target is `transport_compact`, not SRDF `ready`. Static compact target checks pass. Fresh-state GetMotionPlan succeeds 3/3 with live OctoMap. This is not full task-scene/MTC/postvalidation/execution acceptance.

## Current work / gaps

- Transport launch previously generated a separate historical URDF/SRDF. It now shares resolved description bytes with MoveIt and MTC; three offline attachment configurations pass. Live three-node model/skill service validation is still in progress. Existing MTC dependencies must be sourced (`tools/setup_mtc_humble.sh`).
- Wrist 5 Hz delivery margin is inadequate in some phases; test 10 Hz simulation rendering without changing health TTL.
- W0 final integrated snapshot and runtime model/ABI closure need completion. W1 full load/fault matrix, W2 real detection/masks, W3 multiview, W4 authority, W5 full original READY task context, W6–W7 full navigation matrices remain unaccepted.
- VLA true model, real hardware, and full dynamic pick/carry/place remain deferred.

## Ownership / preservation

Keep the integrated dirty tree; no unrelated changes reverted or committed. Isolated build/install under `runs/task_chain_20260921`. Baseline failures and old datasets retained. Active camera session is `runs/task_chain_20260921/camera_session_v2/session/session.json`, domain 89, canonical warehouse. Its query environment is not a claim of full planning-stack dependency completeness; MTC setup is additionally required.

## 2026-09-22 continuation checkpoint

- C++ MoveGroupInterface discovery is now bounded to 2 s per action server. In planning-only mode ExecuteTrajectory is absent; its previous default infinite wait caused the 60 s PlanSkill timeout. Three independent launches now give 9/9 successful transport_compact plans, all four nodes exit cleanly per run, and the three planner model hashes agree. See transport_planning_result.md and transport_lifecycle_summary.json. No execution was requested.
- Final selected build passes for manipulation, transport and perception components. Existing manipulation CTest 1/1 and offline launch/model checks pass. integration_manifest.json freezes 1112 selected files and 98 ELF dependency resolutions, with no missing libraries. This does not prove arbitrary ABI compatibility. No system package was overwritten.
- Owned camera v2 stopped cleanly; zero members remained in its launch process group. v3 uses 10 Hz wrists with unchanged 640x320 intrinsics scaling, extrinsics and 250 ms health TTL. In the 60 s static capture each RGB-D produced 599 exact groups/clouds. After the separately recorded first 2 s, all four RGB-D health states are OK; left/right wrists max sampled age 200 ms, versus 264/284 ms in the preceding 30 s 5 Hz trace. Different durations are reported explicitly. See wrist_rate_comparison.json. No full-load acceptance is claimed.
- W0 remains unaccepted as a whole: wrong-overlay runtime, clock-reset admission, and full context/ABI rejection matrix are incomplete. Unload warnings remain even in clean exits. Pause at original 90-minute deadline; do not reopen as another nominal task.
- W1 remains unaccepted as a whole: task-owned wrist activation, multi-minute/full-load matrix and fault/recovery epoch tests are incomplete. Camera-only pairing and this short static health check are independent checkpoints only.
- W2-W7 integrated gates are not advanced. Prior W3/W6 isolated prerequisites remain historical checkpoints; W8 and VLA/hardware remain deferred.

At 2026-09-22 00:07 +08:00, W0 is PAUSED_TIMEOUT and W1 PAUSED_DEPENDENCY. Owned v3 camera supervisor exited 0; its launch process group has no remaining members. No simulation from this task is intentionally left running. Source/evidence are preserved. Resume requires an explicit decision on the timed-out task; retries must not silently reset its budget.
