# Mainline scene revalidation diagnosis

Issue first observed 2026-09-25 05:56 +08; checkpoint 06:56. Earlier payload-path investigation retains its original timestamps separately.

Only the executor revalidation mismatch failure branch saves its actual request PlanningScene and independent readback as ROS CDR, with stage/context and static/occupancy signature differences. Success behavior, time limits, collision logic and failure reason remain unchanged. New binary and hold-only macro target built successfully. mtc_plan_test and scene_binding_test passed.

The reader now checks the already fixed kp=3.0 as well as idle_position_hold=true. Camera, scene, planner and profile configuration are frozen in frozen_baseline.json; this extends the scene38 baseline only by diagnostic code and that reader check.

Runtime scene39/domain52 uses the same navigation warehouse and single-box Goal. Real complete transfer acceptance remains pending. M1 owns exact scene-field analysis; root owns build and simulation; M5 only verifies evidence of actual progress.
