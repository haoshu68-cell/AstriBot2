# User-fixed idle hold and simulation base motion limits

User instruction on 2026-09-25 fixes `idle_position_hold=true`; `idle_position_kp=3.0` remains unchanged. Source default, configuration overlay and mandatory pre-Goal parameter readback agree. AGENTS.md records the standing instruction.

The separately authorized simulation profile permits 0.05 rad base rotation and 0.10 rad/s non-navigation angular speed. Translation remains 0.02 m and linear speed 0.02 m/s. Profile defaults false and is rejected without use_sim_time. Collision, ledger, freshness, arrival and final measured-stop checks are unchanged. Results under this profile are not acceptance under the previous strict limits.

Build: isolated MTC, native and chassis candidates succeeded. CTest: execution_guard_boundaries 1/1, wheel_math/wheel_reversal 2/2; native trajectory_digest, payload_scene, mtc_plan and scene_binding 4/4 passed with the new library path. M5's 15 profile/readback offline cases and CLI/report checks are separately archived.

Runtime: scene38/domain53, current navigation warehouse, graphical Gazebo and read-only RViz. Actual runtime readback confirms idle_position_hold=true, idle_position_kp=3.0, use_sim_time=true; the loaded chassis executable is the new isolated candidate (see scene38_parameters.json and scene38_chassis_identity.json). Scene38 confirmed GRASP_CONFIRM, then stopped before ATTACH_CONFIRM with MTC_SCENE_CHANGED_DURING_REVALIDATION; parent canceled with resources_released=true, independent measured stop passed, stack stopped with no owned PIDs remaining. It did not complete PICK/NAV/PLACE. Mainline full PICK-NAV-PLACE remains unaccepted.

Source snapshots are archival because the shared worktree contains unrelated ongoing changes. Older active overlay dependencies remain until replacements have passed and safe retirement is verified.

Read-only follow-up: three grasp stages were confirmed; no physical attachment command was accepted/applied, so actual-payload replanning was not reached. Ordinary occupancy revalidation received a successful service response but the subsequent independent PlanningScene readback differed. The compared snapshots were not archived; a specific changed field or root cause cannot be established. No code or threshold was changed in this follow-up.
