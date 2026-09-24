# Planning-only GripperCommander — 2026-09-24

The integration problem retains the parent timer starting at 10:35 +08. The old planning callers used `GripperCommander::configure`, which created a real FollowJointTrajectory action client and `/joint_states` subscription. The native executor's existing single-controller-owner graph check therefore encountered a competing controller client. No exclusive-control check or remap is weakened by this fix.

The new nonvirtual `configureForPlanning(model, config, detail)` delegates to the same geometry configuration implementation with controller endpoint creation disabled. The original three-argument `configure`, `GripperConfig`, and all Commander data members retain their prior shape; the ELF comparison reports no removed Commander strong symbols. An already configured planning instance has no action client. `moveTo` and `sendTrajectory` explicitly return `kInvalidInput` with `GRIPPER_PLANNING_ONLY`; `open`, `close`, and a valid `closeToWidth` also reject through `moveTo`. Reconfiguration removes old endpoints and clears measured-state availability. Only the PlanSkill and MTC planning call sites select the new entrypoint.

## Evidence

Artifacts are in `/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/gripper_planning_only`.

- `red.log` and `red_test_source.cpp`: the original configure method with the real recorded robot URDF/SRDF creates both forbidden planning endpoints. Both absence assertions fail. No controller server was created and no goal was sent.
- `ctest.log`: both test programs pass. The new program has 4 passing cases: no planning controller client or JointState subscription; equality with the original geometry at 61 angles and 6 widths; immediate rejection from four public execution entrypoints; removal of prior execution endpoints when switching to planning.
- Tests used localhost-only ROS domain 181. `/proc` precheck was empty, the test checked that its own node was the only discovered node, and the postcheck is saved. This is isolated ROS graph and geometry evidence, not a robot execution trial.
- `abi_symbols.json`: no old Commander strong symbol removed; only the two nonvirtual methods were added. No config/data-member layout changed.
- `handoff.json`, `mtc_planner_ldd.log`, and `transport_skill_planner_ldd.log`: both installed planning executables load the new gripper and core libraries from this prefix; both reference `configureForPlanning`. The geometry native library remains the required `narrow_arms_20260923_1902/install_v2` candidate.
- `build_red.log`, `build_green.log`, `build_mtc_pinned.log`: builds used `--parallel 1`. The first MTC configure found old dependencies because package-level local_setup did not update CMAKE_PREFIX_PATH; the failed log is retained. Explicit new msgs/manipulation CMake directories fixed that selection. Final MTC build succeeded; existing payload-cache explicit-constructor and serial LTO warnings are retained.

## Integration

Source `/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/gripper_planning_only/overlay.bash`. It first loads the prior payload-transition dependency chain, then the new manipulation and MTC prefixes. It also sets CMAKE_PREFIX_PATH explicitly for subsequent builds. Original payload-transition/M2 installations were not overwritten. The candidate retains the new payload-transition service, but a first PREGRASP-to-Hold trial does not invoke that service.

`changes.patch` includes only this item: Commander header/implementation, the two planning call sites, the new test, and four CMake test lines against the task-start CMake snapshot. Before this item, manipulation CMake/package.xml already had the perception-message dependency and grasp-candidate-gate node/test/export edits; `dual_arm_planner.cpp` already had a bounded MoveGroupInterface wait edit. Those inputs were preserved, are present in the rebuilt package, and are not attributed to this fix. Baseline copies and unchanged checks are retained.

No simulation, controller action, native executor policy, or graph admission code was changed. M2's actual first-segment admission/execution/Hold validation remains the next separate evidence layer.

## Optional fixture follow-up

The test source now uses per-test `SetUp` to report explicit GTest SKIP before ROS initialization when URDF/SRDF/domain fixture variables are absent or empty. A supplied domain other than 181 is still an error. Per-test setup is necessary because this installed GTest version does not automatically propagate a suite-setup SKIP to each test body. The original 4-PASS fixture evidence is preserved. After M2 released the window, only the test target was rebuilt: no-fixture CTest reports four explicit SKIPPED cases (`no_fixture.gtest.xml`), and the supplied model/domain-181 fixture again reports four COMPLETED/PASS cases (`fixture_guard.gtest.xml`). No install command was run; all candidate runtime hashes still match `handoff.json`.

`optional_fixture_guard.patch` records this follow-up only. `standalone_test_registration.patch` is relative to HEAD: it adds this test's standalone BUILD_TESTING block and its required `ament_cmake_gtest` package test dependency, excluding the unrelated grasp-gate/perception additions already present in the shared working tree. The prior `changes.patch` remains the pre-guard, task-start-baseline patch; apply the follow-up separately when reproducing that snapshot.
