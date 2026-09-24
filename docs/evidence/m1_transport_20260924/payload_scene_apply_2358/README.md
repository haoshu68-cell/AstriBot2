# scene21: redundant world removal during attachment

Issue start: 2026-09-24 approximately 23:58 Asia/Shanghai. The coordinator authorized the smallest change in `src/payload_scene.cpp`, `test/payload_scene_test.cpp` and the test dependency in `CMakeLists.txt`. This window did not build, install, launch ROS/Gazebo or execute the candidate tests. The coordinator owns those steps and Git archival. The earlier payload-timing investigation remains separate; this scene produced no stale diagnostic and does not establish that intermittent issue is fixed.

## Observed first cause

Scene root: `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene21_world70`.

The result journal records GRASP_CONFIRM success, remaining-plan revalidation, physical submission and `physical_applied_scene_submission` for command 1 / source revision 2, followed by `PAYLOAD_SCENE_APPLY_REJECTED`. First rejection feedback is at steady 20930.184948363 / ROS 95.959. `transport_skills.log` line 602 contains MoveIt's warning at wall 1790265461.754301632: it tried to remove `transport_box_01`, which no longer exists in the world scene.

The parent ends with status 6, resources_released false, `RESOURCE_RECOVERY_REQUIRED:PAYLOAD_SCENE_APPLY_REJECTED`, and separate cleanup reason `GEOMETRY_UNCONFIRMED`. Its domain70 journal remains quarantined. Process shutdown is not business-resource release, and this correction does not clear that journal. The result excerpt retains the full original path/hash and indexed first feedback; the warning excerpt is separate from later shutdown messages.

## Framework evidence

Installed packages report MoveIt 2.5.9. The corresponding local source is `runs/task_chain_20260921/dependencies/moveit2-2.5.9/`:

- `moveit_core/planning_scene/src/planning_scene.cpp:1158`: `setPlanningSceneDiffMsg` processes robot-state attachment updates before world-object operations, then combines the world-operation return values.
- The same file at line 1461: attachment ADD with explicit geometry removes an existing world object with the same ID before installing the attached body. Its explicit geometry is used, including the conservative physical dimensions.
- The same file at line 1759: world REMOVE returns false when the object is already absent.
- `moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp`: the diff path returns this result; `moveit_ros/move_group/src/default_capabilities/apply_planning_scene_service_capability.cpp` forwards it as service success.

The previous application diff supplied both a complete attached ADD and a world REMOVE for that same ID. This ordering explains a false service result after a state-changing attachment step. The exact scene21 attached-body state after rejection was not independently read back in this review; a false service result must not be treated as proof that no scene mutation occurred.

## Minimal change and acceptance

Only the redundant world REMOVE in the attach branch is removed. The complete actual attached body, conservative dimensions, source inventory checks, requirement for exactly one preexisting world object, detach branch, independent full readback and unrelated-scene comparison remain unchanged. No application retry, ignored service failure, tolerance relaxation or fallback state is introduced.

One new test uses the installed real `planning_scene::PlanningScene`, a legal three-link URDF/SRDF, a target box, a separate world obstacle and another attached object. It first reconstructs the old duplicate-removal diff and requires false application despite actual attachment. It then applies the new diff and verifies true application, conservative dimensions and independent readback. Finally, it actually detaches the object, checks the measured placement pose and retained other objects, and confirms an actual unrelated-world mutation is rejected by the readback validator. Existing five tests remain. MoveIt core is linked only to this test under BUILD_TESTING; no new runtime entry is added.

All six tests, candidate construction and installed-library binding are pending coordinator verification at this freeze. A successful core test will demonstrate framework/application semantics, not full Transfer, performance, long-term stability or hardware acceptance.

The frozen hashes are in `manifest.json`; the complete before/after copies and precise patch preserve preexisting shared-file changes. Build targets are `payload_scene_test` and `trajectory_executor`. The coordinator must keep the actual test library resolution with the result, then separately validate the ordinary full scene without reusing the quarantined domain70 resource state.

## Coordinator build and actual core-test result

The coordinator built the candidate and ran `payload_scene_test` at 2026-09-25 00:08:33. The original XML and log record **6/6 passing tests, zero failures, errors or disabled cases**. This includes the real installed MoveIt test: the reconstructed old request returns false after attachment; the corrected request returns true; its actual attached conservative dimensions, actual detach/placement and independent readback checks pass; an unrelated world change is rejected.

The test dependency listing resolves `libpayload_scene.so` from `runs/m1_transport_20260924/continuous_build`, `libmoveit_planning_scene.so.2.5.9` from `/opt/ros/humble/lib`, and the scene-signature library from the frozen `rigid_grasp_2302/mtc_install`. Read-only checks matched all three frozen source/test/CMake hashes. `verified_core_test/` contains unmodified build/test logs, the dependency listing and XML, with original paths/hashes and the test/library hashes checked after execution. The original handoff manifest remains the historical pending-validation snapshot.

The warning about removing an absent world object `box` is deliberately triggered by the legacy negative case and remains in the raw log. The fixed-joint fixture also logs missing robot-link collision geometry and an empty JointState; those messages are retained. Its assertions exercise PlanningScene attachment/message semantics and readback, not a moving robot's kinematics, collision-planning quality or physical grasping.

The new installation and coordinator-owned scene22 are separate from these core tests. The complete parent Transfer, long-duration stability, performance and hardware remain unaccepted here. Scene21's domain70 resources remain quarantined; six passing core tests neither resolve nor release them. Product files remain frozen and no test was rerun by this window.
