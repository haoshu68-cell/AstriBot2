# Actual payload transition revalidation — 2026-09-24

Problem started 10:35 +08, independent from the already delivered first-segment review. This item changes only `astribot_s1_transport_mtc`, `astribot_transport_msgs`, and its evidence. It does not complete grasp/place execution.

`/transport/revalidate_payload_transition` accepts `context_id`, `transaction_id`, `start_index`, and full `PlanningScene`; it returns `success`, `reason`, and both identifiers. Only index 4 after the cached PICK `ATTACH_CONFIRM` or PLACE `DETACH_CONFIRM` is valid. The original 120-second cache deadline remains unchanged. The old `/transport/revalidate_manipulation` interface and behavior remain intact.

The new operation compares the full input scene after excluding joint samples, observation stamps, the one cached target object, and occupancy bytes. Other world bodies, attached bodies, ACM, transforms, and octomap metadata must match. Actual attached geometry must use the same planned attachment link and link-local frame; detached geometry must use the planning frame. Empty/malformed body geometry and failed MoveIt object processing are rejected. The owner remains responsible for authoritative inventory, scene readback, resource ownership, and execution authorization.

Each remaining stage retains its own temporary ACM. Its scene and every trajectory waypoint receive the actual target geometry, or remove the target attachment after detach. Validation uses the existing external trajectory validator on a separate copy; the original emitted joint path and timing are preserved. Only a fully successful check replaces the cache. A confirmed transaction permits retries with the same actual scene and new occupancy; another transaction or changed confirmed geometry is rejected. Later occupancy checks read the updated cache.

## Evidence

Artifacts: `/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/payload_transition`.

- `ctest_final.log`: all 4 test programs passed, including 11 new cases plus existing execution guard, joint margin, and canonical octomap programs. `build_final.log` has no compiler warnings/errors.
- `red.log`, `red_result.json`, `old_body_probe.patch`, `reproduce_old_body.py`: isolated differential copy reproduces the old octomap-only handling. The inflated-body test fails with `accepted invalid transition` (expected exit 1). This is a controlled offline differential probe, not a ROS run of the historical executable; production sources were not reverted.
- New cases cover attach, detach, inflated collision, each waypoint's body/frame, original path/timing and stage ACM preservation, immutable cache on failure, other body/world/ACM/TF/metadata rejection, context/index/expiry, invalid shapes/state/link, transaction retry binding, and a subsequent occupied voxel that collides only with the confirmed larger body.
- `changes.patch`, `result.json`, and `overlay_check.json` bind the source diff, hashes, interface fields, and actual loaded native libraries.
- First `build.log` preserves the initial const-waypoint compile error. Although `CMAKE_BUILD_PARALLEL_LEVEL=1` was requested, colcon passed `-j28`; this first attempt exited, and all subsequent builds used direct `cmake --build ... --parallel 1`. No live system operation occurred.

## Isolated overlay

Source `runs/mainline_20260924/payload_transition/overlay.bash`. It loads `/opt/ros/humble`, the existing `narrow_arms_20260923_1902/overlay_v2.bash` dependency chain, MTC dependencies, then the new messages package and new MTC package. Actual `_geometry_native` resolves to `narrow_arms_20260923_1902/install_v2`; messages and `_transport_scene_native` resolve to this candidate prefix. Existing `mtc_export/install`, shared installs, and the M2 frozen environment were not written.

Rebuild command after sourcing that overlay:

```bash
cmake --build runs/mainline_20260924/payload_transition/build/astribot_s1_transport_mtc --parallel 1
cmake --install runs/mainline_20260924/payload_transition/build/astribot_s1_transport_mtc
ctest --test-dir runs/mainline_20260924/payload_transition/build/astribot_s1_transport_mtc --output-on-failure
```

Evidence is limited to offline build and synthetic C++ scene/trajectory checks. M1 still must invoke this service at the physical transition and independently bind its ledger/version and scene readback before acknowledging; ROS service integration and real simulation grasp/place remain unverified.
