# Camera reference installation review — 2026-09-21

Scope: provisional mounting YAML, source xacro expansion, warehouse raw-frame aliases and revision, MoveIt profile selection, support geometry, and the isolated simulation validation script. Source was read-only; no ROS/Gazebo process was started, stopped, queried or reconfigured by this reviewer. Earlier GraspNet implementation was excluded. Final collision-model live domain-88 validation belongs to the coordinator and was still in progress at this snapshot.

## Final conclusion

The identified P2 is **closed** by the final source and offline evidence reviewed below. No remaining reproducible P1/P2 defect was found within this scope. This does not establish measured physical calibration, all-pose planning validity, motion execution or hardware acceptance.

## Closed finding and reproduction history

**P2 — newly enabled mounting contacts were missing from the planning collision policy.** The following describes the pre-fix source; the final disposition appears after the geometric analysis.

`ws_robot/src/astribot_s1_description/urdf/astribot_s1_sensors.xacro:154-157` adds each support bracket as real collision geometry. For both wrists the profile at `config/camera_mounts_reference_sim.yaml:63-64` and `:73-74` deliberately penetrates the fixed gripper-base surface to represent attachment. Meanwhile `ws_robot/src/astribot_s1_moveit_config/launch/move_group.launch.py:63-66` now enables this profile by default for simulation, and the current SRDF (`config/astribot_s1.srdf:247-250`) excludes only wrist-camera versus arm links 6/7, not wrist-camera versus gripper base.

Independent offline reproduction: expand the current source xacro with the reference profile and all cameras; compute zero-pose FK; read the actual `astribot_gripper_base_link_collision_mesh.obj` selected by the expanded URDF; transform its triangles into each camera-link frame; perform triangle/axis-aligned-box SAT against the expanded camera collision boxes. For **each** wrist, the housing intersects zero gripper collision triangles, while the new bracket intersects **62**. Neither camera/gripper-base pair is in the SRDF collision exclusion set. These are rigid installation contacts, so changing arm or finger posture cannot remove their relative intersection. Under the current planning collision policy they can invalidate every starting state. The reviewer has confirmed collision-geometry intersection and missing policy entries; the coordinator was asked to confirm the resulting contact pairs using offline MoveIt, rather than treating visual-mesh clearance as a planning check.

The head/torso reference camera bodies also overlap the parent links' conservative sphere/box collision envelopes, and no matching camera-parent entries exist. Independent sphere/box calculations confirm:

| Reference installation pair | Housing intersection | Bracket intersection |
|---|---|---|
| `head_rgbd_camera_link` / `astribot_head_link_2` | sphere radius 0.11 m, nearest box distance 0.093270 m | nearest distance 0.091993 m |
| `head_stereo_left_camera_link` / `astribot_head_link_2` | nearest distance 0.085018 m | nearest distance 0.078245 m |
| `head_stereo_right_camera_link` / `astribot_head_link_2` | nearest distance 0.085018 m | nearest distance 0.078112 m |
| `torso_rgbd_camera_link` / `astribot_torso_link_4` | box overlap widths 0.02011 / 0.08 / 0.03 m | overlap widths 0.005336 / 0.02 / 0.012 m |

Together with the two wrist camera/gripper-base contacts, these six pairs form the candidate minimum fixed-installation ACM additions, subject to the coordinator's actual MoveIt contact output. **Do not add all six unconditionally to the shared SRDF:** the original calibrated torso camera is attached to `astribot_torso_base`; only the reference profile attaches it to `astribot_torso_link_4`. Disabling that pair globally would hide a moving torso collision in original/hardware mode. Apply installation exclusions only when the expanded model confirms a common rigid component (or use a properly selected reference-specific policy). Preserve camera collision with moving fingers, other robot links, payload and environment. This finding and the original-profile constraint were sent to the coordinator immediately. No P1 or second concrete defect was identified in this bounded review.

### Follow-up: seventh contact and volume-preserving deduplication

The coordinator's real offline MoveIt result `moveit_collision_red.json`, inspected by this reviewer, confirms **seven** camera contact pairs in 15 states: the six installation pairs above plus `torso_rgbd_camera_link` / `astribot_torso_link_3`. Other reported contact sets are empty. The listed states vary arm/gripper postures and zero/home torso configurations; they do not sweep torso joint 4. The seventh pair must not be added to an allowed-collision list because the camera and torso link 3 are separated by an active joint.

The reviewer compared `astribot_s1_torso_wheel.xacro:318-362`, the vendor whole-body URDF and the SDK per-part `astribot_torso.urdf`. The joint-4 origin/axis and link-3 cylinder placement agree; this is not a false zero convention. Joint 4 rotates the upper body around its local Z. The original link-3 cylinder has radius 0.09 m and length 0.35 m, spanning approximately Z=0.005–0.355 m when expressed in link 4. The actual link-3 visual STL spans Z=-0.079559–0.074985 m in that same frame. The camera housing starts at Z=0.105 m and its bracket at 0.114 m, giving at least 30.0 mm visual-mesh vertical clearance that is invariant under joint-4 yaw. This explains why visual audits were clear while the conservative planning cylinder collided; it does not authorize replacing or shrinking that cylinder.

The coordinator proposed removing only duplicate camera collision volume already contained in the **same fixed parent** collision box. Independent numerical verification supports that design:

| Volume in torso-link-4 coordinates | X (m) | Y (m) | Z (m) |
|---|---|---|---|
| Existing parent box | [-0.1075, 0.0925] | [-0.135, 0.135] | [0.0772, 0.4572] |
| Original full housing | [0.07239, 0.10239] | [-0.04, 0.04] | [0.105, 0.135] |
| Retained exposed housing | [0.0925, 0.10239] | [-0.04, 0.04] | [0.105, 0.135] |
| Full bracket | [0.068054, 0.073390] | [-0.01, 0.01] | [0.114, 0.126] |

The dropped housing slab and entire bracket are contained in the unchanged parent box. Thus the union of the fixed assembly's collision volumes is exactly preserved; this relation holds at every robot posture because the transform to the parent is fixed. The exposed collision box should have size `[0.00989, 0.08, 0.03]` and camera-local center `[-0.006945, 0, 0]`, while the complete visual housing/bracket remain. No torso-link-3 collision exemption is needed. Accounting for the vendor's rounded rotations, a conservative bound over joint 4 in [-1.2, 1.2] is cylinder X < 0.086517 m, leaving at least 5.98 mm to the retained housing's rear plane.

### Final fix verification

- `_prepare_robot_descriptions` now expands the URDF once before the MoveIt/RViz node actions and derives the semantic description from that exact model. `_fixed_camera_mount_semantic` adds only six named installation contacts and only if each pair is connected entirely through actual fixed joints. It adds no torso-link-3 exemption. An empty mount profile leaves the shared semantic file unchanged, including in hardware-default mode; the shared SRDF SHA-256 is unchanged from the red snapshot.
- The reference profile contains the exact exposed torso housing box verified above. The full visual housing and bracket remain; only redundant collision volume is removed. `test_torso_collision_dedup_preserves_complete_robot_volume` reads the actual expanded URDF, proves rigid-parent identity, checks all box intervals, verifies the bracket's containment, and proves the housing split shares the parent's exact boundary and retains the original exterior. Original camera profiles are unaffected.
- The reviewer independently reran the source camera tests: **26/26 passed**. The saved final CTest report has three passing suites; their XML contains **29 cases total (2 + 1 + 26)**. The older `description_full_green.txt` with 28 cases is historical, not the final count.
- The reviewer independently reran `verify_camera_mount_planning.py`: all four checks passed. They establish exactly six additions with no previous pairs removed, byte-identical original/hardware semantic descriptions, and no exemption across a deliberately changed revolute attachment.
- `moveit_collision_green.json` contains **24 within-bounds states**, including torso yaw -1.2/0/+1.2 and gripper opening 0/0.465/0.93. Every camera-contact and other-contact list is empty. All six external probes placed at actual camera collision-shape centers are detected. Both removed-volume probes are detected by `astribot_torso_link_4`. The reviewer inspected the actual C++ check and parsed these records; the coordinator ran this FCL executable.
- The reviewer independently resolved the current launch and verified that its URDF and semantic strings exactly match the files used by the offline FCL probe. The installed top-level xacro, sensor macro and mount YAML match the current source byte for byte. All seven original calibration hashes remain unchanged.

The source and recorded offline evidence satisfy the requested fix. The frame/visual changes remain provisional simulation references, and final live sensor validation after the coordinator's collision-model restart is a separate evidence step.

## Verified behavior

- The mount YAML explicitly declares `provisional_reference`, `simulation_only_pending_complete_calibration`, and revision `2026092101`. Its six complete records override parent, translation, rotation, housing and bracket geometry while retaining intrinsics in the original camera profiles. The reviewer independently verified all seven hashes in `original_profile_hashes.json`, including the original robot calibration JSON.
- The top-level xacro's omitted or empty `camera_mounts_profile` retains the imported mounts. Tests cover all six original/reference parents and origins, custom overrides, unique frame/sensor names, standard optical rotation, stereo baseline, opaque housing behind the entrance plane and the final collision-volume proof. Final test counts are given above.
- The wrist camera-link RPY `[pi, 0, -pi/2]` composed with the standard optical RPY `[-pi/2, 0, -pi/2]` gives arm-link-7 to optical `Rx(pi/2)`, translation `[0.006, -0.088, -0.060]`. Relative to the actual gripper-base fixed transform, this is the requested `[0, -0.060, 0.040]`, identity rotation to within 0.265 micrometres / 3.673e-6 radians. The residual is the existing vendor `1.5708` angle rounding. Both sides intentionally use the same local face; neither the photos nor this review establish measured physical calibration.
- `_prepare_camera_mounts` is executed before consumers. Independent launch-parameter evaluation gives revision `2026092101` and torso native parent `astribot_torso_link_4` in reference mode; empty mode restores revision 1 and `astribot_torso_base`. The stereo-right original optical-parent chain resolves to the head body for the Gazebo raw-frame name. Static aliases remain identity edges from each optical frame to its native Gazebo sensor frame.
- Independent evaluation confirms MoveIt `use_sim_time=true` selects the reference profile, `false` selects an empty profile, and explicit overrides are respected. This verifies the documented default separation; an explicitly supplied provisional profile can still be selected by the caller. The standard eight `verify_vision_launch.py` checks also passed without executing Node actions.
- Housing fronts stay at least 2 mm behind the reference entrance plane. The supports remain small opaque visual and collision boxes; the wrists attach to the fixed palm side rather than moving fingers. The existing offline `final_source_overlay` audit uses the expanded source's actual brackets and records zero housing/physical-visual-surface intersections. Its three tested gripper poses have clear center rays, zero head/torso/stereo self-hit rays, and 14–34 peripheral wrist self-hit rays out of 1025. These are sampled, zero-arm-pose visual ray checks, not all-pose visibility or planning collision acceptance.
- `camera_reference_session.py` is a launch/validation wrapper: it uses isolated domain 88 and partition ownership checks, retains canonical warehouse launch, writes scaled auxiliary camera profiles separately, and sends no hardware command, navigation goal or arm trajectory. The reviewer did not run it or inspect live process state. Its six-camera capture rate and useful fields of view require the coordinator's live result.

## Source snapshot

| File | SHA-256 |
|---|---|
| `camera_mounts_reference_sim.yaml` | `56fb2e4ea13f631e04353e0441bd4567f99992f241db91d4812b0a8053220966` |
| `astribot_s1.xacro` | `f243ed6f7994de5e2b49bdc5eae0013bddcd4d91cf9702289bafbc1fda852b91` |
| `astribot_s1_sensors.xacro` | `0d50f11d29d61933c04af9f854aef8598bd907533d9d9d0218f0ac7ddc7cbce1` |
| `warehouse_sim.launch.py` | `3657fd4657a80e5a718b480752f3517b40c05912838ab21425f73721b32df2a7` |
| `move_group.launch.py` | `3c0b873bcd040d688ff1efae968ced901da3b34f595fb41106a2ddd32762ea30` |
| `astribot_s1.srdf` | `1a272c8af5508d0c78c40685bc1afcd2c38bbe3678bd4d3e69bac829647c668c` |
| `camera_reference_session.py` | `674a03eceb95aa0ec34cede615f10e6a2d2126db6c28234e38148f85c5a4f4e6` |
| `verify_camera_mount_planning.py` | `1ea0407a739cd9e15d339345d54b2e5eaae6cffc54bdfd5abb717c0628daef11` |
| `camera_mount_collision_check/check.cpp` | `508f8d657e3145997ebdc9e6a380409c1fea1fa107b8d49c7e86dcfce6acbc2a` |
| Tested/current launch `reference.urdf` | `3b9f81b1be79ab051807bc81c634602ec4ac4e1707c8ec4e1586cf4ab8fe0935` |
| Tested/current launch `reference.srdf` | `7b1afa31711f458f100c3057c0109dd958a0a330a975756309a79b6ad46d048c` |

No approval to execute motion follows from these checks. Physical extrinsic calibration, all-pose collision/occlusion, manipulation execution and hardware validation remain outside this review.
