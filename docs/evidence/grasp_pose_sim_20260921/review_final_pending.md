# Follow-up source review — live validation pending

Date: 2026-09-21. This review covers landed source fixes and the frozen known-CAD core. It is not a live Action acceptance report. The coordinator is still investigating the real camera-health stream and generating the new model/runtime evidence. No ROS/Gazebo process or GPU inference was operated by this reviewer.

## Outstanding finding

**P2 — ROS clock rollback detection can race with the health callback.** In the reviewed `ws_robot/src/astribot_s1_manipulation_perception/src/manipulation_perception_server.cpp:281-294`, `common()` reads `now_ns` before acquiring `health_mutex_`; the health callback at lines 151-163 reads and updates `last_ros_time_` while holding that mutex. A valid interleaving is: inference worker samples t1, health callback locks and stores t2 > t1, then inference worker locks and compares t1 against t2. This falsely reports `CLOCK_ROLLBACK` and increments the generation on a correctly advancing clock. Frequent chunk guards increase exposure. Sample the compared clock value inside the same critical section as comparison/update. This was sent to the coordinator immediately; its eventual fix/build/runtime status must be recorded separately.

No P1 was identified. No additional reproducible low-coverage false acceptance was found in the registered box-union core during this source review.

## Initial findings verified in source

- FPS now uses bit-reversed lane rank (`tools/vision/graspnet_torch_ops.py:20-34`). The reviewer reran the prior 513-point counterexample on CPU and obtained the required `[0,256]`. Full CPU/CUDA tests and v2 exports are separately owned by the backend work; prior model binaries are not retroactively validated.
- Grasp proposals are structurally validated before filtering (`manipulation_perception_server.cpp:420-477`). Nonpositive scores and zero widths increment `filtered_candidate_count`; raw count is preserved; an empty usable set returns `NO_USABLE_GRASPS`. Nonfinite values, improper rotations and other invalid geometry still abort. Collision flags remain false and positive raw scores use `s/(1+s)` with explicit nonprobability semantics.
- SHA-256 processing calls a guard per 64 KiB and rejects files over 1 GiB (`:37-60`). `check_inflight()` checks cancellation, total monotonic budget, current context and validity (`:339-352`); `run()` supplies remaining request time and checks again on each worker poll (`:354-383`). This bounds computation/preparation more closely; ordinary filesystem reads themselves are still synchronous, so it is not a hard-real-time I/O guarantee.
- Pose observations now preserve planning scene and envelope values (`:569-572` and `ObjectPoseObservation.msg`). `visibility_model_used` makes an uncomputed visibility value explicit (`:581-586`); the fixture now writes null for the unused metric.
- `sim_pose_evaluate.py:14-30` removes a previous summary/result, rejects exit codes other than 0/2, and enforces agreement between process status and JSON success. The prior stale-success path is closed in source.

## Registered CAD visibility and ambiguity checks

Reviewed `ws_robot/src/astribot_object_pose_core/src/registration.cpp:99-196,279-359` and its tests/documented scope:

- Geometry is explicitly an object-frame union of axis-aligned boxes, validated against outward sampled CAD surfaces. Arbitrary mesh visibility and unknown-object CAD are not implemented.
- The camera origin is transformed into each candidate object frame; front-facing samples are tested for self-occlusion by segment/slab intersections. External occlusion and image-crop loss are not removed from the expected-visible denominator, retaining a conservative missing-data penalty.
- With visibility geometry, success requires at least 75% expected-visible coverage, 60 expected-visible samples, 65% observed-scene support, RMSE at most 5 mm and support from two nonparallel CAD-normal axes. Total-model coverage is retained independently; its 35% fallback is used only without visibility geometry.
- Distinct alternatives with comparable scene support and independently adequate visible coverage/RMSE cause ambiguity rejection. Largest-primitive and observed-principal-axis half-turn challenges supplement global PPF hypotheses. This is a bounded search, not a proof that every possible alternative was explored.
- Existing tests cover a fully visible two-face view below 35% total coverage, missing distinguishing features, single-direction stepped surfaces, incorrect registered geometry and competing views. No truth pose enters candidate generation or scoring.

## Evidence inspected, not rerun as live acceptance

- Durable ROS workspace XML under `runs/grasp_pose_sim_20260921/ros_ws/build/` records 28 manipulation-perception tests, 40 perception-components tests and 10 manipulation tests, all with zero failures/errors. These precede the new clock-race finding.
- `ws_robot/src/astribot_object_pose_core/tests/evidence/final_23_tests.txt` records 23/23 offline core/CLI tests passing.
- `simulation/pose_results_visibility_v2/summary.json` records seven actual rendered RGB-D replays: `far` and `clear` pass the fixed 20 mm/10 degree acceptance, with errors 2.942 mm/0.577 degrees and 2.093 mm/0.435 degrees respectively. Five other views return `no_acceptable_pose` and null truth errors. This remains offline replay, not live Action or physical grasp evidence.
- At review time fresh pose Action validation was not complete; camera-health failures were under investigation. The live result, new model hashes, C++ equivalence and new repeated Grasp Action results must be qualified after they exist.

## Reviewed source hashes

These identify this review snapshot, including the then-outstanding clock sampling race:

| File | SHA-256 |
|---|---|
| `manipulation_perception_server.cpp` | `33ebba649ca75bd02781f60349975ccd4321c0abf51752829d76e3313478be48` |
| `registration.cpp` | `e677971c732f38aa5964c26ae2c0741a45987a3f58796512d8cd721738577226` |
| `graspnet_torch_ops.py` | `89d9ed6e48dcdb73f10747db3450e9c4361cae7077cb8a7901f6665511cd07a1` |
| `sim_pose_evaluate.py` | `c3cf0f737ab727e51880bfe34818af72d3cd1699b8aa1229d150c78a34d23d0f` |

Assessment: initial P2 fixes are present, with one clock-ordering P2 sent for correction and live verification still pending. Hardware, VLA, general-mesh/unknown-object perception, full-robot collision/IK and physical grasp success were considered and left outside the explicitly authorized scope; no conclusion about them is implied.
