# execution_response_scene02 observer evidence

Observer exit 130; 64.288336611 s captured, 2 s warmup, 62.288336611 s post-warmup. Both analyzer statuses INCOMPLETE. Writer dropped=0, queue peak=19. No 300-second PASS.

One parent submission is recorded. The first EXECUTING feedback to first FIRST_STAGE_HOLD_CONFIRMED feedback interval is steady 82289.863465815–82298.966628639 s (9.103162824 s), ROS 70.704–79.029 s. The exact probe records were checked against these boundaries. ACK.mode AttributeError interrupted verification and cleanup; probe disposition is STOP_UNPROVEN_HOLD_NOT_CANCELLED_OWNING_SIMULATION_TEARDOWN_REQUIRED, parent_terminal absent. Process teardown is not action release proof.

Configured=41, full-window present=34, post-warmup present=33. /tf_static retains 8 startup samples and has 0 post-warmup samples. Whole-capture absent: both wrist raw health/projection health/cloud (6 topics) and /cmd_vel_policy_input.

Metrics below use adjacent received samples. Source gaps are received header stamp differences, not complete sensor acquisition history. Counts are strict >250 ms. Motion-window boundary silence and intersecting pairs are also in observer_summary.json; they add no extra >250 ms depth gaps.

## Entire capture, including startup

| Depth | Samples | Source max / >250 count | Receive p99 / max (ms) | Receive >250 count | Initial / terminal silence (ms) |
|---|---:|---|---|---:|---|
| head_rgbd | 1039 | 250.000000 ms / 0 | 161.936695 / 844.833896 | 3 | 825.093946 / 3.241910 |
| torso_rgbd | 998 | 250.000000 ms / 0 | 171.798847 / 343.844721 | 4 | 805.707611 / 24.194410 |
| left_wrist_rgbd | 911 | 250.000000 ms / 0 | 206.287567 / 264.959382 | 4 | 1718.713426 / 118.999467 |
| right_wrist_rgbd | 894 | 300.000000 ms / 3 | 208.810384 / 796.764903 | 7 | 817.889792 / 10.342716 |

## Post-warmup capture

| Depth | Samples | Source max / >250 count | Receive p99 / max (ms) | Receive >250 count | Initial / terminal silence (ms) |
|---|---:|---|---|---:|---|
| head_rgbd | 1031 | 250.000000 ms / 0 | 161.407999 / 263.179775 | 2 | 50.149690 / 3.241910 |
| torso_rgbd | 990 | 250.000000 ms / 0 | 163.049524 / 284.012883 | 2 | 26.299875 / 24.194410 |
| left_wrist_rgbd | 905 | 250.000000 ms / 0 | 206.318596 / 264.959382 | 4 | 33.366138 / 118.999467 |
| right_wrist_rgbd | 885 | 300.000000 ms / 3 | 203.568729 / 430.805072 | 6 | 41.794608 / 10.342716 |

## Execution feedback window

| Depth | Samples | Source max / >250 count | Receive p99 / max (ms) | Receive >250 count | Initial / terminal silence (ms) |
|---|---:|---|---|---:|---|
| head_rgbd | 142 | 250.000000 ms / 0 | 184.067557 / 263.179775 | 1 | 25.316292 / 51.815453 |
| torso_rgbd | 138 | 150.000000 ms / 0 | 178.329835 / 197.845048 | 0 | 5.251364 / 12.401805 |
| left_wrist_rgbd | 131 | 250.000000 ms / 0 | 249.710126 / 264.959382 | 2 | 12.176221 / 3.548861 |
| right_wrist_rgbd | 126 | 300.000000 ms / 1 | 239.795125 / 369.340998 | 1 | 18.363438 / 60.587275 |

## Motion health

| Stream | N/state | Capture-age p99 / max (ms) | Same-capture steady-age max (ms) | Epoch |
|---|---|---|---:|---|
| /perception/camera_health/head_rgbd | 182 / {'OK': 182} | 89.380000 / 120.000000 | 54.525845 | gazebo_camera:82207957405158:659910483:g6 |
| /perception/projection_health/head_rgbd | 455 / {'READY': 455} | 82.380000 / 90.000000 | 81.590576 | 3154587-82207945637378:6 |
| /perception/camera_health/torso_rgbd | 182 / {'OK': 182} | 63.190000 / 65.000000 | 51.685958 | gazebo_camera:82207965410556:4140620482:g5 |
| /perception/projection_health/torso_rgbd | 455 / {'READY': 455} | 64.000000 / 70.000000 | 100.076095 | 3154589-82207964993092:7 |

All four observed motion health streams have valid/nonnegative/available capture ages, no invalid/STALE, one epoch each. Wrist health is unavailable, not healthy. Same-capture steady age is observer persistence, not acquisition latency.

Whole capture has raw head STALE=8, torso STALE=9; projection head OUTPUT_STALE=22, torso OUTPUT_STALE=19, all within start+0.310–1.464 s. Multiple epoch labels are received during warmup, with final labels first seen by start+1.444 s; no later or motion-window epoch change. Because health topics are transient local, these warmup observations alone do not prove physical source restart during the capture.

## Guard raw state

Full capture: 3210 messages; empty reason 2653, WAITING_FOR_EXECUTION_EVIDENCE 2, EXECUTION_WITHIN_BOUNDS 555. Guard armed at source stamp70.606, healthy at70.636. Motion window: 456 messages with reason EXECUTION_WITHIN_BOUNDS; active/healthy true for453 and false for3. Maximum recorded joint error=0.032049613341732286 rad.

The inactive/healthy=false transition is receive steady82298.913337903, data.stamp78.976, followed by false samples78.996 and79.014. This is before the first hold feedback receipt by53.290736 ms; raw reason remains EXECUTION_WITHIN_BOUNDS. Do not equate that retained reason with active guard health or with lease release. Last native status remains phase3 / FIRST_STAGE_HOLD_CONFIRMED, authority_reason HOLDING, hold_confirmed=true.

## Motion receive gaps >250 ms

| Depth | Receive endpoints (steady s) | Source endpoints (s) | Receive / source gap (ms) |
|---|---|---|---|
| /camera/raw/right_wrist_rgbd/depth_image | 82296.658881597–82297.028222595 | 76.950–77.250 | 369.340998 / 300.000000 |
| /camera/raw/head_rgbd/depth_image | 82292.882932927–82293.146112702 | 73.550–73.800 | 263.179775 / 250.000000 |
| /camera/raw/left_wrist_rgbd/depth_image | 82291.354784985–82291.619744367 | 72.100–72.350 | 264.959382 / 250.000000 |
| /camera/raw/left_wrist_rgbd/depth_image | 82298.580092937–82298.844498254 | 78.650–78.900 | 264.405317 / 250.000000 |

## SHA256

- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02/m5_observer/manifest.json`: `fdfe0741e8ebdf533baa635927469a8e50d8962145cf0ff53690daf48f42b2c5`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02/m5_observer/events.jsonl`: `e0ad9db785509a5a2b13cce46fc0ba7cefd08d6b8805c310dd39f4a1e253ad9d`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02/m5_observer/session_snapshot.json`: `147897861dac03a53201bd7739f2ce7150ed9e475a3a0f1dc416d615579aeb86`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02/observer_pre_action.json`: `eb04fe21b4e8212939de3b421ad9ee91c29c0731cc52c5eb248a6398a9b4c683`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02/first_stage/result.json`: `75ad591a37b165c773a06a55ee137ce0717c9426210a684cccdaca3aded37224`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/execution_response_scene02_motion/observer_analysis.json`: `78470f6fd1472858094277af6fe1e5c25954dc0256e9128044982f3783572f9b`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/tools/vision/analyze_m5_capture.py`: `a2b60b66408bd59d2643fd3e5f3f74d22aa31c2612d13ee73267d229b91dc35b`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/execution_response_scene02_motion/observer_summary.json`: `080db40931de6db1e54ffba258e8800b51a006f432392170954f3e177ccacd4f`
