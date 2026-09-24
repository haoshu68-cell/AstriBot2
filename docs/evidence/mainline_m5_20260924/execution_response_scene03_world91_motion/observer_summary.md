# execution_response_scene03_world91 observer evidence

Observer exit 130; 78.127651064 s captured, 2 s warmup, 76.127651064 s post-warmup. Both existing-analyzer statuses INCOMPLETE. Writer dropped=0, queue peak=18. No 300-second sampling PASS.

One PREGRASP parent submission is recorded. First EXECUTING to first FIRST_STAGE_HOLD_CONFIRMED feedback interval: steady82893.838213904–82906.853869587 s (13.015655683 s), ROS59.023–71.419 s. Owner and probe reports pass; probe resource_disposition=RELEASE_CONFIRMED and parent status=5/TASK_CANCELED with resources_released=true. This confirms the scoped first-stage/cancel-release result, not full PICK completion.

Configured=41, full-window present=35, post-warmup present=34. /tf_static retains10 startup samples with0 post-warmup. Whole-capture absent: left/right wrist raw health, projection health and cloud (6 topics). All16 raw streams are present.

Metrics use adjacent received samples. Source gaps mean received header-stamp differences, not full sensor acquisition history. Counts use strict >250 ms. Motion-window initial/terminal silence and intersecting receive-gap pairs are also reported; intersecting pairs add no extra >250 ms depth gaps. Percentiles use linear interpolation. Independent post-warmup summary values were compared to the existing analyzer for all16 raw streams and matched exactly.

## Entire capture including startup

| Depth | Samples | Source max (ms) / >250 count | Receive p99 / max (ms) | Receive >250 count | Initial / terminal silence (ms) |
|---|---:|---|---|---:|---|
| head_rgbd | 1294 | 200.000000 / 0 | 158.436891 / 704.422891 | 2 | 530.450880 / 3.520079 |
| torso_rgbd | 1215 | 250.000000 / 0 | 164.280028 / 371.060385 | 3 | 516.660696 / 24.231665 |
| left_wrist_rgbd | 1142 | 250.000000 / 0 | 202.798774 / 370.649744 | 5 | 520.517330 / 15.836794 |
| right_wrist_rgbd | 1179 | 250.000000 / 0 | 165.480083 / 238.987611 | 0 | 1230.339553 / 53.427985 |

## Post-warmup capture

| Depth | Samples | Source max (ms) / >250 count | Receive p99 / max (ms) | Receive >250 count | Initial / terminal silence (ms) |
|---|---:|---|---|---:|---|
| head_rgbd | 1282 | 200.000000 / 0 | 156.524768 / 272.870018 | 1 | 64.445884 / 3.520079 |
| torso_rgbd | 1203 | 250.000000 / 0 | 159.861639 / 302.937348 | 1 | 40.681510 / 24.231665 |
| left_wrist_rgbd | 1130 | 250.000000 / 0 | 202.120229 / 281.427171 | 3 | 49.167172 / 15.836794 |
| right_wrist_rgbd | 1169 | 250.000000 / 0 | 163.157321 / 238.987611 | 0 | 55.410098 / 53.427985 |

## Execution feedback window

| Depth | Samples | Source max (ms) / >250 count | Receive p99 / max (ms) | Receive >250 count | Initial / terminal silence (ms) |
|---|---:|---|---|---:|---|
| head_rgbd | 206 | 150.000000 / 0 | 158.623496 / 162.485590 | 0 | 10.520448 / 35.038340 |
| torso_rgbd | 207 | 150.000000 / 0 | 160.043584 / 187.056434 | 0 | 38.187014 / 5.637256 |
| left_wrist_rgbd | 195 | 250.000000 / 0 | 213.102105 / 281.427171 | 1 | 0.848197 / 0.624658 |
| right_wrist_rgbd | 191 | 200.000000 / 0 | 205.256867 / 214.292246 | 0 | 4.119295 / 45.793970 |

## Motion health

| Stream | N/state | Capture-age p99 / max (ms) | Same-capture steady-age max (ms) | Epoch |
|---|---|---|---:|---|
| /perception/camera_health/torso_rgbd | 260 / {'OK': 260} | 67.690000 / 85.000000 | 53.391682 | gazebo_camera:82824550051118:1728091346:g5 |
| /perception/projection_health/torso_rgbd | 650 / {'READY': 650} | 61.510000 / 71.000000 | 57.765722 | 3191819-82824561071974:5 |
| /perception/projection_health/head_rgbd | 647 / {'READY': 647} | 81.000000 / 88.000000 | 60.275364 | 3191817-82824556450420:5 |
| /perception/camera_health/head_rgbd | 260 / {'OK': 260} | 82.410000 / 85.000000 | 53.546850 | gazebo_camera:82824542091793:3914239521:g5 |

All observed motion health streams have valid, available, nonnegative capture ages; no invalid/STALE and one epoch each. Wrist health is unavailable, not healthy. Same-capture steady age is observer persistence, not acquisition latency. Projection pending slot max during motion: head1, torso0; not DDS queue occupancy.

Whole-capture raw STALE: head6, torso13; projection OUTPUT_STALE: head23, torso24. All invalid records are within startup+0.018–1.227 s. Received epoch changes are confined to warmup. These transient-local startup observations alone do not establish source restart during the capture.

## Guard raw state

Full capture3908 messages: empty reason1978; WAITING_FOR_EXECUTION_EVIDENCE2; EXECUTION_WITHIN_BOUNDS1928. Armed at data.stamp58.912 s, healthy at58.938 s. Motion window652 messages all retain EXECUTION_WITHIN_BOUNDS;647 active/healthy=true and5 false. Max sampled joint error=0.03226525098039773 rad.

First inactive/healthy=false: receive steady82906.788152627 s, data.stamp71.354 s,65.716960 ms before the first hold feedback receipt. The reason remains EXECUTION_WITHIN_BOUNDS. Retained reason alone is not active guard health. Final native status is phase0/TASK_CANCELED, authority_reason RELEASED, hold_confirmed=false; the independent probe parent result confirms resources_released=true.

## Motion receive gaps >250 ms

| Depth | Receive endpoints (steady s) | Source endpoints (s) | Receive / source gap (ms) |
|---|---|---|---|
| /camera/raw/left_wrist_rgbd/depth_image | 82896.963903262–82897.245330433 | 61.950–62.200 | 281.427171 / 250.000000 |

## SHA256

- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/m5_observer/manifest.json`: `701eaf399557273772e97f316e507ff0a4dfbabfa00099ff70f7c86f4ee6e38d`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/m5_observer/events.jsonl`: `59c5676c2ffadd5e4d46147c3d4bf9af124cbe66ac63ad63bc55e9a8a60f9cd6`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/m5_observer/session_snapshot.json`: `dd38a6a19dd897647c5d81b0754b2da3bce4f125ea873cb1f38d088ed22a4978`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/observer_pre_action.json`: `bfaffff71dde1c46af41363eccf6d833da87632781dba955d966eaad10f643f4`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/first_stage/result.json`: `fe455982f0c126cdad7ec763eb9c7e285c7696abc05f9422d621e0326e8e4e29`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/result.json`: `6d039dfa4049b0e95558e75da96d93d8257e8ebbdbd7e6ab0613dc11c5abe513`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/execution_response_scene03_world91_motion/observer_analysis.json`: `f44467a5fd75ef070e5dc012b82ba9dd121b434a08e496421309114cbb419a1d`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/tools/vision/analyze_m5_capture.py`: `a2b60b66408bd59d2643fd3e5f3f74d22aa31c2612d13ee73267d229b91dc35b`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/execution_response_scene03_world91_motion/observer_summary.json`: `23eb7b242dbac7b989181afe8fe7380e6776a2630560ce58ef699587590fd54a`
