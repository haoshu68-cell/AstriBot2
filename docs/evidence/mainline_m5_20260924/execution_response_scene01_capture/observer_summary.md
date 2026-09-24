# execution_response_scene01 observer offline analysis

This is a 44.631505915 s interrupted observer capture (exit 130), with 2 s warmup and 42.631505915 s metrics window. Both analyzer statuses are INCOMPLETE. There were 0 parent submissions and 0 feedback records; no motion coverage acceptance.

Configured topics: 41; present full window: 35; present post-warmup: 34. Writer dropped 0, queue peak 17. 83249 raw metadata events.

/tf_static: 10 full-window samples, 0 post-warmup samples. Startup data is retained; post-warmup MISSING is not whole-capture absence.

Whole-capture absent topics: /perception/camera_health/left_wrist_rgbd, /perception/projection_health/left_wrist_rgbd, /manipulation/camera/left_wrist_rgbd/points, /perception/camera_health/right_wrist_rgbd, /perception/projection_health/right_wrist_rgbd, /manipulation/camera/right_wrist_rgbd/points

All following raw metrics are post-warmup. Source range is header stamp in received messages; receive gap is observer steady time. Stereo >250 ms counts are descriptive only because its configured budget is null.

| Raw topic | N | Source range (s) | Source gap max (ms) | Receive gap p99 (ms) | Receive gap max (ms) | >250 ms count |
|---|---:|---|---:|---:|---:|---:|
| /camera/raw/head_rgbd/image | 794 | 23.500–64.400 | 150.000000 | 107.635115 | 209.397951 | 0 |
| /camera/raw/head_rgbd/camera_info | 802 | 23.600–64.500 | 100.000000 | 106.047115 | 201.658763 | 0 |
| /camera/raw/head_rgbd/depth_image | 737 | 23.550–64.450 | 200.000000 | 150.033210 | 210.042649 | 0 |
| /camera/raw/torso_rgbd/image | 737 | 23.550–64.500 | 200.000000 | 136.614279 | 209.328453 | 0 |
| /camera/raw/torso_rgbd/camera_info | 820 | 23.550–64.500 | 50.000000 | 69.925947 | 118.706666 | 0 |
| /camera/raw/torso_rgbd/depth_image | 690 | 23.550–64.500 | 200.000000 | 160.164726 | 270.467859 | 1 |
| /camera/raw/left_wrist_rgbd/image | 715 | 23.550–64.500 | 150.000000 | 155.652456 | 244.780113 | 0 |
| /camera/raw/left_wrist_rgbd/camera_info | 800 | 23.550–64.500 | 150.000000 | 105.083669 | 205.856474 | 0 |
| /camera/raw/left_wrist_rgbd/depth_image | 668 | 23.550–64.450 | 250.000000 | 200.547559 | 250.833482 | 1 |
| /camera/raw/right_wrist_rgbd/image | 697 | 23.600–64.500 | 250.000000 | 160.975527 | 257.389531 | 2 |
| /camera/raw/right_wrist_rgbd/camera_info | 805 | 23.550–64.500 | 100.000000 | 105.408416 | 190.972880 | 0 |
| /camera/raw/right_wrist_rgbd/depth_image | 643 | 23.550–64.450 | 350.000000 | 171.186221 | 365.507455 | 3 |
| /camera/raw/head_stereo_left/image_raw | 203 | 23.600–64.400 | 400.000000 | 307.807006 | 521.372798 | 4 |
| /camera/raw/head_stereo_left/camera_info | 205 | 23.600–64.400 | 200.000000 | 265.949589 | 309.697724 | 4 |
| /camera/raw/head_stereo_right/image_raw | 204 | 23.600–64.400 | 400.000000 | 282.129237 | 413.147560 | 4 |
| /camera/raw/head_stereo_right/camera_info | 204 | 23.600–64.400 | 400.000000 | 274.729651 | 521.820357 | 3 |

All 16 raw streams have no duplicate source stamps or source regression. Left stereo CameraInfo has one source-age sample of -9 ms relative to latest asynchronously observed /clock; no clamp applied.

## Health

| Health stream | Full invalid/state | Invalid steady offset (s) | Post-warmup state | Capture-age p99/max (ms) | Same-capture steady age max (ms) |
|---|---|---|---|---|---:|
| /perception/camera_health/head_rgbd | {'OK': 884, 'STALE': 13} | 0.007034733–1.356894000 | {'OK': 853} | 82.000/140.000 | 109.965516 |
| /perception/projection_health/head_rgbd | {'INVALID': 25, 'READY': 2193} | 0.147372628–1.385559475 | {'READY': 2126} | 82.000/100.000 | 119.708371 |
| /perception/camera_health/torso_rgbd | {'OK': 881, 'STALE': 13} | 0.158011347–1.366049047 | {'OK': 853} | 65.000/121.000 | 110.598064 |
| /perception/projection_health/torso_rgbd | {'INVALID': 27, 'READY': 2197} | 0.154639628–1.373051101 | {'READY': 2130} | 63.000/70.000 | 97.919590 |

Head/torso raw STALE (13 each) and projection OUTPUT_STALE (25/27) occur only in warmup. Wrist health has no samples in the whole capture, so its age/STALE status is unavailable. Head/torso projection pending slot sampled max is 1, not DDS queue occupancy. Capture ages use data.capture_stamp, not health publication header; all post-warmup capture ages are available and nonnegative.

## Guard and phase

Guard: 2224 full-window / 2132 post-warmup typed records; raw reason is the empty string in every record, active=false, healthy=false, empty context_id, joint_stamp=base_stamp=0. data.stamp is 22.933–64.517 s; receive steady is 81978.289881279–82022.722730005 s. Header source_ns remains null by message contract. Empty reason is preserved, not replaced with a fabricated diagnostic. Native hold status: 885 records, all phase 0 / WAITING_FOR_TASK, authority_reason IDLE, hold_reason NO_HOLD.

## Four depth worst-gap association

Raw bag process window: steady 81989.751067665–81995.289124833 s. Temporal overlap is not causal attribution. Nearby clock interval means all consecutive clock pairs intersecting the depth receive-gap interval.

| Depth | Receive endpoints (steady s) | Source endpoints (s) | Raw bag overlap | Nearby /clock max receive gap (ms) | Endpoint image source age (ms) | Endpoint last-clock receive age (ms) |
|---|---|---|---|---:|---|---|
| head_rgbd | 82019.782675217–82019.992717866 | 61.700–61.900 | False | 6.688516 | 20/29 | 3.645989/2.675914 |
| torso_rgbd | 81992.431111025–81992.701578884 | 35.350–35.500 | True | 17.451281 | 6/17 | 2.824686/2.196320 |
| left_wrist_rgbd | 82013.543249174–82013.794082656 | 55.600–55.850 | False | 5.724060 | 14/8 | 1.088334/4.494743 |
| right_wrist_rgbd | 82003.585953735–82003.951461190 | 45.850–46.200 | False | 4.753069 | 17/23 | 1.170507/0.533539 |

Only torso worst depth gap overlaps the raw bag process window; the other three do not. No nearby clock regression. Whole post-warmup clock max receive gap is 22.129319 ms, source advance/steady span observed ratio 0.9616157835. This does not identify a cause of camera gaps.

## SHA256

- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01/m5_observer/manifest.json`: `4e4a75df59886eba5eb5a5deffe86663653fdac1e3f9e4173ffbe183d912ce91`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01/m5_observer/events.jsonl`: `1e852913fc44adcaf15d4fee637756c9f950fda07241d2d5d2d7f5ebc13f67e9`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01/m5_observer/session_snapshot.json`: `0dd02f22aea1d9791746c1c069a620a59cf699e93a4d5db9c4cfe29bb38ad1e6`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/execution_response_scene01_capture/observer_analysis.json`: `65d6136bd3977aafe5f8a92e9f0735681d471ecffff63299fba23c33082b7070`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01/observer_pre_action.json`: `5739d8b032f6f3868e8a80baff804aa33c75e6b265d9ea084a8afff485d0cc3a`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01/used_run_first_stage.py`: `13af6d905e43c747890a6e34b776792b749ede957737fd72b22e0f2eeb1fc3ae`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01/used_verify_first_stage.py`: `11d3ec039751a99b76d5dd7edabb75f710463f84f83349aa69e291bb2e5779f5`
- `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01/used_observer_topics.json`: `8afd2cf3f187d80f388366e8b1eaaa0bee950a5ca27fd16ecdf9240c5d1f8bad`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/tools/vision/analyze_m5_capture.py`: `a2b60b66408bd59d2643fd3e5f3f74d22aa31c2612d13ee73267d229b91dc35b`
- `/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/mainline_m5_20260924/execution_response_scene01_capture/observer_summary.json`: `ba3d7f22761e922793728c3bbd58e0f93891fe7ebbaeffb554408514643bbf0d`
