# Task chain, fresh timing round 2026-09-22

User authorized restarting each task's 90-minute clock. Original attempts and failures remain in ../task_chain_20260921/task_timing.json. No VLA, hardware, or complete dynamic pick/carry/place validation.

W0 completed 00:52:43 within a **pinned installed simulation startup baseline**. Evidence: w0_acceptance.json and integration_manifest_clean.json. Includes actual wrong overlay/library rejection, bounded C++ Gazebo URDF fetch (one actual lost first request recovered), three cold starts, and verified owned cleanup. Gazebo time rewind invalidated camera health but **did not recover automatically within 15 seconds**; controlled cold restart recovered. This limitation is preserved, not marked fixed.

W1 started 00:52:43; deadline **02:22:43 +08**. Retries do not reset it.

- Main navigation launch now forwards six-camera switches and profiles; two offline tests pass. First full launch rejected incompatible fixed_v2/off configuration; corrected P4 launch underway. No motion success claimed.
- C++ wrist ACQUIRE/RENEW/RELEASE service and bounded cooperative lease implemented. Six core tests, 17 initial health tests, and 24 checks with real Gazebo images passed. Owner rejection, release, expiry, token replacement and source epoch checked. Controls ROS processing, not sensor power or Gazebo rendering; full transport integration remains pending.
- Additional test reproduced old queued images being accepted after new activation. Added activated_at and capture-generation rejection; updated build/regression underway in a separate overlay to preserve binaries used by running simulation.
- 600 s four-RGB-D baseline: 5990 exact triples per camera. After 2 s warmup: head 2 STALE; right wrist 1 STALE; torso/left 0. Not a continuous-health pass. 250 ms ROS/wall freshness unchanged. Short builds overlapped; this is not an exclusive-host benchmark. Added wall-age/ROS-age diagnostics; recorded age alone cannot distinguish simulation pause from delivery delay.

Current evidence: implementation_status.json, w1_baseline_health.json, ../../../../runs/task_chain_20260922/wrist_session_live/report.json.

01:35 update: the 600 s SLAM/Nav2-idle run with the initial wrist relay failed
sampled health: left STALE 2199/RATE_LOW 7222, right STALE 2030/RATE_LOW 6685;
head STALE 3, torso STALE 2. These counts include ~40 Hz wrist health callbacks
and ~20 Hz head/torso callbacks, so percentages must use each camera's own sample
count. v3 stream counters then showed complete relay reception/forwarding but
continued downstream gaps; queue/clock edge fixes alone did not fix that live
failure. The relay lacked the existing large-image DDS transport used by the
Gazebo bridge. Explicit reuse of that profile is now under a 600 s comparison.

An actual renewal was rejected when caller clock led server by 1 ms
(113.909 s vs 113.908 s). The validation client retries only this proven ordering
case, retaining original ID/timestamp within a 200 ms bound. Core adds explicit
REQUEST_CLOCK_AHEAD diagnosis; 7 pure tests pass. No freshness/lease tolerance
was increased. C++ deferred-frame tests: 8 pass; health: 18; projection: 18.

02:05 update: final v6 C++ session/health build passed 53 cases in four CTest groups;
path tracking passed six CTest groups. Large-image DDS profile removes the severe
STALE/RATE_LOW seen with default transport in the matched 600 s comparison.
The older comparison still had 11 wrist inactive health samples; the future
heartbeat ordering fix removed these in the next 600 s window, which still had
7 total stale samples during support startup/build disturbances. No continuous
freshness acceptance is inferred from short checks or recorder completion.

Final moving-load attempt: both arm compact plans executed and V2 envelope
handshake succeeded after loading current navigation consumers. One navigation
goal was sent and rejected. Live zone status is ZONES.NO_CONTEXT; installed map
manager lacks the source's simulation_static_map_yaml parameter. The physical
social_sim geometry stream was also absent in this manually assembled session.
The old outer report masked the action rejection with a missing-samples timeout;
reporting order is corrected, and replay of the saved failed protocol yields FAIL
with ACTION_REJECTED. Historical reports remain unchanged.

Session mode is static-map navigation with ground-truth localization. VOXEL SLAM
computation is also running (launch_slam=true, map/odom output disabled for this
backend), as shown by actual process logs and launch conditions. This is not
SLAM-localized navigation. The v6 capture is still running; no MPPI movement passed.

02:09: final v6 600 s sampled freshness passes all four RGB-D cameras: head 11960, torso 11959, left 23067, right 23060 valid health samples after warmup; no invalid samples, max callback gap 69.16 ms. This is a stationary window with static-map navigation and VOXEL SLAM computation plus failed navigation admission, not MPPI moving acceptance. CUDA pressure started 02:08:35 on frozen historical cloud; results pending. Explicit wrist launch DDS profile scoping now passes 2 observer tests after a recorded failure; no source-install substitution during capture.

02:20: GraspNet CUDA compute pressure finished 600.07 s: 296 workers succeeded, one final invocation canceled by the workload deadline, zero completed-worker failures. Camera capture lasted 620 s: all four sources passed sampled freshness with zero invalid samples after warmup (head 12360, torso 12360, left 23835, right 23836). Max health callback gap 74.82 ms. This is frozen-cloud compute pressure, not current-camera model accuracy or actual MPPI motion. All owned simulation and camera processes exited; final supervisor reports remaining_owned_pids=[]. Map-manager build then failed on upstream exploration session_archive.hpp missing from its stale install; current dependency build is bounded by remaining task time.

Final pause: W1 90 minute deadline 02:22:43 reached. Fresh map-manager configure/build succeeded (38.2 s); static map context CTest passed (two cases, 3.23 s). These are build/isolated ROS evidence only; no live navigation reintegration after the update. No further tasks started. Both 600 s and 620 s camera windows passed sampled freshness; all owned runtime already stopped. Final status recorded 2026-09-22T02:23:10.283772+08:00.
