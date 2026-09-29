# Final source review — 2026-09-21

Scope: final camera-health matching logic and its 14 tests; warehouse bridge DDS configuration; Action deadlines, context/clock checks and model-output validation. Source was reviewed read-only. The reviewer started no ROS/Gazebo process and ran no GPU inference. The reviewer independently evaluated the eight offline launch-contract checks and inspected the coordinator's completed live evidence.

## Review conclusion

No remaining reproducible P1/P2 defect was found within the reviewed source scope. This is a bounded source-review conclusion, not proof of absence of defects or execution acceptance.

**Closed P2 — the loopback profile previously overrode `localhost_only:=false`.** The earlier implementation unconditionally applied the local profile to the entire bridge, including clock, TF and lidar, breaking the launch's external-machine mode. The final `warehouse_sim.launch.py:44-62` resolves the environment after launch arguments are available: automatic local mode selects the bounded loopback profile; automatic LAN mode returns an empty override and inherits the caller's environment; an explicit profile remains supported. The bridge is created by `OpaqueFunction`, so those selections are evaluated at launch execution time.

The reviewer independently ran `tools/vision/verify_vision_launch.py` with the ROS Humble, repository and durable candidate overlays: all eight checks passed, including all three profile selections. This script only evaluates launch actions and parameters; it does not start nodes. The first two reviewer invocations omitted the repository underlay and failed to resolve the logging package; adding that existing underlay resolved the environment prerequisite without changing source. No cross-machine communication test was performed, so LAN network operation beyond preservation of the caller's configuration remains untested here.

## Fixes and behavior verified

- `camera_health_node.cpp:118-163` buffers metadata for at most 32 pending samples per stream, matches frame/resolution-compatible triples within the configured skew, and consumes each sample at most once. Unmatched future callbacks cannot retimestamp the matched snapshot or renew its wall-clock receipt time.
- `camera_health_node.cpp:249-334` retains capture-time and steady-clock freshness, frequency, frame/resolution, calibration and rollback checks. Malformed images clear matches; calibration changes remain latched invalid; clock rollback clears matches and changes source epoch. The 14 source tests include delayed-depth matching, partial callback arrival, expired old matches, persistent skew, malformed data, frame changes, low rate and rollback. `health_sync/regression_green.txt` records all 14 passing; the reviewer inspected that evidence rather than launching tests again.
- The bridge XML declares 16 MiB SHM, 4 MiB maximum SHM message size, a UDPv4 `127.0.0.1` whitelist and disabled built-in transports. The package CMake installs the `config` directory. Python launch syntax and these XML values were independently checked without launching a process.
- The XML's loopback behavior has upstream implementation support: Fast DDS v2.6.10 joins multicast groups on whitelisted interfaces and restricts outgoing sockets to allowed interfaces. This source inspection does not claim a packet-capture test. The old launch description claiming that the whitelist only affects unicast is not a basis for reporting a leak. Sources: [UDPv4Transport](https://github.com/eProsima/Fast-DDS/blob/v2.6.10/src/cpp/rtps/transport/UDPv4Transport.cpp#L329-L362), [UDPTransportInterface](https://github.com/eProsima/Fast-DDS/blob/v2.6.10/src/cpp/rtps/transport/UDPTransportInterface.cpp#L336-L354).
- `manipulation_perception_server.cpp:281-296` now samples, compares and stores ROS time under `health_mutex_`, closing the previously reported ordering race. Request deadlines use one monotonic start, remaining worker budget and periodic in-flight checks; chunked hashing checks cancellation/context. Nonpositive-score or zero-width proposals are individually filtered, structural/numeric errors still abort, filtered/raw counts are retained, and an empty usable set fails. Collision flags remain false. Null unused pose visibility metrics are separated by `visibility_model_used`; pose scene/envelope values are preserved.

## Evidence limits and prior-record correction

`health_sync/under_inference_summary.json` records 60.0007 wall-clock seconds, 59.298 seconds of advancing simulation time, 593 exact-stamp RGB/depth/intrinsics triples per camera, and 1199/1199 health messages OK per camera. This demonstrates transport/synchronization health during that loaded interval; it does not establish useful head-camera scene visibility or physical calibration. The earlier ten-second `after_transport_and_sync_summary.json` is preserved separately.

The reviewer parsed both final files in `live_actions_v2/`: GraspNet has 20/20 accepted successful Action results from 20 distinct capture timestamps, and known-model pose has 10/10 from 10 distinct timestamps. Grasp wall latency is 1.175–1.277 s (median 1.210 s); pose is 1.572–1.633 s (median 1.598 s). All 640 returned grasp candidates retain false collision flags, the declared nonprobability score mapping, and model revision `5acc1ff77600a6e1b8a5a814b363956c9e12bf74abf89d85b94ba6f66cfd1b7c`. Pose observations retain scene/envelope revisions and visibility-model use; reported visible coverage is 0.775838. These are coordinator-run live Gazebo inference trials on a static clear scene using an HSV fixture mask, not YOLO segmentation, hardware validation, motion execution or successful grasps. The reviewer did not independently rerun live inference.

Model v2 evidence is separately recorded in `graspnet/v2/`: model/operator hashes changed after the FPS fix, while prior models were retained. This review does not retroactively treat historical failed live reports or older model outputs as success.

The earlier `review_final_pending.md` was written while implementation continued: its clock-race code excerpt preceded the coordinator's fix, but its subsequently collected server SHA already identifies the corrected source. That report's finding must not be attributed to the recorded `33ebba...` hash. The clock issue is closed by the source inspection in this report.

## Source snapshot

| File | SHA-256 |
|---|---|
| `camera_health_node.cpp` | `5570c03e5abb87c5048fbcfa5d18405df6f8b775935dd195a02fe444d7f7055b` |
| `camera_health_test.cpp` | `71633b277667361a482dcb82e993311921ca79e192c13854e4efefcd3fbc81a3` |
| `warehouse_sim.launch.py` | `8f3a5b00c14f6da5cca49e7a75498e9789fba1cf49d55af787eeeb8d0a8be411` |
| `verify_vision_launch.py` | `da895dd877682ea272665b9b0aace6b2c679ce205178040de573bfb2edb4b218` |
| `camera_bridge_fastdds.xml` | `c5a12257a092857018470c55ad851a4f148a888af7e480cbe0b062a1181e2b18` |
| `manipulation_perception_server.cpp` | `33ebba649ca75bd02781f60349975ccd4321c0abf51752829d76e3313478be48` |

Assessment: the identified launch regression is closed; no remaining concrete defect was found in the reviewed health/Action/configuration fixes. Fresh Action inference now has the separately recorded live evidence above. Hardware, VLA, full-robot collision/IK and physical grasp validation were considered and excluded because they are outside the authorized scope.
