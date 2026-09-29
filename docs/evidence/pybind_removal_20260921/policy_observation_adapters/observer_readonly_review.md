# Read-only observer integration review

No root-owned file changed and no ROS process was started by this reviewer. Parent is running the isolated ROS differential suite. Review snapshot hashes (root may subsequently change these files):

- `ws_robot/src/astribot_s1_navigation_policy_native/src/policy_observer_node.cpp`: `d4814ad3f572545cf6ee54c4bef5f40d9c90e41b091403a2d084bf86d75861eb`
- `ws_robot/src/astribot_s1_navigation_policy_native/src/policy_observer_core.cpp`: `e9f08e0bfce8b9f7cc4227bc9bfb7b044c7733ebf85ef3ca511cb9352b84737a`
- `ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_observer_ros.py`: `fbab5c58b2c1899282576360ed7909ace20adff15f25f5da119ef4aa16ee4cb1`

Confirmed concrete finding: constructor `adapter_options` used a separate `integer` parser and direct `get<double>()`. Python accepts configured max_points `"32_768"`, calibration_epoch `"１２３"`, and position_variance_m2 `"0.0025"`; that native startup code rejected them. Reported to parent, who requested the now-public `adapter_integer` and `adapter_float` thin wrappers. Parent owns applying and verifying them in the ROS node. This finding is independent of the explicitly retained signed-64-bit and lone-surrogate adapter gaps.

Verified by source comparison against frozen production behavior: scan mailbox cap five, last transformable arrival selection, stale removal, future/delayed-TF retention, process-plan retry with latest transform, capture-time scan transforms, health update after fusion/clear evidence, and rollback clearing odometry/path/pending scans/pending plan keep the same ordering. Parent's raw `last_packet` revision defers malformed empty-packet sensor validation until the original health transaction; hashable non-string sensors pass through fusion ingest before health rejection, while unhashable sensors fail at lookup. Resolution iterates in transport order and retains earlier clears if a later ID is malformed.

Map helper preserves ignored origin z, finite metadata acceptance, occupied threshold 65, 5x5 dilation with two-cell exterior queries and equality of positive/negative zero in map identity. No additional confirmed queue, source-time, TF, rollback or partial-update discrepancy was found in this read-only pass. This is a scoped review result, not exhaustive equivalence or a replacement for the parent's ROS tests.
