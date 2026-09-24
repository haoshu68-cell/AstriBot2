# M4 asymmetric entrance adapter — 2026-09-24 10:57 +08

This independent issue retains its 10:57 start; M1 retains its original 10:35 start.
Only corridor_adapter.py and the existing test_fixed_corridor.py were changed.

The adapter now bounds lateral displacement to the actual centering_target,
including a retained target. Fixed mode uses its asymmetric offset; legacy uses
zero. The 0.3 m bound and both sweep checks are unchanged.

RED: 5 new test methods, 3 failures (both mirrored legal entries and the expected
second sweep call). GREEN: all 21 methods, including 16 existing tests, passed.
The test compiles the complete production advance method from its AST and calls
the real FixedCorridorPolicy/legacy CorridorPolicy; ROS message/TF/publication and
world sweep inputs are substituted. This proves branch wiring and refusal
preservation, not live clearance sensing, navigation, or M4 passage acceptance.

Reproduce from /home/yjh/WorkSpace/astribot_sdk_ros2:

```sh
PYTHONPATH=ws_robot/src/astribot_s1_navigation_policy:/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/narrow_arms_20260923_1902/install_v2/astribot_s1_robot_geometry/local/lib/python3.10/dist-packages python3 -m unittest discover -s ws_robot/src/astribot_s1_navigation_policy/test -p test_fixed_corridor.py -v
```

No ROS was started, no install was changed, and no Git index was modified.
See red.log, green.log, fix.patch, and result.json.
