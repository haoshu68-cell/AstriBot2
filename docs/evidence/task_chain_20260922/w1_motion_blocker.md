# Motion workload attempt: not accepted

Both compact arm trajectories completed (MoveIt/controller SUCCEEDED), then SetFixedEnvelope waited for controller/global_costmap/local_costmap/planner/policy ACKs. Only protection acknowledged. Navigation goal count is zero; the old protocol field motion_goals_sent refers to navigation only and must not be interpreted as no arm motion.

Loaded paths: w1_motion_loaded_plugins.json. Root install path-tracking library has no navigation_geometry_mode or ACTUAL_COSTMAP_FOOTPRINT_APPLIED strings; source and ws_robot install do. Live ListParameters confirms controller/planner do not declare navigation_geometry_mode. Root installed Python navigation policy also lacks V2 code. These are stale overlay consumers, not READY_RIGHT planning failures.

Mapping mode additionally reports ZONES_UNAVAILABLE because there is no selected navigation map. Keep that guard. After a corrected isolated overlay build, use the existing baseline static-map activation path for the MPPI-only comparison. SLAM+camera evidence remains a separate case; this does not prove simultaneous SLAM-based navigation.
