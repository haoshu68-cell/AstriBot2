# Frozen map→odom decomposition contract

Sources frozen verbatim on 2026-09-21 into the shared native package's
`test/reference/astribot_s1_perception/`; tests prepend only that explicit oracle
root, verify the node's actual `__file__`, and never fall back to production imports.

- `map_odom_decompose.py`: SHA256 `d8c1b4d1fc1340c1f32a61d2ee400bffd1aa624cd25ea7945fcb1b34f86a314b`.
- `map_odom_tf_node.py`: SHA256 `e259583aeb3cc4a54a89a77fc6acc6a3ee1a7221dde8c89dcf6fb158343b4d8e`.
- Original namespace initializer is empty. Root owns its common relay reference.

## Interfaces and ownership

Executable name remains `map_odom_tf_node`, node `map_odom_tf`. Sole application
output is dynamic TF `map_frame→odom_frame`. Inputs are latest TF from
`slam_world_frame→slam_base_frame` and `odom_frame→base_frame`. Default frames are
`map→aft_mapped`, `odom→astribot_torso_base`, output `map→odom`. There are no command,
control, map or odometry publishers. TF listener uses reliable/volatile depth100
`/tf`, reliable/transient-local depth100 `/tf_static`; broadcaster uses standard
reliable/volatile depth100 `/tf`. ROS parameter services/events, clock and rosout
are infrastructure. Both source bases must identify the same physical point;
this precondition is inherited, not verified by the node.

## Geometry and counters

Output SE(2) = SLAM pose composed with inverse odometry pose. Yaw uses only `z,w`:
`atan2(sin(2 atan2(z,w)),cos(2 atan2(z,w)))`, without quaternion normalization.
Angles preserve the Python ±pi and signed-zero results; do not replace with fmod.
Output quaternion x/y are zero; height is directly `slam.z-odom.z`, with no finite
validation. x/y/theta Pose2D construction rejects nonfinite values. Runtime
inverse/composition overflow raises DecompositionError and terminates with code2.

Tilt is `hypot(qx,qy)>max_tilt_rad`, not an Euler-angle bound. Despite the field
name `rejected_tilt`, excess tilt only increments the counter and warns: output
continues using the planar yaw. NaN tilt/threshold comparison is false; negative
threshold warns even for zero tilt. Preserve these inherited policies.

Jumps compare only translation distance to the previous successful result, using
strict `> jump_report_m`. Pure yaw change is not a jump. First update has zero
jump distance; every successful calculation increments updates. Jumps increment
count and maximum only when above threshold; the correction is never filtered.

## Source age, clock and ordering

Each tick performs SLAM latest lookup first; failure returns immediately without
querying/counting odom. Only a successful SLAM lookup proceeds to odom. Latest
lookups are not synchronized to a common timestamp. Each source's age is checked
at its own lookup completion. Height is cached before tilt/Pose2D handling.

Age arithmetic deliberately matches Python: node now is `nanoseconds*1e-9`;
source stamp is `sec+nanosec*1e-9`; subtract the floating seconds. Exact expiry is
fresh (`age>max` rejects); future stamps reject only `age<-0.05`. A nominal decimal
50ms may land just beyond the threshold due to floating-point subtraction (for
example `10-10.05`), so the oracle, not decimal intuition, defines that boundary.
Nonfinite now/stamp raises before any bypass. `max_source_age_sec<=0` returns fresh
age0 before zero/future/expiry checks. Otherwise `stamp<=0` rejects, including
static zero-stamped TF. NaN maximum disables only the upper-age comparison, not
zero/future rejection. These are existing escape hatches, not new recommendations.

Python Humble `Buffer()` has no ROS clock reset callback. Its `can_transform`
waits on SYSTEM_TIME in 20ms sleeps, uses latest time0, and breaks on system-clock
rollback over3s. `TransformListener` has no dedicated thread; the single executor
cannot consume TF/ROS-clock/watchdog callbacks during that wait. Native therefore
uses BufferCore + same-executor listener and the same system-time polling, not a
ROS-clock Buffer or threaded-listener shortcut. `use_clock_thread(false)` keeps
C++ clock updates in this executor too. Only Lookup/Connectivity/Extrapolation
are recoverable misses; other TF exceptions terminate nonzero.

Publish timer `1/publish_rate`, fixed2s startup watchdog, and optional report timer
all use ROS time. Paused ROS clock pauses nonzero timers; rollback does not clear
TF cache. Missing default timeout is0.2s per attempted edge. C++ uses the static
single-threaded executor's collected-ready-set traversal, preserving callback
progress for valid zero-nanosecond timers instead of timer starvation.

The startup watchdog checks only whether `updates` has ever exceeded zero. If not,
when a2s ROS timer callback observes `now-start>=source_timeout_sec`, exit1. After
one success the watchdog is permanently disabled, even if both sources disappear;
stale checks stop output but the process stays alive. No post-start liveness
supervisor or cache-clearing policy is added.

## Parameters and exits

All12 custom parameters are snapshots at construction/ROS-timer creation, with
normal statically typed ROS declarations. Typed runtime setter success changes
the parameter store but none of these cached values or timers. `use_sim_time`
keeps the standard dynamic ROS behavior.

| Parameter | Type/default | Original acceptance/consumption |
| --- | --- | --- |
| slam_world_frame/slam_base_frame/map_frame/odom_frame/base_frame | string; map/aft_mapped/map/odom/astribot_torso_base | world must equal map; others passed through TF |
| publish_rate | double20 | strict >0; NaN/zero/negative cause DecompositionError2; +inf produces0ns timer |
| tf_timeout_sec | double0.2 | duration conversion in each lookup; negative finite means no waiting; NaN/Inf/out-of-int64 fail on first tick with code1 |
| jump_report_m | double0.30 | strict >0; +inf accepted; NaN/nonpositive code2 |
| max_tilt_rad | double0.10 | no additional validation; warn-only comparison |
| source_timeout_sec | double60 | no validation; NaN/nonpositive trigger first2s watchdog, +inf disables timeout |
| report_period_sec | double10 | create timer iff >0; NaN/nonpositive disable report; +inf fails timer construction1 |
| max_source_age_sec | double1 | no additional validation; bypass/comparison rules above |

Wrong ROS parameter types are rejected during declaration before Python's float
conversion. The intended normal SIGINT/ExternalShutdown path exits0, watchdog exits1,
DecompositionError exits2, other startup/runtime exceptions exit nonzero (tested1).
The archived busy-SIGINT stress observation separately reproduced a historical
rclpy `take_message` RuntimeError exit1; this is not treated as a normal successful
shutdown or silently removed from evidence.
Miss/stale/tilt warnings retain first3/every100 suppression. Periodic logs preserve
updates, jumps/max, tilt, per-side misses/staleness and last pose. Cosmetic prose
is not a wire contract; meaningful counters and severity are retained.

## Scope and limitations

This is the existing map→odom role, not a SLAM estimator or control owner. Pure
math and isolated real-ROS TF evidence cannot certify Voxel-SLAM, Gazebo, physical
accuracy, base-frame calibration, hardware, or the full navigation tree. Inherited
warn-only tilt, unchecked height, disabled/NaN age gates, asynchronous latest
source times and startup-only watchdog behavior are intentionally documented
rather than silently corrected during migration.
