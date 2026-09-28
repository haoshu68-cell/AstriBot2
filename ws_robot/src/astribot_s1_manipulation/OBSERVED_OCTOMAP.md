# Point cloud integration evidence

`ObservedPointCloudUpdater` is the only configured point-cloud updater in this
branch. It derives directly from MoveIt's `OccupancyMapUpdater`. It retains the
existing log-odds and self-filter behavior; it does not wrap the
old updater or run a second map writer. `moveit_ros_perception` remains a build
and runtime dependency for its existing `ShapeMask` implementation. The old
updater is instantiated only by differential tests.

For each successful integration it publishes `/moveit/observed_octomap` with the
original cloud header, callback time, source name, cloud-to-map and sensor-to-map transforms, and
the full resulting Octomap. Serialization and revision assignment happen under
the same tree write lock as integration. The process epoch and revision identify
the immutable snapshot; consumers must order by revision because callbacks from
different sensors can publish in a different order after releasing the lock.

The packet includes full-depth keys traversed by measured rays that are also
free in that exact snapshot. Rays ending on the self-filter and model clearing
are excluded. Keys absent from this list do not establish current free space.
This list does not prove that every point in a voxel was observed, that gaps
between rays are safe, or that calibration and sensor accuracy are sufficient.

Duplicate source stamps remain duplicates in the packet, even if another
integration advances the revision. Empty clouds and invalid-depth-only clouds
have no free-ray evidence. Missing sensor/model transforms publish no integration
packet. Malformed byte layouts fail explicitly before map access. Accepted XYZ
layout is native little-endian FLOAT32, consecutive XYZ fields (possibly after
other fields), packed rows and per-point padding; this matches the upstream
iterator/self-filter path used here. Exceptions during integration/serialization
propagate after releasing the RAII lock; no partial integration is reported as
successful.

This is evidence transport, not a CLEAR/RISK authority. Source freshness,
calibration/structural epochs, current-to-first-point motion, execution binding,
coverage of the swept volume and the physical reaction/braking budget still
require the next acceptance stages. No online automatic resume consumes this
topic yet. `callback_stamp` is after the TF message filter; processing time does
not include waiting in that filter. Full map serialization increases write-lock
time and must be measured with the actual map and sensor load before acceptance.

The first owned Gazebo readback exposed that the project RGB-D producer publishes
points already transformed into `astribot_torso_base`. Its frame origin is not the
camera origin. The upstream updater used that frame origin for rays and its
range filter assumed sensor-relative points. Each configured updater therefore
now requires the actual optical `sensor_frame`. Both transforms are queried at
the source stamp; the TF filter waits for both. Rays and range clipping use the
camera origin, while robot containment still uses the point cloud's frame.
Missing camera transforms do not fall back to the cloud frame origin. The test
with base-frame points and a translated camera failed before this correction;
same-input parity with upstream remains tested when cloud and sensor frames
coincide. This is a correction to the observed frame mismatch, and does change
the wrongly cleared cells in that previous base-frame integration.

## Source and validation

The updater source/header are derived from the local MoveIt 2.5.9 source under
`runs/task_chain_20260921/dependencies/moveit2-2.5.9/moveit_ros/perception/pointcloud_octomap_updater`
in the original workspace. BSD notices are retained. Source SHA-256 before the
change:

- `src/pointcloud_octomap_updater.cpp`: `92ff23404819d6bce3d73283e5af1a44e1872f3f37aaa0340e4ed3c5f272985d`
- `include/moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h`: `1840c99b7830f60beaf2ce16741a6c8cf2a9b33315ecfc8974c800d73d781c8d`

`observed_pointcloud_test` uses the actual plugin loader, ROS subscriptions,
OccupancyMapMonitor, TF and upstream point-cloud updater. It checks identical
map bytes for identical input, source stamps, concurrent sources, immutable
snapshots, model clearing, invalid/empty clouds and missing transforms.
`ASTRIBOT_OCTOMAP_BENCHMARK=1` additionally measures 30 alternating pairs with
640×360 synthetic points and 5 cm map resolution. Timing includes publishing
through the filtered-cloud callback, without full robot self-filter geometry.

The private build needs the existing `moveit_ros_perception` installation on its
prefix path. It does not rebuild or modify that shared dependency. Evidence and
exact binary hashes are in `docs/evidence/arm_dynamic_recovery_20260928/observed_octomap`.
