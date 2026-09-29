# Native perception adapters

`map_odom_tf_node` owns the existing map→odom decomposition edge.
`map_domain_relay` forwards only OccupancyGrid from a remote ROS domain to a
different local domain, with reliable/transient-local depth 1 on both ends.
The independent native package has no Python/pybind runtime dependency.
Launch/configuration remain in `astribot_s1_perception`.

Build in an owned prefix with ROS 2 Humble. Do not overlay an old installation
and assume obsolete Python wrappers have disappeared; use a clean installation.
Test-only historical Python references are never installed.

Migration contracts, limitations, raw traces and reproduction commands live in
`docs/evidence/pybind_removal_20260921/map_odom/` and `map_relay/` at repository root.
Isolated TF/map tests do not establish full SLAM, navigation or hardware acceptance.
