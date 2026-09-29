# MoveIt 2.5.9 isolated simulation candidates

The two patches apply to the upstream 2.5.9 tree, not the system installation.
The source archive and extracted tree are preserved under
`runs/task_chain_20260921/dependencies/`. Record the archive hash before reuse.
Apply both patches with `patch -p1` from a fresh extracted source root.

Build `moveit_ros_perception` from its explicit package path to the isolated
`runs/task_chain_20260921/moveit_sensor_plugin/{build,install,log}` directories.
After sourcing that install, build `moveit_ros_planning` and
`moveit_ros_move_group` from their explicit package paths to the separate
`runs/task_chain_20260921/moveit_shutdown_fix/{build,install,log}` directories.
Use colcon with explicit --base-paths, --build-base, --install-base and --log-base;
do not scan the repository root or replace /opt/ros/humble.
The recorded session/query_env.sh specifies the tested overlay order.

`integration_manifest.json` records selected source, installed libraries and ldd
resolution. `ready_shutdown_fixed` preserves the failed TEM-only experiment;
`ready_shutdown_v2` has three clean planning-only lifecycles with both patches.
Later transport_bounded_client* evidence includes three independent full
planning-service lifecycles. No execution/cancel lifecycle or leak-free claim is
made; class_loader warnings still require a separate investigation.
