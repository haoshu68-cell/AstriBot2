# Frozen navigation planner comparison

`verify_fixed_hold_expiry.py` captures the observed global costmap, the installed
V2 polygon and version, the actual start pose, and the deployed planner result.
This executable compares the installed Smac 2D / Hybrid / Omni Lattice search
plugins against that snapshot. It never activates a controller or publishes a
geometry acknowledgement. Do not interpret a candidate path as execution approval.

After sourcing the selected workspace overlay:

```bash
cmake -S tools/sim/planner_benchmark -B /tmp/nonhome_planner_benchmark -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/nonhome_planner_benchmark -j 2
/tmp/nonhome_planner_benchmark/planner_benchmark \
  <planner_input.json> <results.json> \
  /opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json
```

The costmap is configured with the polygon before lifecycle configure;
`setRobotFootprint` alone leaves the default radius-mode flag unchanged. The
frozen map already contains its inflation costs. No live costmap update layer
is activated in the benchmark. Unknown and lethal cells remain collisions.
Every candidate receives the same conservative filled-polygon sweep check.

`accepted_for_comparison` means that this geometric check, endpoint compatibility
and basic corridor check passed. Reverse direction semantics, controller yaw,
strict task route witnesses, fresh geometry leases, and dynamic stopping still
need runtime validation. Planning time excludes configure, uniform checking and
execution. Preserve each snapshot's geometry hash with its results.
