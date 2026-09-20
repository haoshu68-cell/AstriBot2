# Whole-robot geometry

Conservative geometry and acquisition-time state for fixed non-home navigation.
The ROS adapter publishes geometry evidence; it does not command the robot.

The C++ extension `_geometry_native` implements convex hulls, filled-polygon/AABB
distance, batch scan free-space certification, capture-time scan projection and
occupied-cell generation, per-joint acquisition caches and source-time selection.
Python remains the ROS/model adapter during migration.
Build dependencies include `pybind11-dev` and `python3-dev`; NumPy is required by
the binding adapter. Build this package before navigation policy or transport,
and source the resulting install setup. A Python source directory alone is not
a complete runtime installation. Do not overwrite a library used by a live stack.

Single-configuration builds default to `RelWithDebInfo` when no build type is
specified. Explicit `Debug`/`Release` selections remain unchanged. This keeps
optimization enabled without changing floating-point rules (`-ffast-math` is
not enabled). Rebuild the consuming navigation policy as well as this library:
an older installed Python adapter can still execute the scalar path even when
the native extension is present. Verify the loaded module paths and run the
differential tests against the same overlay used for simulation.

`JointSnapshot.update()` retains the strict immediate-input API.
`receive()` buffers at most 64 future packets and applies them once the observed
clock catches up. `snapshot()` preserves each joint's acquisition timestamp and
the earliest original deadline. Missing/stale/skewed joints reject; rollback
clears both applied and pending data and increments the clock epoch. Malformed
messages and overflow reject; the ROS adapter invalidates in-flight geometry.

Scan clearing requires fresh coverage of every corner and bracketing ray.
Occluded, NaN, negative-infinite and unmeasured rays retain occupancy. Geometry
kernels do not decide source freshness or grant navigation authority. Polygon
distance checks include obstacles inside the polygon; hull construction removes
only non-left turns, without a tolerance that could discard outward vertices.
Scan projection retains the existing map occupancy threshold/neighborhood,
range boundaries and cell identity. It uses the original capture-time TF and
does not change the sensor's acquisition deadline.

Verification after sourcing the built overlay:

```bash
python3 -m pytest -q ws_robot/src/astribot_s1_robot_geometry/test
python3 tools/sim/benchmark_geometry_kernels.py --output /tmp/geometry-kernels.json
```

The NumPy/scalar reference functions are retained for differential validation.
Offline timing is not a guarantee of ROS end-to-end deadlines; see
`docs/NONHOME_CPP_REPAIR_20260919.md` for the simulation evidence and open gates.
