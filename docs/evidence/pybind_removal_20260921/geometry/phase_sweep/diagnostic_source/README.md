# Whole-robot geometry

Conservative geometry and acquisition-time state for fixed non-home navigation.
The ROS adapter publishes geometry evidence; it does not command the robot.

`geometry_state` is a direct C++ ROS producer. It reads URDF collision geometry,
OBJ/ASCII STL/binary STL bounds, measured joint state (including mimic chains),
attached primitives and projection configuration, and publishes the same geometry
and attachment evidence topics. It preserves bounded asynchronous computation,
input identity checks, acquisition deadlines, attachment filtering acknowledgements
and height coverage checks. It does not load Python or issue robot commands.
This entry point changes only after a fresh build/install of this package. The
existing shared workspace installation is not replaced by editing these sources;
the migration validation uses separate installations under `/tmp`.

`geometry_core` is an exported C++ interface target. `geometry_kernels.hpp` exposes
convex hulls, filled-polygon/AABB distance, scan free-space certification and
capture-time occupied-cell projection without Python types. `robot_model.hpp`
provides model geometry; `joint_snapshot.hpp`, `source_time.hpp` and
`geometry_state_core.hpp` provide acquisition and completed-work checks.

The temporary `_geometry_native` compatibility adapter calls these same C++
kernels for consumers that have not migrated yet. It is still required by the
Python observer, fixed-envelope and transport paths; replacing this producer
alone does not remove those dependencies. Python model/polygon code remains as
the differential analysis oracle. Configure
`-DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF` for a C++-only installation once the
selected application does not require those legacy consumers.
Core build dependencies include Eigen, TinyXML2, OpenSSL and nlohmann JSON.
Compatibility builds additionally need `pybind11-dev` and `python3-dev`; NumPy is
required by the binding adapter. Build this package before navigation policy or transport,
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
ROS_DOMAIN_ID=115 ROS_LOCALHOST_ONLY=1 GEOMETRY_STATE_CPP=/absolute/install/astribot_s1_robot_geometry/lib/astribot_s1_robot_geometry/geometry_state python3 -m pytest -q ws_robot/src/astribot_s1_robot_geometry/test
python3 tools/sim/benchmark_geometry_kernels.py --output /tmp/geometry-kernels.json
```

The NumPy/scalar reference functions are retained for differential validation.
The protocol test starts only its own C++ producer and read-only fixture services
in domain 115, verifies the child's actual domain and absence of Python mappings,
checks attachment revision compatibility and original source deadlines, then
stops only its own process. This is isolated ROS protocol evidence, not a Gazebo,
closed-loop navigation or hardware acceptance result.
Offline timing is not a guarantee of ROS end-to-end deadlines; see
`docs/NONHOME_CPP_REPAIR_20260919.md` for the simulation evidence and open gates.
