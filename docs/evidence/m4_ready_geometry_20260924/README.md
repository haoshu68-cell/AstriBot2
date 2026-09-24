# Historical READY reserved polygon — six offline corridor cases

Run:

```sh
python3 /home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/m4_ready_geometry_20260924/verify_ready_corridor_geometry.py --output /tmp/m4_ready_geometry_replay.json
```

Result: **6/6 PASS**. `result.json` records the exact input/source/kernel/script
hashes, full 60-vertex polygon, source identity/timestamps, route inputs and
outcomes. The original source `fixtures/result.json` matches the digest recorded
in `ready_arm_geometry.json`. No historical timestamp was refreshed.

The measured reserved footprint is 0.880742848 m wide, with lateral bounds
[-0.367902637, 0.512840211] m. Its ideal parallel-wall center offset is
-0.072468787 m. Existing clearance/payload/boundary/tracking margins are
0.08 + 0 + 0.025 + 0.05 = 0.155 m per side. Thus the ideal zero-heading
reserved-polygon width condition is 1.190742848 m. This is not a live recommended
passage width: map cells, allowed heading errors, occupancy and sensor evidence
are outside these pure geometric checks.

| Case | Constructed corridor/route | Actual geometric outcome |
|---|---|---|
| Wide straight | width 2.0 m, zero lateral offset | admitted |
| Base-only comparison | width 1.060371424 m | configured 0.62 m base rectangle admitted; measured whole-body reservation rejected |
| Correct asymmetric offset | width 1.230742848 m, offset -0.072468787 m | admitted |
| Opposite offset | same width, offset +0.072468787 m | rejected |
| Turn before full exit | length 3 m; 90-degree corner at x=3.160024673 m | native continuous-turn kernel rejected |
| Turn after full swept exit | same length; corner x=3.767847413 m | native kernel and route geometry admitted |

The nominal base rectangle is explicitly a configured comparison geometry, not
another measured whole-body observation. READY is the measured nonhome posture
from ready_scene01; there are no attached objects in that recorded geometry.
These six cases add actual 60-vertex/non-symmetric READY data coverage to the
previous synthetic-rectangle tests; the previous 21-test suite was not rerun.

`FixedCorridorPolicy.support_bounds`, `lateral_interval`, `target_offset` and
`route_fits` execute normally. `route_fits` uses the installed native
`corridor_turns_outside` implementation; its result is also recorded directly.
No evaluate/admission call, envelope acceptance, Hold or navigation permission
is fabricated. No ROS/Gazebo node or motion was started, no product or installed
file changed. The outcomes are historical pure geometry, not current Hold,
installed-footprint, actual world clearance or closed-loop M4 acceptance.
