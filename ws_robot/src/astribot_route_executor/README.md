# C++ loop route executor

`loop_route_executor` owns one route and sends one `NavigateToPose` goal at a time
through `/route/navigate_to_pose`. After every successful result and dwell it advances,
wrapping to the first waypoint indefinitely. Failure, rejection or preemption ends the route.
It never publishes velocity commands. Closing the RViz client does not stop the robot-side loop.

Typed services: `~/start` (`StartLoopRoute`) and `~/cancel` (`CancelLoopRoute`).
Start requests must include `expected_boot_id` from the current status. Requests
from a previous executor boot are rejected. Duplicate responses include `active`
and `state`, including terminal states. Rebuild external clients together with
the service package when upgrading this interface.
`~/status` is a transient-local JSON stream containing route ID, poses, index, cycle count
and execution state. Cancel acceptance is not a terminal result or proof of physical standstill.
A lost terminal result blocks new routes as `CANCEL_UNCONFIRMED`.

The navigation launch starts this node idle. Do not run a second instance in the same namespace.
For standalone deployment use `route_executor.launch.xml` with the target ROS domain and clock.

See [the design and validation notes](../../../docs/RVIZ_LOOP_ROUTE_DESIGN_20260919.md).
