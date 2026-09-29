# Native navigation task arbiter

`task_arbiter_cpp` is the navigation-only owner for the six public Nav2 actions.
Operator priority 100, route 50, exploration 10; equal priority replaces. There
is one active task and at most one pending task, across both action types.
Replacement requests cancellation once and waits for the old backend terminal
result. A cancel ACK, rejection, or pending handover timeout never releases an
unresolved backend owner. No velocity, joint, SDK, or whole-robot ownership is
added.

Frontends: `/{navigate_to_pose,navigate_through_poses}`, `/route/*`,
`/exploration/*`. Backends: `/navigation_executor/*`. Goals, feedback and results
are forwarded unchanged. Status uses `/navigation/execution_status`, reliable
transient-local depth 10; task ID is the frontend UUID, sequence is increasing,
and stamps use the ROS clock. Ownership/server deadlines use steady time.

`handover_timeout_s` defaults to 10 seconds, valid startup range `(0, 60]`.
Its value is sampled only at startup, matching the frozen Python implementation.
A later successful parameter set changes the parameter store but not the timeout.
`use_sim_time` retains ROS node behavior. Backend goal-response and result waits
have no release deadline; missing responses retain ownership, using asynchronous
callbacks instead of blocking executor threads. Restart recovery is not provided.

`navigation_geometry_mode=fixed_v2` checks the current `NavigationEnvelopeV2`
before all six public actions are accepted and while they are queued or running.
It uses the shared `EnvelopeEvidence` timing selection, binds the admitted
session/version/geometry/hold/payload identity, and reads `robot_base_frame`
(default `astribot_torso_base`). Revocation, expiry, or an identity change cancels
only this task's backend goal. A late backend acceptance is also canceled;
ownership remains held until the backend terminal result. A recovered heartbeat
does not resume the old goal. Failure reasons are emitted as envelope failures,
not preemption. No envelope ACK, footprint calculation, velocity writer, or arm
resource release is added. `legacy` preserves the existing forwarding behavior.

The frozen migration oracle lives in the policy package's `test/reference` and
is never installed as a runtime node. `test/test_arbiter_protocol.py` runs identical
fake Nav2 inputs against Python and C++. `test/ownership_test.cpp` tests pure
admission/ownership transitions. These tests do not establish physical stopping,
real Nav2, Gazebo, or hardware acceptance. Detailed contract and raw evidence are
in `docs/evidence/pybind_removal_20260921/task_arbiter/`.
