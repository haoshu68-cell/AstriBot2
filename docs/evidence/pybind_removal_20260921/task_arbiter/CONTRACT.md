# Frozen navigation arbiter contract (2026-09-21)

Oracle: `ws_robot/src/astribot_s1_navigation_policy/test/reference/task_arbiter_node.py`,
byte-for-byte source SHA256 `c898f9d0cc459d3cf6b6b28d9199fbcb5c2c367a669e72fb0b11e98c302090a3`.
Scope: navigation goal arbitration only, with no new whole-robot task/resource bus.

## Wire API

| Interface | Semantics |
| --- | --- |
| `/navigate_to_pose`, `/navigate_through_poses` | Nav2 actions, source `operator`, priority 100 |
| `/route/navigate_to_pose`, `/route/navigate_through_poses` | source `route`, priority 50 |
| `/exploration/navigate_to_pose`, `/exploration/navigate_through_poses` | source `exploration`, priority 10 |
| `/navigation_executor/navigate_to_pose`, `/navigation_executor/navigate_through_poses` | Backend Nav2 action clients; goal fields and feedback/results forwarded unchanged |
| `/navigation/execution_status` | `NavigationExecutionStatus`, reliable transient-local KEEP_LAST 10 |
| ROS action internal send_goal/get_result/cancel_goal services; feedback/status topics | Standard Humble action protocol and default QoS; no additional application service |
| ROS node parameter services/events, rosout, optional `/clock` | Standard node infrastructure; no velocity, joint, SDK, or controller command publisher |

Node name is `navigation_task_arbiter`. Each accepted task is keyed by the frontend
UUID as 32 lowercase hexadecimal characters. Backend IDs are independently assigned
by the action client. No retry or UUID reuse is introduced. Status source is the
admission source; sequence begins from monotonic nanoseconds and increments once
per status emit. Status stamps use ROS time (including zero/paused/rollback sim
time); phase deadlines use monotonic/steady time and remain live without `/clock`.

## Admission and lifecycle

Single shared active owner across both action types, at most one pending task.
A reservation protects the interval between goal admission and accepted callback.
Reject when reserved, pending already exists, or incoming priority is lower than
active. Equal/higher priority replaces. On accept, mark old active preempted,
request its cancel if backend handle is known, then emit new ACCEPTED. The front
is executing even while waiting to acquire the backend owner.

Pending task waits up to `handover_timeout_s`; cancel before acquisition finishes
CANCELED/USER_CANCEL. Expiry finishes FAILED/PREVIOUS_TASK_NOT_TERMINAL and does
not clear the old active owner or its preempted flag. After acquisition, a separate
full timeout waits for backend availability. Unavailable expiry is
FAILED/EXECUTOR_UNAVAILABLE. If availability arrives after preemption/cancel, finish
PREEMPTED or CANCELED/CANCELED_BEFORE_DISPATCH without sending a backend goal.

After send, an unresolved goal response retains ownership. A late accepted goal
response still triggers required cancel before emitting EXECUTING. Rejection is
FAILED/EXECUTOR_REJECTED. Goal/feedback forwarding does not rewrite pose, frame,
stamp, behavior tree, duration, recovery count, or through-poses remaining count.
Feedback forwards while the frontend is active, including cancel/preemption waits.

Backend cancel is sent once and emits CANCELING with PREEMPTED or USER_CANCEL.
Cancel response/rejection is deliberately not consumed for ownership. A missing
cancel response, missing result, or pending handover expiry cannot authorize new
backend dispatch. Front cancel is accepted for a known task, but result remains
pending until backend terminal. No physical stop confirmation is claimed.

Backend terminal result: preempted tasks become frontend ABORTED and status
PREEMPTED/HIGHER_OR_EQUAL_PRIORITY_TASK; otherwise only backend SUCCEEDED becomes
frontend/status SUCCEEDED, all other codes produce frontend ABORTED/status FAILED,
reason EXECUTOR_RESULT. If frontend cancel was requested, frontend/status is
CANCELED regardless of those choices; reason and backend action_status are kept.
Backend action status passes through (4 success, 5 canceled, 6 aborted). Pre-dispatch
failures use action_status 0 and a default empty result. Only finish releases the
matching UUID; stale generation cannot release another owner's slot.

## Parameters and timing

`handover_timeout_s`: declared double, default 10, startup finite `(0,60]` validation. Humble rclpy rejects integer, boolean,
and string overrides at declaration before `float(value)` is reached; C++ keeps
that DOUBLE-only behavior (including nonzero startup failure).
Python reads once into `self.timeout`: runtime setter success updates the ROS
parameter store but does not change active or subsequent task deadlines. Native
preserves that existing consumption timing (it is not a dynamic timeout feature).
`use_sim_time` remains the standard ROS dynamic clock setting. No QoS/topic parameter
is declared; standard ROS remaps continue to work.

Python polls future completion/pending owner every 10 ms and occupies executor
threads, with no bound on goal-response/result waiting. Native uses callbacks and
10 ms steady polling only for pending owner/backend discovery deadlines; it keeps
unresolved ownership without blocking an executor thread. This is an implementation
change, not a new timeout/retry/recovery protocol. Startup invalid configuration
returns nonzero. Shutdown is bounded in native: no detached worker/future join;
no new stop/recovery guarantee is added. Python's shutdown-time status publication
can fail after its context is invalid, so shutdown event delivery is not promised.

## Validation boundary

Tests use real ROS2 actions/services/topics with controlled fake Nav2 on private
remapped endpoints and domains 150-154. They do not establish real Nav2 lifecycle,
Gazebo, physical stop, controller behavior, hardware acceptance, restart recovery,
or whole-robot resource ownership. Backend disconnect with unknown outcome retains
ownership indefinitely; no silent lease expiry is added.
