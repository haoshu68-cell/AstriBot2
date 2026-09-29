# W1 wrist session contract

Two optional C++ wrist session gates accept a bounded ACQUIRE/RENEW/RELEASE
service from the task owner. Tokens bind camera, owner, execution and process
instance. A maximum 2 s lease, fresh request stamp, monotonic renewal and
wall/ROS expiry bound work. Duplicate acquire does not extend expiry. Cancel,
lease loss, clock rollback and old tokens cannot activate a prior session.

The gate subscribes to raw wrist RGB/depth/Info only while requested and forwards
unchanged capture headers to private manipulation topics. It never publishes
base or joint commands or disables the navigation head/torso streams. Session
state is control metadata, not camera health. Health requires both complete fresh
sensor data and a fresh active session; release invalidates health immediately.

This controls ROS computation/subscriptions, not physical sensor power or Gazebo
rendering. Render rate/activation is a separate driver capability. Full transport
execution remains deferred; service-level owner/cancel behavior is tested with
an isolated caller before any later task stage integrates the interface.

## Capture generations and clock ordering

`CameraSessionState.activated_at` is the first ROS capture time allowed by this
lease generation, preserved on renewals. `header.stamp` is the status publication
time and must never substitute for an image timestamp. CameraHealth rejects
buffered triples older than activation. A reproduced unit case previously marked
such a triple healthy; the new test rejects it.

Image and /clock delivery are asynchronous. The gate now stores at most four
messages per stream, retaining immutable shared pointers. A capture slightly
in the future waits for the corresponding clock tick; it is never forwarded
while future-dated. The same unchanged 250 ms ROS/wall limit bounds waiting.
Wall expiry, release and rollback clear pending work. A 20 ms wall timer advances
waiting frames; status is published at approximately 60 ms. Stream counters
report received, forwarded, clock-deferred, rejected, overflow and pending data.
They are source-node counters, separate from observer receive timing.

Short session tests require a fresh response within 0.8 s after activation.
They do not assert that all samples remain healthy. An earlier version tested
one sample exactly at 0.8 s and failed when that sample reported RATE_LOW,
although earlier new-generation frames were healthy. The original failures
remain recorded; the duration-wide health analysis still fails on every invalid
sample after warmup and every observed health gap above 250 ms.

## Consumption contract and limits

- Acquire with camera/owner/execution/request identity, a recent request stamp,
  and a lease <=2 s; renew with the returned token before both clocks expire.
- Release on task cancellation and stage exit. If release delivery fails, local
  wall expiry bounds subscriptions and downstream health heartbeat expiry
  revokes availability. This is cooperative ROS ownership, not authentication.
- Read private clouds together with valid camera health, matching source and
  capture-time TF. A cloud alone is not observation or execution authority.
- Full transport stage wiring, atomic object/scene version association, sensor
  hardware power control and stereo depth remain pending. No task layer may
  assume these service tests prove them complete.
