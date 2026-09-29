# Cross-domain map relay migration contract

Oracle: the original `astribot_s1_perception/map_domain_relay.py`, frozen byte for
byte in the native package's `test/reference/astribot_s1_perception/`; original
Git identity and SHA256 are in `oracle.json`. No test reference is installed.

- A temporary node reads five statically typed startup parameters using the
  environment's ROS domain. The process destroys that reader/context, then
  creates two independent contexts using explicit local and remote domain IDs.
  Global node/namespace/topic remaps and scoped parameter files remain effective.
- Equal domains return 1. The existing launch continues to skip the relay for
  equal domains. Different domains do not grant any motion authority: only one
  OccupancyGrid subscription and publisher cross the boundary, remote to local.
- Both endpoints use RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1. Each received map
  is republished, including identical content, empty cells, mismatched dimensions
  and nonfinite metadata. Every field and payload byte stays unchanged. The
  fingerprint affects logging only, never forwarding or validation.
- Initial-map timeout uses system wall time (`time.time()` in Python), strict
  `elapsed > timeout`, after each remote `spin_once(0.2)`. A map dispatched in
  that iteration can be published before the same iteration reports timeout.
  No further timeout runs after the first successful initial wait. Zero,
  negative, NaN and infinite DOUBLE values keep inherited comparison semantics.
- Temporary-reader parameters are snapshots; the later local/remote nodes do
  not expose those declared parameters for live reconfiguration. Wrong ROS
  parameter types fail at declaration. Negative domains fail initialization;
  they must not become the C++ sentinel meaning environment-default domain.
- Ready-process SIGINT exits cleanly after executor cleanup; SIGTERM retains
  the original OS-default termination. Initial timeout/equal domains return 1;
  other startup errors return nonzero. No timeout or cancellation permits
  continuing with an accidentally selected default/robot domain.

Verification runs only isolated loopback DDS contexts (remote 162, local 163,
bootstrap 164). It does not test cross-machine networks, actual robot maps,
Nav2 lifecycle, SLAM quality, physical control, or hardware acceptance.

Inherited limitations are explicit: no map sanity/size/rate validation, no
rolling liveness timeout after first map, wall-clock adjustment sensitivity,
and no guarantee to deliver every map under overload with depth-1 QoS. These
are preserved policies, not safety properties newly established by this port.
