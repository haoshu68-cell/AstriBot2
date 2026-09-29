# Sensor synchronization evidence

C++ timing metadata monitor and vendor-neutral trigger backend **interface**.
No physical timing-board backend or camera driver metadata adapter is included yet.
It neither changes image timestamps nor authorizes robot motion.

See [system design](../../../docs/SENSOR_HARD_SYNC_DESIGN_20260922.md) for hardware topology,
stream capability checks, staged deployment and acceptance boundaries.

Build the package in an isolated overlay, source that overlay, then use:

```bash
ros2 launch astribot_sensor_sync sync_monitor.launch.py reference_clock_epoch:=ACTUAL_CLOCK_EPOCH
```

The launch defaults to rejecting simulated evidence. It does not start pulses.
Keep the epoch tied to the actual session/time-mapping authority; changing it or
restarting a source requires a controlled monitor restart and fresh evidence.
Default `UNCONFIGURED` is deliberately not a production time domain.

Interfaces are relative topics, allowing namespace isolation:

- `sensor_sync/trigger_edges`: actual board edge readback (`TriggerEdge`).
- `sensor_sync/frame_timing`: device sidecar (`SensorTiming`), stream identities
  distinguish color/depth and left/right imagers.
- `sensor_sync/status`: per-sample result, clock-only versus trigger match, plus
  source watchdog failures (`SyncStatus`).

The capture stamp is acquisition time in the declared common clock domain, not
ROS callback receipt time. `hardware_associated` requires device counter metadata,
not nearest-neighbor timestamp pairing. `clock_uncertainty_ns` is a measured bound,
not a configured desired accuracy. Invalid data must not set `clock_locked`.
An exposure-start versus exposure-midpoint convention must be fixed by the driver;
retain raw device stamps and conversion provenance in its logs. Changing that
mapping or convention changes the clock/source epoch.

`valid` verifies this metadata contract, not physical wiring, image geometry,
calibration, object pose, or safety. It must be joined to the exact original frame
before being used by a future fusion/health adapter. No such adapter is enabled by
default in this stage.

Validation:

```bash
ctest --test-dir YOUR_BUILD/astribot_sensor_sync --output-on-failure
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 \
  ws_robot/src/astribot_sensor_sync/test/verify_sync_ros.py --output NEW_EVIDENCE_DIR
```

Select an unused ROS domain before running the synthetic test. It creates only its
own monitor and publishers and stops only its own process group. Test evidence is
explicitly synthetic, not hardware or Gazebo exposure verification.
