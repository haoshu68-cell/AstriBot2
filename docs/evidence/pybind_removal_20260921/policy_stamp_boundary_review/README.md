# Remaining Stamp representation and failure-stage gap

This is an offline audit, not a fix or a ROS executor acceptance result. results.json compares the frozen Python authority with the Stage 6B string candidate's health/adapter probes and the installed Humble `rclpy.time.Time`. No ROS node or robot was started. Candidate binary hashes are recorded in the result.

The difference remains reachable through configuration: finite positive `sensor_timeout_s=1e10` and `1e100` are accepted by the original Profile and SensorHealthRegistry. Python computes an arbitrary-precision expiry and fails when constructing the ROS time output. The current native registry rejects them in its constructor because it stores nanoseconds in int64. For `1e300`, multiplying seconds by 1e9 already becomes infinity, so the original registry also fails at construction. Those two failure stages must not be conflated.

Image-only JSON observations also expose the gap without TF: capture or expiry beyond int64 is accepted by the original pure Stamp model but rejected by the native candidate. The reference adapter fixture uses fake clock/TF ports; these image cases avoid TF. Actual metric observations have a separate installed rclpy Time boundary, which must stay at its original point. Negative and huge input values must not be normalized, capped, or rejected sooner merely to fit the native type.

## Next implementation boundary

1. Add regression inputs for exact capture, expiry, epoch and subtraction, including int64 limits and large finite timeout construction. Preserve Python float multiplication and integer-conversion overflow ordering.
2. Widen the pure Stamp domain and dependent fusion/health keys, histories and expiration calculations, using the existing arbitrary Integer type. Keep wire Version counters and actual ROS clock/message fields separately bounded.
3. Preserve Python integer-to-float conversion and rounding at prediction/age boundaries. Review the existing int64 geometry snapshot interface before passing widened values; changing units or saturating time would change core logic.
4. At TF and health serialization, retain the original negative/time-range checks and callback mutations before failure. Add paired installed ROS tests to verify constructor versus callback/output failure and absence of fabricated normal results.
5. Repeat affected differential suites, sanitizers, installed observer regression and paired performance. Only then close this gate. Full P2/P3/P4/P5/H2 controller and binding/entry retirement remain subsequent work.

Reproduce with ROS Humble sourced and `/usr/bin/python3 reproduce.py --health-probe <owned policy_health_probe> --adapter-probe <owned policy_observation_adapters_probe>`. These probes can be built by the string checkpoint's isolated reproduce.sh. The script does not initialize rclpy or connect to a running graph. File/ELF hashes in source_manifest.json associate this audit with its actual inputs.
