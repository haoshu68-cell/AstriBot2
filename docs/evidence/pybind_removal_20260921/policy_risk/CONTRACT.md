# Pure navigation risk migration contract

Authority is the byte-identical nine-file oracle frozen from Git 694a99d8 under
`astribot_s1_navigation_policy_native/test/reference/policy_risk`.
This stage provides typed C++ cores only. It does not switch either policy ROS
entry, remove an active Python consumer or retire a binding.

- Preserve uncertainty-expanded current and previous boxes; prediction rows keep
  track input order, time order, explicit zero-offset duplicates, empty models,
  full covariance growth and all obstacle owners. Nonfinite or inverted derived
  prediction bounds are rejected.
- Preserve exact polygon geometry and the existing circumscribed-circle lower
  bound. Close pairs use the existing filled-polygon box-distance C++ kernel.
  Rectangle fast paths call existing navigation_math directly.
- Continuous twist sweep retains source begin/end intervals, frame origin,
  constant-twist formula, midpoint/bound subdivision and 11 depth iterations.
  A work bound does not grant clearance. A rejected sibling prevents further
  refinement for that owner after the complete frontier has updated results.
- Risk keeps current clearance separate from forecast risk. Measured speed can
  exceed a proposed cap; the cap cannot erase current motion. Current/path owners
  and stopping-sweep immediate owners remain independent. Unknown observations
  block; no observations alone do not synthesize an immediate collision. An
  absent route disables future route hits, but retains actual-motion protection.
- Reachable finite odometry may overflow derived forecast distance; preserve the
  original empty-route fallback or final tangent instead of adding rejection in
  the native finite-distance fast path. Movement classification retains the
  original compensated Euclidean norm at its strict 0.1 m threshold.
- Existing final route tangent, zero-length segments, endpoint behavior,
  stopping delay, prediction-horizon comparison, moving threshold and 1e-12
  clearance reserve are unchanged. Final predicted geometry reuses translation
  including its zero-velocity signed-zero behavior.

Typed inputs are finite RobotState, validated immutable contracts, finite
validated profile/envelope values, XY paths and explicit equal-length row arrays.
Python broadcasting belongs to the validation adapter; production C++ callers
supply typed rows. Arbitrary Python object types, ragged dynamic arrays and
integers beyond the native ROS-compatible int64 domain are not runtime ABI.

Evidence covers offline math, errors and state integration only. ROS queues,
profile/envelope adapter, observer/controller behavior, leases, cancellation,
full Nav2/Gazebo and hardware acceptance remain the next migration stages.
