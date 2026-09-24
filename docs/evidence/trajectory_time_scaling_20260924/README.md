# MTC uniform time-scaling candidate

Offline build and tests passed. No ROS/Gazebo process was started for this work, and this is not closed-loop acceptance.

- MTC prefix: `install/astribot_s1_transport_mtc` under this directory.
- Explicit scene candidate: `trajectory_time_scaling:=2.5`, or `trajectory_time_scaling_2p5.yaml` passed as a ROS parameter file to `transport_mtc_planner`.
- Default is 1. The parameter is finite and at least 1, and changes require restarting the planner. Existing native timeouts and guard/controller thresholds are unchanged.
- First planning: existing IPTP, then q unchanged, dt multiplied by s, velocity divided by s, acceleration divided by s squared, then existing geometry validation and serialization/cache binding.
- Both revalidation services use the const geometry checker on the bound path. They neither rerun IPTP nor apply scaling again. Payload revalidation retains its two historical scaling arguments for compatibility, but does not use them to retime.

`install/astribot_s1_manipulation` is a copy of the verified `gripper_planning_only` installed package with symlinks dereferenced; only the inline `external_trajectory_validator.hpp` was replaced. It is not a rebuilt manipulation package. All 47 other files match their baseline content. The compiler dependency file and compile_commands record the new header actually used by the candidate MTC executable.

CTest: 6/6 targets passed. GTest suites: time scaling 6/6, canonical scene 5/5, payload transition 16/16. The time-scaling suite links installed JTC 2.53.3 and samples its quintic interpolator at corresponding normalized times. Repeated payload revalidation preserves complete owner/cache trajectory messages, including t/v/a. The copied test XML in `sealed_test_results` was sealed after the root task's independent CTest rerun at approximately 12:40.

`result.json` binds source/binary/header hashes and dependency provenance. `change.patch` contains only this task's changes relative to the saved `before` files, excluding earlier canonical-scene work. Build logs retain existing initializer and serial-LTO warnings.

The original scene did not record the guard's raw reason; position-response lag remains a candidate cause, not a uniquely proven cause. The existing discrete linear geometry checks do not prove the JTC spline's swept collision volume. Real completion, all child terminal results, measured Hold and consumer ACKs still require a new owned run. The original issue clock remains 12:15–13:15 and was not restarted by this task.
