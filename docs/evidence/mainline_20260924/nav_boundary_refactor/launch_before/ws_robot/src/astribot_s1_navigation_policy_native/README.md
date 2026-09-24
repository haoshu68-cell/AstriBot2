# Native navigation runtime

This package contains the direct command-frame, scan, envelope and final
protection executables already selected by the recorded migration launch files.

The new `policy_contracts`, `policy_fusion`, `policy_health`, `policy_sweep` and
`policy_risk` targets implement typed policy state and numerical computation.
They are internal static libraries for the forthcoming complete native observer
and controller. They do not create an additional ROS entry or replace the
remaining Python policy controller yet.

`ASTRIBOT_BUILD_PYBIND=OFF` builds the package and its native tests without Python
bindings. The existing optional binding is still required by unmigrated Python
consumers and must be deleted only together with their complete validated entry
migration. No new binding or Python fallback was added by these core targets.

Pure core tests use explicit frozen source under non-installed `test/reference`.
The registered tests compare persistent fusion, source health/calibration,
continuous polygon sweep and world risk against identical inputs. Probes use
JSON only at the test boundary; production math operates on typed C++ values.

See `docs/CPP_POLICY_MIGRATION_NEXT_20260921.md` at the repository root for the
remaining ROS adapters and P2/P3/P4/P5/H2 state machines, and
`docs/evidence/pybind_removal_20260921/policy_risk/` for scope and reproduction.
Offline core validation does not establish whole-stack or hardware acceptance.
