# Five geometry consumers — source handoff

Task start: 2026-09-24 approximately14:30, following the user's architecture correction. This subtask changes the geometry ACK participants to `global_costmap`, `local_costmap`, `planner`, `controller`, `policy`. The final command protection component is no longer a geometry-commitment participant. Placement of motion constraints and retirement of the former command writer are separate coordinator-owned changes.

## Changes

- FixedEnvelopeCore and FixedStationNavigation now require the five consumers. Unknown/legacy protection ACKs are ignored by their existing consumer filters.
- The core test explicitly verifies that all five suffice, four plus protection cannot replace a missing controller, and a protection negative ACK does not revoke the five-consumer authorization. Existing source/Hold/expiry/negative-controller-or-planner checks remain.
- The existing three navigation-helper tests use five normal ACKs; the missing-controller case additionally emits the legacy protection ACK and still requires no navigation submission. These ROS tests have not been run under the new contract by this task.
- Current fixed-navigation, fixed-Hold, payload-mass and ACK-rate verification scripts use the same five-consumer set. Set-equality checks filter unknown consumers so an extra legacy ACK does not create a false mismatch. The mass verifier's existing subset check already has that behavior.
- No executor-local ACK list exists. Its unfinished combined task continues to depend on the helper and remains at the14:30 uncompiled checkpoint; no new combined fixture, protocol or simulation was started.

The payload ledger, geometry completeness/revisions, Hold, actual source lifetimes, envelope limits, independent scene readback and required five-consumer ACK matching remain in place. No source/time/geometry threshold was relaxed by this subtask.

## Evidence and validation boundary

`before/` and `before_manifest.json` preserve all11 inspected files before modification; ten changed. `contract.patch` and `manifest.json` record the exact scope and hashes. Historical six-ACK snapshots and raw experimental evidence were not rewritten. The older Python-oracle migration A/B harness is a historical six-ACK comparison and is not evidence for this new five-ACK contract.

Python syntax checks passed for the five touched scripts/tests. `python3 -m unittest discover -s tools/sim -p test_fixed_mass.py` passed16/16 offline tests; see `mass_probe_unit.log`. This task did not build C++, launch ROS or run the three helper protocols. The coordinator owns the unified C++ build and will attach its actual results separately. Source handoff is not a claim that the entire architecture migration or combined robot task has passed.
