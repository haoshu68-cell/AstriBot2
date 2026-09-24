# First Transfer Goal: station observation after epoch binding

This is the coordinator-authorized correction of the actual scene06 first failure, discovered at 2026-09-24 18:41 Asia/Shanghai. The coordinator's existing mainline clock and historical pauses are retained; recording or freezing this correction does not restart them. This window owns only the two source/test changes below. The coordinator owns builds, installation, tests, actual scenes and Git commits.

## Confirmed first cause

The scene06 result reports `PARENT_REJECTED`, one Python parent submission, no accepted lease, `admission_uncertain: false` and completed cleanup. The first changed executor status is raw observer line 39024: `STATION_WORLD_OBSERVATION_STALE`, authority `IDLE`, empty lease. The preceding initial status at line 20 is `WAITING_FOR_TASK`. Original Goal and executor shutdown log, exact status-line excerpts, a derived result excerpt and source hashes are retained in `scene06_first_cause/`. The manifest identifies the full original result and observer stream; the derived excerpt is not a replacement for their raw evidence. Controller state topic records are not controller Goal submissions.

The previous source ordering was deterministic on the first Transfer Goal:

1. The world inbox starts at generation zero. Its `receive` method drops any callback with ticket zero.
2. `validate_transfer_goal` called `observe_stations`, which drained the still-unbound inbox and required a fresh snapshot.
3. `admit(..., true)` would call `full_ready`, the only place binding a nonzero inbox generation, only after that station check returned.

`payload_world_base` does not return early merely because `payload_frames_` is absent: it can update the snapshot without frames. The unavailable sample is caused by the zero-generation inbox boundary. This repair preserves that boundary rather than accepting unbound data or binding during initialization.

## Minimal correction and explicit semantic change

- Static Goal/schema/context/station-box validation and authoritative full EMPTY inventory remain before acceptance. Existing geometry, claims, exclusive ownership, service and resource admission remain in place.
- `full_ready` still creates frames from the confirmed physical robot identity and binds the inbox. It now also clears the cached world snapshot at that same boundary. Old-generation callbacks, queued samples and captures before the new binding cannot populate the new frame context; no old cached snapshot survives a new binding.
- After acceptance, `PICK_PREPARE` waits for a capture at or after operation start. If none arrives within 3 seconds on the steady clock, it fails with `STATION_WORLD_OBSERVATION_STALE`. This waiting bound follows the existing preparation timeout scale; the 300 ms source/steady evidence lifetime is unchanged.
- The first fresh snapshot binds both real station identities once for this parent. Invalid, missing, duplicate or moved stations fail immediately. Further checks during navigation-revocation waiting, the existing scene-query callback and PLACE preparation compare the original identities; they do not rebind changed entities.
- Station checks still precede scene submission. Scene application and independent readback must complete before planner submission and controller dispatch. The existing callback's `observe_stations(..., false)`, scene/EMPTY checks, motion safety checks, cancellation, cleanup and first-failure latch are unchanged.

The API outcome changes deliberately: a live station-observation failure now terminates an **accepted parent during preparation, before any child motion**, instead of rejecting the Goal before acceptance. Acceptance reserves the task; it does not establish physical station validity or authorize motion. The previous static and confirmed-EMPTY rejection behavior remains unchanged. This is a preparation-order correction, not a claim of identical Goal admission outcomes.

## Verification and freeze

Only `src/hold_executor.cpp` and the existing `test/payload_frames_test.cpp` changed. The existing `RebindingDropsQueuedAndInFlightPreviousEpoch` test now additionally asserts that cold unbound input is unavailable, an in-flight prebinding ticket cannot seed the first epoch, and a new-generation sample can become available. Its previous queued/old-generation/prebinding-capture rejection and post-rebind acceptance checks remain. This does not itself exercise the entire parent Action.

No new Transfer fixture, helper matrix, build, install or ROS process was run by this window. The coordinator will build the frozen candidate, run the existing payload-frame/inbox and fixed-station-scene tests, then assign the sole next actual scene. Actual acceptance, zero child dispatch before station validation, complete Transfer success and recovery are pending those results; performance and hardware acceptance are not measured here.

Frozen parent SHA256: `fce8a257698362b027950325b8bf7f3aae5244281c3e35658fb473f94a1bd9e3`.

Frozen payload-frame test SHA256: `5d8c7c88dd86b617db988d05a73742c7ef06d59cbf04e23b1b8b2b3aff4befcb`.

`before_manifest.json`, exact before/after copies, `changes.patch` and `manifest.json` preserve the correction. The helper and its previously passed six cases are unchanged and are not rerun as part of this fix. The handoff manifest records the untested-at-freeze state; later coordinator results should be appended with their own evidence and binary bindings.

## Coordinator build and unit results appended after freeze

The coordinator completed the new build and installation at `runs/mainline_20260924/transfer_station_binding_1841/native_install`. Both original GTest XML reports are timestamped 2026-09-24 18:50:12: payload frames/inbox passed 4/4 and fixed-station scene passed 4/4, with zero failures, errors or disabled tests. These are the coordinator's first runs of this candidate. The payload-frame log retains the deliberately exercised Humble timeout-overload and missing-frame diagnostics from the existing negative TF test; its expected assertions passed. Unit-test elapsed times are not runtime performance measurements.

Read-only verification matched all seven candidate binary/library hashes and all six recorded source bindings. The installed executor SHA256 is `3fd0bfa945b9b597dd3b3e90abb0cce9be6cba7ee22efeba28d0cc7b53b19763`. Its recorded dependency resolution uses the new prefix for the navigation helper, whose SHA256 remains `34a08ca131511f2f0e54d53014b59557b118ac21e6d6bf2c8f333f6652dc913d`, and `libarm_hold_core.so`. The six helper tests are the earlier verified result, not a new rerun.

Eight unmodified coordinator records (candidate binding, build/install logs, both test logs/XML reports and executor dependency listing) are copied in `verified_build_and_unit/`, with a manifest of their original paths and hashes. The source/test freeze hashes above are unchanged. This window did not build, install, run tests, launch ROS or make a Git commit.

The coordinator has dispatched M2's actual scene07. Real parent acceptance, station confirmation before motion and completion of the full Transfer remain pending actual-scene evidence. The eight unit passes do not establish complete simulation, long-duration stability, performance or hardware acceptance.
