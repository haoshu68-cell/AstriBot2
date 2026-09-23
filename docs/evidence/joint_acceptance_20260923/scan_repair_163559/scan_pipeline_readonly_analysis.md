# Scan freshness / return-90 read-only diagnosis

Evidence status: existing integrated-simulation records plus bounded offline profiling; no ROS node started, no thresholds changed, no runtime source edits. Written 2026-09-23 before 16:43 +08.

## Confirmed from original return90_01

- Goal 2: 723 observation samples, 724 state samples; 510 required-coverage rejections are scan STALE / ACQUISITION_EXPIRED, including 492 near-stationary rejection samples. Independent source probe P95 receipt age 58 ms / sampled max69 ms; its TF buffer is not the policy buffer.
- Goal 2 recorded observation errors are all zero and last_error empty. This does NOT exclude internal can_transform false, because that path queues silently.
- 723 of 724 sampled path_risk_status values are CLEAR; the remaining one EXPIRED. All 724 corridor states are NORMAL. This trace does not demonstrate a 90-degree geometry/planner defect; frequent freshness HOLD explains lack of sustained permission.
- Policy requires clear_hold_s=0.6s after invalid input. behavior.py74–76 resets clear_at,89–91 requires the interval. Repeated fresh/stale alternation also yields127 CLEAR_CONFIRMATION states; success of one fresh sample cannot resume movement.

## Paired stage ages

See paired_freshness.json for input hash, exact pairing method, all selected rows and nearest-rank statistics. Exact integer pairing yields653 pairs,484 stale. Of these194 were already older than300ms before snapshot;290 crossed300ms during snapshot/risk before decision. The initial654/485/291 console estimate included one exactly5ms boundary sample because binary floating subtraction yielded4.999999999995ms; the durable evidence corrects this and preserves that explanation.

Observation stamp is after process_scans and before snapshot. Therefore this divides post-scan compute from earlier aging, but it cannot separate callback delay, mailbox wait, TF readiness or scan compute. The later age difference is ROS time; CPU/wall stage timings are separate. Percentiles must not be added.

## Source paths and findings

All source references below are relative to ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/ unless stated. Current fusion.py equals frozen I0_2 install source (hash in profile JSON); current observer differs only by parked R0 diagnostics.

- observer_node.py83–110: one mutually-exclusive processing group serializes tick, map and observation adapters. Scan/odom use another default group. Three executor threads at445. TF listener uses its own ReentrantCallbackGroup in installed Humble transform_listener.py. scan_lock at234–261 only swaps bounded5-frame mailbox, not the full computation. No confirmed long-held scan lock.
- observer_node.py239–265: age/TF precheck and most recently arrived ready frame selection; can_transform failure keeps pending without incrementing errors. R0 must measure the policy buffer.
- observer_node.py277–319: native projection followed by per-cell Python Observation construction, ingest, stale-track free-space clearing, health commit. Freshness check uses beginning time; subsequent calculations still consume original capture budget. Final decision rejects expired data correctly.
- fusion.py173 creates an association index over all tracks each ingest;272–284 scans all retained tracks for free-space evidence. Static occupied cells remain until observed free; do not cap/drop them just to lower latency.
- fusion.py293–324 loops all tracks every snapshot; region only controls predictions, not membership. Each result triggers ports.py53–68 validation and WorldSnapshot81–93 repeated validation.
- geometry_native.cpp70/82 releases GIL for existing scan kernels; line80 copies the whole map mask under GIL on each scan. Exact runtime share is unmeasured.

## Bounded synthetic profile

Frozen Profile + ConservativeFusion,3411 padded static occupied cells,10 snapshots,0.459s total wall under cProfile. No live stack or raw recorded scans; this is a hotspot experiment, not a performance acceptance result. See frozen_fusion_snapshot_profile.{json,txt}. TrackedObstacle.__post_init__ accumulated0.172s of0.458s total; WorldSnapshot.__post_init__0.028s. The Python snapshot body itself0.153s. Cached profile attributes and validation are visible in the call tree. Native snapshot should not simply be enabled: it still packs32-column rows and rebuilds Python geometry per track, and has historical slowdown evidence.

## Minimal next discriminating experiment

Use R0 only on unchanged behavior to record capture→policy callback→internal tracking/map TF checks→selection→finish→constraint publication; retain epochs and last successful sequence, and compare timing with exact scene/binary/environment identity. If callback and TF-ready are fast but selection old, repair mailbox scheduling. If TF-ready is delayed, isolate TF dispatch without substituting latest TF. If finish→constraint dominates, move validated snapshot derivation/bulk construction across a C++ boundary while keeping geometry/provenance/version/lease identical.

Output contracts for any snapshot seam: stable track ID/order and full membership; unchanged spatial geometry and capture provenance; snapshot stamp now and current version; spatial occupancy variance0; region only changes prediction_model relevance; unassociated and sensors preserved; reset/epoch semantics unchanged. Do not change freshness or clear-confirmation gates.
