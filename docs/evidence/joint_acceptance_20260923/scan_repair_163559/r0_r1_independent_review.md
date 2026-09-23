# R0/R1 independent return review

Evidence level: existing complete navigation logs and frozen source comparison. This reviewer did not start ROS, run a simulator, or rerun either scenario. Machine-readable counts, event windows and input hashes are in `r0_r1_independent_review.json`.

## Same-window results

Only samples received between each return navigation start and terminal action event are included; cleanup is excluded. These denominators intentionally differ from the navigation owner's at-or-after-start timing reports.

| Metric | scan_r0_01 | scan_tf_01 |
|---|---:|---:|
| Return ROS window | 58.594–237.711 | 43.674–78.102 |
| Wall duration, s | 180.021, timeout/cancel | 34.671, success |
| Policy state samples | 727 | 135 |
| REQUIRED_COVERAGE_UNAVAILABLE | 469 | 0 |
| CONTINUE / HOLD | 88 / 639 | 113 / 22 |
| Scan capture→policy receive P95, ms | 74 | 70 |
| Policy receive→select P95, ms | 240.918 | 85.146 |
| Select→finish P95, ms | 72.562 | 81.663 |
| Capture→constraint publish P95 / max, ms | 462 / 683 | 276 / 322 |
| Publish age >300 ms | 473 | 2 |

R1 return arrival error: 0.661341 mm and 0.056693° in the recorded localization frame. The 36-sample, 0.7-second stop window reports zero drift and zero command. Cleanup completed and owned holds released. R1 demonstrates one successful return scenario; it is not universal navigation acceptance.

R0's 469 required-coverage rejections are scan STALE/ACQUISITION_EXPIRED; 455 occur near stationary. Every strict-window path-risk state was CLEAR, corridor NORMAL and envelope snapshots READY_FIXED. The robot did move: final sampled pose approximately x=.296559, y=.014351, yaw=2.621872. The terminal status is caller timeout/cancel, not a planner NO_PATH result. Scan expiry is a strong, time-correlated obstruction; the logs do not establish it as the sole possible cause. Six INPUT_UNAVAILABLE samples also exist, including float-boundary and envelope-readiness cases documented in `r0_return_causal_boundary.json`.

## Source and experiment controls

Native TF is the runtime variable. R1 loaded binding SHA256 `495852d20675cf01274a349da749307ca1be151aa0a3cb111907ff71742fae21`; every strict-return observation reports the C++ backend. Frozen observer AST comparison finds scan/process_scans/process_scan/static_mask/static_at/mapping unchanged. Only initialization/configure_tf/destruction/tick diagnostics changed. Policy, native math and controller/critic artifacts remained unchanged for this R0/R1 comparison. Occupancy selection/filtering was not relaxed; different TF scheduling can legitimately select different captures and therefore produce different instantaneous tracks.

The validation tool also changed: OwnHold feedback now binds its Action UUID. The exact R0 and R1 hashes are recorded in JSON. start_nav, finish_nav, measured_stop and positive scenario functions are AST-identical; acquisition/feedback/ownership bookkeeping changed. This is a runtime single-variable comparison with an explicitly disclosed validation ownership repair, not literally an identical complete source tree.

## Two stale-at-publication proposals remain in R1

These are matched using each state record's own successful scan trace and health deadline, not an unrelated nearest observation:

| Decision ROS s | Publication ROS s | Capture ROS s | Valid until ROS s | Age ms | Motion |
|---:|---:|---:|---:|---:|---|
| 49.056 | 49.105 | 48.800 | 49.100 | 305 | CONTINUE / CLEAR |
| 51.068 | 51.122 | 50.800 | 51.100 | 322 | CONTINUE / CLEAR |

Both lie inside the strict return window. Their decision-time scan health was VALID; start-maneuver work took 46.579 and 52.183 ms before publication. The old policy checked health before this work and gave the subsequently published message a fresh stamp and full lease. The old final protection then stamped and leased its forwarded constraint again. Consequently business PASS did not satisfy the publication deadline invariant.

The observed `/cmd_vel` stream contains nonzero angular commands after both publications, and tracking is in ALIGN_START. However Twist carries no proposal identity or capture stamp, and R1 did not record the independent protection state needed to join every command to source evidence. This establishes stale-evidence proposals, not that a specific physical command was unsafe or that independent protection lacked fresh scans.

## Follow-up boundary

The next candidate must recheck the same decision evidence at publication, preserve its absolute deadline through final forwarding, and audit both wire stages. It must not claim all policy world/odometry evidence is newly certified: this slice covers the policy's required scan evidence and final protection's independently used scan/odom evidence. Clock/watchdog, collision, envelope and cancellation checks remain separate acceptance items. R1 itself used neither publication repair and must not be relabeled after later code changes.
