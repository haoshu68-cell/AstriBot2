# Actual payload collision checkpoint

Normal single-box scene27 (domain64) passed PREGRASP, GRASP_APPROACH and GRASP_CONFIRM, submitted/applied the physical attachment, and rejected the unchanged remaining TRANSPORT_POSTURE path. Exact first/bad/last states are in collision_snapshot.log; original full logs and hashes are in summary.json.

Independent offline FK using this scene's actual URDF and registered station in the same planning frame found that the arm starts descending and tilting before leaving the station. First conservative vertical clearance is +24.264 mm; first bad index41 of788 is −0.257 mm, while nominal physical geometry still has +6.590 mm clearance. This is not evidence of using the wrong stage start. The 788 states are the validated samples, not a continuous-volume proof.

The first inward 120 mm scene-only exit candidate had +27.003 mm endpoint side clearance but scene28 rejected its Cartesian LIFT for arm/head self-collision before sending controller goals. Parent resources were released, and owned simulation processes were cleaned. Endpoint clearance is not full-path acceptance.

The next candidate changes only the ordinary fixture exit to horizontal robot-forward 120 mm with the same 30 mm lift. Endpoint clearance is +34.338 mm; full MTC and actual attached-body path validation remain mandatory. This is not an implemented general conservative-payload predictor or replan protocol.

Scene27 business state remains UNRESOLVED/quarantined despite measured chassis stop and successful outer process cleanup; old journals are retained. Scene25 TF preparation failure and scene26 empty publisher observation are deferred at the user's request, with no gate relaxation. The direct collision investigation retains its original 00:26:36 start and 01:26:36 cutoff on 2026-09-25 Asia/Shanghai.

## Final checkpoint

The final vertical60 candidate passed initial planning but scene31 still rejected actual TRANSPORT_POSTURE at119/840. Independent full-corner/15-axis SAT analysis in scene31_clearance.json confirms a real conservative-box/station overlap after the path descends and tilts; a higher lift endpoint alone is insufficient. Nominal geometry can have a corner below the table top while still not overlap horizontally, so minimum Z alone must not be used as a collision test.

Scene31 confirmed PREGRASP, GRASP_APPROACH and GRASP_CONFIRM plus physical attachment; LIFT, loaded navigation and PLACE remain unexecuted. The video has545 decoded/index/metadata frames and is a failed-run recording. Business state is still quarantined/UNRESOLVED; measured chassis stop and outer process cleanup are separate. Final details and absolute paths are in scene31_final_checkpoint.json. All further candidate search stopped at the original one-hour checkpoint; no safety thresholds or geometry were reduced.
