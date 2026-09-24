# M1 full executor integration checkpoint

2026-09-24, Asia/Shanghai. Checkpoint prepared at 12:25. Original integration start approximately 11:28; the 12:28 checkpoint is not reset by the reporting turn or by the shared simulation window.

## Scope and present status

Development candidate only. Source changes stay in `ws_robot/src/astribot_s1_transport_native`. The first-stage `next_install` remains frozen. The full candidate uses `runs/m1_transport_20260924/full_build` and `full_install`. No deployment, robot motion, or full PICK/PLACE acceptance is claimed.

The same C++ node and ResourceAuthority now expose `ManipulationToHold` for the six-stage MTC operation, alongside the existing bounded `PlanToHold` commissioning action. Both share admission, persistent journal, resource lease, controller ownership and cancellation. No Python runtime backend or additional motion gateway was added.

Full source includes sequential ARM / GRIPPER / ATTACH or DETACH stage dispatch; internal stationary hold confirmation; physical Gazebo command client; actual URDF fixed-chain and top-level robot-model world-frame binding; physical inventory pairing; scene application and full readback; independent payload ledger and geometry convergence; remaining-path payload revalidation and a second full scene readback before the next motion.

The full action requires an already legal READY posture, explicit simulation payload registration and the actual robot description. It does not prepare zero-start postures or change clearance limits. Attached scene frames use actual URDF known links. The canonical scene helper is owned and verified separately by the mainline coordinator.

## Safety boundaries added during integration

- During a bounded physical payload transaction, all 22 actual joint states must remain fresh and within the pre-transaction geometry bounds. A temporarily unconfirmed attachment ledger does not authorize another motion.
- An unresolved physical command, scene application, ledger reconciliation or revalidation prevents resource release. Cancellation can reconcile an already submitted physical command but cannot start a new motion.
- Motion history survives per-stage child tracker replacement, so cancellation between stages still needs measured settling.
- World pose inbox binding clears the prior queue, increments a callback generation and rejects captures older than the new binding. An in-flight callback from the previous generation cannot append after the reset.
- All original scene geometry frames are validated before the permitted target object is removed for unrelated-scene comparison.
- Detach checks retain the measured world placement target and require independently separated observations with the existing placement/stability bounds.

## Validation evidence so far

- First full configure and single-job build succeeded: `runs/m1_transport_20260924/full_configure_1208.log`, `full_build_1213.log`.
- First CTest run: **9/10 targets passed**. `payload_scene_test` failed twice with `MTC_OCTOMAP_FRAME_UNSUPPORTED`; its fixture omitted the required `astribot_torso_base` outer octomap frame. Original failure retained in `full_ctest_1213.log`. The fixture was corrected to carry the actual canonical outer frame; the complete rerun passed **10/10 targets** (`full_ctest_1222.log`).
- New `payload_frames_test`: **3/3** cases passed, including queued and in-flight previous-generation world poses; `payload_scene_test`: **4/4** passed, including validation before target stripping and an old ledger revision waiting instead of authorizing progress. Newer revisions/source/clock/ledger epochs are rejected.
- The first build had one explicit-time-constructor warning in new payload_scene code; corrected and rebuilt. Final execution-source build: `full_build_1221_clock.log`.
- Build/test work paused during M2's exclusive actual simulation window, then resumed only after the coordinator released it.
- Installed payload components: **4/4 targets passed** (9 command, 5 client, 3 frame/inbox, 4 scene cases), loading the new `full_install` libraries.
- The current installed executor passed **2/2 first-stage synthetic ROS protocols**: domain 218 normal (six JTC submissions, confirmed hold, cancel, released), and 219 pending planner cancellation (no JTC submission, planner terminal before release). Both journals end in `resource_handoff_committed`, phase 0, with zero `RESOURCE_CLOCK_RESET` entries.
- The coordinator identified an existing same-tick clock-snapshot defect. The full source now takes fresh ROS/steady times at stop cleanup and final status; ResourceAuthority's genuine rewind rejection is unchanged. Dedicated execution-guard RED/GREEN regression belongs to the coordinator in domains 220/221; these are not claimed by this checkpoint.
- Both owned protocol drivers exited normally; no owned build/test process remains active. Journals and locks are preserved.
- The `trajectory_scene_binding` static library is publicly installed/exported as `astribot_s1_transport_native::trajectory_scene_binding`, with install includes and MTC/MoveIt dependencies. The installed ELF resolves the new native libraries and the coordinator's canonical-scene library; see `full_checkpoint/installed_ldd.txt`.
- Exact source, installed ELF and evidence hashes: `full_checkpoint/manifest.json`. Installed `trajectory_executor` SHA256: `e41da3d6afabfb0f34f5e4e1bf218c4d27561306833b58cfa16e37eaf2b6fa71`.

## Remaining acceptance work

1. Implement and run the normal six-stage PICK and PLACE protocol fixture through the physical-command / independent-ledger / scene / revalidation barriers. Component tests alone do not prove this integration. Include one normal delayed-ledger cycle and unexpected new-revision rejection in that full protocol; only the component gate is verified so far.
2. Review the integrated cancellation and unresolved-transaction paths, then hand off to the coordinator for exact Git records and a separately scheduled actual simulation window.
3. Actual six-stage motion, performance comparison and long-duration stability remain **not tested**. M3 integration remains out of scope until the normal full path is accepted.

## Handoff boundary

This is a compilable, partially verified development checkpoint, not completion of the original six-stage task. The remaining full protocol and actual motion acceptance remain open. The coordinator owns exact Git staging/commits; no shared HEAD/index/reset operation was performed here. Do not replace the accepted first-stage artifact or enable this full candidate merely because the component and first-stage checks passed.

## Re-entry

Use the reviewed mainline payload_transition overlay for messages and explicitly choose `runs/mainline_20260924/canonical_scene/install/astribot_s1_transport_mtc/share/astribot_s1_transport_mtc/cmake` for the MTC CMake package. Prepend its `lib` after the full candidate build/install libraries. Do not use the same-named old MTC or native libraries from an earlier overlay. Do not remove or reuse persistent resource journals or domain locks. Do not rebuild the frozen `next_install` from this development source.
