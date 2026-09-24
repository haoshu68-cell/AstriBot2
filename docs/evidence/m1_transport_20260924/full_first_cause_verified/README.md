# Full six-stage candidate: required protocol regressions passed

2026-09-24, resumed approximately14:05 after the13:44 checkpoint. Earlier starts, pauses, failures and journals remain unchanged. This is a synthetic ROS protocol acceptance record for the standalone six-stage C++ action, not the later combined PICK→NAV→PLACE task or actual Gazebo/hardware acceptance.

## Minimal repair

The first stopping cause is now latched separately from mutable cleanup diagnostics. The Action result, feedback, status primary reason and final quarantine reason preserve it. The status `cleanup_reason` separately exposes later failures such as GEOMETRY_UNCONFIRMED. The latch resets only when starting a new admitted task. Resource safety checks, timestamps, expiry, settling, raw-observation guards and release conditions were not relaxed.

The original domain26 driver exit1 is preserved at the13:44 checkpoint. The new fixture retains that final-result assertion and adds checks that Action/journal first causes agree and that the cleanup diagnosis remains observable.

## Five required cases

| Domain | Scenario | Result |
|---|---|---|
|31|Late unpaired revision during final scene readback|PASS;18 prior JTC,0 later,0 payload commit, no Hold; RAW_REVISION_CHANGED retained; phase5 quarantine, resources not released|
|27|Same revision becomes UNKNOWN/incomplete inventory without new diagnostic|PASS;18 prior JTC,0 later,0 payload commit, no Hold; RAW_UNCONFIRMED retained; phase5 quarantine, resources not released|
|28|Normal PICK|PASS;30 JTC across5 motion stages,1 physical attach and payload commit; final Hold, cancel, released|
|29|Normal PLACE|PASS;30 JTC across5 motion stages,1 physical detach and payload commit; final Hold, cancel, released|
|30|PICK with legal150ms ledger-input delay|PASS; old capture/300ms lifetime preserved, real C++ ledger convergence followed by remaining-path protocol and full readback, normal Hold/cancel/release|

All five drivers exited0; all15 owned child processes exited0 and none of their recorded PID/start-ticks identities remained live at final audit. Per-stage child UUID/terminal and independent source/steady500ms settling assertions passed. Journals and locks remain preserved, including the quarantined cases. The previously failed232 admission and retired233+ domain allocation are not counted as passed tests.

Controllers, MTC service results and physical Ignition endpoint are synthetic. The executor and independent payload ledger are actual C++ processes. These cases do not validate real MTC collision planning, Gazebo grasp physics, perception, actual full motion, performance or long-duration stability. The normal standalone Action intentionally retains Hold until canceled; these tests do not claim normal completion of the future combined task.

## Frozen identity and provenance

- trajectory_executor SHA256: `ea92495653cd0c292e6b54623c125f72c9468aaa2b3fd750d219e381f201edac`.
- hold_executor SHA256: `c5e119d795f96aa020a28130ee971a5cc7ab4cb4119f81dec529b57541702467`.
- libpayload_scene SHA256: `40d1d72919231d53fd72ea16c7b2a2b738bf871e28971a508b354a036b8592f4`.
- Frozen driver SHA256: `f89653311375f7411a7db4c6afdbd7654b128504bb9312af9137f246c30e09ac`.

`verified_native_source.tar.gz` reconstructs the13:44 archived41-file package, replacing only `src/hold_executor.cpp` and `test/verify_full_manipulation.py`. Each archive member was hash-verified. It deliberately excludes the coordinator's subsequent ResourceAuthority/helper/combined-Action edits, which were not inputs to this candidate. See `source_manifest.json` and `first_cause.patch`.

The installed tree is frozen separately at `/home/yjh/WorkSpace/astribot_sdk_ros2/runs/m1_transport_20260924/full_six_stage/first_cause_repair/verified_native_install.tar.gz`, SHA256 `50c447c3ab8d50eab32d4dfa394312172334f4e57c3036a7ac08ffe2278fd917`. Full source, binary, linked-library, five-case, copied-evidence and process identities are in `manifest.json`. The executor's inspected dynamic dependency list resolves all libraries and contains no Python/pybind library; this is an ELF-scope claim, not all-project binding removal.

Build used the existing isolated full_build configuration and reviewed payload-transition overlay, with `cmake --build runs/m1_transport_20260924/full_build --target trajectory_executor hold_executor -j1`, then `cmake --install runs/m1_transport_20260924/full_build`. Both targets completed; build/install logs are archived. No broader rebuild or additional regression matrix was run.

## Reproduction and next boundary

Restore the source/install snapshots into isolated paths and use the recorded overlay/library identities. The existing protocol wrapper is `runs/m1_transport_20260924/full_six_stage/run_protocol.bash`; select a fresh approved domain and unique partition rather than reusing these journals/locks. Preserve original failure evidence and exact runtime identity. The coordinator owns Git staging/commits and real simulation scheduling.

The five-case gate is complete. Further testing is stopped here. The next separately authorized task is the combined parent Action and same-lease continuation contract; it must use a new build/install candidate and cannot inherit this five-case acceptance without its own integration validation.
