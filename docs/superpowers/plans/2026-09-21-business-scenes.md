# Business scenes implementation plan

Goal: Implement the user-approved five business scene types and persistent equipment annotations in the existing RViz workstation.
Architecture: C++ catalog owns immutable revisions of devices and scene bindings. Map manager owns scene activation and map transactions; operator backend remains the authority gateway. ScenePage uses existing map picking and MarkerArray rendering.
Spec: User-approved design in this conversation: shared or independent maps, equipment save/load, revision checks, no automatic business motion.
Execution: Inline in the current dirty workspace to preserve the existing integrated plugin. Build the three affected packages in a separate /tmp build/install prefix; do not restart shared simulation.

- [x] Catalog tests first: persistence, idempotency, stale revisions, invalid poses, device retirement, shared maps and delayed scene activation.
- [x] C++ scene/device catalog, backward-compatible store migration, atomic map/scene commitment.
- [x] Gateway and map-manager authority, idle, context and recovery checks; bounded status snapshots.
- [x] C++ Qt ScenePage: five types, scene/instance, map selection, device list, pose/dock/wait marking, save/load/edit/retire, hide and focus.
- [x] Integrate exclusive point picking, device markers, scene-aware route/plan invalidation and recorder context.
- [x] Build and run catalog, gateway, manager and Qt tests in isolated test domains. Record limitations and operation guide.

Constraints: no direct velocity or arm commands; no inferred hardware capabilities; old maps/versions retained; stale localization blocks navigation; state after restart is observed, never automatically replayed.

Verification: 16 test executables / 64 GoogleTest cases passed. One new Workstation fixture initially omitted mandatory map floor/transaction fields; corrected the fixture and reran the two UI suites successfully. No shared runtime restart or hardware acceptance. See docs/evidence/business_scenes_20260921/verification.json.
