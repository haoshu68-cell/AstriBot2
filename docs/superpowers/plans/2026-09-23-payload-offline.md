# Versioned payload state offline implementation plan

> Implement inline with executing-plans, C++ runtime and evidence-qualified tests.

**Goal:** Make confirmed empty/loaded state explicit and versioned, connect it to geometry, and audit the dependent arm hold/fixed-envelope contract without starting simulation.

**Spec:** The user's approved attachment-state design in this conversation and `docs/RGBD_CUDA_PROCESS_RECOVERY_20260922.md`, next-blocker section. This is continuation of that plan, not a new approval request.

## Global constraints and logical review

- Current worktree includes uncommitted implementations required by this change. Preserve them; snapshot only affected files and build in `runs/task_chain_20260923_payload_offline`. No reset, shared install overwrite, simulator, ROS runtime launch, hardware or motion commands.
- One coherent W1 offline continuation: start 2026-09-23 00:13:31 +08, pause at 01:43:31 +08 if unfinished. No internal step resets that budget.
- Explicitly confirmed empty is different from unavailable. A newly empty PlanningScene is not physical confirmation. The producer is pinned to environment/session/source; simulated truth is labelled simulation.
- Retain source capture time, receive steady time, clock/source epoch and monotonic semantic revision; periodic publication does not renew evidence. An empty-loaded-empty sequence must not reuse an execution version.
- The C++ ledger reconciles independently supplied attachment evidence against a full scene readback. It has no motion interface and never manufactures empty evidence. Existing task execution remains responsible for changing physical attachments and the scene.
- Persistent records describe the last observation, not permission after restart. Failure to persist prevents confirmation. Unknown/pending/conflict invalidates permission without clearing the last known load.
- Consumer must enforce ledger and source freshness, frame/shape validity and revision at async geometry completion, alongside existing filter/coverage checks.
- Review scope: duplicate/late evidence, stale full scene, node/source/clock restart, same content after load transition, partial/ambiguous/unsupported geometry, durable-write failure, and invalidation while geometry is in flight.

## Task 1: Attachment state through the geometry consumer (offline)

Files: new `astribot_payload_msgs`, new C++ `astribot_s1_payload_state`, geometry consumer/header/tests and dependency declarations.

- [x] Write failing C++ tests for confirmed empty versus missing, explicit physical source plus independent scene readback, revision/epoch, deadlines, malformed geometry and journal failures.
- [x] Implement typed full-state messages, bounded C++ reconciliation core, durable journal and ROS adapter; runtime adapter compiles but is not started in this offline task.
- [x] Integrate explicit state consumption into geometry; old direct-scene mode remains an explicit compatibility mode only, not default proof of physical state.
- [x] Verify C++ tests and compilation in independent install paths; include empty/left/right/dual load, offset shapes, scene mismatch, restart/late response and geometry invalidation boundaries. Record unsupported geometry as rejected, never silently omitted.
- [x] Audit relevant diff and preserve red/green evidence, source hashes and exact verification scope.

## Task 2: Dependent hold and envelope offline contract

Begin only after Task 1's offline contract passes. Re-read resource/hold ownership and reference existing `ros_backend.py`, `fixed_hold.py`, `FixedEnvelopeCore`. Separate missing runtime publisher from an actual rejection bug. Verify available pure C++ transaction tests and add regression for attachment change/empty-loaded-empty while waiting for ACK. Do not fabricate runtime hold or ACKs.

- [x] C++ task-owned ArmHold core with typed resource/action inputs, stable observation, controller claims and bounded request construction.
- [x] Fix and verify expiry gaps, duplicate sequence, initialization order, callback revocation, clock epoch and ACK gates.

## Task 3: Offline composition verification

- [x] Review logic before starting: direct composition of actual cores, with physical/resource/TF inputs explicitly identified as deterministic fixtures.
- [x] Verify confirmed empty, dual distinct load admission, version transitions, unknown mass, scene conflict and expiry through the combined chain.
- [x] Retain the same overall deadline. This verification does not instantiate a physical source, resource server or ROS executor.

## Deferred by current authorization

Live physical-source adapters, real scene service reconciliation, controller ownership, original live consumer ACK reproduction, actual SLAM/MPPI workload, TF/source restart and camera coverage matrices remain NOT RUN pending simulation permission. Offline contract tests do not advance these gates. VLA/hardware/full pick-carry-place remain deferred.

## Rulings

- User explicitly requested offline implementation now following the presented design; no repeated design approval. Use a source snapshot and independent build/install rather than a new clean checkout that loses this unfinished chain.
- Reconciliation adapter is read-only toward MoveIt: existing execution owns scene mutations. This avoids two scene writers and tests evidence matching rather than treating an apply response as physical success.
