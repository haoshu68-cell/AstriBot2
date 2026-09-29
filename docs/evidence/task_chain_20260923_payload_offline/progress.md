# Offline W1 continuation — 2026-09-23 payload state

Plan: docs/superpowers/plans/2026-09-23-payload-offline.md

Task 1 logic reviewed before changes: authoritative full-state evidence and scene readback are independent inputs; confirmed empty requires both; clocks, versions, persistence and async consumer invalidation are gates. No simulation or ROS nodes will be started. Runtime C++ and validation evidence skills apply. Baseline snapshot is in runs/task_chain_20260923_payload_offline/baseline.

Pre-flight interface conflict: geometry currently treats successful scene read as attachment confirmation and hashes content only. New ledger must retain a monotonic identity and source deadline across content-preserving transitions. Consumer completion must revalidate that identity and deadline, and cannot renew source evidence through scene polling.

Independent reviewer (requesting-code-review skill) reproduced four defects with pure C++ probes: negative revision high-water missing, overflow recovery reused a capture, geometry completion failed to clamp a tightened lease, and expired out-of-order packets revoked fresh evidence. Added red regressions for all four before fixing. Additional red cases cover ledger startup identity, new-ledger UNKNOWN retirement, source-epoch consistency, future same-context ordering and callback-to-worker queue age. Whole-stack acceptance remains false.

Build diagnostics preserved separately: direct-CMake package has share/package/local_setup rather than root local_setup (first geometry configure invalid); ROS primitive dimensions use BoundedVector, requiring explicit iterator copy; geometryRequire takes const char*, requiring c_str. These compilation failures were corrected without changing acceptance thresholds.

Task 1: offline implementation and scoped verification complete. 46 payload GTests + 5 attachment-geometry GTests, 4 native geometry executables, 26 geometry differential pytest cases and 5 launch-parameter pytest cases passed. C++ ledger adapter and geometry node compiled; neither was started. Four independent-review defects fixed with red/green regression.

Task 2 logic review before implementation: separate task/resource authority, physical settling, controller claims, source/receipt leases, cancellation and envelope-consumer ACK. Add a C++ in-process authority core suitable for an owning task client; no daemon may synthesize owner/hold evidence from configuration or zero velocities. The actual resource owner/trajectory completion adapter remains a live integration boundary; do not declare the old missing publisher fixed. Preserve the same overall 01:43:31 deadline.

Task 2 offline contract passed: 25 hold GTests, 8 fixed-envelope flow GTests and existing native authority executable. Three independent review defects and additional clock/ACK/callback revocation cases now pass. Task-owned ROS adapter is not implemented by this library.

Task 3 logic review before verification: exercise the actual Ledger → Consumer → confirmedAttachments → ArmHold → FixedEnvelopeCore combination, while declaring joints/resource/action/physical inputs as deterministic fixtures. Check empty/loaded versions, dual distinct geometry, mismatch/expiry and late ACK; no fixture can count as physical inventory or ownership evidence. Same overall deadline remains in force.

Final offline verification complete at 2026-09-23T01:15:35.482969+08:00: 91 GTest cases, 5 native executables and 31 pytest cases passed. Seven full core-composition cases included. No ROS nodes/simulation/hardware/VLA started. Full W1 acceptance remains false; actual full physical inventory source and formal resource/executor runtime wiring remain implementation work, followed by live validation. Elapsed 62.1 minutes, within the unchanged 90-minute task budget. Source hashes, build context and scoped diff check recorded.
