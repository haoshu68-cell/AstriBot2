# scene15 physical payload state: diagnostic handoff

The coordinator assigned the scene15/domain76 first-cause review and subsequently authorized only a minimal diagnostic in `payload_client.cpp` and its existing test. The issue began at 23:30 on 2026-09-24; this handoff does not restart that timer. No ROS process, build, installation or test was run by this window. The coordinator owns those operations and Git archival.

## Confirmed boundary

The source result is `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene15_world76/full_transfer/result.json`. Its first `PAYLOAD_STATE_STALE` feedback is at steady 19284.9108786 / ROS 95.362, stage `ATTACH_CONFIRM`. The subsequent renewal response at steady 19284.911290887 is rejected with the existing stopped-state reason; the driver requests cancellation later at steady 19285.211384473 / ROS 95.599. `ResourceAuthority::renew` does not check payload data: `grant` rejects STOPPING and the service returns the already recorded reason. Renewal rejection is a consequence rather than the original payload failure.

The domain journal has 18 child submissions and 18 child terminal records, three confirmed stages, and zero physical submissions or payload commits. Its lines 66–68 record GRASP_CONFIRM confirmation, remaining-plan revalidation starting at index 3, then the payload-state failure. The physical command was not sent. Final parent status is 5, `success=false`, `resources_released=true`, reason `PAYLOAD_STATE_STALE`; the resource handoff returns the journal to phase 0. This is safe failure cleanup, not successful transfer.

`PAYLOAD_STATE_STALE` comes from the physical Ignition state client, not the ROS ledger consumer or `check_payload_raw`. In this stage the call is `advance_payload`'s FRAME observation. `scene15_first_cause_excerpt.json` retains the exact indexed feedback, renewal tail and journal records, and the full source file path and hash.

The independent `payload_physical_state.log` contains 75 samples from source 94.201 to 95.681 s with a maximum source gap of 20 ms, no source regression, command_id 0, attached false and empty error throughout that window. Its `ign topic` capture lacks receipt timestamps. It demonstrates an independently observed source sequence, not its timely delivery to the executor. The simultaneously fresh confirmed ROS ledger is also a separate path.

## What remains unknown

Three checks originally shared the same error:

1. Poll time precedes receipt, receipt precedes capture, source age reaches 300 ms, or poll steady time precedes receipt steady time.
2. Elapsed steady time exhausts the original remaining lifetime, or the deadline addition cannot be represented.
3. The nonrenewable capture deadline is reached.

The Ignition callback can update the inbox after `tick` samples its ROS/steady pair and before `observation` copies the inbox. That ordering makes a newer receipt fail the first check. It is a source-level possible interleaving, not a reproduced scene15 root cause. The callback also discards a state whose source stamp is ahead of its local ROS clock; a run of such discards could leave an old accepted state. Actual local reception gaps, steady expiry and clock reversal remain distinguishable alternatives. Existing scene15 records do not identify the failed predicate or the executor's accepted receipt.

## Diagnostic-only candidate

The original three conditions, evaluation order, 300 ms limit, capture/deadline updates and `PAYLOAD_STATE_STALE` exception text remain unchanged. On the first stale failure in a client's lifetime, one stderr line prefixed `PAYLOAD_STATE_STALE_DIAGNOSTIC` records JSON containing the condition, model, source stamp, poll ROS/steady pair, accepted receipt ROS/steady pair, prior/current capture and deadline at that check, comparison flags, and the last future-dropped stamp/receipt. Future-drop tracking never admits a sample or extends its lifetime. stderr is captured by the executor's existing process log; no logging dependency, ROS endpoint or public API was added.

The existing repeated-capture test now checks two identical stale failures, exactly one diagnostic, the expected receipt-lifetime condition and the timing values. It does not claim to reproduce the callback race. Once the actual failure branch is known, a deterministic offline ordering case should target that branch while retaining the existing same-capture and expiry rejection tests; state-machine changes require a separate coordinator decision.

Frozen source SHA256: `9547b3aeca86b468cb1f08e516ac9619b5a2e9e5f94f2eb5da02ed37ddb6f9f7`.

Frozen test SHA256: `9c5e60215ff467c2ffa962ec87c8d30b5b3650b1665c32671a604556b41f4d36`.

Before/after copies, `changes.patch`, `before_manifest.json` and `manifest.json` preserve this scope. Build targets are the existing `trajectory_executor` and `payload_client_test`; run the existing five client tests in their owned private Ignition partitions before the coordinator's same-scene observation. Compilation, test execution and diagnostic-candidate simulation are pending at handoff. No performance or stability improvement is claimed.

## Paused before validation

After handoff, the coordinator relayed the user's updated priority: retain intermittent-failure evidence for later focused diagnosis and advance the ordinary mainline without extending this investigation. The diagnostic source and test remain frozen at the hashes above; no build, test, installation or runtime switch occurred for this candidate. The coordinator reports scene16/domain75 continues with the previous executor SHA256 `84b8c2d127607a14e3936672e6597f4ea6daf395e7f609717c778695a0999b99` and original installed payload-client SHA256 `0ae0e9f70e702d8bbc9959f30965e4cfab3cf72a366236ff2d3889a06ff3e08d`. This window did not launch or validate scene16. This pause is neither a passed test nor a resolved diagnosis. Resume from the archived before/after files and existing five client tests only when the coordinator reassigns the issue; retain its 23:30 starting point and prior evidence.
