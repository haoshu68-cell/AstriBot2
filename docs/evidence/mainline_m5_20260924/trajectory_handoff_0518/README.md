# M5 trajectory handoff validator freeze

Date: 2026-09-25. Coordinator: `01a0c405-627f-7742-b500-e057726ed2e7`.

Production file: `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/verify_full_transfer.py`.

Frozen SHA256: `40e36820d4398e62d22c27f0426a9f3fea2db3f070e8cdf1955d259a0a393eb9`.

## Scope

Only the successful parent path gains a journal acceptance gate. The existing journal filter binds owner, lease ID, epoch, and capture start. Already-validated ATTACH/DETACH physical transactions identify the two operation contexts and their suffix adoptions. Each operation must contain indices 4 and 5: PICK LIFT/TRANSPORT_POSTURE and PLACE RETREAT/STOW. Returned and adopted digests must equal the digest recorded from the final active-controller goal. Point counts and ordered joint names must also match.

For each suffix stage, the accepted response must repeat the immutable send metadata. Its controller/UUID must bind a successful terminal event and the subsequent stage confirmation, with ordered events and increasing generations inside each operation. Old child_submission events, earlier stages, and hold-controller responses do not substitute for active trajectory evidence. PICK may report transport_replanned=false; PLACE may not report true. Reported journal indices address the filtered world_journal sequence, not raw file line numbers.

Original success, EMPTY, scene, geometry, stop, and cleanup code is byte-for-byte unchanged after removing the added block. Failed parents reach the existing exception path before this new gate, preserving their first cause. No new subscriptions, ROS processes, builds, GPU activity, or simulation were introduced by M5.

## Validation

- `check_offline.py`: 17/17 targeted cases passed, executing the actual production block. Two positive cases cover PICK replanned true and false; negative cases cover missing/duplicate adoption, wrong context or transaction, unequal fingerprints/counts/joint order, response generation, terminal UUID or failure, stage UUID, reversed ordering, missing PLACE suffix, and forbidden PLACE replanning.
- Original final-world geometry check: 7/7 passed. Only its output destination was redirected to `geometry_regression_results.json`; historical evidence files were not rewritten.
- Syntax parsed successfully. Removing the new block restores `verify_full_transfer.py.before` exactly.

Reproduce the new offline check from the repository root:

```sh
python3 docs/evidence/mainline_m5_20260924/trajectory_handoff_0518/check_offline.py
```

## Evidence boundary

The new journal fixture is entirely synthetic and uses synthetic digest strings. It validates matching, rejection, and ordering; it does not recompute trajectory hashes, execute controllers, prove physical tracking, or prove successful simulation. Existing geometry regression retains its documented mixture of real scene21 geometry and synthetic completed transactions. Neither result upgrades a failed historical run to success.

The native implementation records the hash after assigning final goal.trajectory, and copies its metadata into the goal response. The offline gate proves returned→adopted→sent identity under that native event contract. It does not independently prove unchanged trajectories relative to the original pre-revalidation plan; that invariant remains the native adoption contract's responsibility. Whole-scene acceptance, video review, and hardware acceptance remain outside this offline freeze. Root owns scene32/domain59 and subsequent runtime acceptance.

Files include before/after snapshots, minimal patch, fixture, targeted results, original geometry regression results, and hash manifest. No commit was made by M5.
