# scene33: PREGRASP geometry rejection, suffix not exercised

Closed run: `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene33_world58` (domain 58).

This is an offline review of saved journal, result, video metadata, and runtime identity files. M5 did not start ROS, GPU, builds, observers, or video decoding. The owner already decoded the original video. The frozen verifier was not changed.

Task `fixed_transfer_6b44552c9c9f4b79b578d46a8859c7f3`, parent context `8422b9709e7a4a95bd0f898c904c4420`, goal UUID `d5cf11a757ed4ec7935ab472561b9dfe`, lease `4a0943f8-fcb6-4b45-a83b-b99cb8f32188_1`.

## Journal findings

Indices below are zero-based positions in the original `full_transfer/result.json` arrays.

| Evidence | Result |
|---|---|
| `executor_journal[6]` | Initial MTC planner succeeded. |
| `executor_journal[8]` | PREGRASP stage 0, generation 1, active arm_left_controller sent a 400-point, 7-joint trajectory; transaction is empty because no payload transition occurred. |
| `executor_journal[14]` | Accepted UUID `6ccc248c4102987d6c7b29595c0b3662`; all immutable send metadata match event 8. |
| `executor_journal[20]` | Stop requested with reason GEOMETRY_UNCONFIRMED. |
| `executor_journal[22]` | Same active controller/UUID canceled: result_code=5, success=false. |
| `executor_journal[29]` | resource_handoff_committed, reason RELEASED. |

The sent/response digest is `7005cb22571e35431e7b8dff4e7bfdb8b30f3329f91abbfb6ceb952baef15e75`. This validates that the new send/response metadata are emitted consistently during PREGRASP. It does **not** validate returned→adopted→sent payload suffix identity: there are zero physical_submission, payload_suffix_adopted, payload_transaction_confirmed, or stage_confirmed events. No complete PICK, LIFT, NAVIGATE, PLACE, replan, or suffix handoff is accepted.

Parent feedback first reports GEOMETRY_UNCONFIRMED at `feedback_records[609]`, received steady=40592.625421523 and ROS=83.185 s. The rejected renewal response was received at steady=40592.682605989, ROS=83.237 s. The verifier preserved `RENEW_REJECTED:GEOMETRY_UNCONFIRMED`, then requested cancellation. Journal `issued_at=83.047 s` is retained as recorded; this archive does not reinterpret it as a precisely measured fault-onset timestamp. Geometry root-cause analysis belongs to the separately assigned reviewer.

Parent terminal status=5, success=false, resources_released=true; resource disposition RELEASE_CONFIRMED. Cleanup complete=true with no cleanup errors. The stop check has 36 samples over 0.7 s, ROS 84.8–85.5, zero recorded base drift/rotation/speed/commands. The saved stack session is stopped with remaining_owned_pids=[]. Failure, release, and cleanup success are separate conclusions.

## Original video and runtime identity

Owner full decode: 473 frames, matching 473 metadata frames and 473 sidecar rows. M5 independently checked contiguous indices 0–472; no new decode or per-frame pixel inspection was performed. Metadata marks PREGRASP execution at frame 284 and GEOMETRY_UNCONFIRMED at frame 308. Those are status annotations, not independent physical completion evidence. The 10 fps output gives 47.3 s nominal playback, whereas the recorder reports 86.884485107 s wall duration; playback speed is not a timing benchmark.

The original mp4, jpg, metadata, sidecar, result files, and saved runtime manifests are indexed with SHA256 in `runtime_video_review.json`. Recorded executor and MTC executable hashes match the current corresponding files; the runtime association comes from owner-saved process identities, not a new live process inspection.

Used verifier SHA256 is `40e36820d4398e62d22c27f0426a9f3fea2db3f070e8cdf1955d259a0a393eb9`, matching the frozen candidate. Original source files were not modified. `stage_excerpt.json` retains the full 30-event journal, relevant feedback transitions, renewals, and cleanup result. `verification.json` records the bounded cross-checks and explicitly labels suffix handoff NOT_EXERCISED.
