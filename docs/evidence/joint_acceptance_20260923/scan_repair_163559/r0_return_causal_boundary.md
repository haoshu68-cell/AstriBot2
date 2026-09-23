# R0 return-90 attribution boundary

Window: ROS58.594→237.711, wall180.021s, bounded by the second nav_started and its canceled terminal. This is the probe's180s timeout followed by USER_CANCEL, not a planner NO_PATH terminal. Counters differ from r0_analysis_return.json because that report includes post-return-start cleanup; this report excludes post-terminal samples.

- 727 policy states:469 REQUIRED_COVERAGE_UNAVAILABLE,164 CLEAR_CONFIRMATION,6 INPUT_UNAVAILABLE,88 CLEAR. All469 coverage rejections name scan STALE/ACQUISITION_EXPIRED;455 occur near stationary.
- All727 sampled path risk statuses CLEAR and corridor states NORMAL; one return path with17 poses was published.7019 envelope samples all READY_FIXED/allowed. This excludes these sampled specific rejection reasons, not all possible planning/control defects.
- 3600 controller samples:3164 POLICY_HOLD,168 ALIGN_START,268 FOLLOW. Sampling is about50ms and counts are not independent failures or exact duration fractions.
- R0 trace independently demonstrates651 unique waiting scans with internal tracking TF unavailable (map not yet checked), plus capture→receipt P9574ms, receipt→first-checked-ready P95240.946ms, selection→finish P9572.562ms. This identifies a major internal delay, not a source-stamp freshness problem alone.
- Source/TF waiting correlates with stale health, HOLD, repeated recovery confirmation and partial return progress. Robot still moved from approximatelyx0.800 to0.297; there was no sustained successful return.

Do not write that scan expiry is the sole failure cause. Among six INPUT_UNAVAILABLE samples, tight observation/state pairs show three floating300ms boundary rejects, two envelope_ready=false despite sampled authoritative READY_FIXED wire state, and one unmatched preceding observation (144.5ms reception gap). The latter cannot be attributed from this pairing. Keep all original thresholds and semantics during R1.

R1 comparison: verify only private C++ TF receiver/recorded adapter changed, exact loaded hash and same world/controller/profile/camera context; compare policy-internal receipt→TF-ready and decision capture age, sampled stale/hold/clear-confirmation rates, controller phases, actual arrival/stop, envelope state and cleanup. A successful R1 can support the chosen TF repair under this scene; residual freshness tails, different safety rejection or another return failure remain separate findings.

Structured counts, exact input hashes and six other rejection observations: r0_return_causal_boundary.json.
