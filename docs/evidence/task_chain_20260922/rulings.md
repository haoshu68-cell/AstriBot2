# Rulings for this resumed task chain

- User explicitly reset timing; old rounds remain in task_timing.json history. Each actual task starts its own new 5400-second budget.
- W0 verifies the pinned, installed dependency/interface set. Arbitrary ABI compatibility is not a finite acceptance condition. Source drift is reported separately and is not overwritten or silently promoted. Risk: pending newer navigation source is not covered by this baseline.
- Gazebo 6 time-only rewind does not promise immediate camera recovery. The original 15-second automatic-recovery probe remains FAILED. W0 clock-reset acceptance tests invalidation plus explicit owned cold restart; no old action resumes. W1 retains automatic/recovery policy and concurrency as separate work. Risk: availability requires a session restart after rewind; this is an operational workaround, not an upstream engine fix.
- class_loader unload warnings remain a limitation, not a claim of leak-free execution; long-duration/execution lifecycle belongs to W9. No crash-free hardware/execution claim follows from planning-only runs.
