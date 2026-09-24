# Stop cleanup clock sampling fix

The independent execution guard can stop a motion from inside tick(). stopping() records a fresh steady/ROS clock pair in ResourceAuthority, but the same tick previously attempted release with its earlier pair. This caused a false RESOURCE_CLOCK_RESET despite monotonic simulation time. Cleanup now samples a new pair before stop barriers. Genuine clock rewind rejection is unchanged.

RED: original first-stage binary, 34 received monotonic clock samples, six child cancellations and release, but a false reset in the journal. GREEN: same guard fault, 36 received monotonic samples, six cancellation acknowledgements and canceled terminals, released parent, no reset. Eight CTest targets (73 cases) passed. Both owned protocol sessions exited. This is isolated protocol evidence, not a new Gazebo acceptance.

The product patch contains only the clock-sampling change. The independent source snapshots include prior validation-only CDR diagnostics; they do not install or enable the unfinished six-stage executor. Artifact and raw result identities are retained in manifest.json. The next trajectory-capture diagnostic is a separate candidate.
