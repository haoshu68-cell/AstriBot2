# Read-only contracts/fusion review (2026-09-21)

Two concrete issues were found and reproduced against the initial native build:
- BearingCone 3D norm tolerance changed acceptance at the 1e-6 boundary.
- 2D norm rounding changed association at 0.5 and provenance deduplication at 0.05,
  altering track count or conservative union geometry.

The owner corrected norm computation without widening thresholds. Re-running the
exact reproductions confirms parity. No outstanding actionable state discrepancy
was found in the reviewed frozen sources.

Independent validation: 47 owner tests passed; 8,000 generated bearing tolerance
cases passed; 106,000 vectors with 2D and 3D norms (212,000 comparisons) matched
Python exactly across normal/subnormal exponents and admission thresholds;
101 additional stateful scenarios / 16,005 operations passed. These include epoch
change, clock rollback, capture freshness and input preflight, deduplication,
association/source ordering, callback batch-size failures, unassociated
resolution, dynamic envelope bounds, and legal ROS timestamp maximum
2147483647999999999 ns.

Source and binary hashes: source_hashes_final.json. Reproduction scripts:
bearing_boundary.py, association_boundary.py, state_checks.py, norm_stress.py
(with norm_probe.cpp rebuilt against the frozen header). Results are in the
adjacent JSON files. No ROS, installs, source edits or Git actions were used in
this review. All evidence is pure/offline and does not certify runtime entry,
closed-loop simulation or hardware.
