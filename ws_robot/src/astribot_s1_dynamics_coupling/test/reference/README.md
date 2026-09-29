# Frozen pre-migration Python oracle

These three Python sources were copied byte-for-byte from the production package
on 2026-09-21 before the C++ migration. Their original paths and SHA256 values are
in `docs/evidence/pybind_removal_20260921/dynamics_coupling/baseline_manifest.json`.

They are only importable after a test explicitly adds this `reference` directory
to its Python path. CMake installs no Python package, console script, reference,
test, or bindings. Always set `ASTRIBOT_BRIDGE_NATIVE_KERNELS=0` for the baseline;
the frozen optional-native import is historical source, not a runtime fallback.

Preserved behavior and known gaps are recorded in the migration evidence README.
Do not modify these files to make a comparison pass. Change production C++ and
explicit tests when a separately authorized contract change is required.
