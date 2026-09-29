# Retired Python reference installation cleanup

The 17 previously replaced Python modules were moved to
`tools/migration/python_reference/<original_package>/`. Every moved file is
byte-identical to both its pre-move source and Git checkpoint `ee14a567`;
`relocation_manifest.json` records the original path, new path and SHA256.
The production package directories no longer contain these modules.

The reference namespace is explicitly enabled by source regression tests and
standalone validation drivers only. There is no production environment hook,
console registration, launch selector or runtime fallback. Installed/source
production imports cannot discover the old modules. Live `polygon.py`,
`protection.py`, controller/observer and device bridge consumers remain intact.

## Verification performed

| Check | Result |
|---|---|
| Retirement/source-import contract | Red: 17 failed, 1 passed; after relocation: 18 passed |
| Fresh installs of navigation, policy and wheel packages, plus a fresh compatibility-enabled geometry install | All 4 package locations resolved to owned prefixes; all 17 retired modules absent; live polygon/protection still discoverable |
| Ordinary production source import discovery | All 17 absent, including when shared ROS underlays are available |
| Geometry, posture, wheel, entry selection, envelope and clock regressions | 83 passed |
| Fixed envelope differential/protocol, final protection, legacy envelope and costmap scan wire protocols | 226 passed in 165.63 s |
| Arm limiter and body/world command parameter protocols | 52 passed, 12 skipped in 61.36 s; the 12 unchanged skips are old-Python exceptions for the deliberately stricter C++ invalid/read-only parameter contract |
| Fresh geometry Release build and CTest | 5/5 passed |
| Final explicit child bootstrap smoke | 8 ROS cases passed; wheel Python/C++ replay matched active commands and stale-feedback zero output |
| Reference provenance / standalone tools | All 17 reference import locations verified; live helpers resolve to current source; five standalone benchmark/validation imports and direct projection unittest passed |
| Inventory | 17 explicit validation references; no production importer found for any of them |

These are installation, offline and isolated ROS checks. No shared installation,
Gazebo stack or hardware was changed. Runtime C++ algorithms were not modified
by this cleanup; no new runtime performance claim is made from a file move.
Normal installs and ordinary source imports were tested directly. An actual
new symlink installation was not run; physical removal also removes the old
module from the source directory that such an installation would expose.

`regression_initial.log` records a collection failure because the old
`_navigation_math_native` benchmark binding was absent from the test environment.
The corrected run built the historical test bindings in an owned temporary
prefix, then passed. `test_bindings_build.log` records the first script's wrong
assumption that a standalone CMake install creates a prefix-level
`local_setup.bash`; `test_bindings_build_retry.log` sources the generated
package-level setup file and succeeded. Neither failure was hidden or counted
as a production behavior failure.

## Reproduction and provenance

- Run `python3 -m pytest -q tools/migration/test/test_retired_python_packaging.py`.
- Build geometry with `ASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=ON` in an owned
  prefix to validate the still-required polygon/binding installation.
- Run `tools/migration/verify_retired_python_install.py --work <fresh-directory>
  --geometry-prefix <owned-geometry-prefix>`. The tool builds/installs only the
  three Python launch/remaining-runtime packages into that new directory.
- For regression tests, source ROS Humble and required message underlays, then
  source the owned geometry package's `share/astribot_s1_robot_geometry/local_setup.bash`.
  Point `PYTHONPATH` at the independently built test math bindings. Source-tree
  pytest enables historical references through `ws_robot/src/conftest.py`.
- ROS protocol candidates used here are the previously validated native
  binaries from `/tmp/codex_navigation_validation_20260921/build` and
  `/tmp/codex_arm_parameters_20260921/build`. The geometry candidate was rebuilt
  under `/tmp/codex_reference_retirement_20260921/geometry_build`.

The old geometry evidence's `reproduce.sh` describes the layout at its original
checkpoint. When replaying from the current source layout, additionally copy
the four files from `tools/migration/python_reference/astribot_s1_robot_geometry/`
into the explicitly frozen `baseline_src/astribot_s1_robot_geometry/` directory
before using that script's benchmark commands. Do not replace its historical
snapshots/manifests or prepend the current test overlay ahead of that frozen
baseline. Its driver verifies the loaded model's baseline path.

The inventory snapshot here is contemporaneous with this cleanup, before the
next coupling/arbiter changes: 85 remaining own runtime Python files, one
camera calibration compatibility entry and 17 validation references. It is not
a declaration that the full project is free of pybind.

Independent review found and closed two validation-only omissions: the scan occupancy benchmark now explicitly enables the reference namespace, and all five old-node subprocess drivers enable it before importing any ROS package helper. The final loader and wheel smoke logs cover these corrections. `oracle_imports.json` records the actual imported paths and hashes. The existing untracked `tools/sim/verify_multi_load_dynamic.py` belongs to a separate worktree change; only this task's exact import/documentation hunk and before/after hashes are retained in `untracked_validation_consumer.patch` / `.json`, without incorporating that full unrelated script into the record branch.
