# Follow-up integration and Git delivery checks

Three implementation checkpoints were recorded on the existing local migration
branch without changing shared HEAD/index or shared installations:

- `262f604e`: 17 old modules moved out of production packages.
- `bb94029d`: native arm/chassis Twist coupling, old Python production package removed.
- `fa91842f`: native task arbiter, old Python module/console removed, single native launch.

Source/ELF hashes in both new node manifests were independently recomputed and
matched. All four frozen Python files match the pre-migration `ee14a567` blobs
byte-for-byte. Root repeated ownership CTest 1/1, coupling core replay 1/1 and
its dependent arm-math test 4/4. The shared source entry/launch checks passed
18/18; extracting the recorded Git tree and running its scoped launch/entry
checks passed 21/21. Counts differ because these selections and the excluded
concurrent source tests are different, not because skipped failures were pooled.

`entries.json` uses ament package-index resolution across explicit owned `/tmp`
prefixes. All ten migrated executables resolve to ELF with complete shared-library
resolution and no Python/pybind library. The new arbiter and coupling are included.
Fresh remaining-Python package installation exposes only the two policy runtime
entries and two navigation diagnostics; the wheel launch package exposes no
Python executable. This does not remove older artifacts from shared installs.

The package reference-discovery validator was repeated after the arbiter console
removal. All 17 centralized old modules remain absent from production source and
fresh installed packages, with polygon/protection helpers preserved. Coupling's
separate install contains no Python runtime, and its installation evidence is in
its own manifest. These probes do not launch a shared ROS/Gazebo stack.

The latest read-only inventory finds 410 source files, 22 resolved Python console
entries, 82 remaining own runtime files and 20 validation-only historical modules;
no production importer points to those 20 references. The older snapshot is
retained. The whole-project pybind audit still fails intentionally: three own
binding definitions and the vendor SDK chain remain. Unit/isolated-node evidence
is not real Nav2, physical stop, whole-stack, long-duration or hardware acceptance.

`recorded_source_manifest.json` fingerprints the scoped source in the Git index
being committed. `recorded_scope.json` records intentional differences from the
shared worktree. The separately owned untracked validator's exact migration patch
is stored under `reference_retirement/`; its full source is not imported into this
record branch. No push or shared-branch merge was performed.
