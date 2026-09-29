# Validation-only Python references

These 17 files were moved byte-for-byte from the production source packages
after their runtime roles migrated to C++. The relocation manifest and SHA256
values are in `docs/evidence/pybind_removal_20260921/reference_retirement/`.

No ROS package installs this directory. It has no console registration, launch
entry or environment hook. Production package imports and ordinary/symlink
installations must not discover these retired modules. Do not add this directory
to a runtime launch environment.

`ws_robot/src/conftest.py` explicitly enables these references in regression test
processes. Standalone replay tools opt in with `reference_bootstrap.enable()`;
owned oracle subprocesses prepend this directory to their validation-only
`PYTHONPATH`. The original module names are retained to preserve test fixtures
and relative imports. Live shared helpers and compiled test extensions remain
in their original source/installation packages.

Historical geometry performance replay deliberately uses its frozen
`--python-root` baseline. Do not enable this reference overlay in that driver.
To construct a new frozen baseline, copy the geometry production package plus
the four geometry reference files to an owned temporary package directory,
then build/copy its test binding there. Existing historical evidence remains
unchanged and still identifies the exact original source hashes.

The remaining live Python controller/observer, polygon and device SDK imports
are separate migration work; this relocation does not eliminate their bindings.
