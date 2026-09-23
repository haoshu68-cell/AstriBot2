# Assembly S0 profile preparation

These are offline geometry/CAD preparation and validation scripts. They do not
start ROS/GPU work, create a physical scene, grant execution permission, or
implement the robot's runtime matching policy. Production runtime remains C++.

`generate_profiles.py` consumes the explicit metre-based DESIGN_ONLY revision2
spec and writes a new immutable directory. Normal-offset polygon halfplanes,
analytic circles and clipped circles define the straight insertion section.
The D hole is the disk of radius R+c intersected with x<=a+c, with sharp joins.
DXF uses millimetres; circles and D arcs remain analytic entities.

```bash
python3 -B -m unittest discover -s tools/assembly -p 'test_*.py' -v
python3 -B tools/assembly/generate_profiles.py \
  --spec docs/assets/assembly_20260923/scene_spec_v2.yaml \
  --output docs/assets/assembly_20260923/profiles_next --yaw-step-deg 0.5

# Independent reader must pass before treating an export as usable CAD.
PYTHONPATH=/home/yjh/.cache/astribot/assembly/s0_20260923/python \
  python3 -B tools/assembly/verify_profiles.py \
  --input docs/assets/assembly_20260923/profiles_next \
  --output docs/evidence/assembly_s0_20260923/dxf_readback_next.json
```

Preparation requires Python3/PyYAML. Independent readback requires NumPy/SciPy
and ezdxf1.4.3. The current reader is installed only in the above task cache from
the official PyPI wheel; SHA256 is
`e5f2cf7f904cb78390f6df8a374c2cfe304bb50fce215768eab0e94677c32a17`.
Do not install packages or start heavy jobs during another task's exclusive
runtime window. Preserve source spec, producer bytes, input/output hashes and
failed attempts. `profiles_v1` failed independent DXF import; validated output
is `profiles_v2`, with rejection/readback evidence in the S0 report.

The screening matrix fixes translation to zero and samples yaw every0.5degrees.
Positive clearance is an explicit geometric witness; tangency is separate.
A negative sampled result is not proof that arbitrary translations/rotations
cannot fit. `business_success` is always false in the geometry tool: matching
an assigned instance/model/slot is owned by the future task runtime.

This substep excludes 3D stem/shoulder union, chamfers, mounting/fasteners,
gripper opening/withdrawal, IK, contact dynamics and physical manufacturing
tolerances. Those have separate phase gates.
