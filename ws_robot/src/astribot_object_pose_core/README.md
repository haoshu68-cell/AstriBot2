# Known-CAD object pose registration

This ROS-independent C++ library estimates a rigid **camera_from_object** transform
from a known CAD surface and a segmented camera point cloud. It does not read
Gazebo state, require a pose seed, command the robot, or infer unknown-object CAD.

## API and coordinate contract

```cpp
#include <astribot_object_pose_core/registration.hpp>
const auto result = astribot::object_pose::estimate(model, scene, options);
// Consume result.camera_from_object only when result.success is true.
```

`model` is Nx6 `CV_32F`: x y z nx ny nz, positions in **metres** in the object
frame, normals pointing out of the CAD surface. `scene` is Nx6 with camera-frame
points and outward normals, or Nx3. Nx3 normals are estimated by local PCA and
oriented toward the **camera origin**, so an arbitrary translated world-frame
cloud is not a valid Nx3 input. Input must already be segmented to the target.
Frames, timestamps, camera calibration, segmentation provenance, model revision,
and service lifetime are the caller's responsibility.

The exported target is `astribot_object_pose_core::registration`.
`find_package(astribot_object_pose_core REQUIRED)` loads the dependency exports.

## Method and gates

1. Reject empty, sparse, nonfinite, malformed and covariance-planar point clouds.
   Validate normal vectors, options and declared proper-rotation symmetries.
2. OpenCV PPF builds a CAD point-pair hash and searches unknown rotation and
   translation; no PCA pose or identity-orientation fallback is returned.
3. Up to 32 global hypotheses are refined with robust point-to-point ICP.
   Correspondences run from observed points to the model, preserving missing
   surfaces under partial visibility. Refinement uses at most 600 uniformly
   selected observation rows, up to 45 iterations, normal agreement and 10%
   trimming. A single spatial KD-tree performs exact nearest-neighbor search
   with zero approximation epsilon. Quality scoring always uses **all input points**.
4. Principal-axis half-turns of the observed geometry challenge the winning
   hypothesis to expose orientation ambiguity hidden by PPF voting. These are
   ambiguity probes, not the delivered global pose estimator.
5. Reject insufficient coverage, excessive inlier RMSE, or competing distinct
   poses with comparable support. A rejection never produces a usable pose in
   the CLI JSON.

Default gates: >=60 input points, <=12000 points per cloud; 8 mm correspondence
radius; >=65% scene coverage; <=5 mm scene inlier RMSE. Without registered solid
geometry, >=35% total model coverage is a conservative fallback. With a registered
union of box primitives, acceptance instead requires >=75% expected-visible CAD
coverage, >=60 expected-visible samples, and at least two nonparallel supported
normal directions. Each direction needs >=20 observed inliers and >=5% of all
scene points. The same fixed gates apply to every view.
The independent test acceptance is <=20 mm translation and <=10 degrees rotation;
those pose-error values are not estimator inputs.

`model_coverage` is the fraction of all CAD samples with a scene neighbor inside
8 mm and agreeing normal (dot product >.35). `total_model_coverage` in JSON is
an explicit alias. `visible_model_coverage` divides matched visible CAD samples
by all CAD samples analytically expected to be visible at that candidate pose.
`visible_model_points` reports its denominator. `scene_coverage` is the converse
fraction of all observed samples with a CAD neighbor. With registered geometry,
only expected-visible CAD samples can support scene inliers. `coverage` in JSON is an
alias for model coverage. `rmse_m` is the root mean square distance of **scene
inliers only**, not a probability, covariance, full-cloud error or ground-truth
pose error. Uneven point density affects these sample-count fractions.

Ambiguity comparison defaults: rotations differing by >=.20 rad or translations
by >15 mm are distinct; alternatives within .035 coverage on both sides and
1 mm RMSE are rejected without geometry. With registered geometry, a distinct
alternative is rejected whenever it has comparable scene support and independently
passes the 75% visible-coverage and 5 mm RMSE gates, even if another candidate
scores somewhat better. This prevents hidden protrusions from being treated as
proof of a unique orientation. The largest registered primitive's half-turns
provide additional challenges around its own center. Finite global search cannot certify that all possible
poses were explored. Keep segmentation, lifetime and downstream collision/IK
gates independent.

## Registered CAD visibility

`Options::visibility_boxes` contains object-frame box centers and full side
lengths, in metres. Every model sample must lie on the exposed union boundary
with a consistent outward face normal. Incorrect or nonfinite geometry is
rejected. The candidate transform maps the camera origin into the object frame;
front-facing CAD samples are then ray-tested against every box using analytic
slab intersections. Neither ground truth nor camera intrinsics enter this test.

The method does not clip against an image frustum or ROI: geometry outside the
observed crop still counts as missing, a conservative choice. Occlusion by the
robot or external scene objects also counts as missing. Only self-occlusion by
the registered CAD is removed from the denominator. This visibility mode is
explicitly limited to registered axis-aligned box unions in their object frame.
General meshes and unknown CAD are not implemented.

For this CAD, exact ray audits give only 19.7% total visible samples from the long
front, 10.7% from the right, 28.4% from front/right, and 45.4% from front/right/top.
Therefore a total-surface threshold alone is not an observability measure. The
75% *visible* coverage, normal diversity, scene support and competing-pose gates
must be used together.

`models/asymmetric_union.visibility.json` declares the same three primitives as
the sample generator, using `schema: astribot.box_union/1`, `units: m`, and
`boxes: [{center_m: [x,y,z], size_m: [x,y,z]}]`. Bind this file to the model ID and
revision in the calling service registry, never to a client-supplied object pose.

## Symmetry

The default is an asymmetric model. Common object-axis half-turn symmetries are
checked and rejected if undeclared. This is not a general symmetry-discovery
proof. Supply every nonidentity member of a known finite rotation group as
`Options::symmetry_rotations`; identity is implicit, rotations act around the
object-frame origin, and each supplied rotation is checked against the CAD.
Accepted equivalent poses are `T_camera_object * S_object_object`, and
`symmetry_equivalent` is true. They do not have a unique object orientation.

`--symmetry box` declares the three 180-degree rotations of a centered rectangular
box with distinct side lengths. `--symmetry square_prism_z` declares the complete
8-element proper rotation group of a centered square prism (7 nonidentity members
in `Options`): four quarter turns around Z, and those turns composed with a
half turn around X. It requires a visibility model containing exactly one positive
box with x=y and z different (1 micrometre comparison tolerance), centered at the
origin, and matching CAD bounds. A cube is rejected: it needs 24 rotations through
the C++ API. The estimator still checks the declared rotations against CAD points;
this declaration does not make orientation unique or relax fit/visibility gates.
An invalid declaration returns `invalid_square_prism_z_model` and exit status 2.
`continuous_symmetry=true` / `--symmetry continuous`
rejects unique full-6D output (`continuous_symmetry_unobservable`).

## CLI

With the isolated install:

```sh
/tmp/astribot_pose_core_install/astribot_object_pose_core/lib/astribot_object_pose_core/object_pose_register --model MODEL.xyz --scene SCENE.xyz \
  --visibility-model REGISTERED_CAD.visibility.json --output /absolute/result.json
```

The executable is installed at
`<install>/astribot_object_pose_core/lib/astribot_object_pose_core/object_pose_register`.
Use its absolute path (the ROS libexec directory is not added to `PATH`). XYZ
text accepts blank lines and `#` comments, with consistently three or six
whitespace-separated columns. CAD requires six columns. `--output` atomically
publishes JSON separately from stderr diagnostics; without it, JSON goes to
stdout. Use a fresh output path for each invocation. The parent process can
terminate the CLI for a hard timeout or cancellation; the library is synchronous
and OpenCV PPF is not cooperatively interruptible.

Exit status: 0 accepted; 2 input or registration rejected; 64 command-line usage
error; 74 output I/O error. Additional options: `--symmetry none|box|square_prism_z|continuous`,
`--max-candidates N`, `--max-points N`, `--visibility-model REGISTERED.json`.

Stable JSON schema:

```json
{
  "schema": "astribot.object_pose/1",
  "method": "known_cad_ppf_icp",
  "success": false,
  "reason": "no_acceptable_pose",
  "camera_from_object": null,
  "coverage": 0.0,
  "model_coverage": 0.0,
  "total_model_coverage": 0.0,
  "visible_model_coverage": null,
  "visible_model_points": 0,
  "observable_normal_directions": 0,
  "scene_coverage": 0.0,
  "rmse_m": null,
  "ambiguous": false,
  "symmetry_equivalent": false,
  "candidate_count": 0
}
```

On success the transform is a 4x4 nested row-major JSON array. Nonfinite metrics
serialize as null. Rejection pose matrices are null even if diagnostics contain
a best unsuccessful candidate internally.

## Reproducible CAD and independent verification

`models/asymmetric_union.xyz` is the exposed union surface of three boxes,
sampled on a 6 mm grid at cell centers. The source geometry (centers and full
side lengths in metres) is:

| Box | Center | Size |
|---|---|---|
| A | (0, 0, 0) | (.180, .100, .080) |
| B | (.035, .015, .065) | (.070, .060, .050) |
| C | (.115, -.020, -.005) | (.060, .045, .040) |

Regenerate without simulator data or third-party model downloads:

```sh
python3 tools/generate_reference_cad.py --output models/asymmetric_union.xyz
```

Build and test without overwriting the workspace install:

```sh
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/astribot_pose_core_log build \
  --base-paths ws_robot/src/astribot_object_pose_core \
  --build-base /tmp/astribot_pose_core_build \
  --install-base /tmp/astribot_pose_core_install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
ctest --test-dir /tmp/astribot_pose_core_build/astribot_object_pose_core --output-on-failure
```

Tests exercise unknown large rotations, 0.5/0.85/1.6 m depths, 1 mm point noise,
partial observations, camera-facing Nx3 points and normal estimation, malformed
and nonfinite inputs, planes, a wrong scaled CAD, undeclared/declared box
symmetries, hidden distinguishing geometry, continuous symmetry, invalid options,
actual CLI JSON/file/exit contract, registered geometry mismatch, analytic
self-occlusion, complete two-face views below 35% total coverage, one-direction
stepped faces, featureless boxes, and ambiguous partially visible views. These are offline geometric tests;
they do not imply physical grasping, real-camera accuracy or robot acceptance.

## Recorded verification (2026-09-21)

The final isolated build passed all 23 CTests in 13.86 seconds (`-j2`). The
installed package target was also linked by a separate CMake consumer. Raw
first-failure and final test records are retained under `tests/evidence/`.
A 400-point refinement optimization was rejected after it broke ambiguity and
large-rotation tests; the final implementation retains 600 refinement samples.

A real Gazebo RGB-D cloud, reduced to XYZ only, completed registration in 1.63 s
with visible coverage .76746, scene coverage .99844, three normal directions,
and 2.427 mm inlier RMSE. This estimator-side run did not read pose ground truth;
pose-error scoring belongs to the independent simulation validation record.
Latency is a measured sample, not a real-time guarantee; callers must continue
to enforce their own hard timeout and input/result lifetime limits.
