# FoundationPose P0 preparation and isolated validation

`prepare_p0.py` prepares the approved P0 assets without ROS/GPU side effects.
This directory also contains explicitly separate environment operations, launch
and offline validation tools. None is a Python perception runtime or grants robot
execution permission. The estimator is the upstream C++ Isaac ROS backend;
project runtime integration remains C++.

## Run

The explicit historical dataset is the seven single-frame Gazebo captures from
the 2026-09-21 PPF/GraspNet comparison. Do not substitute the current camera frame
or calibration into these files.

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
# Historical v2 command, retained for provenance only.
# Do not repeat this whole-archive network download; use the fixed checkout below.
python3 -B tools/vision/foundationpose/prepare_p0.py \
  --output runs/foundationpose_p0_20260923/prepared_v2 \
  --cache /home/yjh/.cache/astribot/foundationpose/p0_20260923 \
  --dataset docs/evidence/grasp_pose_sim_20260921/simulation/snapshots \
  --download-timeout-seconds 240 --network-budget-seconds 300

# CURRENT RECOMMENDED CHECK: offline, into a NEW evidence directory.
# Expected exit 2: the legacy full-archive gate is still missing, even though
# the separately verified Git source checkout is now available.
python3 -B tools/vision/foundationpose/prepare_p0.py \
  --output runs/foundationpose_p0_20260923/offline_recheck_v1 \
  --cache /home/yjh/.cache/astribot/foundationpose/p0_20260923 \
  --dataset docs/evidence/grasp_pose_sim_20260921/simulation/snapshots --offline

python3 -B -m unittest discover -s tools/vision/foundationpose -p 'test_*.py' -v
```

Requirements: Python 3, NumPy, Pillow and curl. These are preparation tools only;
they do not provide Isaac ROS or prove that the backend runs. `--help` documents
all arguments. Exit 0 means CPU preparation succeeded; 2 means the manifest
records blocked downloads; 1 means an input/integrity/output conflict rejected
the run. **Exit 0 never means P0 inference or integration acceptance.**

Network transfer has a total per-artifact timeout, no implicit retry, and a
shared finite budget. A failed transfer is removed without publishing its final
name. Model sizes and SHA256 values are pinned to the official exact-version NGC
file listing; the listing is checked and archived too. A local calculated SHA256
and a publisher-listed SHA256 are recorded as separate facts. The NGC signature
is not cryptographically verified by this script.

Existing cache bytes must match their first receipt and the pinned hash. A rerun
with identical output is a no-op and preserves the files' modification times.
Changed script, inputs, assets or manifest are rejected instead of silently
overwriting an existing evidence directory. Use a new output name such as
`prepared_v3` for revised preparation. An interrupted cache publish can be retried
against the receipt that was already saved. A blocked run is evidence: preserve
it and use another output directory for a successful retry.

## Frozen inputs

- `isaac_ros_common`: `fcf4d9e17f8f0a7f47f1d22d6a18421ce3768c01`
- `isaac_ros_pose_estimation`: `9caca619bcc9d637b3107e17c1a77132c9d7863b`
- NGC `nvidia/isaac/foundationpose:1.0.0_onnx`
- Project CAD input:
  `ws_robot/src/astribot_object_pose_core/models/asymmetric_union.visibility.json`

The fixed GitHub commit archives are saved without executing or extracting the
code tree. Their license/notice members are copied as regular files under
`licenses/`. The NGC exact-version model metadata and its Model EULA link are
saved independently. Neither the ROS package license nor the NVlabs research
license is assigned to the NGC weights. Transitive dependencies, GXF runtime
artifacts, image digest and installed Debian versions remain unfrozen until the
isolated supported environment is actually prepared.

The first actual run `prepared_v1` is retained with a real `BLOCKED` result:
the large pose-estimation archive exceeded the 120-second transfer limit. Its
partial file was discarded, while completed models, metadata and licenses stayed
in the cache. `prepared_v2` also exceeded its 240-second archive transfer limit
and remains `BLOCKED`. It preserves the geometry JSON and preparation-script
bytes. No further whole-archive retry was performed. Both manifests retain the
real `curl exit 28` failure; the completed model/CAD/dataset checks remain usable.

A separate, bounded `git fetch --filter=blob:none --depth=1` saved only the pinned
pose repository commit/tree in
`/home/yjh/.cache/astribot/foundationpose/p0_20260923/source_metadata/isaac_ros_pose_estimation.git`.
Its tree is `46c29b8943e753ad0f12bb5235ced0902a0ee763`; 179 unique blobs were
absent at that metadata-only stage. That fetch did not check out files, run repository scripts, or
turn either blocked archive result into a pass. The record is
`docs/evidence/foundationpose_p0_20260923/source_metadata.json`.

A subsequent authorized, bounded checkout materialized all 181 Git-tracked files
(2,783,038 bytes) at
`/home/yjh/.cache/astribot/foundationpose/p0_20260923/source_checkout/isaac_ros_pose_estimation-9caca619bcc9d637b3107e17c1a77132c9d7863b`.
Hooks and LFS smudge/process filters were disabled; the 12 LFS test-file pointers
remain pointers. No repository code was executed. The verified HEAD, tree,
every file hash, license and LFS payload sizes are in
`docs/evidence/foundationpose_p0_20260923/source_checkout.json`. This usable
source checkout is separate evidence; it does not rewrite either failed archive
manifest and is not an archive including LFS assets. Container/transitive
runtime dependencies still require their own freeze.

Read `source_checkout.json` alongside the legacy `manifest.json`: the former is
the current usable pinned source evidence; the latter preserves the historical
archive failure. Subsequent environment preparation should use the fixed
checkout directly, not fetch the large codeload archive again.

The separately prepared official single-frame fixture is in
`/home/yjh/.cache/astribot/foundationpose/p0_20260923/official_sample`.
Its four PNG payloads match the pinned commit's LFS SHA256 and byte sizes; the
camera JSON, OBJ and MTL were copied from the fixed checkout. See
`docs/evidence/foundationpose_p0_20260923/official_sample.json` and
`official_sample_adapter_notes.json`. No model was run. The original checkout
still contains its LFS pointer files and was left clean.

Do not pass this official fixture through the project dataset's input assumptions:

- Official RGB is a 640×480 RGBA PNG; the fixed upstream test reads it using
  OpenCV's default BGR path, which drops alpha, then converts to RGB8.
- Official depth is a 640×480 uint16 grayscale PNG in millimetres. The fixed
  `isaac_ros_foundationpose_pol.py` lines 151–157 explicitly converts to float32
  and divides by 1000 before publishing `32FC1` metres. Convert exactly once.
- Its mask has 3252 foreground pixels, including 605 pixels with zero depth.
  These remain invalid depth; no filling or mask modification was performed.
- Its camera frame is `tf_camera`, D is zero, and the fixture has no capture
  timestamp. It does not establish online freshness, registration/tracking
  performance or physical calibration.
- The project historical fixtures remain 640×360 RGB8, float32 metres and their
  original frame/calibration. Their stricter mask/depth checks are not silently
  weakened to accept a different official test fixture format.

## Output boundary

`manifest.json` contains provenance, hashes, geometry/data checks and explicit
`BLOCKED` / `NOT_RUN` stages. It is an evaluator/operator artifact.

Give the future estimator access to **only**:

- `algorithm_inputs/<scenario>/{rgb.png,depth.npz,mask.png,camera.json}`
- `cad/{centered.obj,material.mtl,albedo.png,registry.json}` as needed for assets

The image directory contains no object pose, setup pose, ground truth or point
cloud generated from truth. `camera.json` only carries measured image geometry,
timestamps, source metadata and the historical fixture mask label.

Keep `evaluation_only/` and `manifest.json` outside the estimator mount. The
former retains original `camera_info.json`, `scene.xyz`, `truth.json` and
`truth_setup.json` for separate scoring. Filesystem directory separation is not
an OS access-control boundary: the future container/worker launcher must enforce
the allowed mount list. The preparation script does not launch that worker.

All seven masks are **HSV magenta color fixtures**, not general instance
segmentation. All seven captures use calibration revision 1 and historical
`astribot_s1/astribot_torso_base/torso_rgbd_sensor`. The current raw torso evidence
uses `astribot_s1/astribot_torso_link_4/torso_rgbd_sensor`; neither is renamed here.
Seven isolated frames cannot measure tracking, recovery or end-to-end freshness.
The later independent truth samples are usable only under their recorded static
target and capture-time camera-TF assumptions.

## CAD coordinates

The generator partitions coordinates at every input box boundary and emits only
exposed cell faces. Shared faces inside the union disappear. Checks cover edge
incidence, opposite edge orientation, vertex links, connectivity, signed volume
and bounds. Inputs must explicitly declare metres; no unit guessing is allowed.
Coordinates are normalized to 1 pm to avoid floating-point sliver cells.

`asymmetric_union.obj` preserves the original business object frame. The render
mesh `centered.obj` subtracts the AABB center. `registry.json` permanently stores
both transforms using `T_A_B` to map points in B into A:

```text
T_camera_object = T_camera_mesh * T_mesh_object
```

The magenta material is a uniform simulation fixture matching the historical
scene's nominal material, not a measured texture. Geometric centering is checked
offline; the actual Isaac decoder's returned frame still needs P0 inference
verification. Do not apply a second centering compensation by assumption.

## Remaining P0 gates

The preparation script checks model bytes without importing ONNX. A later
separate CPU-only container check in `onnx_check_v2` passed ONNX 1.16.0 full_check
and input contracts for both models; `onnx_check_v1` retains its real tmpfs
noexec loader failure. Neither check is model inference. Docker/Toolkit and a
device visibility probe passed; the fixed TensorRT base image is available.
Isaac ROS/GXF/backend compatibility, FP32 engines, official/project inference,
coordinate accuracy, tracking and inference latency remain NOT_RUN.

Resource ownership is tracked in
`docs/FOUNDATIONPOSE_ASSEMBLY_COORDINATION_20260923.md`. The initial window was
explicitly released at 15:31; Docker/Toolkit installation and a separate device
visibility probe then passed. At 15:57 navigation resumed exclusive regression:
the large image pull was interrupted with exit 130, and no GPU/ROS container may
start until a new explicit release. A release was received at16:08 and the image
download completed at16:30. At16:36 all task containers were gone and resources
were explicitly returned to navigation. FP-BACKEND-01 is deferred at its first
one-hour checkpoint; the next independent work is CPU assembly geometry.
Time expiry alone does not release resources.

## Isolated runtime tools (execution status is separate)

- `Dockerfile.p0` is a candidate minimal backend image, based on fixed official
  TensorRT 24.08 amd64 digest, ROS Humble and Isaac ROS FoundationPose 3.2.14.
  The unavailable common-generated image tag and the candidate's different
  provenance are recorded in the P0 report. It is not yet a validated replacement
  for the full Isaac ROS developer image.
- Its minimal v3 build context contains only the Dockerfile, the three pinned
  keys `ros_key.gpg` / `isaac_repos.key` / `cuda-3bf863cc.pub`, and the official fixed
  `nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb` release asset. The latter follows
  the pinned common recipe; ELF inspection confirms FoundationPose directly
  requires `libcvcuda.so.0` and `libnvcv_types.so.0`. Fetch keys from the official ROS
  rosdistro and NVIDIA Isaac repositories over verified HTTPS; the Dockerfile
  rejects changed hashes. ROS HTTP follows the pinned upstream common recipe
  and retains `signed-by` verification. No insecure registry/TLS flags are used.
- Actual base inspection confirms TRT10.3.0.26; fixed common requires10.3.0.30.
  Resolve the v3 APT plan and align the complete runtime/tool family before
  creating engines. Check actual loaded libraries and any pip Python binding
  residue; an image tag or Python's abbreviated10.3.0 string is insufficient.
  Contexts v1/v2 preserve earlier candidates and must not be mistaken for v3.
- `build_engines.sh` is an explicit GPU operation. Mount only `/models` read-only
  and a new `/output` directory. It verifies both model hashes, disables TF32 and
  FP16, bounds each conversion at 600 seconds and writes the successful engines
  atomically. It saves command/version/hash/timing and whole-device 500 ms GPU
  samples. Apply a caller deadline within the resource window. Engine generation
  uses `--skipInference`; it cannot establish object-pose accuracy.
- `p0_fixture.launch.py` starts only the C++ FoundationPose component.
  `validate_fixture.py` is a bounded, one-shot replay/validation script, not a
  production ROS service. Official uint16 mm depth is converted once, project
  float32 metre depth and original capture stamps are preserved. The official
  sample's missing capture timestamp is labelled synthetic, never fresh capture.
- Output checks reject empty detections, mismatched array/detection frame or
  timestamp, nonfinite values, malformed quaternion, nonpositive box dimensions
  or a center behind the camera. PASS means numeric/header sanity only. Pose
  error, decoder coordinate convention and tracking remain separate gates.
  First-publication-to-first-output time includes queueing/retry/transport;
  it is not reported as pure model latency.
- `run_fixture.sh` starts that backend and validator inside one disposable
  container, preserving their separate logs/exit codes. The caller must enforce
  a 180-second container-internal deadline (plus bounded shutdown) and confirm
  removal of its owned container; the shell trap alone is not a cleanup proof.

Runtime invocation must use `--network none`, a private container namespace,
unprivileged user, read-only input mounts, dropped capabilities and no robot,
host DDS, Docker socket, `evaluation_only/` or manifest mount. Build networking
and inference networking are separate. The input loader has been exercised on
the official sample and seven project frames; the launch/backend has NOT_RUN
status until actual logs prove otherwise.

`check_onnx.sh` is a separate CPU inspection operation. It uses pinned wheel-only
ONNX1.16.0/protobuf4.25.3, no GPU mapping, a read-only root and a temporary tool
directory that permits shared-library execution. Its temporary installation
does not modify the image or host. Package origins/hashes are in the pip report.

CPU checks:

```bash
python3 -B -m unittest discover -s tools/vision/foundationpose -p 'test_*.py' -v
bash -n tools/vision/foundationpose/build_engines.sh
```

Official sources:

- [Isaac ROS release-3.2 FoundationPose](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html)
- [NGC exact-version file metadata](https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0_onnx/files)
- [NGC exact-version model metadata](https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0_onnx)
