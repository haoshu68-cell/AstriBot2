# Local validation, 2026-09-21

Evidence is model inference and offline replay of actual rendered Gazebo RGB-D.
It does not establish live Action acceptance, collision freedom, inverse
kinematics, physical grasp success, or hardware performance.

## Artifacts

Persistent artifacts live under
`runs/grasp_pose_sim_20260921/models/` (gitignored):

- `checkpoint-rs.tar`: SHA-256
  `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`.
- `graspnet_cuda.pt`: SHA-256
  `0f5c4a9fe2a6a83b6ef4675cc0c2b599e2929213ab308100def1aaed57178c95`.
- `graspnet_cpu.pt`: SHA-256
  `70326173d46614c99021f34abb9857fe142b74a81c2147b823fb50fbb7b3e594`.
- `download_provenance.json`, full upstream license, pinned source archive,
  both export reports, C++ equivalence reports and all replay outputs.
- `runtime_paths.json` and `cuda_dependencies.txt` identify persistent runtime
  locations and exact installed package versions.

CUDA worker:
`/home/yjh/.cache/astribot/graspnet/build_cuda/graspnet_worker`.
Its libraries resolve under
`/home/yjh/.cache/astribot/graspnet/torch-2.5.1-cu124`; no `/tmp` runtime dependency
was found by `ldd`. CPU worker and libraries use `build_cpu` and
`torch-2.5.1-cpu` in the same cache. Both exports are device-specific.

## Checks

- Official source commit `280c215129f759ed8649cb4e89fc5dfee55f4f80`, unchanged.
- Real pretrained checkpoint, epoch 18, strict load of all 162 state-dictionary
  entries. The official direct download was quota-limited; the mirrored
  checkpoint and its provenance limits are detailed in README.
- Six compatibility tests cover sphere/cylinder strict boundaries, ordering,
  padding, absent neighborhoods, FPS origin exclusion and CUDA thread-lane tie
  order, gather/interpolation and nearest neighbors.
- C++ binary-input CTest passed. End-to-end empty, sparse, NaN and negative-depth
  probes all returned exit code 2 and created no final grasp file.
- The complete `graspnet_setup.sh` CPU workflow was replayed from cached source,
  weights and libraries in a separate persistent prefix: export, changed-input
  equality, fresh single-job build and CTest passed. Evidence is
  `docs/evidence/grasp_pose_sim_20260921/graspnet/cached_cpu_rebuild.log`.
  This confirms the cached rebuild path, not a fresh-machine dependency install.
- Official RGB-D demo: 513,688 observed points, fixed-seed sample of 20,000,
  331 decoded grasps. Python eager and C++ output maximum absolute error was
  zero on CPU and GPU. Translated input was compared between eager and exported
  models: error zero on CPU, at most `2.69e-7` on GPU. The model output changes
  with input and the export retains dynamic candidate selection.
- GPU Python eager first/second model calls: 1621/294 ms. Isolated C++ GPU demo
  process: 1.26 s wall time, including 271 ms load and 629 ms inference.

## Actual Gazebo RGB-D replay

The validation script consumes only `scene.xyz` and `camera_info.json`.
It does not read ground-truth pose. Inputs were segmented by a magenta color
fixture mask, not by YOLO. Metadata reports source epoch `gazebo_camera`,
calibration revision 1 and frame
`astribot_s1/astribot_torso_base/torso_rgbd_sensor`; XYZ values are projected
camera optical coordinates in metres. Per-capture times and health are retained
in each JSON report. Replay is historical and does not bypass service freshness.

| Capture | Observed points | GPU candidates | Maximum raw score | GPU process wall time |
|---|---:|---:|---:|---:|
| close | 5844 | 1019 | 0.5842403 | 1.193 s |
| tilted | 2389 | 749 | 0.6814706 | 1.213 s |
| yawed | 2186 | 1022 | 0.8926520 | 1.212 s |

Report: `models/gazebo_cuda_validation/report.json`. All output rotations passed
proper-rotation checks. Scores are uncalibrated upstream values and can exceed
1; official demo maximum was 1.4119. No collision flag is asserted.

CPU produced the same candidate counts. Its measured process times were
7.146/5.893/4.910 s, so this evidence does not support a 5 s deadline on CPU.
GPU replay timings fit within that interval on this host, but live service
acceptance is a separate validation owned by the action-server integration.

The original legacy CUDA extension could not be built without nvcc, so bitwise
parity to that original implementation is not established. Torch-compatible
operator tests, eager/export equivalence, and C++ equality qualify the adapted
backend. The source and pretrained network were not retrained or replaced.
