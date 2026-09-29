# GraspNet inference worker

This package separates LibTorch and its C++ ABI from ROS. `graspnet_worker` runs
the official GraspNet-baseline network with pretrained RealSense weights, after
an offline export. It owns no robot command interfaces. Python is used for model
preparation and equivalence checks; the deployed worker is C++.

## Wire contract

```
graspnet_worker --model /absolute/model.pt \
  --input /private/job/cloud.bin --output /private/job/grasps.bin --device cuda
```

Input is headerless little-endian IEEE754 float32, N by 3 XYZ in metres in the
camera optical frame (positive Z). N must be 2048 through 2,000,000 inclusive;
nonfinite or nonpositive depths fail. Capture frame, timestamp, calibration,
and scene versions are checked by the calling ROS component, which also owns
deadlines and cancellation. The worker uses exactly 20,000 sampled existing
points; a fixed Torch CPU random seed makes the upstream-style sampling
reproducible. This samples observed points and does not generate synthetic grasps.

Output is headerless float32, M by 17, using GraspNet's decoded ordering:
`score,width,height,depth,R00,R01,R02,R10,R11,R12,R20,R21,R22,x,y,z,object_id`.
Distances are metres; R maps the GraspNet gripper frame into camera optical.
M can be zero and is at most 1024. A temporary output is renamed only after a
successful complete inference; exit code 2 indicates failure. Standard output
contains timing and model provenance as JSON. Standard error contains errors.
An empty file with exit code zero is a valid no-candidate result.

Scores are the upstream model's score times tolerance normalization; they are
not calibrated success probabilities. Object id remains upstream's `-1`.
Candidates have no collision, inverse-kinematics, reachability, or execution
approval. The caller must not set those flags based on inference success.

## Build

One-command preparation (requires Python 3.10, NumPy, SciPy, Pillow, pip, git,
curl, CMake and a C++17 compiler) uses a persistent isolated cache:

```
bash tools/vision/graspnet_setup.sh "$HOME/.cache/astribot/graspnet/runtime" cuda
```

Use `cpu` for a CPU-only wheel/export. The script downloads pinned source and
checkpoint, checks the checkpoint hash, tests operators, exports on official
demo RGB-D, verifies changed-input equivalence, and builds with one compiler job.
No ROS/Gazebo process is started. The complete CPU script was successfully
replayed with cached source, checkpoint and dependencies, including fresh
export, changed-input verification, compilation and CTest. An empty-machine
network installation was not repeated. GPU stages were separately verified.

The default build runs only protocol tests and needs no Torch installation:

```
cmake -S ws_robot/src/astribot_graspnet_runtime -B /tmp/astribot_graspnet_build
cmake --build /tmp/astribot_graspnet_build -j1
ctest --test-dir /tmp/astribot_graspnet_build --output-on-failure
```

To build the isolated worker, additionally set
`-DGRASPNET_TORCH_ROOT=/tmp/astribot_graspnet_deps/torch` and
`-DGRASPNET_TORCH_CXX11_ABI=0` (verify the ABI flag from the installed PyTorch).
CUDA libraries are supplied by the isolated wheel installation. Do not link
these libraries into the ROS process. Use a private job directory and configure
the executable/model at node startup, rather than accepting executable paths
from remote goals.

## Model and licensing boundary

Upstream source: <https://github.com/graspnet/graspnet-baseline>, pinned commit
`280c215129f759ed8649cb4e89fc5dfee55f4f80`. Preserve the complete upstream LICENSE
and source beside prepared artifacts. The upstream license restricts the model
and its derivatives to noncommercial research; this package's Apache license
does not relicense the model or exported artifact.

Official checkpoint link:
<https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view>.
On 2026-09-21 both direct Drive download forms reported quota exceeded. The
public official Drive and Baidu pages described a 12,468,415-byte checkpoint.
The downloaded artifact came from a pinned mirror:
<https://huggingface.co/AIGeeksGroup/GeneralVLA/blob/cf04f9c258588a2e1f46ecb22268f20b087c917c/checkpoints/v1/checkpoint-rs.tar>.
Its SHA-256 is
`60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`, also recorded
by <https://huggingface.co/dgrachev/a2_pretrained/commit/801483b4210c1d49667ae737ceee7dc97c5c8ddc>.
These are matching mirrors, not a cryptographic attestation by the original
publisher. The exporter requires this exact hash and strict state-dictionary
loading; it never falls back to random network weights or geometric candidates.

The compatibility operators preserve the upstream algorithms, including first
in-radius neighborhood selection, strict cylinder endpoints and origin exclusion
in farthest-point sampling. They use scripted Torch operations instead of the
legacy CUDA extension because this environment has no nvcc. This is an adapted
backend, not evidence of bitwise equivalence to the original CUDA implementation.
Final local numerical and timing evidence belongs in the generated export report.
