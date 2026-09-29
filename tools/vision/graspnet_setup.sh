#!/usr/bin/env bash
# Isolated preparation only. Does not start ROS, Gazebo, or robot controllers.
set -euo pipefail
graspnet_prefix="${1:-${XDG_CACHE_HOME:-$HOME/.cache}/astribot/graspnet/runtime}"
graspnet_device="${2:-cuda}"
if [[ "$graspnet_prefix" != /* || ( "$graspnet_device" != cuda && "$graspnet_device" != cpu ) ]]; then
  echo 'Usage: graspnet_setup.sh /absolute/cache-prefix [cuda|cpu]' >&2
  exit 2
fi
graspnet_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
graspnet_source="${graspnet_prefix}_source/baseline"
graspnet_deps="${graspnet_prefix}_deps"
graspnet_models="${graspnet_prefix}_models"
graspnet_build="${graspnet_prefix}_build"
graspnet_commit=280c215129f759ed8649cb4e89fc5dfee55f4f80
graspnet_checkpoint_hash=60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868
mkdir -p "$graspnet_deps" "$graspnet_models" "${graspnet_prefix}_source"
python3 -c 'import numpy, scipy, PIL'  # preparation-only prerequisites
if [[ ! -d "$graspnet_source/.git" ]]; then
  git clone https://github.com/graspnet/graspnet-baseline.git "$graspnet_source"
  git -C "$graspnet_source" checkout --detach "$graspnet_commit"
fi
[[ "$(git -C "$graspnet_source" rev-parse HEAD)" == "$graspnet_commit" ]]
git -C "$graspnet_source" diff --quiet HEAD -- models pointnet2 utils
if [[ "$graspnet_device" == cuda ]]; then
  PYTHONPATH="$graspnet_deps" python3 -c 'import torch; assert torch.__version__ == "2.5.1+cu124"' 2>/dev/null ||
    python3 -m pip install --target "$graspnet_deps" --upgrade --no-cache-dir 'torch==2.5.1'
else
  PYTHONPATH="$graspnet_deps" python3 -c 'import torch; assert torch.__version__ == "2.5.1+cpu"' 2>/dev/null ||
    python3 -m pip install --target "$graspnet_deps" --upgrade --no-cache-dir 'torch==2.5.1+cpu' --index-url https://download.pytorch.org/whl/cpu
fi
if ! echo "$graspnet_checkpoint_hash  $graspnet_models/checkpoint-rs.tar" | sha256sum --check --status; then
  curl --fail --location --max-time 300 \
    'https://huggingface.co/AIGeeksGroup/GeneralVLA/resolve/cf04f9c258588a2e1f46ecb22268f20b087c917c/checkpoints/v1/checkpoint-rs.tar' \
    --output "$graspnet_models/checkpoint-rs.tar.download"
  echo "$graspnet_checkpoint_hash  $graspnet_models/checkpoint-rs.tar.download" | sha256sum --check
  mv "$graspnet_models/checkpoint-rs.tar.download" "$graspnet_models/checkpoint-rs.tar"
fi
export PYTHONPATH="$graspnet_deps${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CUBLAS_WORKSPACE_CONFIG=:4096:8
python3 "$graspnet_repo/tools/vision/graspnet_test_ops.py"
python3 - "$graspnet_source" "$graspnet_models/official_demo_cloud.bin" <<'PY'
import sys
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.io import loadmat
p = Path(sys.argv[1]) / 'doc/example_data'
depth = np.asarray(Image.open(p/'depth.png'))
mask = np.asarray(Image.open(p/'workspace_mask.png')) > 0
meta = loadmat(p/'meta.mat')
k = meta['intrinsic_matrix']
z = depth / meta['factor_depth']
y, x = np.indices(depth.shape)
xyz = np.stack(((x-k[0,2])*z/k[0,0], (y-k[1,2])*z/k[1,1], z), -1)
xyz[mask & (depth > 0)].astype('<f4').tofile(sys.argv[2])
PY
python3 "$graspnet_repo/tools/vision/graspnet_prepare.py" --source "$graspnet_source" \
  --checkpoint "$graspnet_models/checkpoint-rs.tar" --cloud "$graspnet_models/official_demo_cloud.bin" \
  --output "$graspnet_models/graspnet_${graspnet_device}.pt" --device "$graspnet_device"
graspnet_abi="$(python3 -c 'import torch; print(int(torch._C._GLIBCXX_USE_CXX11_ABI))')"
cmake -S "$graspnet_repo/ws_robot/src/astribot_graspnet_runtime" -B "$graspnet_build" \
  -DCMAKE_BUILD_TYPE=Release -DGRASPNET_TORCH_ROOT="$graspnet_deps/torch" \
  -DGRASPNET_TORCH_CXX11_ABI="$graspnet_abi"
cmake --build "$graspnet_build" -j1
ctest --test-dir "$graspnet_build" --output-on-failure
echo "worker=$graspnet_build/graspnet_worker"
echo "model=$graspnet_models/graspnet_${graspnet_device}.pt"
