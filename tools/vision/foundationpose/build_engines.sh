#!/usr/bin/env bash
# Offline operation script, run only in a GPU container during the owned window.
# Caller must bind official models read-only to /models and a NEW output directory
# to /output. A generated engine is not proof of correct object-pose inference.
set -euo pipefail
umask 022

test -d /output
test -z "$(find /output -mindepth 1 -maxdepth 1 -print -quit)" || {
  echo 'Refusing to overwrite a nonempty evidence directory' >&2; exit 2;
}
exec > >(tee /output/build.log) 2>&1
date -Ins > /output/started_at.txt
sampler_pid=''
cleanup() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$sampler_pid" ]]; then
    kill "$sampler_pid" 2>/dev/null || true
    wait "$sampler_pid" 2>/dev/null || true
  fi
  date -Ins > /output/finished_at.txt
  printf '%s\n' "$result" > /output/exit_code.txt
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '%s\n' \
 '06ad19f2c3598cb76733feec084d3f6802e7ff143882ec42ba368df7e38ae094  /models/refine_model.onnx' \
 '49a4f5f094358913670733ec31e856b96271c869f9949aa3a0361cf7cf8f0be8  /models/score_model.onnx' \
 | sha256sum --check - | tee /output/models.checked.txt
trt_binary=$(command -v trtexec || true)
if [[ -z "$trt_binary" && -x /opt/tensorrt/bin/trtexec ]]; then
  trt_binary=/opt/tensorrt/bin/trtexec
fi
if [[ -z "$trt_binary" && -x /usr/src/tensorrt/bin/trtexec ]]; then
  trt_binary=/usr/src/tensorrt/bin/trtexec
fi
test -n "$trt_binary"
"$trt_binary" --help > /output/trtexec_help.txt 2>&1
dpkg-query -W -f='${Package}\t${Version}\n' | sort > /output/packages.tsv
nvcc --version > /output/nvcc_version.txt
if [[ -f /usr/local/cuda/version.json ]]; then
  cat /usr/local/cuda/version.json > /output/cuda_version.json
fi
nvidia-smi --query-gpu=name,uuid,driver_version,memory.total,memory.used --format=csv \
  > /output/gpu.before.csv
sha256sum "$trt_binary" > /output/trtexec.sha256

# Whole-device usage sampled every 500ms, not exact engine-only peak memory.
nvidia-smi --query-gpu=timestamp,uuid,memory.used,utilization.gpu \
  --format=csv --loop-ms=500 > /output/gpu.samples.csv &
sampler_pid=$!

for name in refine score; do
  maximum=42
  [[ "$name" != score ]] || maximum=252
  command=("$trt_binary" "--onnx=/models/${name}_model.onnx"
    "--saveEngine=/output/${name}.partial.plan"
    --minShapes=input1:1x160x160x6,input2:1x160x160x6
    --optShapes=input1:1x160x160x6,input2:1x160x160x6
    "--maxShapes=input1:${maximum}x160x160x6,input2:${maximum}x160x160x6"
    --noTF32 --skipInference)
  printf '%q ' "${command[@]}" > "/output/${name}.command.txt"
  printf '\n' >> "/output/${name}.command.txt"
  date -Ins > "/output/${name}.started_at.txt"
  timeout --signal=TERM --kill-after=15s 600s "${command[@]}" \
    > "/output/${name}.log" 2>&1
  test -s "/output/${name}.partial.plan"
  mv "/output/${name}.partial.plan" "/output/${name}.plan"
  sha256sum "/output/${name}.plan" > "/output/${name}.sha256"
  date -Ins > "/output/${name}.finished_at.txt"
done
printf '%s\n' 'FP32 (TF32/FP16 disabled); build only; real fixture inference remains separate' \
  > /output/BUILT
