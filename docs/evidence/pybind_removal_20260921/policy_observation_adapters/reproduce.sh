#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../../.." && pwd)
build=${POLICY_ADAPTERS_BUILD_DIR:-/tmp/codex_policy_observation_adapters_20260921}
mkdir -p "$build"
native="$repo/ws_robot/src/astribot_s1_navigation_policy_native"
geometry="$repo/ws_robot/src/astribot_s1_robot_geometry/include"
flags=(-std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Wpedantic)
suffix=""
if [[ ${POLICY_ADAPTERS_SANITIZE:-0} == 1 ]]; then
  flags=(-std=c++17 -O1 -g -ffp-contract=off -Wall -Wextra -Wpedantic -fno-omit-frame-pointer -fno-pie -no-pie -fsanitize=address,undefined)
  suffix="_sanitized"
fi
g++ "${flags[@]}" -I"$native/include" -I"$geometry"   "$native/src/policy_contracts.cpp" "$native/src/policy_observation_adapters.cpp"   "$native/test/policy_observation_adapters_probe.cpp"   -o "$build/policy_observation_adapters_probe$suffix" > "$build/build$suffix.txt" 2>&1
probe="$build/policy_observation_adapters_probe$suffix"
if [[ ${POLICY_ADAPTERS_SANITIZE:-0} == 1 ]]; then
  # Keep the host audit preload while placing ASan first for the child probe.
  cat > "$build/probe_sanitized_wrapper" <<WRAPPER
#!/bin/sh
exec env LD_PRELOAD="$(g++ -print-file-name=libasan.so)\${LD_PRELOAD:+:\$LD_PRELOAD}" "$probe" "\$@"
WRAPPER
  chmod +x "$build/probe_sanitized_wrapper"
  probe="$build/probe_sanitized_wrapper"
  export ASAN_OPTIONS=detect_leaks=1:halt_on_error=1
  export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
fi
POLICY_OBSERVATION_ADAPTERS_PROBE="$probe" python3 -m pytest -q   "$native/test/test_policy_observation_adapters.py" > "$build/test$suffix.txt" 2>&1
cat "$build/test$suffix.txt"
