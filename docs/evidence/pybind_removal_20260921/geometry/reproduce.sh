#!/usr/bin/env bash
# Run from the repository root, using an unused work/evidence destination.
# No shared install, simulator or non-owned process is modified.
set -eo pipefail
repo="$(pwd)"
work="${GEOMETRY_REPLAY_WORK:-/tmp/geometry-replay-$(date +%s)}"
evidence="${GEOMETRY_REPLAY_OUTPUT:-$work/evidence}"
test ! -e "$work"
mkdir -p "$work" "$evidence"
source /opt/ros/humble/setup.bash
source "$repo/ws_robot/install/setup.bash"
package="$repo/ws_robot/src/astribot_s1_robot_geometry"
cp -a "$package" "$work/baseline_src"
cmake -S "$work/baseline_src" -B "$work/baseline_build" \
  -DCMAKE_BUILD_TYPE=Release -DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=ON -DBUILD_TESTING=OFF
cmake --build "$work/baseline_build" --target _geometry_native -j2
cp "$work/baseline_build/"_geometry_native*.so "$work/baseline_src/astribot_s1_robot_geometry/"
cmake -S "$package" -B "$work/cpp_build" -DCMAKE_BUILD_TYPE=Release \
  -DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX="$work/cpp_install"
cmake --build "$work/cpp_build" -j2
cmake --install "$work/cpp_build"
ctest --test-dir "$work/cpp_build" --output-on-failure -j1
python3 "$package/test/prepare_geometry_timing_probe.py" --output "$work/diagnostic_src"
cmake -S "$work/diagnostic_src" -B "$work/diagnostic_build" \
  -DCMAKE_BUILD_TYPE=Release -DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF -DBUILD_TESTING=OFF
cmake --build "$work/diagnostic_build" -j2
python3 "$package/test/prepare_geometry_timing_probe.py" --output "$work/fault_src" --compute-delay-ms 120
cmake -S "$work/fault_src" -B "$work/fault_build" \
  -DCMAKE_BUILD_TYPE=Release -DASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF -DBUILD_TESTING=OFF
cmake --build "$work/fault_build" -j2
export ROS_DOMAIN_ID=115 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONPATH="$work/baseline_src:${PYTHONPATH:-}"
python3 "$package/test/validate_geometry_completion_faults.py" \
  --cpp "$work/fault_build/geometry_state_timing" --diagnostic-source "$work/fault_src" \
  --output "$evidence/completion_faults"
# Coordinate a quiet CPU window with other users before the following two runs.
python3 "$package/test/benchmark_geometry_state_ab.py" \
  --python-root "$work/baseline_src" \
  --cpp "$work/cpp_install/lib/astribot_s1_robot_geometry/geometry_state" \
  --cpp-build-directory "$work/cpp_build" --output "$evidence/wall_ab" \
  --groups 3 --warmup 2 --seconds 6
python3 "$package/test/analyze_geometry_state_ab.py" "$evidence/wall_ab"
python3 "$package/test/geometry_state_phase_sweep.py" \
  --python-root "$work/baseline_src" --cpp "$work/diagnostic_build/geometry_state_timing" \
  --diagnostic-source "$work/diagnostic_src" --diagnostic-build "$work/diagnostic_build" \
  --output "$evidence/phase_sweep" --warmup 2 --seconds 3
