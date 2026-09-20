#!/usr/bin/env bash
# Build the repository's numerical dependency for the current host architecture.
set -euo pipefail
SLAM_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
SLAM_DEP_PREFIX="${ASTRIBOT_SLAM_DEP_PREFIX:-$SLAM_PROJECT_ROOT/ws_robot/deps/gtsam}"
SLAM_DEP_BUILD="$SLAM_PROJECT_ROOT/ws_robot/deps/build_gtsam"
mkdir -p "$SLAM_PROJECT_ROOT/ws_robot/deps"
touch "$SLAM_PROJECT_ROOT/ws_robot/deps/COLCON_IGNORE"
SLAM_EIGEN_PREFIX="$SLAM_PROJECT_ROOT/ws_robot/deps/eigen"
cmake -S "$SLAM_PROJECT_ROOT/ws_robot/src/astribot_eigen_vendor" \
    -B "$SLAM_PROJECT_ROOT/ws_robot/deps/build_eigen" \
    -DASTRIBOT_EIGEN_STANDALONE=ON -DCMAKE_INSTALL_PREFIX="$SLAM_EIGEN_PREFIX"
cmake --install "$SLAM_PROJECT_ROOT/ws_robot/deps/build_eigen"
cmake -S "$SLAM_PROJECT_ROOT/ws_robot/third_party/gtsam" \
    -B "$SLAM_DEP_BUILD" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$SLAM_DEP_PREFIX" \
    -DEigen3_DIR="$SLAM_EIGEN_PREFIX/share/eigen3/cmake" \
    -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DCMAKE_INSTALL_RPATH="$SLAM_DEP_PREFIX/lib"
cmake --build "$SLAM_DEP_BUILD" --parallel "${ASTRIBOT_BUILD_JOBS:-2}"
cmake --install "$SLAM_DEP_BUILD"
echo "GTSAM_DIR=$SLAM_DEP_PREFIX/lib/cmake/GTSAM"
