#!/usr/bin/env bash
set -euo pipefail
REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$REPO/scripts/env.sh"
drone_source_profile ros
CERES_ROOT="$DRONE_CERES_ROOT"
mkdir -p "$DRONE_RUNTIME_DIR/checks"
drone_prepend_path PKG_CONFIG_PATH "$DRONE_OPENCV_ROOT/lib/pkgconfig"
drone_prepend_path LD_LIBRARY_PATH "$DRONE_CUDA_ROOT/lib64"
drone_prepend_path LD_LIBRARY_PATH "$DRONE_OPENCV_ROOT/lib"
drone_prepend_path LD_LIBRARY_PATH "$CERES_ROOT/lib"
# pkg-config emits compiler flags; this installed OpenCV prefix has no spaces.
OPENCV_FLAG_TEXT=$(pkg-config --cflags --libs opencv)
read -r -a OPENCV_FLAGS <<< "$OPENCV_FLAG_TEXT"
g++ -std=c++14 -O2 -Wno-deprecated-declarations "$REPO/scripts/check_gpu.cpp" \
    -I"$CERES_ROOT/include" -I/usr/include/eigen3 \
    -L"$CERES_ROOT/lib" -Wl,-rpath,"$CERES_ROOT/lib" \
    "${OPENCV_FLAGS[@]}" -lceres -lglog -lgflags -lpthread \
    -o "$DRONE_RUNTIME_DIR/checks/check_gpu"
"$DRONE_RUNTIME_DIR/checks/check_gpu"
