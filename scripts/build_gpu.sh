#!/usr/bin/env bash
# Build against the Jetson CUDA installation, in a private prefix.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ "${DRONE_CLEAN_BUILD:-0}" != 1 ]]; then
    exec env -i HOME="$HOME" USER="${USER:-z}" \
      PATH=/usr/bin:/bin:/usr/local/cuda-10.2/bin \
      DRONE_CLEAN_BUILD=1 DRONE_BUILD_JOBS="${DRONE_BUILD_JOBS:-2}" \
      bash "$ROOT/scripts/build_gpu.sh" "$@"
fi
JOBS=${DRONE_BUILD_JOBS:-2}
mkdir -p "$ROOT/runtime"/{local,build,logs,workspaces}
[[ -d "$ROOT/runtime/opencv-src" ]] || cp -a "$ROOT/dependencies/opencv" "$ROOT/runtime/opencv-src"
[[ -d "$ROOT/runtime/ceres-src" ]] || cp -a "$ROOT/dependencies/ceres-solver-2.1.0" "$ROOT/runtime/ceres-src"
/usr/bin/python3 - "$ROOT/runtime/opencv-src" "$ROOT/runtime/ceres-src" <<'PY'
import os, sys, time
from pathlib import Path
now = time.time()
for directory in sys.argv[1:]:
    for path in Path(directory).rglob('*'):
        if path.is_file() and not path.is_symlink() and path.stat().st_mtime > now:
            os.utime(str(path), (now, now))
PY
case "${1:-all}" in
  opencv|all)
    modules=(core imgproc imgcodecs highgui calib3d features2d flann video videoio ml photo objdetect cudaarithm cudaimgproc cudaoptflow cudafilters cudawarping cudev)
    options=()
    for path in "$ROOT/runtime/opencv-src/modules/"*; do
      [[ -d "$path" ]] || continue
      name=${path##*/}
      enabled=OFF
      for wanted in "${modules[@]}"; do [[ "$name" != "$wanted" ]] || enabled=ON; done
      options+=("-DBUILD_opencv_${name}=$enabled")
    done
    mkdir -p "$ROOT/runtime/build/opencv"
    cd "$ROOT/runtime/build/opencv"
    cmake "$ROOT/runtime/opencv-src" \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ROOT/runtime/local/opencv" \
      -DWITH_CUDA=ON -DCUDA_ARCH_BIN=7.2 -DCUDA_ARCH_PTX= \
      -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 -DWITH_CUBLAS=ON -DENABLE_FAST_MATH=ON -DCUDA_FAST_MATH=ON \
      -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF \
      -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
      -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_GTK=ON -DWITH_QT=OFF -DWITH_VTK=OFF \
      -DWITH_OPENEXR=OFF -DWITH_IPP=OFF -DWITH_LAPACK=OFF -DWITH_TBB=ON \
      "${options[@]}"
    cmake --build . -- -j"$JOBS"
    cmake --build . --target install -- -j"$JOBS"
    [[ "${1:-all}" == all ]] || exit 0
    ;;&
  ceres|all)
    mkdir -p "$ROOT/runtime/build/ceres"
    cd "$ROOT/runtime/build/ceres"
    cmake "$ROOT/runtime/ceres-src" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$ROOT/runtime/local/ceres" -DCUDA=ON \
      -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 -DBUILD_SHARED_LIBS=ON \
      -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF
    cmake --build . -- -j"$JOBS"
    cmake --build . --target install -- -j"$JOBS"
    [[ "${1:-all}" == all ]] || exit 0
    ;;&
  vins|all)
    [[ -d "$ROOT/runtime/workspaces/vinsf_gpu_ws" ]] || cp -a "$ROOT/workspaces/vinsf_gpu_ws" "$ROOT/runtime/workspaces/"
    /usr/bin/python3 "$ROOT/scripts/prepare_workspace_scripts.py" vinsf_gpu_ws
    /usr/bin/python3 "$ROOT/scripts/prepare_vins.py"
    set +u
    source /opt/ros/melodic/setup.bash
    set -u
    export LD_LIBRARY_PATH="$ROOT/runtime/local/opencv/lib:$ROOT/runtime/local/ceres/lib:/usr/local/cuda-10.2/lib64:${LD_LIBRARY_PATH:-}"
    cd "$ROOT/runtime/workspaces/vinsf_gpu_ws"
    catkin_make -j"$JOBS" -l"$JOBS" -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python2 -DCATKIN_ENABLE_TESTING=OFF \
      -DCATKIN_WHITELIST_PACKAGES='cv_bridge;camera_models;vins;image_geometry' \
      -DOpenCV_DIR="$ROOT/runtime/local/opencv/share/OpenCV" \
      -DCeres_DIR="$ROOT/runtime/local/ceres/lib/cmake/Ceres" \
      -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 -DDRONE_CUDA_ARCH=72
    ;;
  *) echo 'Usage: build_gpu.sh [opencv|ceres|vins|all]' >&2; exit 2 ;;
esac
