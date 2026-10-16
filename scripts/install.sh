#!/usr/bin/env bash
# Reproduce this Jetson/Melodic installation without overwriting system libraries.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ -f /opt/ros/melodic/setup.bash && -x /usr/local/cuda-10.2/bin/nvcc ]] || {
    echo 'This installer requires Ubuntu 18.04 ARM64, ROS Melodic and JetPack CUDA 10.2.' >&2
    exit 1
}
mkdir -p "$ROOT/runtime"/{workspaces,local/bin,logs,build}
for source in "$ROOT/workspaces/"*; do
    [[ -d "$ROOT/runtime/workspaces/${source##*/}" ]] || cp -a "$source" "$ROOT/runtime/workspaces/"
done
python3 - "$ROOT" <<'PY'
import json, os, sys, time, shlex
from pathlib import Path
root = Path(sys.argv[1])
kalibr = Path.home() / 'kalibr_workspace'
host_config = root / 'runtime/host.env'
if not host_config.exists() and (kalibr / 'devel/setup.bash').is_file():
    host_config.write_text('DRONE_KALIBR_WORKSPACE=' + shlex.quote(str(kalibr)) + '\n')
for entry in json.load(open(str(root / 'metadata/SYMLINKS.json'))):
    if not entry['destination'].startswith('workspaces/'):
        continue
    path = root / 'runtime' / entry['destination']
    if path.is_file() and not path.is_symlink() and path.read_bytes() == entry['target'].encode():
        path.unlink()
        path.symlink_to(entry['target'])
now = time.time()
for path in (root / 'runtime/workspaces').rglob('*'):
    if path.is_file() and not path.is_symlink() and path.stat().st_mtime > now:
        os.utime(str(path), (now, now))
PY
bash "$ROOT/scripts/install_user_packages.sh"
bash "$ROOT/scripts/install_opencv.sh"
bash "$ROOT/scripts/build_camera.sh"
bash "$ROOT/scripts/build_control.sh"
bash "$ROOT/scripts/build_gpu.sh" all
bash "$ROOT/scripts/check_ceres_cuda.sh"
mkdir -p "$ROOT/runtime/build/gpu-check"
cd "$ROOT/runtime/build/gpu-check"
cmake "$ROOT/scripts/gpu-check" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$ROOT/runtime/local" \
    -DOpenCV_DIR="$ROOT/runtime/local/opencv/share/OpenCV" \
    -DCeres_DIR="$ROOT/runtime/local/ceres/lib/cmake/Ceres"
cmake --build . --target install -- -j2
/usr/local/cuda-10.2/bin/nvcc -arch=sm_72 "$ROOT/scripts/cuda_device_check.cu" -o "$ROOT/runtime/local/bin/drone_cuda_check"
bash "$ROOT/scripts/check_kalibr.sh"
bash "$ROOT/scripts/install_shell.sh"
bash "$ROOT/scripts/doctor.sh"
