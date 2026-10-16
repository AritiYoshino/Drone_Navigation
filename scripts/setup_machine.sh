#!/usr/bin/env bash
# Reuse this Jetson's BSP/CUDA/ROS and build isolated, local workspaces.
set -euo pipefail
REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$REPO/scripts/env.sh"
MODE=${1:---build}
case "$MODE" in
    --configure-only|--build) ;;
    *) echo "Usage: $0 [--configure-only|--build]" >&2; exit 2 ;;
esac
JOBS=${DRONE_BUILD_JOBS:-2}
ADAPT="$DRONE_WORK_ROOT/drone_nav_adapt"
mkdir -p "$ADAPT/logs" "$REPO/config" "$DRONE_RUNTIME_DIR" "$HOME/.local/bin"
for pair in "realsense_ws:$DRONE_RS_WS" "vinsf_gpu_ws:$DRONE_VINS_WS" \
            "Fast-Drone-250-master:$DRONE_FAST_WS" "kalibr_ws:$DRONE_KALIBR_WS"; do
    name=${pair%%:*}; target=${pair#*:}
    if [[ ! -d "$target/src" ]]; then
        [[ ! -e "$target" ]] || { echo "Existing incomplete workspace: $target" >&2; exit 1; }
        cp -a "$REPO/workspaces/$name" "$target"
    fi
done
python3 - "$DRONE_RS_WS" "$DRONE_VINS_WS" "$DRONE_FAST_WS" "$DRONE_KALIBR_WS" <<'PY'
from pathlib import Path
import os, sys, time
now = time.time()
for root in sys.argv[1:]:
    for path in Path(root, 'src').rglob('*'):
        if path.is_symlink() or not path.is_file():
            continue
        info = path.stat()
        if info.st_mtime > now + 1:
            os.utime(str(path), (info.st_atime, now))
PY
# Never replace an existing machine's connection settings.
if [[ ! -f "$REPO/config/local.env" ]]; then
    {
        printf '# Local Jetson configuration; USB FCU default. For UART use /dev/ttyTHS0:921600.\n'
        for variable in DRONE_WORK_ROOT DRONE_ROS_ROOT DRONE_RS_WS DRONE_VINS_WS DRONE_FAST_WS \
                        DRONE_KALIBR_WS DRONE_NATIVE_PREFIX DRONE_CERES_ROOT DRONE_OPENCV_ROOT \
                        DRONE_CUDA_ROOT DRONE_FCU_URL; do
            printf 'export %s=%q\n' "$variable" "${!variable}"
        done
    } > "$REPO/config/local.env"
fi
ln -sfn "$REPO/scripts/drone_nav.sh" "$HOME/.local/bin/drone-nav"
python3 - "$HOME/.bashrc" "$ADAPT" <<'PY'
from pathlib import Path
import shutil, sys
path, backup = Path(sys.argv[1]), Path(sys.argv[2]) / 'bashrc.before-drone-nav'
if path.exists() and not backup.exists():
    shutil.copy2(str(path), str(backup))
text = path.read_text() if path.exists() else ''
# Loading arbitrary old catkin overlays in every terminal contaminates builds.
lines = []
for line in text.splitlines():
    if line.strip().startswith('source ') and any(item in line for item in
          ['realsense_ws/devel/setup.bash', 'Prometheus/devel/setup.bash',
           'rplidar_ws/devel/setup.bash', 'amovlab_ws/devel/setup.bash']):
        line = '# drone-nav: load a named profile explicitly: ' + line
    lines.append(line)
text = '\n'.join(lines) + '\n'
if '# >>> drone-nav local commands >>>' not in text:
    text += '''
# >>> drone-nav local commands >>>
case ":$PATH:" in
    *":$HOME/.local/bin:"*) ;;
    *) export PATH="$HOME/.local/bin:$PATH" ;;
esac
# <<< drone-nav local commands <<<
'''
path.write_text(text)
PY
if [[ "$MODE" == --configure-only ]]; then
    echo "Local paths and drone-nav command configured. Open a new terminal."
    exit 0
fi
[[ -x "$DRONE_CUDA_ROOT/bin/nvcc" && -r "$DRONE_ROS_ROOT/setup.bash" ]] || {
    echo "Install the matching Jetson CUDA toolkit and ROS Melodic first." >&2; exit 1;
}
GPU_ARCH=$(python3 - <<'PY'
import ctypes
c = ctypes.CDLL('libcudart.so.10.2')
n, a, b = ctypes.c_int(), ctypes.c_int(), ctypes.c_int()
assert c.cudaGetDeviceCount(ctypes.byref(n)) == 0 and n.value > 0
assert c.cudaDeviceGetAttribute(ctypes.byref(a), 75, 0) == 0
assert c.cudaDeviceGetAttribute(ctypes.byref(b), 76, 0) == 0
print(str(a.value) + str(b.value))
PY
)
if [[ ! -f "$DRONE_CERES_ROOT/lib/libceres.so" ]]; then
    mkdir -p "$ADAPT/build/ceres"
    (
        cd "$ADAPT/build/ceres"
        cmake "$REPO/dependencies/ceres-solver-2.1.0" \
            -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$DRONE_CERES_ROOT" \
            -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
            -DSUITESPARSE=OFF -DCUDA=ON \
            -DCUDA_TOOLKIT_ROOT_DIR="$DRONE_CUDA_ROOT" \
            -DCUDA_NVCC_FLAGS="-gencode arch=compute_$GPU_ARCH,code=sm_$GPU_ARCH"
        make -j"$JOBS"
        make install
    ) > "$ADAPT/logs/ceres-setup.log" 2>&1
fi
# Patch only the working copy, retaining the first original of each changed file.
python3 - "$DRONE_VINS_WS" "$DRONE_CERES_ROOT" "$DRONE_OPENCV_ROOT" "$DRONE_CUDA_ROOT" "$GPU_ARCH" <<'PY'
from pathlib import Path
import re, shutil, sys
root, ceres, opencv, cuda, arch = sys.argv[1:]
values = {'OpenCV_DIR': opencv + '/share/OpenCV',
          'Ceres_DIR': ceres + '/lib/cmake/Ceres',
          'CMAKE_INSTALL_RPATH': ceres + '/lib;' + cuda + '/lib64'}
for path in Path(root, 'src').rglob('CMakeLists.txt'):
    text = before = path.read_text()
    for name, value in values.items():
        text = re.sub(r'set\(' + name + r'\s+"[^"]*"\)',
                      'set({} "{}")'.format(name, value), text)
    text = re.sub(r'arch=compute_\d+,code=sm_\d+',
                  'arch=compute_{0},code=sm_{0}'.format(arch), text)
    if text != before:
        backup = path.with_name(path.name + '.before-drone-nav')
        if not backup.exists():
            shutil.copy2(str(path), str(backup))
        path.write_text(text)
PY
python3 - "$DRONE_RS_WS" <<'PY'
from pathlib import Path
import re, shutil, sys
root = Path(sys.argv[1], 'src/realsense-ros/realsense2_camera')
for path in root.rglob('*.h'):
    before = path.read_text()
    text = re.sub(r'#include\s+"/usr/local/include/(librealsense2/[^\"]+)"',
                  r'#include <\1>', before)
    if text != before:
        backup = path.with_name(path.name + '.before-drone-nav')
        if not backup.exists():
            shutil.copy2(str(path), str(backup))
        path.write_text(text)
PY
# Repair this machine's inconsistent cv_bridge include export locally.
COMPAT="$ADAPT/ros_compat/cv_bridge/cmake"
mkdir -p "$COMPAT"
mkdir -p "$ADAPT/ros_compat/include"
ln -sfn /usr/include/opencv2 "$ADAPT/ros_compat/include/opencv2"
ln -sfn /usr/include/opencv "$ADAPT/ros_compat/include/opencv"
cp "$DRONE_ROS_ROOT/share/cv_bridge/cmake/"*.cmake "$COMPAT/"
python3 - "$COMPAT" "$DRONE_ROS_ROOT" "$ADAPT/ros_compat/include" <<'PY'
from pathlib import Path
import re, sys
for path in Path(sys.argv[1]).glob('*.cmake'):
    text = path.read_text()
    includes = sys.argv[2] + '/include;' + sys.argv[3]
    text = text.replace('include;/usr/local/include;/usr/local/include/opencv2', includes)
    if path.name == 'cv_bridgeConfig.cmake':
        text, count = re.subn(r'set\(_include_dirs\s+"[^"]*"\)',
                             'set(_include_dirs "' + includes + '")', text)
        if count != 1:
            raise SystemExit('Unexpected installed cv_bridge config: ' + str(path))
    path.write_text(text)
PY
build_workspace() (
    name=$1; ws=$2; shift 2
    unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
    set +u
    source "$DRONE_ROS_ROOT/setup.bash"
    set -u
    export PATH="$DRONE_CUDA_ROOT/bin:$PATH"
    cd "$ws"
    catkin_make -j"$JOBS" -l3 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python2 "$@" \
        > "$ADAPT/logs/$name-setup.log" 2>&1
)
echo 'Building GPU VINS (OpenCV 3.3) ...'
build_workspace vins "$DRONE_VINS_WS" \
    -DOpenCV_DIR="$DRONE_OPENCV_ROOT/share/OpenCV" \
    -DCeres_DIR="$DRONE_CERES_ROOT/lib/cmake/Ceres"
echo 'Building planning/control and camera (OpenCV 3.2) ...'
build_workspace fast "$DRONE_FAST_WS" -DOpenCV_DIR=/usr/share/OpenCV \
    -Dcv_bridge_DIR="$COMPAT" -DCATKIN_ENABLE_TESTING=ON \
    -DCMAKE_CXX_FLAGS="-I$ADAPT/ros_compat/include"
build_workspace realsense "$DRONE_RS_WS" -DOpenCV_DIR=/usr/share/OpenCV \
    -Dcv_bridge_DIR="$COMPAT" -Drealsense2_DIR="$DRONE_ROS_ROOT/lib/cmake/realsense2" \
    -DCMAKE_CXX_FLAGS="-I$ADAPT/ros_compat/include" -DCATKIN_ENABLE_TESTING=OFF
bash "$REPO/scripts/setup_kalibr.sh"
bash "$REPO/scripts/check_gpu.sh"
python3 "$REPO/scripts/doctor.py"
