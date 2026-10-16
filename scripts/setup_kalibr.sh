#!/usr/bin/env bash
# Build the migrated ROS Melodic/Python 2 Kalibr in a separate user workspace.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
kalibr_ws="${DRONE_KALIBR_WS:-$HOME/kalibr_ws}"
native_prefix="${DRONE_NATIVE_PREFIX:-$HOME/third_party/install}"
package_cache="${DRONE_KALIBR_PACKAGE_CACHE:-$HOME/third_party/kalibr-debs}"
log_dir="${DRONE_ADAPT_LOG_DIR:-$HOME/drone_nav_adapt/logs}"
mode="${1:---all}"
case "$mode" in
  --all|--deps-only|--build-only|--verify) ;;
  *) printf 'Usage: %s [--all|--deps-only|--build-only|--verify]\n' "$0" >&2; exit 2 ;;
esac
mkdir -p "$native_prefix" "$package_cache" "$log_dir"

runtime_environment() {
  export PATH="$native_prefix/usr/bin:$PATH"
  export LD_LIBRARY_PATH="$native_prefix/usr/lib/$(gcc -dumpmachine)${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  if [[ -f /usr/lib/$(gcc -dumpmachine)/libgomp.so.1 ]]; then
    export LD_PRELOAD="/usr/lib/$(gcc -dumpmachine)/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
  fi
  export PYTHONPATH="$native_prefix/usr/lib/python2.7/dist-packages${PYTHONPATH:+:$PYTHONPATH}:/usr/lib/python2.7/dist-packages"
  export MATPLOTLIBDATA="$native_prefix/usr/share/matplotlib/mpl-data"
  export TEXMFCNF="$native_prefix/etc/texmf/web2c"
}

install_dependencies() {
  command -v apt-get >/dev/null
  command -v dpkg-deb >/dev/null
  # Resolve against the host's package database, then unpack only additions into
  # the user's prefix. This never runs package maintainer scripts or sudo.
  LC_ALL=C apt-get -s --no-install-recommends install \
    python-scipy python-matplotlib python-igraph python-pyx \
    texlive-latex-base texlive-fonts-recommended dvipng \
    > "$package_cache/apt-plan.txt"
  awk '/^Inst / { print $2 }' "$package_cache/apt-plan.txt" > "$package_cache/packages.txt"
  while read -r package; do
    if ! compgen -G "$package_cache/${package}_*.deb" >/dev/null; then
      (cd "$package_cache" && apt-get download "$package")
    fi
  done < "$package_cache/packages.txt"
  shopt -s nullglob
  local debs=("$package_cache"/*.deb)
  if ((${#debs[@]})); then
    for package_file in "${debs[@]}"; do
      dpkg-deb -x "$package_file" "$native_prefix"
    done
    (cd "$package_cache" && sha256sum ./*.deb > SHA256SUMS)
  fi
  # Debian generates namespace package __init__.py files in its post-install
  # hook. User-level extraction needs the equivalent for Python 2 explicitly.
  local backports_dir="$native_prefix/usr/lib/python2.7/dist-packages/backports"
  if [[ -d "$backports_dir" && ! -e "$backports_dir/__init__.py" ]]; then
    printf '%s\n' 'from pkgutil import extend_path' \
      '__path__ = extend_path(__path__, __name__)' > "$backports_dir/__init__.py"
  fi
  configure_tex
}

configure_tex() {
  # Debian TeX normally writes formats and caches via root package hooks.
  # Relocate its configuration and generate only the formats Kalibr uses.
  python3 - "$native_prefix" <<'PY'
from pathlib import Path
import os
import sys
prefix = Path(sys.argv[1])
# Package symlinks for TeX caches and configuration are absolute on Debian.
# Relocate these before mktexlsr, without touching targets under the host root.
for link in prefix.rglob('*'):
    if link.is_symlink():
        target = os.readlink(str(link))
        if str(target).startswith(('/var/lib/texmf/', '/etc/texmf/',
                                   '/usr/share/texlive/', '/usr/share/texmf/')):
            relocated = prefix / str(target).lstrip('/')
            relocated.parent.mkdir(parents=True, exist_ok=True)
            link.unlink()
            link.symlink_to(relocated)
source = prefix / 'usr/share/texlive/texmf-dist/web2c/texmf.cnf'
if not source.exists():
    raise SystemExit('The local TeX packages are missing. Run --deps-only first.')
config = prefix / 'etc/texmf/web2c/texmf.cnf'
config.parent.mkdir(parents=True, exist_ok=True)
text = source.read_text()
for old in ['/usr/share/texlive', '/usr/share/texmf', '/usr/local/share/texmf',
            '/var/lib/texmf', '/etc/texmf']:
    text = text.replace(old, str(prefix) + old)
text = text.replace('TEXMFVAR = ~/.texlive2017/texmf-var',
                    'TEXMFVAR = ' + str(prefix) + '/var/lib/texmf')
text = text.replace('TEXMFCONFIG = ~/.texlive2017/texmf-config',
                    'TEXMFCONFIG = ' + str(prefix) + '/etc/texmf')
config.write_text(text)
language = prefix / 'var/lib/texmf/tex/generic/config/language.dat'
language.parent.mkdir(parents=True, exist_ok=True)
language.write_text('english hyphen.tex\n')
(prefix / 'var/lib/texmf/web2c').mkdir(parents=True, exist_ok=True)
PY
  runtime_environment
  mktexlsr "$native_prefix/usr/share/texlive/texmf-dist" \
    "$native_prefix/usr/share/texmf" "$native_prefix/var/lib/texmf"
  (
    cd "$native_prefix/var/lib/texmf/web2c"
    # PyX talks to TeX interactively, so save formats with errorstopmode.
    tex -ini -interaction=errorstopmode -jobname=tex plain '\dump' \
      < /dev/null > "$log_dir/kalibr-tex-init.log" 2>&1
    pdftex -ini -etex -interaction=errorstopmode -jobname=latex -progname=latex \
      '\pdfoutput=0 \input latex.ltx' < /dev/null > "$log_dir/kalibr-latex-init.log" 2>&1
  )
  mktexlsr "$native_prefix/var/lib/texmf"
}

build_kalibr() {
  command -v catkin >/dev/null
  [[ -f /opt/ros/melodic/setup.bash ]] || {
    printf 'This migrated workspace needs ROS Melodic.\n' >&2; exit 1;
  }
  if [[ ! -d "$kalibr_ws/src/kalibr" ]]; then
    mkdir -p "$kalibr_ws/src"
    cp -a "$repo_root/workspaces/kalibr_ws/src/kalibr" "$kalibr_ws/src/kalibr"
  fi
  # The migrated numpy_eigen unconditionally builds 246 extra test bindings.
  # Respect CATKIN_ENABLE_TESTING in the working copy to save ARM build time.
  python3 - "$kalibr_ws" <<'PY'
from pathlib import Path
import re
import sys
root = Path(sys.argv[1]) / 'src/kalibr/Schweizer-Messer/numpy_eigen'
cmake = root / 'CMakeLists.txt'
text = cmake.read_text()
snapshot = root / 'CMakeLists.txt.pre-drone-adaptation'
if not snapshot.exists():
    snapshot.write_text(text)
block = '''INCLUDE(src/autogen_test_files.cmake)
add_python_export_library(${PROJECT_NAME}_test\x20
      ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${AUTOGEN_TEST_FILES}
  )'''
if block in text and 'if(CATKIN_ENABLE_TESTING)\n' + block not in text:
    text = text.replace(block, 'if(CATKIN_ENABLE_TESTING)\n' + block + '\nendif()')
# Repeated Boost/Eigen header parsing dominates these 245 tiny generated TUs.
# Keep the NumPy module entry point separate; it owns the NumPy C API table.
# Each converter is included unchanged in groups of eight, with no types cut.
imports = sorted((root / 'src/autogen_module').glob('import_*.cpp'))
pattern = (r'// This file automatically generated by create_export_module.py\n'
           r'#define NO_IMPORT_ARRAY\s*\n#include <NumpyEigenConverter.hpp>\s*\n'
           r'#include <boost/cstdint.hpp>\s*\nvoid (import_\w+)\(\)\s*\{\s*'
           r'NumpyEigenConverter<Eigen::Matrix<\s*'
           r'(?:int|float|double|boost::uint8_t|boost::int64_t),\s*'
           r'(?:\d+|Eigen::Dynamic),\s*(?:\d+|Eigen::Dynamic)\s*'
           r'> >::register_converter\(\);\s*\}\s*')
matches = [re.fullmatch(pattern, path.read_text()) for path in imports]
if len(imports) != 245 or not all(matches):
    raise SystemExit('Unexpected numpy_eigen generated sources; refusing unity adaptation.')
assert len({match.group(1) for match in matches}) == len(imports)
unity = root / 'src/autogen_module_unity'
unity.mkdir(exist_ok=True)
sources = ['src/autogen_module/numpy_eigen_export_module.cpp']
for start in range(0, len(imports), 8):
    batch = unity / ('batch_%03d.cpp' % (start // 8))
    content = '// Local build: preserve every original converter function.\n'
    content += ''.join('#include "../autogen_module/%s"\n' % path.name
                       for path in imports[start:start + 8])
    if not batch.exists() or batch.read_text() != content:
        batch.write_text(content)
    sources.append(str(batch.relative_to(root)))
generated = root / 'src/autogen_files_unity.cmake'
content = 'SET(AUTOGEN_FILES\n' + ''.join('  %s\n' % source for source in sources) + ')\n'
if not generated.exists() or generated.read_text() != content:
    generated.write_text(content)
text = text.replace('INCLUDE(src/autogen_files.cmake)', 'INCLUDE(src/autogen_files_unity.cmake)')
if cmake.read_text() != text:
    cmake.write_text(text)
PY
  # Do not inherit a VINS/OpenCV/Ceres workspace into this independent build.
  unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
  unset PYTHONPATH LD_LIBRARY_PATH
  set +u
  source /opt/ros/melodic/setup.bash
  set -u
  runtime_environment
  local compat_root="${DRONE_ADAPT_DIR:-$HOME/drone_nav_adapt}/ros_compat"
  local compat_args=()
  if [[ -d "$compat_root/include" ]]; then
    compat_args+=("-Dcv_bridge_DIR=$compat_root/cv_bridge/cmake")
    compat_args+=("-DCMAKE_CXX_FLAGS=-I$compat_root/include -Wno-deprecated-declarations")
  else
    compat_args+=("-DCMAKE_CXX_FLAGS=-Wno-deprecated-declarations")
  fi
  catkin config --workspace "$kalibr_ws" --extend /opt/ros/melodic \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
      -DPYTHON_EXECUTABLE=/usr/bin/python2 -DOpenCV_DIR=/usr/share/OpenCV \
      "${compat_args[@]}" \
    --make-args -j"${DRONE_KALIBR_JOBS:-2}"
  PYTHONUNBUFFERED=1 catkin build --workspace "$kalibr_ws" -p 1 -j "${DRONE_KALIBR_JOBS:-2}" \
    --no-status --summarize > "$log_dir/kalibr-build.log" 2>&1 || {
      tail -n 100 "$log_dir/kalibr-build.log" >&2; return 1;
    }
}

verify_kalibr() {
  [[ -f "$kalibr_ws/devel/setup.bash" ]] || {
    printf 'Kalibr has not been built. Run --build-only first.\n' >&2; exit 1;
  }
  unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
  unset PYTHONPATH LD_LIBRARY_PATH
  set +u
  source /opt/ros/melodic/setup.bash
  source "$kalibr_ws/devel/setup.bash"
  set -u
  runtime_environment
  MPLBACKEND=Agg python2 - <<'PY'
import numpy, scipy, matplotlib, igraph, pyx
import cv2
from cv_bridge import CvBridge
import aslam_cv, aslam_backend, kalibr_common, sm
transform = numpy.eye(4)
transform[:3, 3] = [0.1, -0.2, 0.3]
pose = sm.Transformation(transform)
numpy.testing.assert_allclose(pose.T(), transform)
numpy.testing.assert_allclose(pose.inverse().T().dot(transform), numpy.eye(4), atol=1e-12)
point = numpy.array([1.0, 2.0, 3.0])
numpy.testing.assert_allclose(pose * point, point + transform[:3, 3])
bridge = CvBridge()
image = numpy.zeros((4, 6, 3), dtype=numpy.uint8)
message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
assert bridge.imgmsg_to_cv2(message, 'bgr8').shape == image.shape
opencv = set(line.split()[-1] for line in open('/proc/self/maps')
             if 'libopencv_' in line)
assert opencv and all('.so.3.2' in path for path in opencv), sorted(opencv)
print('Kalibr C++ bindings and Python 2 dependencies imported successfully.')
print('NumPy/Eigen matrix and vector conversions verified.')
print('cv_bridge image conversion and OpenCV 3.2 ABI verified.')
PY
  # These legacy argparse entry points print help and return 2 when required
  # calibration inputs are absent. Validate the useful output as well as the
  # accepted exit status so --verify remains meaningful on ROS Melodic.
  local help_log
  for help_log in kalibr-camera-help kalibr-imu-help; do
    set +e
    if [[ "$help_log" == kalibr-camera-help ]]; then
      MPLBACKEND=Agg rosrun kalibr kalibr_calibrate_cameras --help > "$log_dir/$help_log.txt" 2>&1
    else
      MPLBACKEND=Agg rosrun kalibr kalibr_calibrate_imu_camera --help > "$log_dir/$help_log.txt" 2>&1
    fi
    local help_status=$?
    set -e
    [[ "$help_status" == 0 || "$help_status" == 2 ]] || return "$help_status"
    grep -q 'usage:' "$log_dir/$help_log.txt"
  done
  rosrun kalibr kalibr_create_target_pdf "$log_dir/kalibr-target-smoke" \
    --type apriltag --nx 2 --ny 2 --tsize 0.04 --tspace 0.3 \
    > "$log_dir/kalibr-target-smoke.log" 2>&1
  [[ -s "$log_dir/kalibr-target-smoke.pdf" ]]
  printf 'Kalibr verification passed. Logs: %s\n' "$log_dir"
}

if [[ "$mode" == --all || "$mode" == --deps-only ]]; then install_dependencies; fi
if [[ "$mode" == --all || "$mode" == --build-only ]]; then build_kalibr; fi
if [[ "$mode" == --all || "$mode" == --verify ]]; then verify_kalibr; fi
