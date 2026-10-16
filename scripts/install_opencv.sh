#!/usr/bin/env bash
# Restore Ubuntu's OpenCV 3.2 inside the project; the host dpkg database can
# report packages installed even after their shared libraries were removed.
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cache="$project_root/runtime/downloads/deb"
sysroot="$project_root/runtime/sysroot"
mkdir -p "$cache" "$sysroot"
packages=(libopencv-dev libopencv-ts-dev libopencv3.2-jni ros-melodic-cv-bridge)
for component in calib3d contrib core features2d flann highgui imgcodecs imgproc ml objdetect photo shape stitching superres video videoio videostab viz; do
  packages+=("libopencv-${component}-dev" "libopencv-${component}3.2")
done
missing=()
for package in "${packages[@]}"; do
  if ! compgen -G "$cache/${package}_*.deb" >/dev/null; then
    missing+=("$package")
  fi
done
if ((${#missing[@]})); then
  (cd -- "$cache" && apt-get download "${missing[@]}")
fi
for archive in "$cache"/libopencv*.deb; do
  dpkg-deb -x "$archive" "$sysroot"
done
for archive in "$cache"/ros-melodic-cv-bridge_*.deb; do
  dpkg-deb -x "$archive" "$sysroot"
done
python3 - "$sysroot" <<'PY'
import os, re, sys
prefix = os.path.join(sys.argv[1], 'usr')
config = os.path.join(prefix, 'share/OpenCV/OpenCVModules-release.cmake')
with open(config) as stream:
    paths = set(re.findall(r'\$\{_IMPORT_PREFIX\}([^";]+)', stream.read()))
missing = [prefix + path for path in paths if not os.path.exists(prefix + path)]
if missing:
    raise SystemExit('Missing OpenCV files:\n' + '\n'.join(sorted(missing)))
print('OpenCV 3.2 restored:', prefix)
# The host cv_bridgeConfig.cmake was manually changed to export OpenCV 4.8
# while libcv_bridge itself still uses 3.2. Use the original Debian package
# privately and relocate its absolute exported paths into this sysroot.
cv_config = os.path.join(sys.argv[1], 'opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake')
with open(cv_config) as stream:
    text = stream.read()
for old in ('/usr/lib/aarch64-linux-gnu', '/usr/include', '/opt/ros/melodic/lib'):
    text = text.replace(old, os.path.join(sys.argv[1], old.lstrip('/')))
with open(cv_config, 'w') as stream:
    stream.write(text)
print('Private cv_bridge configuration:', cv_config)
PY
