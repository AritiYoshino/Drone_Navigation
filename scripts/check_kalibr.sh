#!/usr/bin/env bash
# Exercise Kalibr's Python/C++ libraries and PDF generator without hardware.
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/env.sh" kalibr
export MPLBACKEND=Agg
mkdir -p "$PROJECT_ROOT/runtime/checks" "$PROJECT_ROOT/runtime/logs"
/usr/bin/python2 -B - <<'PY'
import numpy as np
import scipy.linalg
import cv2
import sm
import aslam_cv
import aslam_cv_backend
import aslam_cameras_april as april
import kalibr_camera_calibration
import kalibr_imu_camera_calibration
from cv_bridge import CvBridge

assert cv2.__version__.startswith('3.2.'), cv2.__version__
matrix = np.array([[4., 1.], [1., 3.]])
rhs = np.array([1., 2.])
solution = scipy.linalg.solve(matrix, rhs)
assert np.linalg.norm(matrix.dot(solution) - rhs) < 1e-12
target = april.GridCalibrationTargetAprilgrid(6, 6, .088, .3)
image = np.arange(15, dtype=np.uint8).reshape(3, 5)
bridge = CvBridge()
message = bridge.cv2_to_imgmsg(image, encoding='mono8')
color = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
assert color.shape == (3, 5, 3)
assert all(np.array_equal(color[:, :, channel], image) for channel in range(3))
print('Kalibr numeric solve, AprilGrid C++ object and cv_bridge conversion: PASS (OpenCV %s)' % cv2.__version__)
PY
cd -- "$PROJECT_ROOT/runtime/checks"
pdf_log="$PROJECT_ROOT/runtime/logs/kalibr_aprilgrid_test.log"
if ! timeout 90s rosrun kalibr kalibr_create_target_pdf \
    --type apriltag --nx 2 --ny 2 --tsize 0.088 --tspace 0.3 \
    kalibr_aprilgrid_test > "$pdf_log" 2>&1; then
    tail -60 "$pdf_log" >&2
    exit 1
fi
/usr/bin/python3 - "$PROJECT_ROOT/runtime/checks/kalibr_aprilgrid_test.pdf" <<'PY'
import os, sys
filename = sys.argv[1]
with open(filename, 'rb') as stream:
    assert stream.read(5) == b'%PDF-'
assert os.path.getsize(filename) > 1000
print('Kalibr diagnostic PDF: PASS (%s)' % filename)
PY
