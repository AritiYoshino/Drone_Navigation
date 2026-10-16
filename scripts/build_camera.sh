#!/usr/bin/env bash
# Build the migrated camera driver and vision bridge against the system ROS OpenCV.
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
jobs=${BUILD_JOBS:-2}
python3 "$project_root/scripts/prepare_workspace_scripts.py" realsense_ws
python3 "$project_root/scripts/prepare_control.py" camera
exec env -i HOME="$HOME" USER="${USER:-z}" PATH=/usr/bin:/bin \
  PROJECT_ROOT="$project_root" BUILD_JOBS="$jobs" \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/melodic/setup.bash
    export LD_LIBRARY_PATH="$PROJECT_ROOT/runtime/sysroot/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
    cd "$PROJECT_ROOT/runtime/workspaces/realsense_ws"
    exec catkin_make -j"$BUILD_JOBS" -l"$BUILD_JOBS" \
      -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="$PROJECT_ROOT/runtime/sysroot/usr/share/OpenCV" \
      -Dcv_bridge_DIR="$PROJECT_ROOT/runtime/sysroot/opt/ros/melodic/share/cv_bridge/cmake" \
      -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
      -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python2.7
  '
