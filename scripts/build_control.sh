#!/usr/bin/env bash
# Build the hardware control/planning workspace without inheriting VINS/OpenCV overlays.
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
jobs=${BUILD_JOBS:-2}
python3 "$project_root/scripts/prepare_workspace_scripts.py" Fast-Drone-250-master
python3 "$project_root/scripts/prepare_control.py" control
exec env -i HOME="$HOME" USER="${USER:-z}" PATH=/usr/bin:/bin \
  PROJECT_ROOT="$project_root" BUILD_JOBS="$jobs" \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/melodic/setup.bash
    export LD_LIBRARY_PATH="$PROJECT_ROOT/runtime/sysroot/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
    cd "$PROJECT_ROOT/runtime/workspaces/Fast-Drone-250-master"
    exec catkin_make -j"$BUILD_JOBS" -l"$BUILD_JOBS" \
      -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="$PROJECT_ROOT/runtime/sysroot/usr/share/OpenCV" \
      -Dcv_bridge_DIR="$PROJECT_ROOT/runtime/sysroot/opt/ros/melodic/share/cv_bridge/cmake" \
      -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python2.7 \
      "-DCATKIN_WHITELIST_PACKAGES=px4ctrl;quadrotor_msgs;uav_utils;cmake_utils;ego_planner;plan_env;path_searching;bspline_opt;traj_utils;rosmsg_tcp_bridge;drone_detect"
  '
