#!/usr/bin/env bash
# Source this file, then call drone_source_profile <ros|camera|vins|fast|kalibr>.
# Each profile starts with an independent ROS/OpenCV environment.

DRONE_REPO_ROOT="${DRONE_REPO_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)}"
if [[ -r "$DRONE_REPO_ROOT/config/local.env" ]]; then
    source "$DRONE_REPO_ROOT/config/local.env"
fi
: "${DRONE_WORK_ROOT:=$HOME}"
: "${DRONE_ROS_ROOT:=/opt/ros/melodic}"
: "${DRONE_RS_WS:=$DRONE_WORK_ROOT/realsense_ws}"
: "${DRONE_VINS_WS:=$DRONE_WORK_ROOT/vinsf_gpu_ws}"
: "${DRONE_FAST_WS:=$DRONE_WORK_ROOT/Fast-Drone-250-master}"
: "${DRONE_KALIBR_WS:=$DRONE_WORK_ROOT/kalibr_ws}"
: "${DRONE_NATIVE_PREFIX:=$DRONE_WORK_ROOT/third_party/install}"
: "${DRONE_CERES_ROOT:=$DRONE_WORK_ROOT/third_party/ceres-cuda}"
: "${DRONE_OPENCV_ROOT:=/usr/local}"
: "${DRONE_CUDA_ROOT:=/usr/local/cuda}"
: "${DRONE_RUNTIME_DIR:=$DRONE_REPO_ROOT/.runtime}"
: "${DRONE_FCU_URL:=/dev/ttyACM0:57600}"
: "${DRONE_CAMERA_SERIAL:=}"
: "${DRONE_TOPIC_TIMEOUT:=120}"
: "${DRONE_VINS_CONFIG:=$DRONE_VINS_WS/src/VINS-Fusion/config/realsense_d435i/px4_d435_vins_gpu.yaml}"
if [[ -z "${DRONE_VINS_DEVEL:-}" ]]; then
    if [[ -f "$DRONE_VINS_WS/devel_gpu/setup.bash" ]]; then
        DRONE_VINS_DEVEL="$DRONE_VINS_WS/devel_gpu"
    else
        DRONE_VINS_DEVEL="$DRONE_VINS_WS/devel"
    fi
fi
export DRONE_REPO_ROOT DRONE_WORK_ROOT DRONE_ROS_ROOT DRONE_RS_WS DRONE_VINS_WS
export DRONE_FAST_WS DRONE_KALIBR_WS DRONE_NATIVE_PREFIX DRONE_CERES_ROOT
export DRONE_OPENCV_ROOT DRONE_CUDA_ROOT DRONE_RUNTIME_DIR DRONE_VINS_DEVEL
export DRONE_FCU_URL DRONE_CAMERA_SERIAL DRONE_TOPIC_TIMEOUT DRONE_VINS_CONFIG

drone_prepend_path() {
    local variable="$1" directory="$2" current
    [[ -d "$directory" ]] || return 0
    current="${!variable-}"
    printf -v "$variable" '%s' "$directory${current:+:$current}"
    export "$variable"
}

drone_source_profile() {
    local profile="${1:-ros}" overlay="" source_directory="" nounset_was_set=""
    case "$profile" in
        ros) ;;
        camera) overlay="$DRONE_RS_WS/devel"; source_directory="$DRONE_RS_WS/src" ;;
        vins) overlay="$DRONE_VINS_DEVEL"; source_directory="$DRONE_VINS_WS/src" ;;
        fast) overlay="$DRONE_FAST_WS/devel"; source_directory="$DRONE_FAST_WS/src" ;;
        kalibr) overlay="$DRONE_KALIBR_WS/devel"; source_directory="$DRONE_KALIBR_WS/src" ;;
        *) printf 'Unknown environment profile: %s\n' "$profile" >&2; return 2 ;;
    esac
    [[ -r "$DRONE_ROS_ROOT/setup.bash" ]] || {
        printf 'Missing ROS setup: %s/setup.bash\n' "$DRONE_ROS_ROOT" >&2; return 1;
    }
    [[ -z "$overlay" || -r "$overlay/setup.bash" ]] || {
        printf 'Workspace has not been built: %s/setup.bash\n' "$overlay" >&2; return 1;
    }
    # Old .bashrc files may already have sourced several incompatible workspaces.
    # Do not import those paths into a camera, controller or GPU VINS process.
    unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
    unset LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH LD_PRELOAD
    unset ROS_ROOT ROS_ETC_DIR ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
    unset CATKIN_SETUP_UTIL_ARGS _CATKIN_SETUP_DIR MATPLOTLIBDATA TEXMFCNF
    export PATH="$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    [[ $- != *u* ]] || { nounset_was_set=1; set +u; }
    if ! source "$DRONE_ROS_ROOT/setup.bash"; then
        [[ -z "$nounset_was_set" ]] || set -u
        return 1
    fi
    if [[ -n "$overlay" ]]; then
        CATKIN_SETUP_UTIL_ARGS="--extend --local"
        if ! source "$overlay/setup.bash"; then
            unset CATKIN_SETUP_UTIL_ARGS
            [[ -z "$nounset_was_set" ]] || set -u
            return 1
        fi
        unset CATKIN_SETUP_UTIL_ARGS
        # --local deliberately skips the base ROS package-path hook.
        drone_prepend_path ROS_PACKAGE_PATH "$source_directory"
    fi
    if [[ "$profile" == vins ]]; then
        drone_prepend_path LD_LIBRARY_PATH "$DRONE_CUDA_ROOT/lib64"
        drone_prepend_path LD_LIBRARY_PATH "$DRONE_OPENCV_ROOT/lib"
        drone_prepend_path LD_LIBRARY_PATH "$DRONE_CERES_ROOT/lib"
        drone_prepend_path PATH "$DRONE_CUDA_ROOT/bin"
    elif [[ "$profile" == kalibr ]]; then
        # The interpreter's default /usr/local path contains GPU OpenCV 3.3.
        # Kalibr's C++ bindings and ROS cv_bridge use the system OpenCV 3.2.
        export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}/usr/lib/python2.7/dist-packages"
        drone_prepend_path PYTHONPATH "$DRONE_NATIVE_PREFIX/usr/lib/python2.7/dist-packages"
        drone_prepend_path LD_LIBRARY_PATH "$DRONE_NATIVE_PREFIX/usr/lib/aarch64-linux-gnu"
        drone_prepend_path PATH "$DRONE_NATIVE_PREFIX/usr/bin"
        # ARM Python extensions using OpenMP may otherwise fail with a static
        # TLS allocation error when libgomp is loaded late in the process.
        if [[ -f /usr/lib/aarch64-linux-gnu/libgomp.so.1 ]]; then
            export LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
        fi
        if [[ -d "$DRONE_NATIVE_PREFIX/usr/share/matplotlib/mpl-data" ]]; then
            export MATPLOTLIBDATA="$DRONE_NATIVE_PREFIX/usr/share/matplotlib/mpl-data"
        fi
        if [[ -d "$DRONE_NATIVE_PREFIX/etc/texmf/web2c" ]]; then
            export TEXMFCNF="$DRONE_NATIVE_PREFIX/etc/texmf/web2c"
        fi
    fi
    export DRONE_ACTIVE_PROFILE="$profile"
    [[ -z "$nounset_was_set" ]] || set -u
}
