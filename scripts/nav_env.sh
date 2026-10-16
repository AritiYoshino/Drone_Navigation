#!/usr/bin/env bash
# Shared paths and isolated ROS profiles for the reference-compatible launcher.
# Sourcing this file alone does not start ROS or select a workspace.
_drone_nav_env_init() {
    local repo name config ros_setup
    local -A inherited=()
    repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || return
    # Explicit shell/environment settings take precedence over optional files.
    for name in ${!DRONE_@}; do inherited["$name"]="${!name}"; done
    for config in "$repo/runtime/host.env" "$repo/config/drone.env" "$repo/config/local.env"; do
        if [ -f "$config" ]; then
            source "$config" || return
        fi
    done
    for name in "${!inherited[@]}"; do
        printf -v "$name" '%s' "${inherited[$name]}"
        export "$name"
    done
    export DRONE_REPO_ROOT="${DRONE_REPO_ROOT:-$repo}"
    ros_setup="${DRONE_ROS_SETUP:-}"
    export DRONE_ROS_ROOT="${DRONE_ROS_ROOT:-${ros_setup%/setup.bash}}"
    export DRONE_ROS_ROOT="${DRONE_ROS_ROOT:-/opt/ros/melodic}"
    export DRONE_ROS_SETUP="${DRONE_ROS_SETUP:-$DRONE_ROS_ROOT/setup.bash}"
    export DRONE_RS_WS="${DRONE_RS_WS:-$DRONE_REPO_ROOT/runtime/workspaces/realsense_ws}"
    export DRONE_VINS_WS="${DRONE_VINS_WS:-$DRONE_REPO_ROOT/runtime/workspaces/vinsf_gpu_ws}"
    export DRONE_FAST_WS="${DRONE_FAST_WS:-$DRONE_REPO_ROOT/runtime/workspaces/Fast-Drone-250-master}"
    export DRONE_RUNTIME_DIR="${DRONE_RUNTIME_DIR:-$DRONE_REPO_ROOT/runtime/navigation}"
    export DRONE_VINS_CONFIG="${DRONE_VINS_CONFIG:-$DRONE_REPO_ROOT/config/vins/px4_d435_vins_gpu.yaml}"
    export DRONE_VINS_DEVEL="${DRONE_VINS_DEVEL:-$DRONE_VINS_WS/devel}"
    export DRONE_TOPIC_TIMEOUT="${DRONE_TOPIC_TIMEOUT:-${DRONE_STARTUP_TIMEOUT:-120}}"
    export DRONE_FCU_URL DRONE_GCS_URL DRONE_TARGET_SYSTEM DRONE_TARGET_COMPONENT
    export DRONE_CAMERA_SERIAL DRONE_CAMERA_USB_PORT DRONE_IMU_RATE DRONE_ROS_PYTHON
    export DRONE_STARTUP_TIMEOUT DRONE_KALIBR_WORKSPACE
}

drone_source_profile() {
    local profile="${1:-fast}" workspace="" setup="" resolved_setup="" had_nounset=0 status=0
    local kalibr_workspace="${DRONE_KALIBR_WORKSPACE:-}"
    case "$profile" in
        ros|base) profile=base ;;
        camera) workspace="$DRONE_RS_WS" ;;
        vins) setup="$DRONE_VINS_DEVEL/setup.bash" ;;
        fast|control) profile=control; workspace="$DRONE_FAST_WS" ;;
        kalibr) workspace="$kalibr_workspace" ;;
        *) printf 'Unknown ROS profile: %s\n' "$profile" >&2; return 2 ;;
    esac
    # Each child starts clean, including shells that inherited the control
    # profile from .bashrc. env.sh reinstates this host's private runtime libs.
    export PATH="$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin"
    unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH
    unset ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ROOT ROS_DISTRO
    unset ROS_ETC_DIR ROS_VERSION ROS_PYTHON_VERSION
    unset LD_PRELOAD DRONE_KALIBR_TLS_PRELOAD MATPLOTLIBDATA TEXMFCNF
    unset CATKIN_SETUP_UTIL_ARGS _CATKIN_SETUP_DIR
    source "$DRONE_REPO_ROOT/scripts/env.sh" "$profile" || return
    case "$profile" in
        camera) resolved_setup="$DRONE_WORKSPACE_ROOT/realsense_ws/devel/setup.bash" ;;
        vins) resolved_setup="$DRONE_WORKSPACE_ROOT/vinsf_gpu_ws/devel/setup.bash" ;;
        control) resolved_setup="$DRONE_WORKSPACE_ROOT/Fast-Drone-250-master/devel/setup.bash" ;;
        kalibr)
            resolved_setup="$DRONE_WORKSPACE_ROOT/kalibr_ws/devel/setup.bash"
            [ -f "$resolved_setup" ] || resolved_setup="${DRONE_KALIBR_WORKSPACE:-}/devel/setup.bash"
            ;;
    esac
    # env.sh normally selects these same builds. Honor explicitly configured
    # workspace paths after restoring the local dependency/ABI environment.
    if [ -n "$workspace" ]; then setup="$workspace/devel/setup.bash"; fi
    if [ -n "$kalibr_workspace" ]; then export DRONE_KALIBR_WORKSPACE="$kalibr_workspace"; fi
    if [ -n "$setup" ] && [ "$setup" != "$resolved_setup" ]; then
        if [ ! -f "$setup" ]; then
            printf '[env] Missing %s workspace setup: %s\n' "$profile" "$setup" >&2
            return 1
        fi
        case "$-" in *u*) had_nounset=1; set +u ;; esac
        source "$setup" || status=$?
        [ "$had_nounset" = 0 ] || set -u
        [ "$status" = 0 ] || return "$status"
    fi
}

_drone_nav_env_init
_drone_nav_env_status=$?
unset -f _drone_nav_env_init
return "$_drone_nav_env_status" 2>/dev/null || exit "$_drone_nav_env_status"
