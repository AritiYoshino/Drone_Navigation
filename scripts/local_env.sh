#!/usr/bin/env bash
# Source one profile per process; camera and GPU VINS use different OpenCV ABIs.
_drone_env_main() {
    local profile="${1:-control}" root prefix workspace setup entry var value filtered part had_nounset=0
    case "$profile" in base|camera|vins|control|kalibr) ;; *)
        printf 'Usage: source scripts/local_env.sh {base|camera|vins|control|kalibr}\n' >&2
        return 2 ;;
    esac
    root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || return
    prefix="$root/runtime/local"
    export DRONE_ROOT="$root" DRONE_WORKSPACE_ROOT="$root/runtime/workspaces" DRONE_LOCAL_PREFIX="$prefix"
    [ ! -f "$root/runtime/host.env" ] || source "$root/runtime/host.env"
    if [ -n "${DRONE_KALIBR_TLS_PRELOAD:-}" ]; then
        if [ "${LD_PRELOAD:-}" = "$DRONE_KALIBR_TLS_PRELOAD" ]; then
            unset LD_PRELOAD
        else
            value="${LD_PRELOAD:-}"
            export LD_PRELOAD="${value#"$DRONE_KALIBR_TLS_PRELOAD:"}"
        fi
        unset DRONE_KALIBR_TLS_PRELOAD
    fi
    # Remove this project's previous profile before selecting another one.
    for var in PATH LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH ROS_PACKAGE_PATH; do
        value="${!var-}"; filtered=""
        while IFS= read -r part; do
            case "$part" in
                "$root/runtime/local"/*|"$root/runtime/sysroot"/*|"$root/runtime/workspaces"/*|"$root/workspaces"/*/devel/*|"$root/workspaces"/*/devel) continue ;;
            esac
            [ -n "$part" ] && filtered="${filtered:+$filtered:}$part"
        done < <(printf '%s' "$value" | tr ':' '\n'; printf '\n')
        printf -v "$var" '%s' "$filtered"
        export "$var"
    done
    setup="${DRONE_ROS_SETUP:-/opt/ros/melodic/setup.bash}"
    if [ ! -f "$setup" ]; then printf '[env] Missing ROS setup: %s\n' "$setup" >&2; return 1; fi
    # ROS Melodic's generated setup scripts are not nounset safe.
    case "$-" in *u*) had_nounset=1; set +u ;; esac
    source "$setup"
    local status=$?
    [ "$had_nounset" = 0 ] || set -u
    [ "$status" = 0 ] || return "$status"
    case "$profile" in
        camera) workspace=realsense_ws ;;
        vins) workspace=vinsf_gpu_ws ;;
        control) workspace=Fast-Drone-250-master ;;
        kalibr) workspace=kalibr_ws ;;
    esac
    if [ "$profile" != base ]; then
        # Full setup also runs ROS's package-path hook for this devel prefix.
        setup="$DRONE_WORKSPACE_ROOT/$workspace/devel/setup.bash"
        if [ "$profile" = kalibr ] && [ ! -f "$setup" ] && [ -n "${DRONE_KALIBR_WORKSPACE:-}" ]; then
            # catkin_tools' linked devel space requires setup.bash to register packages.
            setup="$DRONE_KALIBR_WORKSPACE/devel/setup.bash"
        fi
        if [ ! -f "$setup" ]; then
            printf '[env] Profile %s is not built: %s\n' "$profile" "$setup" >&2
            return 1
        fi
        case "$-" in *u*) set +u ;; esac
        source "$setup"
        status=$?
        [ "$had_nounset" = 0 ] || set -u
        [ "$status" = 0 ] || return "$status"
    fi
    local libraries="" prefixes=""
    case "$profile" in
        camera) prefixes="$prefix/realsense" ;;
        vins) prefixes="$prefix/opencv:$prefix/ceres" ;;
        control) prefixes="$prefix/nlopt" ;;
    esac
    while IFS= read -r entry; do
        [ -n "$entry" ] || continue
        [ ! -d "$entry/lib" ] || libraries="${libraries:+$libraries:}$entry/lib"
        [ ! -d "$entry/lib64" ] || libraries="${libraries:+$libraries:}$entry/lib64"
        [ ! -d "$entry/lib/pkgconfig" ] || export PKG_CONFIG_PATH="$entry/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
    done < <(printf '%s' "$prefixes" | tr ':' '\n'; printf '\n')
    [ -z "$prefixes" ] || export CMAKE_PREFIX_PATH="$prefixes${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    # Missing Ubuntu/ROS libraries are unpacked here without changing /usr.
    # GPU OpenCV must stay ahead of the system OpenCV 3.2 in this search path.
    if [ -d "$root/runtime/sysroot/usr/lib/aarch64-linux-gnu" ]; then
        libraries="${libraries:+$libraries:}$root/runtime/sysroot/usr/lib/aarch64-linux-gnu"
    fi
    [ -z "$libraries" ] || export LD_LIBRARY_PATH="$libraries${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    if [ "$profile" = kalibr ] && [ -d "$root/runtime/sysroot/usr/lib/python2.7/dist-packages" ]; then
        export PYTHONPATH="$root/runtime/sysroot/usr/lib/python2.7/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
        export PATH="$root/runtime/sysroot/usr/bin:$PATH"
        export TEXMFCNF="$root/runtime/sysroot/etc/texmf/web2c"
        # This host reproduced AArch64's static TLS failure when importing cv2
        # after SciPy. Preloading libgomp is limited to the Kalibr process profile.
        entry=/usr/lib/aarch64-linux-gnu/libgomp.so.1
        if [ -f "$entry" ]; then
            case ":${LD_PRELOAD:-}:" in *":$entry:"*) ;; *)
                export DRONE_KALIBR_TLS_PRELOAD="$entry"
                export LD_PRELOAD="$entry${LD_PRELOAD:+:$LD_PRELOAD}" ;;
            esac
        fi
    fi
    if [ -d /usr/local/cuda-10.2/bin ]; then export PATH="/usr/local/cuda-10.2/bin:$PATH"; fi
    [ ! -d "$prefix/bin" ] || export PATH="$prefix/bin:$PATH"
    if [ "$profile" = vins ] && [ -d /usr/local/cuda-10.2/lib64 ]; then
        export LD_LIBRARY_PATH="/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
    # GeographicLib reads the existing bundled EGM96 dataset without /usr writes.
    if [ -f "$prefix/share/GeographicLib/geoids/egm96-5.pgm" ]; then
        export GEOGRAPHICLIB_GEOID_PATH="$prefix/share/GeographicLib/geoids"
    else
        export GEOGRAPHICLIB_GEOID_PATH="$root/system_overrides/usr/share/GeographicLib/geoids"
    fi
    export DRONE_ENV_PROFILE="$profile"
}
_drone_env_main "$@"
_drone_env_status=$?
unset -f _drone_env_main
return "$_drone_env_status" 2>/dev/null || exit "$_drone_env_status"
