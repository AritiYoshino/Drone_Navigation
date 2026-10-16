#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
source "$SCRIPT_DIR/env.sh"
SELF="$SCRIPT_DIR/drone_nav.sh"

usage() {
    cat <<'HELP'
Usage: drone-nav <command> [options]
  doctor [--hardware]  Offline checks; --hardware reads connected devices/topics
  check                Check files, packages and linked libraries without hardware
  vins                 Start MAVROS, camera, GPU VINS and VINS RViz
  start | quick        Also start odometry bridge, EGO planner and planner RViz
  component NAME       Start one component in the foreground
  controller           Manually start px4ctrl after the VINS gravity check
  takeoff              Publish the existing takeoff command (manual use only)
  shell PROFILE        Open an isolated ros/camera/vins/fast/kalibr shell
Options: --dry-run (print only), --headless (processes/logs without a desktop)
Components: master, mavros, imu-rate, camera, vins, bridge, planner, rviz,
            vins-rviz, gravity, controller, goals
Profiles and device URL: config/local.env (optional).
start/quick do not start px4ctrl or publish takeoff/goals.
HELP
}

component_profile() {
    case "$1" in
        master|mavros|imu-rate|vins-rviz) printf '%s' ros ;;
        camera) printf '%s' camera ;;
        vins) printf '%s' vins ;;
        bridge|planner|rviz|gravity|controller|goals|takeoff) printf '%s' fast ;;
        *) printf 'Unknown component: %s\n' "$1" >&2; return 2 ;;
    esac
}

component_command() {
    case "$1" in
        master) COMMAND=(roscore) ;;
        mavros) COMMAND=(roslaunch "$SCRIPT_DIR/launch/mavros_px4.launch" "fcu_url:=$DRONE_FCU_URL") ;;
        imu-rate) COMMAND=(python "$DRONE_REPO_ROOT/system_overrides/opt/ros/melodic/lib/mavros/request_px4_imu_rate.py") ;;
        camera) COMMAND=(roslaunch "$DRONE_RS_WS/src/realsense-ros/realsense2_camera/launch/rs_camera_ego.launch" unite_imu_method:=linear_interpolation "serial_no:=$DRONE_CAMERA_SERIAL") ;;
        vins) COMMAND=(rosrun vins vins_node "$DRONE_RUNTIME_DIR/vins_runtime.yaml" __ns:=/uav1) ;;
        bridge) COMMAND=(roslaunch px4ctrl vins_to_px4_odom.launch) ;;
        planner) COMMAND=(roslaunch ego_planner single_run_in_exp.launch) ;;
        rviz) COMMAND=(roslaunch ego_planner rviz.launch) ;;
        vins-rviz) COMMAND=(rosrun rviz rviz -d "$DRONE_VINS_WS/src/VINS-Fusion/config/vins_rviz_config.rviz") ;;
        gravity) COMMAND=(python "$SCRIPT_DIR/home/check_vins_gravity.py" --settle 8 --duration 2 --threshold-deg 2 --max-resets 3) ;;
        controller) COMMAND=(roslaunch px4ctrl run_ctrl.launch) ;;
        goals) COMMAND=(rosrun ego_planner realtime_goal.py) ;;
        takeoff) COMMAND=(rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand 'takeoff_land_cmd: 1') ;;
    esac
}

wait_topic() {
    [[ "$DRONE_TOPIC_TIMEOUT" =~ ^[1-9][0-9]*$ ]] || {
        printf 'DRONE_TOPIC_TIMEOUT must be a positive number of seconds.\n' >&2; return 2;
    }
    local topic="$1" deadline=$((SECONDS + DRONE_TOPIC_TIMEOUT))
    printf '[startup] Waiting for %s (up to %s s)\n' "$topic" "$DRONE_TOPIC_TIMEOUT"
    until timeout 3 rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
        if (( SECONDS >= deadline )); then
            printf '[startup] Topic unavailable: %s\n' "$topic" >&2
            return 1
        fi
        sleep 1
    done
}

# Every component below goes through roslaunch, and roslaunch auto-starts its
# own roscore when it cannot reach a master. Started in parallel they race for
# port 11311: one wins, the others die with "Address already in use" and
# "run_id on parameter server does not match", which tears the session down.
# Start a single master up front and let the components attach to it.
wait_for_master() {
    [[ "$DRONE_TOPIC_TIMEOUT" =~ ^[1-9][0-9]*$ ]] || {
        printf 'DRONE_TOPIC_TIMEOUT must be a positive number of seconds.\n' >&2; return 2;
    }
    local deadline=$((SECONDS + DRONE_TOPIC_TIMEOUT))
    printf '[startup] Waiting for ROS master (up to %s s)\n' "$DRONE_TOPIC_TIMEOUT"
    # Probe from a subshell with the ros profile: the caller may be a plain
    # login shell without ROS tools on PATH.
    until ( source "$SCRIPT_DIR/env.sh"; drone_source_profile ros; \
            timeout 3 rosnode list >/dev/null 2>&1 ); do
        if (( SECONDS >= deadline )); then
            printf '[startup] ROS master did not come up within %s s\n' "$DRONE_TOPIC_TIMEOUT" >&2
            return 1
        fi
        sleep 1
    done
}

gravity_check() {
    wait_topic /mavros/imu/data_raw || return 1
    wait_topic /uav1/vins_estimator/imu_propagate || return 1
    python "$SCRIPT_DIR/home/check_vins_gravity.py" --settle 8 --duration 2 --threshold-deg 2 --max-resets 3
}

run_component() {
    local component="$1" profile
    profile="$(component_profile "$component")"
    drone_source_profile "$profile"
    case "$component" in
        vins)
            python3 "$SCRIPT_DIR/prepare_vins_config.py" "$DRONE_VINS_CONFIG" "$DRONE_RUNTIME_DIR" >/dev/null
            wait_topic /mavros/imu/data_raw
            wait_topic /uav1/camera/infra1/image_rect_raw
            wait_topic /uav1/camera/infra2/image_rect_raw
            ;;
        bridge|controller)
            gravity_check || { printf '[startup] VINS gravity check failed; %s blocked.\n' "$component" >&2; return 1; }
            ;;
        planner|rviz)
            wait_topic /uav1/vins_estimator/imu_propagate
            wait_topic /uav1/camera/depth/image_rect_raw
            ;;
        gravity) gravity_check; return ;;
        goals) wait_topic /uav1/vins_estimator/imu_propagate ;;
    esac
    component_command "$component"
    exec "${COMMAND[@]}"
}

print_component() {
    local component="$1" profile
    profile="$(component_profile "$component")"
    component_command "$component"
    printf '[%s / %s] ' "$component" "$profile"
    printf '%q ' "${COMMAND[@]}"
    printf '\n'
    if [[ "$component" == vins ]]; then
        printf '  Config: %s -> %s/vins_runtime.yaml\n' "$DRONE_VINS_CONFIG" "$DRONE_RUNTIME_DIR"
    fi
}

check_profile() {
    local profile="$1" packages=() binaries=() package binary linkage failed=0
    drone_source_profile "$profile" || return 1
    case "$profile" in
        ros) packages=(mavros mavros_msgs); binaries=("$DRONE_ROS_ROOT/lib/mavros/mavros_node") ;;
        camera) packages=(realsense2_camera); binaries=("$DRONE_RS_WS/devel/lib/librealsense2_camera.so") ;;
        vins) packages=(vins cv_bridge image_geometry); binaries=("$DRONE_VINS_DEVEL/lib/vins/vins_node" "$DRONE_VINS_DEVEL/lib/libcv_bridge.so") ;;
        fast) packages=(ego_planner px4ctrl quadrotor_msgs); binaries=("$DRONE_FAST_WS/devel/lib/ego_planner/ego_planner_node" "$DRONE_FAST_WS/devel/lib/ego_planner/traj_server" "$DRONE_FAST_WS/devel/lib/px4ctrl/px4ctrl_node" "$DRONE_FAST_WS/devel/lib/px4ctrl/vins_to_px4_odom_node") ;;
        kalibr) packages=(kalibr) ;;
    esac
    for package in "${packages[@]}"; do
        if ! rospack find "$package" >/dev/null; then
            printf '[FAIL] %s package: %s\n' "$profile" "$package" >&2; failed=1
        fi
    done
    for binary in "${binaries[@]}"; do
        if [[ ! -f "$binary" ]]; then
            printf '[FAIL] Missing binary: %s\n' "$binary" >&2; failed=1
        else
            linkage="$(ldd "$binary" 2>&1)" || { printf '[FAIL] ldd %s: %s\n' "$binary" "$linkage" >&2; failed=1; }
            if [[ "$linkage" == *'not found'* ]]; then
                printf '[FAIL] Unresolved dependencies of %s:\n%s\n' "$binary" "$linkage" >&2; failed=1
            fi
        fi
    done
    (( failed == 0 )) && printf '[OK] %s profile\n' "$profile"
    return "$failed"
}

check_all() {
    local profile failed=0 file
    for file in "$DRONE_VINS_CONFIG" "$SCRIPT_DIR/home/check_vins_gravity.py" \
        "$DRONE_RS_WS/src/realsense-ros/realsense2_camera/launch/rs_camera_ego.launch"; do
        [[ -r "$file" ]] || { printf '[FAIL] Missing configuration: %s\n' "$file" >&2; failed=1; }
    done
    for profile in ros camera vins fast kalibr; do
        bash --noprofile --norc "$SELF" _check-profile "$profile" || failed=1
    done
    return "$failed"
}

launch_components() {
    local component terminal_args=(--window) terminal_command pids=() pid status=0
    if (( DRY_RUN )); then
        print_component master
        printf '  (started first, then waited for; the components below attach to it)\n'
        for component in "${COMPONENTS[@]}"; do print_component "$component"; done
        return
    fi
    if (( ! HEADLESS )) && [[ -n "${DISPLAY:-}" ]] && command -v gnome-terminal >/dev/null; then
        # The master gets its own window and must be up before any roslaunch
        # below starts, otherwise they race for port 11311.
        printf -v terminal_command '%q ' bash --noprofile --norc "$SELF" _component master
        gnome-terminal --window --title="drone: master" -e "$terminal_command"
        wait_for_master || return 1
        for component in "${COMPONENTS[@]}"; do
            # Ubuntu 18.04 supports -e; %q keeps repository paths as one argument.
            printf -v terminal_command '%q ' bash --noprofile --norc "$SELF" _component "$component"
            (( ${#terminal_args[@]} == 1 )) || terminal_args+=(--tab)
            terminal_args+=(--title="drone: $component" -e "$terminal_command")
        done
        gnome-terminal "${terminal_args[@]}"
        return
    fi
    mkdir -p "$DRONE_RUNTIME_DIR/logs"
    # EXIT runs after this function's local scope has unwound on an error.
    DRONE_SESSION_TOKEN="drone-nav.$$.$RANDOM.$RANDOM"
    DRONE_SESSION_PIDS=()
    cleanup() {
        local cleanup_pid
        trap - INT TERM EXIT
        # roslaunch nodes may create their own process groups or outlive a
        # crashed parent. The exact environment marker also covers those nodes.
        python3 "$SCRIPT_DIR/stop_session.py" "$DRONE_SESSION_TOKEN" || true
        for cleanup_pid in "${DRONE_SESSION_PIDS[@]}"; do
            kill -KILL -- "-$cleanup_pid" 2>/dev/null || true
            if ! kill -0 "$cleanup_pid" 2>/dev/null; then wait "$cleanup_pid" 2>/dev/null || true; fi
        done
    }
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    # Start the master first and wait for it; see wait_for_master().
    DRONE_SESSION_ID="$DRONE_SESSION_TOKEN" setsid bash --noprofile --norc "$SELF" _component master >"$DRONE_RUNTIME_DIR/logs/master.log" 2>&1 &
    pids+=("$!")
    DRONE_SESSION_PIDS+=("$!")
    printf '[startup] master PID %s, log %s/logs/master.log\n' "$!" "$DRONE_RUNTIME_DIR"
    wait_for_master || return 1
    for component in "${COMPONENTS[@]}"; do
        DRONE_SESSION_ID="$DRONE_SESSION_TOKEN" setsid bash --noprofile --norc "$SELF" _component "$component" >"$DRONE_RUNTIME_DIR/logs/$component.log" 2>&1 &
        pids+=("$!")
        DRONE_SESSION_PIDS+=("$!")
        printf '[startup] %s PID %s, log %s/logs/%s.log\n' "$component" "$!" "$DRONE_RUNTIME_DIR" "$component"
    done
    while :; do
        for pid in "${pids[@]}"; do
            if ! kill -0 "$pid" 2>/dev/null; then
                wait "$pid" || status=$?
                printf '[startup] Component PID %s exited (%s); stopping session.\n' "$pid" "$status" >&2
                return "$status"
            fi
        done
        sleep 1
    done
}

if [[ "${1:-}" == doctor ]]; then shift; exec python3 "$SCRIPT_DIR/doctor.py" "$@"; fi
if [[ "${1:-}" == _component ]]; then shift; run_component "$1"; exit; fi
if [[ "${1:-}" == _check-profile ]]; then shift; check_profile "$1"; exit; fi

DRY_RUN=0 HEADLESS=0 CHECK_ONLY=0 POSITIONAL=()
for argument in "$@"; do
    case "$argument" in
        --dry-run) DRY_RUN=1 ;;
        --headless) HEADLESS=1 ;;
        --check) CHECK_ONLY=1 ;;
        --help|-h) usage; exit ;;
        *) POSITIONAL+=("$argument") ;;
    esac
 done
ACTION="${POSITIONAL[0]:-help}"
if (( CHECK_ONLY )); then check_all; exit; fi
if [[ -z "${DISPLAY:-}" ]] || ! command -v gnome-terminal >/dev/null; then HEADLESS=1; fi
case "$ACTION" in
    help) usage ;;
    check) check_all ;;
    start|quick) COMPONENTS=(mavros imu-rate camera vins bridge planner); (( HEADLESS )) || COMPONENTS+=(rviz); launch_components ;;
    vins) COMPONENTS=(mavros imu-rate camera vins); (( HEADLESS )) || COMPONENTS+=(vins-rviz); launch_components ;;
    component|controller|takeoff)
        COMPONENT="${POSITIONAL[1]:-}"
        [[ "$ACTION" == component ]] || COMPONENT="$ACTION"
        [[ -n "$COMPONENT" ]] || { usage >&2; exit 2; }
        component_profile "$COMPONENT" >/dev/null
        if (( DRY_RUN )); then print_component "$COMPONENT"; else run_component "$COMPONENT"; fi
        ;;
    shell|env)
        PROFILE="${POSITIONAL[1]:-ros}"
        if (( DRY_RUN )); then printf 'source %q; drone_source_profile %q\n' "$SCRIPT_DIR/env.sh" "$PROFILE"; else
            drone_source_profile "$PROFILE"
            exec bash --noprofile --norc -i
        fi
        ;;
    *) printf 'Unknown command: %s\n' "$ACTION" >&2; usage >&2; exit 2 ;;
esac
