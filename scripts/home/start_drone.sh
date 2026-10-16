#!/usr/bin/env bash
# Compatibility for the local drone-start interface; drone-nav is the main entry.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
MODE=localization
WITH_RVIZ=0
OPTIONS=()
while (( $# )); do
    case "$1" in
        --mode)
            (( $# >= 2 )) || { printf 'Missing value for --mode\n' >&2; exit 2; }
            MODE="$2"; shift 2 ;;
        --with-rviz) WITH_RVIZ=1; shift ;;
        --help|-h)
            cat <<'HELP'
Usage: drone-start [--mode localization|full|camera|mavros] [--with-rviz] [options]
  localization -> drone-nav vins (default)
  full         -> drone-nav start (localization, odometry bridge and EGO)
  camera       -> camera only
  mavros       -> MAVROS and IMU rate requester only
Options: --dry-run, --headless, --fcu-url URL, --serial SERIAL, --vins-config PATH.
Use drone-nav controller separately to start px4ctrl.
HELP
            exit ;;
        *) OPTIONS+=("$1"); shift ;;
    esac
done
case "$MODE" in
    localization) ACTION=vins ;;
    full) ACTION=start ;;
    camera|mavros) ACTION="$MODE" ;;
    *) printf 'Unknown mode: %s\n' "$MODE" >&2; exit 2 ;;
esac
(( WITH_RVIZ )) || OPTIONS+=(--headless)
exec bash "$SCRIPT_DIR/../drone_nav.sh" "$ACTION" "${OPTIONS[@]}"
