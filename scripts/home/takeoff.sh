#!/usr/bin/env bash
# Manual command only; never called by startup or checks.
exec bash "$(dirname -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")")/drone_nav.sh" takeoff "$@"
