#!/usr/bin/env bash
# Same entry as the reference drone.
exec bash "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/drone_nav.sh" takeoff "$@"
