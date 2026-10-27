#!/usr/bin/env bash
# Select control/planner, camera, VINS or Kalibr explicitly when needed.
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/scripts/env.sh"
drone_source_profile "${1:-base}"
