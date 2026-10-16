#!/usr/bin/env bash
# Run with sudo once; reconnect/login again to apply the supplementary group.
set -euo pipefail
if [[ "$EUID" != 0 ]]; then
    echo 'Run: sudo bash scripts/setup_serial_permissions.sh' >&2
    exit 1
fi
drone_user=${SUDO_USER:-z}
getent passwd "$drone_user" >/dev/null
usermod -aG dialout,plugdev "$drone_user"
echo "Serial access configured for $drone_user. Log out and back in before testing."
