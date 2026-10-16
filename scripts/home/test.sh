#!/usr/bin/env bash
exec bash "$(dirname -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")")/drone_nav.sh" vins "$@"
