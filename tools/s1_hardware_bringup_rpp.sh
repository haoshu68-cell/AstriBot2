#!/usr/bin/env bash
# Compatibility entry: all startup, verification and vendor protection live in one script.
set -e
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$SCRIPT_DIR/s1_hardware_bringup.sh" "$@" --controller=rpp
