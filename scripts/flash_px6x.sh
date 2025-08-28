#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
PX4="$HERE/firmware/PX4-Autopilot"
pushd "$PX4" >/dev/null
make px4_fmu-v6x_default upload
popd >/dev/null
