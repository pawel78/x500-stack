#!/usr/bin/env bash
set -euo pipefail
pushd firmware/PX4-Autopilot >/dev/null
make px4_fmu-v6x_default
make px4_fmu-v6x_default upload
popd >/dev/null
