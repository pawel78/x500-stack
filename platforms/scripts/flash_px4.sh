#!/usr/bin/env bash
set -euo pipefail
pushd platforms/pixhawk/px4 >/dev/null
make px4_fmu-v6x_default
make px4_fmu-v6x_default upload
popd >/dev/null
