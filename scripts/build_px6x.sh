#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
PX4="$HERE/firmware/PX4-Autopilot"

git -C "$PX4" submodule update --init --recursive
git -C "$PX4" fetch --tags
git -C "$PX4" checkout v1.15.0
git -C "$PX4" submodule update --init --recursive
git -C "$PX4" clean -fdx

pushd "$PX4" >/dev/null
make px4_fmu-v6x_default
ART="$PX4/build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4"
popd >/dev/null

echo "✅ Built: $ART"
ls -lh "$ART"
