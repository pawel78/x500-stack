#!/usr/bin/env bash
set -euo pipefail
# repo root (works whether the script lives at platforms/scripts or elsewhere)
HERE="$(
  git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel 2>/dev/null \
  || { d="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"; while [[ "$d" != "/" && ! -d "$d/.git" ]]; do d="$(dirname "$d")"; done; printf '%s' "$d"; }
)"

PX4_DIR="${PX4_DIR:-$HERE/platforms/pixhawk/px4}"

git -C "$PX4_DIR" submodule update --init --recursive
git -C "$PX4_DIR" fetch --tags
git -C "$PX4_DIR" checkout v1.15.0
git -C "$PX4_DIR" submodule update --init --recursive
git -C "$PX4_DIR" clean -fdx

pushd "$PX4_DIR" >/dev/null
make px4_fmu-v6x_default
ART="$PX4_DIR/build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4"
popd >/dev/null

echo "✅ Built: $ART"
ls -lh "$ART"
