#!/usr/bin/env bash
set -euo pipefail
SUBS=( firmware/PX4-Autopilot )
for S in "${SUBS[@]}"; do
  if [ -d "$S/.git" ]; then
    echo "Cleaning $S ..."
    git -C "$S" reset --hard
    git -C "$S" clean -fdx
    git -C "$S" submodule foreach --recursive 'git reset --hard; git clean -fdx'
  fi
done
echo "Done."
