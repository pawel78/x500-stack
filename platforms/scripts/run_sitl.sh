#!/usr/bin/env bash
set -euo pipefail
pushd platforms/pixhawk/px4 >/dev/null
# Gazebo (Ignition/Harmonic/etc.) target may vary with your container
make px4_sitl gazebo
popd >/dev/null
