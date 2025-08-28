#!/usr/bin/env bash
set -euo pipefail
# Intended for Ubuntu (native or devcontainer). On macOS, use devcontainer/Colima.
sudo apt-get update
sudo apt-get install -y git ninja-build exiftool python3-pip python3-jinja2 \
  cmake build-essential genromfs ccache kconfig-frontends \
  python3-numpy python3-dev python3-setuptools python3-empy \
  python3-toml python3-packaging python3-yaml
# ARM toolchain for NuttX boards
sudo apt-get install -y gcc-arm-none-eabi
