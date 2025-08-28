#!/usr/bin/env bash
set -euo pipefail

# ---- System packages ----
apt-get update
apt-get install -y --no-install-recommends \
  git curl wget ca-certificates openssh-client \
  cmake ninja-build build-essential ccache \
  python3 python3-pip python3-venv \
  python3-dev python3-setuptools \
  python3-empy python3-jinja2 python3-serial \
  python3-numpy python3-packaging python3-yaml \
  python3-lxml \
  gcc-arm-none-eabi

update-ca-certificates || true

# ---- Python deps used by PX4 (v1.15) ----
python3 -m pip install --upgrade pip
# future + pymavlink bits, pyelftools for ELF, toml via stdlib in 3.10 but keep explicit
pip3 install --no-cache-dir \
  future \
  pyelftools \
  toml

# (Genmsg is not on PyPI for this branch; above OS packages cover gen tools.)

echo "[OK] Toolchain ready. You can now run: make px4_fmu-v6x_default"
