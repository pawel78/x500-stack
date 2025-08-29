#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

# ---- System packages ----
apt-get update
apt-get install -y --no-install-recommends \
  git curl wget ca-certificates openssh-client \
  cmake ninja-build build-essential ccache \
  gcc-arm-none-eabi libnewlib-arm-none-eabi \
  binutils-arm-none-eabi gdb-multiarch \
  python3 python3-pip python3-venv \
  python3-dev python3-setuptools \
  python3-empy python3-jinja2 python3-serial \
  python3-numpy python3-packaging python3-yaml \
  python3-lxml
update-ca-certificates || true

# ---- Python deps used by PX4 (v1.15) ----
PY=$(command -v python3)
$PY -m pip install --upgrade --no-cache-dir pip
$PY -m pip install --no-cache-dir \
  pyros-genmsg kconfiglib future pyelftools toml lxml jsonschema
export PYTHON_EXECUTABLE="$PY"   # so CMake uses the same python

$PY - <<'PY'
import sys, jsonschema, genmsg, kconfiglib
print("OK:", sys.executable)
print("jsonschema:", jsonschema.__file__)
print("genmsg:", genmsg.__file__)
print("kconfiglib:", getattr(kconfiglib, "__file__", "?"))
PY

# ---- Git LFS ----
apt-get install -y git-lfs
git lfs install --force
git lfs update --force