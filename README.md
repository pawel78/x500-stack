# X500 / Pixhawk 6X Dev Kit

PX4 firmware + companion stack for the Holybro **X500** running a **Pixhawk 6X** flight controller, with an NVIDIA **Jetson** companion for AI (wildfire detection) and VIO.  
Primary working branch: **[`feat/px6x-firmware`](https://github.com/pawel78/x500-stack/tree/feat/px6x-firmware)**

---

## Repository Layout

```
x500-stack/
├─ .devcontainer/            # VS Code dev container config (Linux toolchain on macOS via Colima)
├─ docker/                   # Dockerfile used by the dev container
├─ firmware/
│  └─ PX4-Autopilot/         # PX4 source code (Git submodule pinned to release tag)
├─ scripts/
│  ├─ setup_px4_toolchain.sh # Installs ARM toolchain + Python deps + Git LFS
│  ├─ build_px6x.sh          # Clean + build Pixhawk 6X firmware
│  ├─ flash_px6x.sh          # Upload firmware over USB
│  └─ clean_submodules.sh    # Reset PX4 submodule state
├─ config/
│  └─ params/                # QGC parameter files (airframe tuning, etc.)
├─ maps/                     # (optional) mapping artifacts (LFS tracked)
└─ README.md
```

---

## Submodules & Pinning Strategy

PX4 lives under `firmware/PX4-Autopilot` as a **Git submodule**.  
A submodule is basically a “repo within a repo,” pinned at a specific commit. We **pin PX4** to a **stable release tag** (currently `v1.15.0`) to guarantee reproducible builds.

**Updating PX4 to a new release:**
```bash
git -C firmware/PX4-Autopilot fetch --tags
git -C firmware/PX4-Autopilot checkout v1.16.0
git -C firmware/PX4-Autopilot submodule update --init --recursive
git -C firmware/PX4-Autopilot clean -fdx
git add firmware/PX4-Autopilot
git commit -m "chore(px4): bump PX4 to v1.16.0 for Pixhawk 6X"
```

---

## macOS Workflow (VS Code Dev Containers)

This repo is designed for **macOS** dev using **VS Code** with Dev Containers and **Colima**. The container provides the exact Linux toolchain PX4 needs.

### One-time setup
```bash
brew install colima docker
colima start --cpu 6 --memory 16 --disk 60   # start Linux VM

# Install VS Code + “Dev Containers” extension
```

### Open in container
1. From the repo root: `code .`
2. Cmd+Shift+P → **Dev Containers: Rebuild and Reopen in Container**
3. Terminal prompt should be `/workspaces/x500-stack` and `uname -a` should show Linux.

---

## Building PX4 for Pixhawk 6X

```bash
# Ensure submodules are initialized
git submodule update --init --recursive

# Install toolchain & deps
./scripts/setup_px4_toolchain.sh

# Build
./scripts/build_px6x.sh

# Firmware output:
# firmware/PX4-Autopilot/build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```

**Flashing**  
- On macOS: open QGroundControl and drag-drop the `.px4` artifact.  
- On Linux: run `./scripts/flash_px6x.sh` with Pixhawk connected via USB.

**Cleanup (PX4 submodule junk):**
```bash
./scripts/clean_submodules.sh
```

---

## Jetson Development (Fire Detection + VIO)

The NVIDIA Jetson companion will host:
- **AI wildfire detection inference** (TensorRT-optimized ONNX/TF/torch models).
- **VIO (Visual-Inertial Odometry)** stack (e.g., Isaac ROS VSLAM or OpenVINS).

Integration:
- MAVLink/RTPS bridge to PX4 over serial/Ethernet.
- ROS 2 nodes for perception, mapping, and autonomy.
- Telemetry/video/alerts backhaul.

### TODOs
- [ ] Jetson setup scripts (`jetson/`): CUDA, TensorRT, ROS 2 Humble.  
- [ ] Deploy fire detection models (`*.onnx`, `*.pt`, `*.tflite`, `*.engine`).  
- [ ] Camera/VIO calibration and launch files.  
- [ ] Systemd services for auto-bringup.  
- [ ] Logging with rosbags (`*.bag`, `*.bag2` → LFS).  
- [ ] Add temporary files (`*.log`, `*.tmp`, `*.pcap`) to `.gitignore`.

---

## Git LFS Strategy

We use **Git LFS** for persistent large assets:

Tracked by LFS (`.gitattributes`):
```
*.bag
*.bag2
maps/**
*.onnx
*.pt
*.tflite
*.engine
```

Ignored by `.gitignore` (derived artifacts):
```
build/
install/
log/
*.tmp
*.pcap
*.log
```

The setup script installs & initializes LFS:
```bash
apt-get install -y git-lfs
git lfs install --force
git lfs update --force
```

---

## Troubleshooting

- **kconfiglib missing**: `python3 -m pip install kconfiglib`
- **genmsg missing**: `python3 -m pip install pyros-genmsg`
- **jsonschema missing**: `python3 -m pip install jsonschema`
- **arm-none-eabi nosys.specs error**: `sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi`
- **Git LFS push error**: `git lfs install && git lfs update --force`

---

## Branch & PR Flow

- Develop on feature branches (e.g., `feat/px6x-firmware`).
- Push and open PRs to `main`.  
- Merge only after build passes and SITL/flight test validated.

---

## Quick Start

```bash
# open container
code .

# rebuild container
Cmd+Shift+P → Dev Containers: Rebuild and Reopen in Container

# inside container
./scripts/setup_px4_toolchain.sh
./scripts/build_px6x.sh
```

Firmware will be at:
```
firmware/PX4-Autopilot/build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```
Flash it via QGroundControl (macOS) or `./scripts/flash_px6x.sh` (Linux).
