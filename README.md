# X500 / Pixhawk 6X Dev Kit

PX4 firmware + companion stack for the Holybro **X500** running a **Pixhawk 6X** flight controller, with an NVIDIA **Jetson** companion for AI (wildfire detection) and VIO.  
Primary working branch: **[`feat/px6x-firmware`](https://github.com/pawel78/x500-stack/tree/feat/px6x-firmware)**

---

## Repository Layout

```
x500-stack/
├─ .devcontainer/                    # VS Code Dev Container config
├─ docker/                           # Base images used by the dev container
├─ config/
│  ├─ params/                        # QGC parameter files
│  ├─ topics_caltech.yaml            # Caltech CART topic map (analysis)
│  └─ topics_x500.yaml               # X500 topic map (analysis)
├─ platforms/
│  ├─ scripts/                       # single home for all scripts/CLIs
│  │  ├─ setup_px4_toolchain.sh      # toolchain/deps bootstrap inside container
│  │  ├─ build_px6x.sh               # build Pixhawk 6X firmware (cross-compile)
│  │  ├─ flash_px6x.sh               # flash firmware over USB (or use QGC)
│  │  ├─ convert_ros1_to_ros2.py     # analysis helpers (bags → ROS 2)
│  │  ├─ extract_ros1.py
│  │  ├─ extract_ros2.py
│  │  ├─ run_mapping.py
│  │  └─ run_fire_detection.py
│  ├─ jetson/                        # Jetson-only deployment & config
│  │  ├─ docker/                     # app images / compose
│  │  ├─ ros2/                       # launch overlays, params
│  │  ├─ system/                     # udev, systemd, nvpmodel, jetson_clocks
│  │  ├─ setup/                      # provisioning / first-boot / Ansible
│  │  ├─ configs/                    # device-local IDs/paths
│  │  └─ README.md
│  └─ pixhawk/
│     ├─ px4/                        # PX4 submodule lives here
│     ├─ params/                     # vehicle .params you flash
│     ├─ airframe/
│     └─ wiring/                     # pinouts, serial maps
├─ src/
│  └─ x500proc/                      # reusable Python package for processing
├─ data/
│  ├─ caltech_cart/                  # Caltech Aerial RGB-Thermal dataset
│  │  ├─ raw_ros1_bags/
│  │  ├─ converted_ros2/
│  │  └─ extracted/
│  └─ x500_logs/                     # flight logs (ROS 2)
│     ├─ ros2_bags/
│     ├─ extracted/
│     └─ sessions/
├─ notebooks/
│  ├─ pawel/
│  └─ zach/
├─ results/
│  ├─ mapping/
│  └─ fire_detection/
├─ docs/
│  └─ research/                      # references, standards, summaries
│     ├─ papers/  standards/  patents/  datasets/  summaries/  bibtex/
│     ├─ reading-log.md
│     └─ README.md
├─ maps/                             # mapping artifacts (LFS)
├─ .gitattributes
├─ .gitignore
└─ README.md
```

---

## Submodules & Pinning Strategy

PX4 lives under `platforms/pixhawk/px4` as a **Git submodule**.  
A submodule is basically a “repo within a repo,” pinned at a specific commit. We **pin PX4** to a **stable release tag** (currently `v1.15.0`) to guarantee reproducible builds.

**Updating PX4 to a new release:**
```bash
git -C platforms/pixhawk/px4 fetch --tags
git -C platforms/pixhawk/px4 checkout v1.16.0
git -C platforms/pixhawk/px4 submodule update --init --recursive
git -C platforms/pixhawk/px4 clean -fdx
git add platforms/pixhawk/px4
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
## Daily Usage with Colima and Docker

This project uses Colima as the Docker runtime on macOS.  Af ther one-time setup instructions, follow these steps for everyday container work.

### Start Colima
Make sure the coliman VM is running before working with containers
```bash
colima start
```
Check status:
```bash
colimna status
```
Expected status output
```bash
INFO colima is running
```
If it says `stopped`, run `colima start` again.

### Confirm Docker is using Colima
Colima installs its own Docker context. Verify with:
```bash
docker context show
```
You should see:
```bash
colima
```
### List Containers
Show all (running and stopped) containers:
```bash
docker ps -a
```
### Start or Restart a Container
if you container is stopped and you want to restart it with logs and keep STDIN open
```bash
docker start -ai <container_name_or_id>
```
### Exec into a Running Container
If you want to open the shell inside the container and do work, this will be the most common use-case do:
```bash
docker exec -it <container_id_or_name> /bin/bash
```
---
## Building PX4 for Pixhawk 6X

```bash
# Ensure submodules are initialized
git submodule update --init --recursive

# Install toolchain & deps
./platforms/scripts/setup_px4_toolchain.sh

# Build
./platforms/scripts/build_px6x.sh

# Firmware output:
# platforms/pixhawk/px4/build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```

**Flashing**  
- On macOS: open QGroundControl and drag-drop the `.px4` artifact.  
- On Linux: run `./platforms/scripts/flash_px6x.sh` with Pixhawk connected via USB.

**Cleanup (PX4 submodule junk):**
```bash
./platforms/scripts/clean_submodules.sh
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

We use **Git LFS** for persistent large assets.

Tracked by LFS (`.gitattributes`):
```bash
*.bag
*.bag2
maps/**
*.onnx
*.pt
*.tflite
*.engine
```

Ignored by `.gitignore` (derived artifacts):
```bash
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

### On-demand download strategy

By default, LFS auto-downloads large files (via the smudge filter).  
To avoid this, we configure **on-demand fetching** so contributors only pull what they need.

#### New clone (recommended)
```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none https://github.com/pawel78/x500-stack.git
cd x500-stack
git lfs install --skip-smudge
git config lfs.fetchexclude "*"
```

#### Existing clone
```bash
git lfs install --skip-smudge
git config lfs.fetchexclude "*"
```

#### Fetch specific assets explicitly
```bash
git lfs fetch --include="data/run_01/*.bag"
git lfs pull  --include="data/run_01/*.bag"
```
---

## Troubleshooting

- **kconfiglib missing**: `python3 -m pip install kconfiglib`
- **genmsg missing**: `python3 -m pip install pyros-genmsg`
- **jsonschema missing**: `python3 -m pip install jsonschema`
- **arm-none-eabi nosys.specs error**: `sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi`
- **Git LFS push error**: `git lfs install && git lfs update --force`

---
