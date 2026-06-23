# ROS Noetic Z1 Arm Simulation

Gazebo simulation of the Unitree Z1 robotic arm with ArUco marker tracking, containerized in Docker.

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Python 3.8](https://img.shields.io/badge/Python-3.8-blue)
![Docker](https://img.shields.io/badge/Docker-required-blue)

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Installation and Dependencies](#installation-and-dependencies)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [AI-Assistance](#ai-assistance)

---

## Overview

We run ROS Noetic inside Docker to simulate the Unitree Z1 arm on Ubuntu 24, which does not natively support Noetic. The workspace implements a full perception-to-control pipeline: a RealSense D435 camera in Gazebo detects a moving ArUco marker, and the Z1 arm follows it in real time using Cartesian control.

The primary use case is a robotics workshop where participants receive boilerplate ROS nodes with TODOs and must complete the vision and control pipelines.

The image uses Mesa software rendering (llvmpipe) by default — no GPU is required to run Gazebo or RViz.

---

## Key Features

- Full Gazebo simulation of the Unitree Z1 6-DOF arm with gripper
- ArUco marker detection via OpenCV and cv_bridge, published as 3D world-frame poses
- Configurable camera modes: end-effector (wrist-mounted) and fixed (static world pose)
- Animated marker with configurable motion patterns: sinusoidal, circular, figure-8, and static
- Smooth Cartesian arm tracking with low-pass filtering and workspace clamping
- Centralized YAML configuration — change marker motion, tracking gain, camera mode, and workspace limits without rebuilding
- Pre-configured RViz layout with robot model, TF tree, camera feeds, and marker pose visualization
- Software rendering baked in — works without NVIDIA drivers or GPU passthrough
- Live file sync via Docker bind mounts — edit source on the host, relaunch inside the container

---

## Installation and Dependencies

### Prerequisites

| Dependency | Version | Notes |
| --- | --- | --- |
| Docker | 24+ | Required |
| Python | 3.8 | Inside the container |
| ROS Noetic | 1.16 | Inside the container |
| X11 / `xhost` | any | Required for GUI (Gazebo, RViz) |

No ROS installation is needed on the host. Everything runs inside the container.

### Build the Docker Image [RECOMMENDED]

```bash
git clone --recursive <repo-url> ros_docker
cd ros_docker
docker build -t ros-z1-aruco-real .
```

If you already cloned without `--recursive`, populate the submodules before building:

```bash
git submodule update --init --recursive
```

The `sdk_z1`, `z1_controller`, and `unitree_ros` directories are git submodules — the
Docker build copies their contents directly and fails if they are empty.

The build installs ROS Noetic, Gazebo, RViz, OpenCV with ArUco support, the RealSense2
ROS driver, and compiles `z1_controller` and `sdk_z1` binaries. First build takes
10-15 minutes.

### Optional — GPU Hardware Rendering

The image uses software rendering by default. To override to NVIDIA GPU rendering,
install `nvidia-container-toolkit` on the host:

```bash
sudo apt-get install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Then pass `-e LIBGL_ALWAYS_SOFTWARE=0 --gpus all` to `docker run`. See [docs/DOCKER_CMDS.md](docs/DOCKER_CMDS.md) for the full command.

---

## Quick Start

**1. Allow X11 forwarding on the host:**

**Linux:**

```bash
xhost +local:docker
```

**Windows:**

Windows has no native X server, so install one. [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
is the standard free option.

1. Launch the X server with access control disabled and native OpenGL off
   (the latter avoids a Gazebo `gzclient` crash):

   ```powershell
   & "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac -wgl
   ```

   Or run `XLaunch` and choose: *Multiple windows* → display `0` → *Start no
   client* → check **Disable access control** and uncheck **Native opengl**.

2. When Windows Firewall prompts, **allow VcXsrv** (on private *and* public
   networks — Docker Desktop's virtual adapter is usually classified as public).
   If you dismissed the prompt, Windows creates *Block* rules that silently stop
   the container from connecting; remove them and add an allow rule from an
   **elevated** PowerShell:

   ```powershell
   Get-NetFirewallApplicationFilter | Where-Object { $_.Program -like "*vcxsrv*" } |
     Get-NetFirewallRule | Where-Object { $_.Action -eq "Block" } | Remove-NetFirewallRule
   New-NetFirewallRule -DisplayName "VcXsrv X server (allow)" -Direction Inbound `
     -Program "C:\Program Files\VcXsrv\vcxsrv.exe" -Action Allow -Profile Any
   ```

3. In the `docker run` commands below, Docker Desktop runs containers in a VM, so
   replace the Linux X11 flags `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`
   with:

   ```
   -e DISPLAY=host.docker.internal:0.0
   ```

   (drop the `-v /tmp/.X11-unix...` mount entirely — it does not exist on Windows).

   Verify the connection from inside the container with `xclock` (a window should
   appear on your desktop) and `glxgears` (confirms OpenGL works).

**2a. Gazebo simulation (simulated camera + animated marker):**

**Linux:**

```bash
docker run -it --rm \
  --name ros-z1-real \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros-z1-aruco-real bash
```

**Windows** (single line; PowerShell does not treat a trailing `\` as a line
continuation):

```powershell
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 ros-z1-aruco-real bash
```

Inside the container (same on both platforms):

```bash
z1_sim        # launches Gazebo + ArUco tracking simulation
z1_unpause    # unpause physics (separate terminal)
z1_rviz       # open RViz (separate terminal)
```

**2b. Real camera mode (physical D435 + Gazebo arm):** [RECOMMENDED for workshop]

USB passthrough works completely differently on Linux (native device
passthrough) vs. Windows (the device must be bridged into Docker Desktop's
WSL2 VM first, via `usbipd-win`). Full detail and troubleshooting:
[docs/DOCKER_CMDS.md — USB Device Passthrough](docs/DOCKER_CMDS.md#usb-device-passthrough-realsense-d435).

**Linux:**

Find the D435 USB bus on the host:

```bash
lsusb | grep RealSense
# example: Bus 004 Device 003: ID 8086:0b07 Intel Corp. RealSense D435
```

```bash
docker run -it --rm \
  --name ros-z1-real \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/bus/usb/004:/dev/bus/usb/004 \
  ros-z1-aruco-real bash
```

**Windows:**

Bridge the D435 into WSL2 with `usbipd-win` (one-time install, then re-attach
after every unplug/replug/reboot):

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list                          # find the D435's BUSID, e.g. 4-3
usbipd bind --busid 4-3
usbipd attach --wsl --busid 4-3
```

`usbipd attach --wsl` attaches to whichever WSL distro is currently default —
not necessarily `docker-desktop`, the distro that actually backs Docker
Desktop's containers. Confirm it landed there specifically:

```powershell
wsl -d docker-desktop -- lsusb       # confirm it landed, note the bus number
```

If it's missing here but shows up under plain `wsl lsusb`, it attached to
the wrong distro — switch the default (`wsl -l -v` / `wsl -s docker-desktop`)
and re-attach.

Then pass that bus into the container (single line; bus 002 in this example):

```powershell
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 --device /dev/bus/usb/002:/dev/bus/usb/002 ros-z1-aruco-real bash
```

Inside the container (same on both platforms):

```bash
z1_real       # launches Gazebo arm + real D435 tracking
z1_unpause    # unpause physics (separate terminal)
z1_rviz       # open RViz (separate terminal)
```

**2c. Real camera mode (physical ZED 2 + Gazebo arm, no ZED SDK):**

**Windows shortcut:** `start_zed_sim.sh` (repo root, run from Git Bash on the
host) checks/repairs VcXsrv (running + `-wgl`), the ZED's `usbipd` attach
state, and the `uvcvideo`/`/dev/videoX` step below, reading the live USB bus
number each run instead of a stale hardcoded one — then launches the
container and auto-opens RViz once ROS is up inside it (polls
`rostopic list` in the background, no separate terminal/command needed):

```bash
bash start_zed_sim.sh                  # launches z1_real_zed, default DICT_4X4_50/5cm/ID 0
bash start_zed_sim.sh z1_real_zed_headless   # or any other alias

# Markers that don't match the repo's default (DICT_4X4_50, 5cm, ID 0):
bash start_zed_sim.sh --dictionary DICT_5X5_50 --marker-size 0.03 --tracking-id 1
```

`--dictionary`/`--marker-size`/`--tracking-id` override `aruco/dictionary`,
`aruco/marker_size`, and `aruco/tracking_id` from `aruco_tracking.yaml` for
this run only (the file itself is untouched). `--tracking-id` is the single
ID the arm actually follows — `marker_ids` in the yaml is documentation only,
the detector never reads it. An unrecognized `--dictionary` value is rejected
before launching; `aruco_detector_node` itself now also fails loudly (instead
of silently falling back to `DICT_4X4_50`) if the resolved value is invalid.

`usbipd bind` needs an elevated shell the first time a device is bound — run
it from an admin Git Bash if that step fails. The manual steps it automates
are below, useful for understanding what broke if the script errors out.

`aruco_detector_node` only subscribes to `/camera/color/image_raw` +
`/camera/color/camera_info` — it doesn't care which physical camera feeds
those topics. `zed_camera_node.py` reads the ZED 2's raw UVC feed directly via
OpenCV/V4L2 (no ZED SDK, no GPU/CUDA needed), crops it to one eye, and
republishes on those same topics, so nothing downstream changes.

USB/WSL2 bridging is the same `usbipd-win` dance as the D435 above, plus one
extra step (the WSL2 kernel doesn't auto-load the `uvcvideo` module needed
for `/dev/videoX` to appear), and — on Windows specifically — a real
bandwidth caveat: full-resolution video reliably arrives corrupted over
`usbipd`'s USB-over-TCP tunnel. Full steps, the `ffplay` test to confirm a
clean mode, and the exact corruption signature to watch for:
[docs/DOCKER_CMDS.md — Diagnosing corrupted video over usbipd](docs/DOCKER_CMDS.md#diagnosing-corruptedgarbled-video-over-usbipd-windows).

`/dev/videoX` is `crw-------` (root-only) — **`--user root` is required** or
`zed_camera_node` fails with `[FATAL] [zed_camera] Could not open /dev/video0`:

```powershell
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 --device /dev/bus/usb/002:/dev/bus/usb/002 --device /dev/video0:/dev/video0 --device /dev/video1:/dev/video1 --user root ros-z1-aruco-real bash
```

```bash
z1_real_zed   # launches Gazebo arm + real ZED 2 tracking
z1_unpause    # unpause physics (separate terminal)
z1_rviz       # open RViz (separate terminal)
```

The camera device path, resolution, framerate, and which eye to publish are
all set in `zed_camera:` in `z1_aruco/config/aruco_tracking.yaml` — edit
there, not the launch file, to change them. The defaults (`1344x376@15fps`,
left eye) are the lowest-bandwidth mode and the only one confirmed
corruption-free over `usbipd` during testing; intrinsics are a rough
FOV-based estimate (not the ZED's factory calibration), so expect some error
in estimated marker distance/pose.

**3. Open RViz (any mode, Terminal 2):**

**Linux:**

```bash
docker exec -e DISPLAY=$DISPLAY -it ros-z1-real bash -ic "z1_rviz"
```

The `-i` (interactive) flag is required: bash only expands the `z1_*` aliases —
and auto-sources `~/.bashrc` — in an interactive shell.

**Windows** (VcXsrv must be running):

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-real bash -ic "z1_rviz"
```

**Windows — full 3-terminal + browser workflow (sim, RViz, workshop UI):**

With VcXsrv running (step 1 above), open three PowerShell windows:

```powershell
# Terminal 1 — start the container and launch the simulation directly
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 -p 8501:8501 -p 8888:8888 ros-z1-aruco-real bash -ic "z1_sim"
```

```powershell
# Terminal 2 — open RViz (docker exec needs its own -e DISPLAY, it does not
# reliably inherit the value from the docker run above)
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-real bash -ic "z1_rviz"
```

```powershell
# Terminal 3 — start the Streamlit workshop UI
docker exec -it ros-z1-real bash -ic "z1_ui"
```

Then open **http://localhost:8501** in a browser (or `http://<host-hostname-or-IP>:8501`
from another machine on the network, e.g. for workshop attendees) for the live-knob
dashboard, alongside the Gazebo/RViz windows VcXsrv forwards to the desktop.

See [docs/STARTUP.md](docs/STARTUP.md) for the full developer reference — roslaunch arguments,
configuration, architecture notes, troubleshooting, and future work.

---

## Project Structure

```txt
ros_docker/
├── Dockerfile                        # ROS Noetic image with all dependencies
├── README.md
├── start_zed_sim.sh                  # Windows host script: checks/repairs VcXsrv+USB, launches ZED sim
├── docs/
│   ├── STARTUP.md                    # Full developer reference — build, run, config, architecture
│   ├── DOCKER_CMDS.md                # Docker command reference
│   ├── QUICKSTART.md                 # All run modes, flags, roslaunch args, config, aliases — one reference
│   ├── CONTROL_GUIDE.md              # (legacy — content merged into STARTUP.md)
│   └── GAZEBO_SIM_GUIDE.md           # (legacy — content merged into STARTUP.md)
│
├── z1_aruco_detector/                # ROS package — perception pipeline
│   ├── src/
│   │   ├── aruco_detector_node.py    # Detects ArUco, publishes 3D marker pose
│   │   ├── marker_mover_node.py      # Animates the ArUco marker in Gazebo
│   │   └── zed_camera_node.py        # Raw-UVC ZED 2 bridge (no ZED SDK) → image_raw/camera_info
│   └── config/
│       └── aruco_tracking.yaml       # Centralized simulation configuration
│
├── z1_arm_tracker/                   # ROS package — control pipeline
│   └── src/
│       └── arm_tracker_node.py       # Sends Cartesian commands to sim_ctrl via UDP
│
├── z1_aruco/                         # ROS package — project-specific launch, worlds, models
│   ├── launch/
│   │   ├── z1_aruco_tracking.launch            # Simulation: Gazebo + simulated camera + marker
│   │   ├── z1_real_camera_tracking.launch      # Real camera: Gazebo arm + physical D435
│   │   └── z1_real_camera_tracking_zed.launch  # Real camera: Gazebo arm + physical ZED 2
│   ├── worlds/
│   │   ├── aruco_tracking.world       # World with arm and ArUco marker
│   │   └── aruco_tracking_real.world  # World with arm only (no marker, no camera)
│   ├── models/
│   │   ├── realsense_d435/            # Gazebo RealSense D435 model
│   │   └── aruco_marker_0/            # Gazebo ArUco marker ID 0 model
│   ├── xacro/
│   │   └── z1_aruco_robot.xacro      # Z1 URDF with end-effector camera additions
│   └── rviz/
│       └── z1_aruco_tracking.rviz     # Pre-configured RViz layout
│
├── unitree_ros/                      # Unitree ROS packages (submodule, unmodified upstream)
│
├── z1_controller/                    # Unitree Z1 FSM controller (C++)
│   └── build/                        # sim_ctrl binary (run from here)
│
├── sdk_z1/                           # Unitree Z1 SDK (C++)
│   └── build/                        # highcmd_basic, lowcmd_development, etc.
│
└── tests/                            # Development test scripts
    ├── test_arm_motion.py
    ├── test_follower_no_camera.py
    └── test_marker_control.py
```

---

## Architecture

Two modes share the same detector and tracker — only the image source changes.

**Simulation mode** (`z1_aruco_tracking.launch`):

```txt
  Gazebo Simulation
  ┌──────────────────────────────────────────────────┐
  │  Z1 Arm  ←── joint controllers                   │
  │  ArUco Marker (animated by marker_mover_node)    │
  │  RealSense D435 camera plugin (URDF, wrist)      │
  └──────────────┬───────────────────────────────────┘
                 │ /camera/color/image_raw
                 ▼
        aruco_detector_node ──► arm_tracker_node
                                       │ MotorCmd
                                       ▼
                               joint controllers
```

**Real camera mode** (`z1_real_camera_tracking.launch`):

```txt
  Physical D435 (USB)          Gazebo Simulation
  ┌─────────────────┐          ┌──────────────────────┐
  │  RealSense D435 │          │  Z1 Arm              │
  │  (real frames)  │          │  (camera TF on wrist)│
  └────────┬────────┘          └──────────────────────┘
           │ /camera/color/image_raw
           ▼
  aruco_detector_node ──► arm_tracker_node
                                 │ MotorCmd
                                 ▼
                         joint controllers
```

### Data Flow

| Topic | Type | From | To |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gazebo plugin or real D435 | aruco_detector |
| `/aruco/marker_pose` | `geometry_msgs/PoseStamped` | aruco_detector | arm_tracker |
| `/aruco/marker_detected` | `std_msgs/Bool` | aruco_detector | arm_tracker |
| `/aruco/debug_image` | `sensor_msgs/Image` | aruco_detector | image_view / RViz |
| `/z1_gazebo/JointXX_controller/command` | `MotorCmd` | arm_tracker | Gazebo joint controllers |
| `/gazebo/set_model_state` | `gazebo_msgs/ModelState` | marker_mover | Gazebo (sim mode only) |

---

## Configuration

All simulation parameters load from a single file at launch time:

```txt
z1_aruco/config/aruco_tracking.yaml
```

Edit this file on the host and relaunch — no rebuild required when using bind mounts.

### Camera Mode

```yaml
camera:
  mode: end_effector   # end_effector (wrist-mounted) or fixed (static world pose)
```

The launch file argument `camera_mode:=end_effector` must match `camera/mode` in the YAML.

### Marker Motion

```yaml
marker:
  motion_pattern: square   # sinusoidal, circular, figure8, square, static
  center: [0.70, 0.0, 0.50]
  amplitude_y: 0.20
  amplitude_z: 0.10
  frequency: 0.2           # Hz
```

### Marker Detection

```yaml
aruco:
  marker_size: 0.05          # physical marker size in metres (border included) — must be exact
  dictionary: DICT_4X4_50
  tracking_id: 0

  # Detection tuning — important for small or distant markers
  min_marker_perimeter_rate: 0.02   # lower = detect smaller/more distant markers (default 0.03)
  error_correction_rate: 0.6        # raise to 0.8 if marker is blurry or poorly printed
```

**`marker_size` must match everywhere:** the YAML value, the Gazebo `model.sdf` box
dimensions (`z1_aruco/models/aruco_marker_0/model.sdf`), and the physical printed marker
must all use the same size. A mismatch compresses all pose estimates by the ratio
`yaml_size / actual_size` — the arm target will be wrong and tracking will fail.

For small markers (≤ 50 mm) the default OpenCV `minMarkerPerimeterRate` of 0.03 can cause
missed detections at distances above ~0.8 m. Set it to 0.02 or lower. See
[docs/STARTUP.md](docs/STARTUP.md) for a detection-range reference table.

### Arm Tracker

```yaml
arm_tracker:
  smoothing_alpha: 0.10    # low-pass filter (0.0=frozen, 1.0=instant snap)
  fixed_x: 0.25            # fixed forward reach (metres) — arm only follows Y and Z
  joint_kp: 150.0          # PD position gain for MotorCmd
  joint_kd: 3.0            # PD velocity gain for MotorCmd
  workspace:
    y: [-0.35, 0.35]
    z: [0.10, 0.75]
```

### roslaunch Arguments

```bash
# Simulation mode
roslaunch z1_aruco z1_aruco_tracking.launch \
  camera_mode:=end_effector \   # end_effector | fixed
  paused:=true \
  gui:=true \
  headless:=false \
  UnitreeGripperYN:=true

# Real camera mode (RealSense D435)
roslaunch z1_aruco z1_real_camera_tracking.launch \
  realsense_serial:="" \        # leave empty for first connected D435
  paused:=true \
  gui:=true \
  headless:=false \
  UnitreeGripperYN:=true

# Real camera mode (ZED 2) — device/resolution/fps/eye come from zed_camera:
# in aruco_tracking.yaml, not roslaunch args
roslaunch z1_aruco z1_real_camera_tracking_zed.launch \
  paused:=true \
  gui:=true \
  headless:=false \
  UnitreeGripperYN:=true
```

---

## Troubleshooting

```txt
Error: "cannot open display"
- Cause: X11 forwarding not enabled on the host
- Solution: Run "xhost +local:docker" before starting the container

Error: arm_tracker logs DRY RUN — not sending commands
- Cause: unitree_arm_interface Python binding failed to import (missing LD_LIBRARY_PATH)
- Solution: Rebuild the image — ENV LD_LIBRARY_PATH is set in the Dockerfile

Error: aruco_detector node starts but /aruco/marker_detected stays false
- Cause: Camera not publishing, or marker not visible in camera frame
- Solution: Check "rostopic hz /camera/color/image_raw" — should be ~30 Hz

Error: Gazebo opens but arm is frozen
- Cause: Gazebo starts paused by default
- Solution: Run z1_unpause or click Play in the Gazebo GUI

Error: gzclient (Gazebo GUI) dies with "Aborted (core dumped)", exit code 134 (Windows)
- Cause: usually no X server is actually running on the host (VcXsrv not started, or
         closed) — the client aborts on connect failure instead of a clean X11 error.
         gzserver and everything else keeps running; only the GUI window dies, easy to
         miss among other startup logs. (If VcXsrv IS running, check "Native opengl" is
         unchecked / "-wgl" is passed — see Quick Start step 1 — as a second cause.)
- Solution: Start VcXsrv (see Quick Start step 1) BEFORE docker run, and verify with
            "Get-NetTCPConnection -LocalPort 6000 -State Listen" on the host.

Error: zed_camera logs [FATAL] Could not open /dev/video0
- Cause: /dev/videoX is crw------- (root-only); the container's default non-root user
         can't open it
- Solution: Add --user root to docker run (see Quick Start 2c)

Error: D435 not found — RS2_USB_STATUS_ACCESS (real camera mode, Linux)
- Cause: udev rules not installed on host, or wrong USB bus passed to docker run
- Solution: Install udev rules (see docs/STARTUP.md section 4.1), unplug/replug D435,
            check bus with "lsusb | grep RealSense" and update --device path

Error: D435 not found — RS2_USB_STATUS_ACCESS (real camera mode, Windows)
- Cause: Windows seeing the camera does not mean WSL2/Docker Desktop sees it —
         there is no native USB passthrough into the WSL2 VM. Even once
         "usbipd attach" succeeds, three things commonly still break it:
         (1) the docker run command is missing --device entirely, (2) the bus
         number is stale (it's reassigned on every "usbipd attach" — recheck
         it), or (3) the container was already running when you attached the
         device (devices are snapshotted at "docker run" time — recreate it).
- Solution: Install usbipd-win, "usbipd bind"/"usbipd attach --wsl" the
            device (re-run attach after every unplug/replug/reboot — it does
            not persist), confirm it landed in the right WSL distro with
            "wsl -d docker-desktop -- lsusb" (not just your default distro),
            then update --device to match that bus number on a freshly
            started container. See docs/DOCKER_CMDS.md#usb-device-passthrough-realsense-d435

Error: TF transform failed — extrapolation into the future (real camera mode)
- Cause: use_sim_time mismatch between Gazebo sim clock and D435 wall-clock timestamps
- Solution: Use z1_real_camera_tracking.launch — it sets use_sim_time:=false

Error: video frames garbled/missing, or ArUco detection fails intermittently (Windows only)
- Cause: usbipd-win tunnels USB over TCP; isochronous video transfers (what webcams use)
         frequently corrupt over that tunnel at high resolution/framerate — confirmed with
         a ZED 2 at its default 2560x720@60fps mode (every frame corrupted) vs. its lowest
         mode, 1344x376@15fps (zero corrupted frames over the same test).
- Solution: Test raw frames with ffplay and drop to the camera's lowest-bandwidth UVC mode.
            See docs/DOCKER_CMDS.md#diagnosing-corruptedgarbled-video-over-usbipd-windows
```

---

## Gallery

### RViz — arm tracking marker in real time

![RViz ArUco tracking](assets/z1-aruco-rviz.png)

### Workshop UI — live knobs (Streamlit)

![Workshop UI](assets/webUI.png)

### ROS node graph

![ROS node graph](assets/rosgraph.png)

### Gazebo simulation (video)

[z1-aruco-gazebo.webm](assets/z1-aruco-gazebo.webm)

### RViz visualization (video)

[z1-aruco-rviz.webm](assets/z1-aruco-rviz.webm)

### Real camera tracking — D435 + Gazebo arm (video)

[z1-aruco-realsense.webm](assets/z1-aruco-realsense.webm)

---

## AI-Assistance

Parts of this workspace were developed with the assistance of large language models
(Claude by Anthropic).

AI-generated code in this workspace controls a simulated robotic arm. Before using
any part of this code with real hardware:

- Review all motion limits and workspace bounds in `aruco_tracking.yaml`
- Validate Cartesian speed limits (`cartesian_speed`) against your physical setup
- Test incrementally at low speeds before enabling full tracking
- This code is provided for simulation and educational use — use caution when
  adapting it for real hardware applications
