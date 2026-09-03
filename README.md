# ROS Noetic Z1 Arm Simulation with Hand Teleoperation

Gazebo simulation of the Unitree Z1 robotic arm following an operator's hand via
a webcam, RealSense D435, or ZED 2 camera, using MediaPipe Hands for real-time
Cartesian control, alongside a live Streamlit control panel.

Workshop UI ([live control panel](z1_teleop/scripts/teleop_ui.py)).

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Python 3.8](https://img.shields.io/badge/Python-3.8-blue)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Hands-orange)
![Docker](https://img.shields.io/badge/Docker-required-blue)

![Unitree Z1](assets/z1.jpg)

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Operator conventions](#operator-conventions)
- [Architecture](#architecture)
- [Install and build](#install-and-build)
- [Quick start](#quick-start)
- [Camera sourcing — Windows reality check](#camera-sourcing--windows-reality-check)
- [Configuration](#configuration)
- [ROS topics](#ros-topics)
- [Shell aliases](#shell-aliases)
- [Troubleshooting](#troubleshooting)
- [AI-assistance](#ai-assistance)

---

## Overview

Gazebo simulation of the Unitree Z1 arm with a full perception-to-control
pipeline: MediaPipe Hands tracks an operator's hand via camera, and the arm
follows it in real time using Cartesian control.

---

## Key Features

- Full Gazebo simulation of the Unitree Z1 6-DOF arm with gripper
- MediaPipe Hands tracking via OpenCV and cv_bridge, published as world-frame
  Cartesian targets
- RealSense D435 or ZED 2 real-camera tracking, alongside a webcam or a
  looping video file for hardware-free demos
- Operator conventions: open-palm/fist clutch to engage or freeze the arm,
  optional pinch-to-gripper control
- Two control laws: a default Cartesian "hand mirror" (no calibration, no TF,
  no depth estimate) and an opt-in joint-mirror mode that drives the Z1
  joints directly from MediaPipe Pose
- Smooth Cartesian arm tracking with low-pass (OneEuro) filtering and
  workspace clamping
- Centralized YAML configuration for hand mapping, gesture conventions,
  camera mode, and workspace limits
- Pre-configured RViz layout with robot model, TF tree, camera feed, and
  hand-tracking debug overlay
- Software rendering baked in, works without NVIDIA drivers or GPU
  passthrough
- Live Streamlit control panel for tuning gesture and joint-mirror
  parameters without restarting

---

## Operator conventions

> Cheat sheet: open palm to drive, fist to freeze. Show your right hand.

1. **Handedness.** The station tracks one hand (`gesture/hand`, default
   `right`). The feed is mirrored, selfie-style, so "right" means your right
   hand as you see yourself on screen.
2. **Clutch.** The arm only follows while engaged:
   - Open palm (four or more fingers extended) engages tracking.
   - Closed fist (one finger or fewer) freezes the arm in place, so you can
     reposition your hand without dragging the arm along with it.
   - A short frame hysteresis keeps this from flickering on borderline poses.
3. **Starts frozen.** The arm always comes up frozen, both on launch and any
   time it loses the hand. Show an open palm to (re)engage. It never lurches
   on start.
4. **Gripper.** A pinch between thumb and index finger drives the gripper:
   fingers together closes it, apart opens it (`gesture/gripper: pinch`,
   on by default).
5. **Lost hand.** If the hand drops out of frame for `gesture/lost_frames`
   frames, the arm freezes and the clutch resets.

The `/hand/debug_image` overlay shows the tracked landmarks, finger count,
clutch state (ENGAGED or FROZEN), the current Y/Z target, and the gripper
value. The border goes green while driving, red while frozen.

---

## Architecture

```txt
 Real camera (webcam / ZED-2 UVC / D435 / video file)
        |  /camera/color/image_raw          (sensor_msgs/Image)
        v
 hand_detector_node  --> /hand/target_pose      (geometry_msgs/PoseStamped, world)
   (MediaPipe Hands) --> /hand/tracking_active   (std_msgs/Bool)  clutch + detection gate
                     --> /hand/debug_image       (sensor_msgs/Image, overlay)
                     --> /hand/gripper_cmd        (std_msgs/Float64, pinch 0..1)
        |
        v
 arm_tracker_node  --> /z1_gazebo/JointXX_controller/command  (MotorCmd x6)
                   --> /z1_gazebo/gripper_controller/command   (MotorCmd)
        ^
 Gazebo: Z1 arm + joint controllers + robot_state_publisher (TF)
```

The image source is swappable between the Gazebo plugin, a webcam, a D435, a
ZED-2 over UVC, or a video file, because every downstream node only ever
looks at the `/camera/color/image_raw` topic.

The default control law is a Cartesian "hand mirror." No calibration, no TF,
no depth estimate, just a direct 2D-to-2D map:

```txt
u, v = normalized image coords of the tracked hand point  (v grows downward)
Y =  map(u, 0..1 -> y_max..y_min)   # mirrored: hand right -> arm right
Z =  map(v, 0..1 -> z_max..z_min)   # inverted: image v is top-down
X =  fixed_x                        # depth held constant
```

Rendering runs on Mesa software rendering (llvmpipe) by default, so no GPU is
needed for Gazebo or RViz, and MediaPipe Hands runs fine on CPU too.

**Packages**

| Package | Role | Key files |
| --- | --- | --- |
| `z1_hand_detector` | Perception | `hand_detector_node.py`, `image_source_node.py`, `zed_camera_node.py` |
| `z1_arm_tracker` | Control | `arm_tracker_node.py` |
| `z1_teleop` | Launch / world / xacro / rviz / config | `launch/*.launch`, `worlds/teleop.world`, `xacro/z1_teleop_robot.xacro`, `rviz/z1_teleop.rviz`, `config/teleop.yaml` |
| `sdk_z1` (submodule) | Z1 SDK (C++) — IK model | upstream `unitreerobotics/z1_sdk` |
| `z1_controller` (submodule) | Gazebo `sim_ctrl` bridge | upstream `unitreerobotics/z1_controller` |
| `unitree_ros` (submodule) | URDF / descriptions / `unitree_legged_msgs` | upstream `unitreerobotics/unitree_ros` |

---

## Install and build

### Prerequisites
- Docker (Docker Desktop on Windows/macOS).
- An X server for the GUI: VcXsrv on Windows (`-wgl`), XQuartz on macOS, native X
  on Linux.
- For the ZED/D435 real-camera paths on Windows: `usbipd-win`.

### Build the image

```bash
git clone --recursive https://github.com/billisandr/ros_z1_teleop.git
cd ros_z1_teleop
docker build -t ros-z1-teleop .
```

If you cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

The first build takes about 10 to 15 minutes: it compiles the Z1 SDK and the
`z1_controller` Gazebo bridge, then runs `catkin_make`. An import smoke test
runs at the end and fails the build loudly if MediaPipe, OpenCV, and
`cv_bridge` don't coexist cleanly.

---

## Quick start

See [docs/QUICKSTART.md](docs/QUICKSTART.md) for the full walkthrough and
[docs/MINIMAL_QUICKSTART.md](docs/MINIMAL_QUICKSTART.md) for commands only.

### A. Hardware-free demo (looping video file)

Start the X server, then:

```bash
docker run -it --rm --name ros-z1-teleop -e DISPLAY=host.docker.internal:0.0 \
    ros-z1-teleop bash -ic "z1_teleop image_source:=video:/path/to/hand.mp4"
```

In another terminal, release physics and open RViz:

```bash
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
docker exec -it -e DISPLAY=host.docker.internal:0.0 ros-z1-teleop bash -ic "z1_rviz"
```

### B. Local webcam (Linux/macOS hosts)

Pass the camera device into the container and use the default `webcam` source:

```bash
docker run -it --rm --name ros-z1-teleop --device /dev/video0:/dev/video0 \
    -e DISPLAY=$DISPLAY ros-z1-teleop bash -ic "z1_teleop"
```

On Windows, the built-in webcam generally can't be passed into Docker at all.
Use a video file or the ZED path instead; see the camera section below.

### C. ZED 2 over USB (recommended on Windows)

A one-shot script, run from Git Bash on the Windows host, handles VcXsrv,
`usbipd` bind/attach, `uvcvideo`, starting the container, and opening RViz:

```bash
bash start_teleop.sh                 # launches z1_teleop_zed
```

Then release physics:

```bash
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
```

---

## Camera sourcing — Windows reality check

Getting a camera into the container is the part of this project that gave us
the most grief on Windows, so here's what actually works:

- Laptop-integrated webcams generally cannot be `usbipd`-attached into the
  Docker Desktop WSL2 VM. Don't assume `/dev/video0` inside the container is
  your built-in webcam, because it usually isn't reachable at all.
- In order of how much trouble they are:
  1. **A bundled or looping video file** (`image_source:=video:/path.mp4`).
     Zero hardware needed, always works, and it's the easiest way to bring
     the station up the first time or to demo it without a camera on hand.
  2. **A ZED 2 or other external USB cam via `usbipd`.** `start_teleop.sh`
     handles the bind/attach/`uvcvideo` sequence for you. Use the
     lowest-bandwidth UVC mode (`1344x376@15fps` is the one we found
     corruption-free; see [docs/DOCKER_CMDS.md](docs/DOCKER_CMDS.md)).
  3. **Run MediaPipe on the host** and publish over the ROS network, pointing
     `ROS_MASTER_URI` at the container. This works but is more setup than
     it's worth unless you're already fighting the other two paths.
- On Linux and macOS, this whole problem mostly doesn't exist: native
  `--device /dev/video0` works for a built-in or USB webcam.

---

## Configuration

All knobs live in [z1_teleop/config/teleop.yaml](z1_teleop/config/teleop.yaml),
loaded at launch. Highlights:

| Block | Key | Meaning |
| --- | --- | --- |
| `control` | `mode` | `cartesian` (default) or `joint_mirror` (staged; see [docs/JOINT_MIRROR.md](docs/JOINT_MIRROR.md)) |
| `hand` | `image_source` | `webcam` / `video:/path.mp4` / `external` |
| `hand` | `flip_horizontal` | selfie view so handedness matches the operator |
| `mapping` | `tracked_point` | `wrist` or `palm_centroid` |
| `mapping` | `fixed_x`, `y_range`, `z_range` | image→world map (mirror/invert) |
| `gesture` | `hand` | `right` / `left` / `either` |
| `gesture` | `clutch` | `palm_fist` / `always_on` / `pinch_hold` |
| `gesture` | `gripper` | `pinch` / `none` |
| `gesture` | `lost_frames`, `hysteresis_frames` | freeze / flicker control |
| `smoothing` | `min_cutoff`, `beta` | OneEuro filter on (u,v) |
| `arm_tracker` | `workspace`, `smoothing_alpha`, `enable_gripper` | control limits + gripper |
| `scene_objects` | `enabled`, `static`, `objects` | spawn cubes/pyramids/cylinders as interaction props (`enabled: false` = clean scene) |

Edit the YAML and relaunch (or `docker cp` it into a running container). No
rebuild needed.

---

## ROS topics

| Topic | Type | Description |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Camera feed (any source) |
| `/hand/target_pose` | `geometry_msgs/PoseStamped` | World-frame target (X fixed) |
| `/hand/tracking_active` | `std_msgs/Bool` | True only when hand present + engaged |
| `/hand/debug_image` | `sensor_msgs/Image` | Landmark + HUD overlay |
| `/hand/gripper_cmd` | `std_msgs/Float64` | Pinch → gripper, 0 (closed) .. 1 (open) |
| `/z1_gazebo/JointXX_controller/command` | `unitree_legged_msgs/MotorCmd` | Per-joint command |
| `/z1_gazebo/gripper_controller/command` | `unitree_legged_msgs/MotorCmd` | Gripper command |

---

## Shell aliases

Available inside the container (defined in `~/.bashrc`):

| Alias | Action |
| --- | --- |
| `z1_teleop` | Launch the station (webcam/video sim) |
| `z1_teleop_zed` | Launch with the ZED 2 bridge |
| `z1_unpause` | Release Gazebo physics |
| `z1_rviz` | Open the preconfigured RViz |
| `z1_hand` | Run the hand detector alone |
| `z1_tracker` | Run the arm tracker alone |
| `z1_cam` | View `/hand/debug_image` |
| `z1_ui` | Launch the live control-panel UI (browser, port 8501) |
| `z1_target` / `z1_active` / `z1_gripper` | Echo the hand topics |
| `z1_joints` | Echo `/z1_gazebo/joint_states` |

---

## Live control panel (UI)

A browser-based control panel, built with Streamlit, lets you tune the
station live, without touching YAML or restarting anything. It sets ROS
params, and the detector and arm tracker re-read the "soft" knobs on every
loop. It's especially handy for joint-mirror calibration
([docs/JOINT_MIRROR.md](docs/JOINT_MIRROR.md)), which otherwise means a lot
of edit-relaunch-repeat cycles.

Start the container with port 8501 published, then run `z1_ui`:

```bash
# the ZED start script already publishes 8501; for a manual run add -p 8501:8501
docker run -it --rm --name ros-z1-teleop -p 8501:8501 \
    -e DISPLAY=host.docker.internal:0.0 ros-z1-teleop bash -ic "z1_teleop" &
docker exec -it ros-z1-teleop bash -ic "z1_ui"
```

Open **http://localhost:8501**. There are tabs for live status, Cartesian
knobs, the joint-mirror per-joint map, gesture settings, and a presets/save
tab that writes current values back to `teleop.yaml`. Changing the control
mode, MediaPipe model settings, or the camera source still needs a relaunch,
and the UI flags this when it applies.

---

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| Gazebo/RViz window never appears | X server not running with `-wgl` (Windows) / `DISPLAY` not set |
| Arm doesn't move | Show an open palm to engage; check `/hand/tracking_active` is true and physics is unpaused (`z1_unpause`) |
| Wrong hand tracked | Set `gesture/hand`; confirm `hand/flip_horizontal: true` |
| No hand detected | Improve lighting; lower `hand/min_detection_confidence`; check `/hand/debug_image` |
| Garbled ZED video over usbipd | Use the `1344x376@15fps` mode; see [docs/DOCKER_CMDS.md](docs/DOCKER_CMDS.md) |
| `mediapipe`/`cv2` import error | Rebuild the image — the in-build smoke test must pass |

---

## AI-assistance

Parts of this workspace, including code, documentation, and debugging
support, were developed with the help of large language models (Claude, by
Anthropic). We reviewed and tested everything before it landed here, but if
you're adapting this for your own work, treat the AI-assisted parts with the
same scrutiny you'd give any code you didn't write yourself.

That matters more here than usual because this code, in its current form,
controls a simulated robotic arm, not a real one. Before pointing any of it
at actual hardware:

- Review the workspace bounds and gains in
  [z1_teleop/config/teleop.yaml](z1_teleop/config/teleop.yaml).
- Test incrementally at low speeds before enabling full tracking.
- Treat this as a simulation and education project, not a hardware-ready
  controller. Use caution, and validate independently, before it touches a
  real arm.

---

<sub>Technical University of Crete · SenseLab</sub>

![TUC](assets/TUC_logo.png) ![SenseLab](assets/senselab_logo.png)
