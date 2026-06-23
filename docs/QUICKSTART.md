# Quick Start — ROS Noetic Z1 Simulation

Every run mode (simulation, RealSense D435, ZED 2), every `docker run`/`docker exec`
flag, every roslaunch argument, every config option, and every in-container alias —
spelled out, Linux and Windows.

## 1. Build the Docker Image

```bash
git clone --recursive <repo-url> ros_docker
cd ros_docker
docker build -t ros-z1-aruco-real .
```

Already cloned without `--recursive`? Populate submodules first:

```bash
git submodule update --init --recursive
```

`sdk_z1`, `z1_controller`, and `unitree_ros` are git submodules — the build copies
their contents directly and fails if they're empty. First build takes 10-15 minutes.

**Rebuild whenever you change a launch file, node script, `CMakeLists.txt`, or the
Dockerfile** — none of the run commands below mount source live, so edits on the
host have no effect until the next `docker build`.

---

## 2. X11 Forwarding (Host)

**Linux:**

```bash
xhost +local:docker
```

**Windows:** no native X server — install [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

```powershell
& "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac -wgl
```

Or `XLaunch` → *Multiple windows* → display `0` → *Start no client* → check
**Disable access control**, uncheck **Native opengl**.

`-wgl` / unchecking Native opengl is not optional — without it, `gzclient` (the
Gazebo GUI) crashes with `Aborted (core dumped)`, exit code 134, as soon as it
tries to render. If you ever see that crash, this is the first thing to check —
along with confirming VcXsrv is actually running at all
(`Get-NetTCPConnection -LocalPort 6000 -State Listen` should show a listener).

Allow VcXsrv through Windows Firewall when prompted (private **and** public —
Docker Desktop's virtual adapter is usually classified public). If you dismissed
the prompt, remove the resulting block rule and add an allow rule from an
**elevated** PowerShell:

```powershell
Get-NetFirewallApplicationFilter | Where-Object { $_.Program -like "*vcxsrv*" } |
  Get-NetFirewallRule | Where-Object { $_.Action -eq "Block" } | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName "VcXsrv X server (allow)" -Direction Inbound `
  -Program "C:\Program Files\VcXsrv\vcxsrv.exe" -Action Allow -Profile Any
```

In every `docker run`/`docker exec` command below, Windows replaces the Linux X11
flags `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` with just
`-e DISPLAY=host.docker.internal:0.0` (the `-v .../X11-unix` mount doesn't exist
on Windows — drop it entirely). Verify the connection with `xclock` (window should
appear) and `glxgears` (confirms OpenGL).

---

## 3. Run Modes

### 3a. Simulation only — no real camera

Gazebo simulates the camera and animates the ArUco marker itself.

**Linux:**

```bash
docker run -it --rm \
  --name ros-z1-real \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros-z1-aruco-real bash
```

**Windows** (single line — PowerShell does not treat a trailing `\` as a line continuation):

```powershell
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 ros-z1-aruco-real bash
```

Inside the container:

```bash
z1_sim        # launches Gazebo + ArUco tracking simulation
z1_unpause    # unpause physics (separate terminal)
z1_rviz       # open RViz (separate terminal)
```

### 3b. Real camera — RealSense D435

USB passthrough differs completely between Linux (native device passthrough) and
Windows (the device must be bridged into Docker Desktop's WSL2 VM via `usbipd-win`).

**Linux — find the bus, then pass it through:**

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

Host prerequisite (Intel's apt repo doesn't support Ubuntu 24/Noble — install udev
rules from upstream instead), one-time:

```bash
sudo curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

If the driver logs `RS2_USB_STATUS_ACCESS`, the udev rules haven't taken effect —
unplug/replug the D435. Fallback (workshop/dev use only): add `--privileged`.

**Windows — bridge via `usbipd-win`:**

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list                          # find the D435's BUSID, e.g. 4-3
usbipd bind --busid 4-3
usbipd attach --wsl --busid 4-3
```

`usbipd attach --wsl` attaches to whichever WSL distro is currently default — not
necessarily `docker-desktop`, the distro that actually backs Docker Desktop's
containers. Confirm it landed there, and note the live bus number (it's reassigned
on every attach — never reuse a number from a previous run):

```powershell
wsl -d docker-desktop -- lsusb
# example: Bus 002 Device 002: ID 8086:0b07 Intel Corp. RealSense D435
```

```powershell
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 --device /dev/bus/usb/002:/dev/bus/usb/002 ros-z1-aruco-real bash
```

Three things that silently break this even when the device is confirmed reachable
in `docker-desktop`: (1) `--device` missing from the actual command you ran, (2) a
stale bus number from an earlier session, (3) the container was already running
when you attached the device (devices are snapshotted at `docker run` time — recreate it).

Inside the container:

```bash
z1_real       # launches Gazebo arm + real D435 tracking
z1_unpause    # unpause physics (separate terminal)
z1_rviz       # open RViz (separate terminal)
```

### 3c. Real camera — ZED 2 (no ZED SDK, no GPU/CUDA)

`aruco_detector_node` only subscribes to `/camera/color/image_raw` +
`/camera/color/camera_info` — it doesn't care which camera feeds them.
`zed_camera_node.py` reads the ZED 2's raw UVC feed directly via OpenCV/V4L2,
crops it to one eye, and republishes on those same topics.

**Windows shortcut — `start_zed_sim.sh`** (repo root, run from Git Bash, NOT the
legacy `C:\Windows\system32\bash.exe` WSL launcher — check which one `bash`
resolves to with `Get-Command bash -All`; if it's the WSL one, invoke Git Bash's
real executable directly instead, from PowerShell):

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./2.CodeRepos/ros_z1_sim_marker-real-camera/start_zed_sim.sh --dictionary DICT_5X5_50 --marker-size 0.05 --tracking-id 0
```

Or, already inside a real Git Bash window:

```bash
bash start_zed_sim.sh --dictionary DICT_5X5_50 --marker-size 0.05 --tracking-id 0
```

What it does, every run: checks VcXsrv is running with `-wgl` (restarts it if not),
checks the ZED 2 is bound + attached via `usbipd` (binds/attaches if not — needs an
elevated shell the *first* time a device is bound), reloads the `uvcvideo` kernel
module in the `docker-desktop` WSL VM (doesn't persist across restarts), reads the
*live* USB bus number (never a stale hardcoded one), removes any stale
`ros-z1-real` container, starts the container with `--user root` and all three
`--device` flags, and auto-opens RViz in the background once ROS is up inside it
(polls `docker exec ... rostopic list` — no separate manual command needed).

Full flag reference:

```bash
bash start_zed_sim.sh [options] [launch_alias]

  --dictionary NAME      ArUco dictionary. One of:
                         DICT_4X4_50 DICT_4X4_100 DICT_5X5_50 DICT_6X6_250 DICT_7X7_1000
                         Default: DICT_5X5_50 (script default — aruco_tracking.yaml's own
                         default is DICT_4X4_50; this script always passes the flag
                         explicitly, overriding the yaml either way)
  --marker-size METRES   Physical marker size, border included. Default: 0.05
  --tracking-id ID       The single marker ID the arm follows — must be one of your
                         printed markers' IDs. Default: 0
                         (aruco_tracking.yaml's marker_ids list is documentation only;
                         the detector never reads it — only one ID is ever tracked)
  launch_alias           Defaults to z1_real_zed. Pass z1_real_zed_headless to skip
                         the Gazebo GUI window (RViz still opens)

# Examples:
bash start_zed_sim.sh                                                  # all defaults
bash start_zed_sim.sh z1_real_zed_headless                             # no Gazebo GUI
bash start_zed_sim.sh --dictionary DICT_5X5_50 --marker-size 0.03 --tracking-id 1
```

An unrecognized `--dictionary` is rejected before launching anything;
`aruco_detector_node` itself also fails loudly (instead of silently falling back to
`DICT_4X4_50`) if the resolved value is ever invalid.

**What the script actually runs** (spelled out, for when you need to run a step
manually or adapt it):

```bash
# usbipd: bind + attach (BUSID from `usbipd list`, re-run after every unplug/replug/reboot)
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3

# Load uvcvideo in the docker-desktop WSL VM so /dev/videoX appears (not persistent)
wsl -d docker-desktop -- modprobe uvcvideo

# Confirm the live bus number before using it below — it's reassigned on every attach
wsl -d docker-desktop -- lsusb

# Main container — --user root is required: /dev/videoX is crw------- (root-only),
# zed_camera_node fails with "[FATAL] Could not open /dev/video0" without it
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 --device /dev/bus/usb/002:/dev/bus/usb/002 --device /dev/video0:/dev/video0 --device /dev/video1:/dev/video1 --user root ros-z1-aruco-real bash -ic "z1_real_zed aruco_dictionary:=DICT_5X5_50 aruco_marker_size:=0.03 aruco_tracking_id:=0"

# RViz, separate terminal — also needs --user root (the main process owns ~/.ros / ~/.rviz as root)
docker exec -it --user root -e DISPLAY=host.docker.internal:0.0 ros-z1-real bash -ic "z1_rviz"
```

**On Windows, video frequently arrives corrupted at full resolution** — `usbipd-win`
tunnels USB over TCP, and isochronous transfers (what webcams use) routinely corrupt
over that tunnel at high bandwidth. Symptom: a wall of
`Dequeued v4l2 buffer contains corrupted data (N bytes)` with `N` varying wildly
instead of staying constant. Fix: drop to the camera's lowest-bandwidth UVC mode —
confirmed clean for the ZED 2 at `1344x376@15fps` (its lowest mode), corrupted at
its default `2560x720@60fps`. Test any mode with `ffplay` before trusting it; full
walkthrough: [DOCKER_CMDS.md — Diagnosing corrupted video over usbipd](DOCKER_CMDS.md#diagnosing-corruptedgarbled-video-over-usbipd-windows).

Mode/device/eye are set in `zed_camera:` in `z1_aruco/config/aruco_tracking.yaml`
(see [§5](#5-config-reference-arucotrackingyaml) below) — not on the command line.

---

## 4. roslaunch Argument Reference

### `z1_aruco_tracking.launch` — simulation, simulated camera + animated marker

```bash
roslaunch z1_aruco z1_aruco_tracking.launch [arg:=value ...]
```

| Argument | Default | Options | Description |
| --- | --- | --- | --- |
| `camera_mode` | `end_effector` | `end_effector`, `fixed` | Camera on wrist (URDF) or static in world (Gazebo model) |
| `paused` | `true` | `true`, `false` | Start Gazebo paused — press Play in GUI to start physics |
| `gui` | `true` | `true`, `false` | Show the Gazebo GUI window |
| `headless` | `false` | `true`, `false` | No rendering — fastest, overrides `gui:=true` |
| `UnitreeGripperYN` | `true` | `true`, `false` | Include gripper controller |
| `debug` | `false` | `true`, `false` | Gazebo debug output |

### `z1_real_camera_tracking.launch` — real camera, RealSense D435

```bash
roslaunch z1_aruco z1_real_camera_tracking.launch [arg:=value ...]
```

| Argument | Default | Description |
| --- | --- | --- |
| `realsense_serial` | `""` | D435 serial number — empty uses the first connected device |
| `paused` / `gui` / `headless` / `UnitreeGripperYN` / `debug` | same as above | |

`use_sim_time` is fixed at `false` (not overridable) — the D435 driver timestamps
in wall-clock time, while Gazebo runs sim time; mixing the two causes
`TF transform failed — extrapolation into the future`.

### `z1_real_camera_tracking_zed.launch` — real camera, ZED 2

```bash
roslaunch z1_aruco z1_real_camera_tracking_zed.launch [arg:=value ...]
```

| Argument | Default | Description |
| --- | --- | --- |
| `aruco_dictionary` | `DICT_4X4_50` | Overrides `aruco/dictionary` from the yaml for this run |
| `aruco_marker_size` | `0.05` | Overrides `aruco/marker_size` (metres, border included) |
| `aruco_tracking_id` | `0` | Overrides `aruco/tracking_id` — the one ID the arm follows |
| `paused` / `gui` / `headless` / `UnitreeGripperYN` / `debug` | same as above | |

These three defaults are the **yaml's** defaults, not `start_zed_sim.sh`'s — the
script always passes all three explicitly regardless (see §3c).
`use_sim_time` is fixed at `false`, same reason as the D435 launch file above.

### `z1.launch` (`unitree_gazebo`) — standard Z1 simulation, no ArUco

```bash
roslaunch unitree_gazebo z1.launch [arg:=value ...]
```

| Argument | Default | Options | Description |
| --- | --- | --- | --- |
| `wname` | `earth` | `earth`, `space`, `stairs`, `building_editor_models` | Gazebo world file |
| `paused` | `true` | `true`, `false` | Start Gazebo paused |
| `gui` | `true` | `true`, `false` | Show the Gazebo GUI window |
| `headless` | `false` | `true`, `false` | No rendering |
| `UnitreeGripperYN` | `true` | `true`, `false` | Include gripper controller |
| `user_debug` | `false` | `true`, `false` | FSM debug output |
| `debug` | `false` | `true`, `false` | Gazebo debug output |

```bash
roslaunch unitree_gazebo z1.launch wname:=stairs paused:=false
roslaunch unitree_gazebo z1.launch headless:=true UnitreeGripperYN:=false
```

---

## 5. Config Reference (`aruco_tracking.yaml`)

`z1_aruco/config/aruco_tracking.yaml`, loaded once at launch time. Edit on the host
and relaunch — no rebuild needed (this file *is* mounted/copied per build, but
roslaunch re-reads it fresh on every `roslaunch` inside an already-built image; a
rebuild is only needed if you change the *path* or add a new file).

| Section | Key fields | Notes |
| --- | --- | --- |
| `aruco:` | `dictionary`, `marker_ids`, `marker_size`, `tracking_id`, detection tuning | `marker_ids` is documentation only — never read by code. Only `tracking_id` (a single ID) is actually tracked. `dictionary`/`marker_size`/`tracking_id` are overridable per-launch (§4) |
| `camera:` | `mode`, `fixed_pose`, `end_effector_offset`, `width`/`height`/`fps`/`fov`, `show_debug_image` | Simulated-camera settings (Gazebo plugin). Not used in real-camera modes |
| `zed_camera:` | `device`, `width`, `height`, `fps`, `eye`, `hfov_deg` | ZED 2 bridge settings (§3c). `width`/`height`/`fps` must be a mode reported by `v4l2-ctl -d /dev/video0 --list-formats-ext` and confirmed clean with `ffplay` — see corruption note in §3c |
| `marker:` | `motion_pattern`, `center`, `amplitude_y`/`amplitude_z`, `frequency`, `radius` | Simulated-marker animation. Not used in real-camera modes (no simulated marker exists) |
| `arm_tracker:` | `cartesian_speed`, `proportional_gain`, `smoothing_alpha`, `workspace`, `enabled`, `track_orientation` | Tracking/control tuning, all modes |
| `joint_limits:` | `j1`–`j6` | Z1 Pro physical joint range, radians |
| `simulation:` | `paused`, `gui`, `headless` | Mirrors the roslaunch args of the same name |

---

## 6. Alias Reference (inside the container)

```bash
# --- Launch (simulation) ---
z1_sim                # roslaunch z1_aruco z1_aruco_tracking.launch
z1_sim_headless        # ... headless:=true paused:=false

# --- Launch (real camera — RealSense D435) ---
z1_real                # roslaunch z1_aruco z1_real_camera_tracking.launch
z1_real_headless        # ... headless:=true

# --- Launch (real camera — ZED 2) ---
z1_real_zed             # roslaunch z1_aruco z1_real_camera_tracking_zed.launch
z1_real_zed_headless    # ... headless:=true

# --- Physics ---
z1_unpause              # rosservice call /gazebo/unpause_physics

# --- Individual nodes (restart one without a full relaunch) ---
z1_detector             # rosrun z1_aruco_detector aruco_detector_node.py
z1_tracker              # rosrun z1_arm_tracker arm_tracker_node.py
z1_mover                # rosrun z1_aruco_detector marker_mover_node.py

# --- Workshop UI ---
z1_ui                   # streamlit run .../workshop_ui.py  → http://localhost:8501
z1_nb                   # jupyter lab .../Z1_Workshop_Companion.ipynb → http://localhost:8888

# --- Visualisation ---
z1_rviz                 # rviz -d .../z1_aruco_tracking.rviz
z1_camera               # image_view on /aruco/debug_image (ArUco overlay)
z1_camera_raw           # image_view on /camera/color/image_raw (no overlay)

# --- Diagnostics ---
z1_nodes                # rosnode list
z1_topics               # rostopic list
z1_pose                 # rostopic echo /aruco/marker_pose
z1_detected             # rostopic echo /aruco/marker_detected
z1_joints               # rostopic echo /z1_gazebo/joint_states
```

All `z1_*` aliases need an **interactive** shell to be visible — `bash -ic "..."`,
not `bash -c "..."` (the `-i` sources `~/.bashrc`, where they're defined).

---

## 7. Multi-Terminal Manual Workflow (no aliases/script)

Useful when adapting a step instead of running it as-is.

```bash
# Terminal 1 — launch
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
roslaunch z1_aruco z1_aruco_tracking.launch
```

**Linux** — Terminal 2, camera feed with ArUco overlay:

```bash
docker exec -e DISPLAY=$DISPLAY -it ros-z1-real bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source ~/catkin_ws/devel/setup.bash && \
   rosrun image_view image_view image:=/aruco/debug_image"
```

**Windows** (`docker exec` needs its own `-e DISPLAY` — it does not reliably
inherit the value from `docker run`):

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-real bash -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosrun image_view image_view image:=/aruco/debug_image"
```

Terminal 3 — RViz:

```bash
docker exec -e DISPLAY=$DISPLAY -it ros-z1-real bash -ic "z1_rviz"
```

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-real bash -ic "z1_rviz"
```

Pre-configured RViz displays: Grid, RobotModel (Z1, Fixed Frame `world`), TF (all
frames incl. `camera_color_optical_frame`/`link06`), Camera ArUco overlay
(`/aruco/debug_image`), Camera raw (`/camera/color/image_raw`, disabled by
default), Camera Frame axes, Marker Pose (red arrow, world frame).

Terminal 4 — monitor detection and tracking:

```bash
docker exec -it ros-z1-real bash
rostopic echo /aruco/marker_detected   # is the marker being detected?
rostopic echo /aruco/marker_pose       # where, in world frame?
rosnode list
rostopic hz /camera/color/image_raw
rostopic hz /aruco/marker_pose
```

---

## 8. Verify the Environment

```bash
echo $ROS_PACKAGE_PATH                  # ROS is sourced
rospack find z1_description             # Unitree packages are found
rospack find unitree_gazebo
rospack find unitree_legged_msgs
ls ~/z1_controller/build/z1_ctrl        # Z1 binaries exist
ls ~/sdk_z1/build/
```

---

## 9. Cleanup

```bash
docker rm -f ros-z1-real      # force stop and remove the container
xhost -local:docker           # Linux only — revoke X11 access
```
