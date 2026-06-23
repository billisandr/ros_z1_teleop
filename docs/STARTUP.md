# Startup & Developer Reference — Z1 Hand-Teleop Station

Developer-oriented companion to [QUICKSTART.md](QUICKSTART.md). Covers the build,
hot-reload loop, rendering, architecture, and troubleshooting. For end-to-end run
steps use the quick starts; for the ZED/USB internals see
[DOCKER_CMDS.md](DOCKER_CMDS.md).

---

## 1. Build the image

```bash
git submodule update --init --recursive
docker build -t ros-z1-teleop .
```

The build compiles the Z1 SDK + `z1_controller` Gazebo bridge, runs
`catkin_make`, and runs an import smoke test
(`python3 -c "import cv2, mediapipe; from cv_bridge import CvBridge"`) that fails
the build if MediaPipe / OpenCV / `cv_bridge` don't coexist.

### Hot-reload a node or config without rebuilding

```bash
# copy an edited file into a running container
docker cp z1_hand_detector/src/hand_detector_node.py \
    ros-z1-teleop:/home/rosuser/catkin_ws/src/z1_hand_detector/src/hand_detector_node.py

# restart just that node inside the container
docker exec -it ros-z1-teleop bash -ic "rosnode kill /hand_detector; z1_hand"
```

`teleop.yaml` is loaded into the parameter server at launch; `docker cp` it in and
relaunch (or re-`rosparam load` + restart the node) to apply changes.

---

## 2. X11 forwarding (host — once per session)

- **Windows:** `& "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac -wgl`
  (the `-wgl` flag is required — Mesa software GL crashes gzclient without it).
- **Linux:** `xhost +local:docker` and run with `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`.
  Revoke with `xhost -local:docker` when done.
- **macOS:** XQuartz with "Allow connections from network clients".

Containers connect via `-e DISPLAY=host.docker.internal:0.0` on Windows/macOS.

---

## 3. Run modes

See [QUICKSTART.md](QUICKSTART.md) §4–6. In short:

```bash
# A. hardware-free demo (video file)
z1_teleop image_source:=video:/path/to/hand.mp4
# B. local webcam (Linux/macOS, --device /dev/video0)
z1_teleop
# C. ZED 2 over USB (Windows): from Git Bash on the host
bash start_teleop.sh
```

Gazebo starts paused — `z1_unpause` to release. `z1_rviz` opens the layout.

### What auto-starts in `z1_teleop_sim.launch`
Gazebo (`teleop.world`, arm only) · the Z1 URDF + joint/gripper controllers ·
`robot_state_publisher` · `image_source_node` (webcam/video) · `hand_detector_node`
· `arm_tracker_node`.

### Useful launch arguments

```bash
# pick the camera source
z1_teleop image_source:=video:/home/rosuser/hand.mp4
z1_teleop image_source:=webcam
# headless (no Gazebo GUI), unpaused
roslaunch z1_teleop z1_teleop_sim.launch headless:=true paused:=false
# disable the gripper passthrough
roslaunch z1_teleop z1_teleop_sim.launch   # then edit arm_tracker/enable_gripper in yaml
```

---

## 4. Configuration

All knobs are in [../z1_teleop/config/teleop.yaml](../z1_teleop/config/teleop.yaml):
`control`, `hand`, `mapping`, `gesture`, `smoothing`, `camera`, `zed_camera`,
`arm_tracker`, `joint_limits`, `simulation`. See the README configuration table.
Inspect what a node is actually using:

```bash
rosparam get /gesture/hand
rosparam get /mapping/fixed_x
rosparam get /arm_tracker/workspace
```

---

## 5. Rendering

The image uses Mesa software rendering by default — both `LIBGL_ALWAYS_SOFTWARE=1`
and `MESA_GL_VERSION_OVERRIDE=3.3` are baked into the Dockerfile via `ENV`.

**Why:** the Intel Meteor Lake GPU (0x7d67) is not supported by Mesa 21 in the
Ubuntu 20.04 base, and NVIDIA GLX rejects indirect X11 connections from Docker.
Mesa llvmpipe gives stable OpenGL 3.1 for Gazebo and RViz with no host GPU setup.
MediaPipe Hands runs on CPU, so no GPU is required anywhere.

To attempt hardware rendering on Linux, override at runtime:

```bash
docker run -it --rm --name ros-z1-teleop --gpus all --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY -e LIBGL_ALWAYS_SOFTWARE=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix ros-z1-teleop bash
```

Requires `nvidia-container-toolkit` on the host. **Windows:** `/dev/dri` does not
exist — software rendering (the default) is the only supported path.

---

## 6. Architecture

```txt
 camera image ──► /camera/color/image_raw
                        │
              hand_detector_node (MediaPipe Hands)
                 ├─► /hand/target_pose      (PoseStamped, world frame)
                 ├─► /hand/tracking_active  (Bool, clutch + detection gate)
                 ├─► /hand/debug_image      (Image, landmark + HUD overlay)
                 └─► /hand/gripper_cmd       (Float64, pinch 0..1)
                        │
                 arm_tracker_node (IK via Z1 SDK model, no UDP)
                 ├─► /z1_gazebo/JointXX_controller/command (MotorCmd ×6)
                 └─► /z1_gazebo/gripper_controller/command  (MotorCmd)
```

The default control law maps normalized image coords (u,v) → world (Y,Z) with X
fixed (PLAN.md §4.2) — **no TF and no `camera_info`** are needed. The arm-tracker
holds X fixed, follows Y/Z, low-pass filters, clamps to the workspace, and (when
settled) points the wrist at the target.

---

## 7. Troubleshooting

### Arm doesn't move
Show an **open palm** to engage (`/hand/tracking_active` must be `true`), and make
sure physics is unpaused (`z1_unpause`). The arm starts and re-acquires **frozen**.

### No hand detected
Check `z1_cam` (`/hand/debug_image`). Improve lighting; lower
`hand/min_detection_confidence`; ensure `hand/flip_horizontal: true` so handedness
matches; verify the image source actually publishes (`rostopic hz /camera/color/image_raw`).

### IK failures — `IK failed for target x:... y:... z:...`
The requested Cartesian position is outside the Z1's reachable workspace:
- `mapping/fixed_x` / `arm_tracker/fixed_x` above ~0.55 m at non-zero Y/Z can
  exceed reach — try `0.25`–`0.40`.
- Keep `mapping/y_range` / `mapping/z_range` within `arm_tracker/workspace`.
- IK uses `checkWorkspace=True` and refuses positions near joint limits.

### `unitree_legged_msgs` not found at Python import
`unitree_legged_msgs` lives in the nested submodule
`unitree_ros/unitree_ros_to_real`. Make sure you cloned with
`git submodule update --init --recursive`, then rebuild. Inside the container,
`source ~/catkin_ws/devel/setup.bash` exposes it.

### Jittery or laggy arm
Tune `smoothing/min_cutoff` and `smoothing/beta` (OneEuro on (u,v)) and
`arm_tracker/smoothing_alpha` (Cartesian target filter). `mapping/tracked_point:
palm_centroid` is steadier than `wrist`. Add a small `mapping/deadzone`.

### TF/clock errors in real-camera mode
Real cameras stamp images with wall-clock time. The launches set
`use_sim_time:=false` for that reason; the default hand-mirror needs no TF, so
clock-skew lookups that plagued the ArUco real-camera path do not occur here.

### Garbled ZED video over usbipd (Windows)
Use the `1344x376@15fps` mode (the only one confirmed corruption-free) and confirm
it with `ffplay` before trusting it — see [DOCKER_CMDS.md](DOCKER_CMDS.md).

---

## 8. Joint-mirror mode & stretch work

- **Joint-mirror mode** (`control/mode: joint_mirror`): MediaPipe Pose → 6 arm
  angles → Z1 joints directly, bypassing IK (PLAN.md §4.3). **Implemented (staged:
  joints 1–3; wrist 4–6 opt-in).** The human→Z1 mapping needs hand-tuning — see the
  calibration guide: [JOINT_MIRROR.md](JOINT_MIRROR.md).
- **Depth from hand size** (`mapping/depth_from_hand_size`): bound X from apparent
  hand size (PLAN.md §4.4). Hook present; not implemented.
