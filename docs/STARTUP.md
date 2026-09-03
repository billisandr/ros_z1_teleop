# Startup & Developer Reference — Z1 Hand-Teleop Station

A developer-oriented companion to [QUICKSTART.md](QUICKSTART.md), covering the
build, the hot-reload loop, rendering, and architecture. For end-to-end run
steps use the quick starts; for the ZED/USB internals see
[DOCKER_CMDS.md](DOCKER_CMDS.md).

---

## 1. Build the image

```bash
git submodule update --init --recursive
docker build -t ros-z1-teleop .
```

The build compiles the Z1 SDK and the `z1_controller` Gazebo bridge, runs
`catkin_make`, and finishes with an import smoke test
(`python3 -c "import cv2, mediapipe; from cv_bridge import CvBridge"`) that
fails the build if MediaPipe, OpenCV, and `cv_bridge` don't coexist.

### Hot-reload a node or config without rebuilding

```bash
# copy an edited file into a running container
docker cp z1_hand_detector/src/hand_detector_node.py \
    ros-z1-teleop:/home/rosuser/catkin_ws/src/z1_hand_detector/src/hand_detector_node.py

# restart just that node inside the container
docker exec -it ros-z1-teleop bash -ic "rosnode kill /hand_detector; z1_hand"
```

`teleop.yaml` is loaded into the parameter server at launch. `docker cp` it in
and relaunch (or re-`rosparam load` and restart the node) to apply changes.

---

## 2. X11 forwarding (host — once per session)

- **Windows:** `& "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac -wgl`
  (the `-wgl` flag is required, since Mesa software GL crashes gzclient
  without it).
- **Linux:** `xhost +local:docker` and run with `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`.
  Revoke with `xhost -local:docker` when done.
- **macOS:** XQuartz with "Allow connections from network clients".

Containers connect via `-e DISPLAY=host.docker.internal:0.0` on Windows/macOS.

---

## 3. Run modes

See [QUICKSTART.md](QUICKSTART.md) §4-6. In short:

```bash
# A. hardware-free demo (video file)
z1_teleop image_source:=video:/path/to/hand.mp4
# B. local webcam (Linux/macOS, --device /dev/video0)
z1_teleop
# C. ZED 2 over USB (Windows): from Git Bash on the host
bash start_teleop.sh
```

Gazebo starts paused; `z1_unpause` releases it. `z1_rviz` opens the layout.

### What auto-starts in `z1_teleop_sim.launch`
Gazebo (`teleop.world`, arm only), the Z1 URDF and joint/gripper controllers,
`robot_state_publisher`, `image_source_node` (webcam/video), `hand_detector_node`,
`arm_tracker_node`, and `scene_spawner_node` (props, if `scene_objects/enabled`).

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
`arm_tracker`, `joint_limits`, `joint_mirror`, `simulation`, `scene_objects`. See
the README configuration table for the short version.

**Scene props:** `scene_objects/enabled: true|false` toggles a handful of
cubes, pyramids, and cylinders dropped into the world as interaction targets,
spawned by `scene_spawner_node` from the `scene_objects:` list. Set
`enabled: false` for a clean arm-only scene, or `static: true` to bolt them
down. Props spawn once at launch, so edit the list and relaunch to change them.

**Tune live without restarting:** run `z1_ui` (browser control panel on port
8501; start the container with `-p 8501:8501`). It sets ROS params and the
nodes re-read the soft knobs each loop; see the README's "Live control panel"
section.

Inspect what a node is actually using:

```bash
rosparam get /gesture/hand
rosparam get /mapping/fixed_x
rosparam get /arm_tracker/workspace
```

---

## 5. Rendering

The image uses Mesa software rendering by default: both
`LIBGL_ALWAYS_SOFTWARE=1` and `MESA_GL_VERSION_OVERRIDE=3.3` are baked into
the Dockerfile via `ENV`.

We ended up here because the Intel Meteor Lake GPU (0x7d67) isn't supported
by Mesa 21 in the Ubuntu 20.04 base image, and NVIDIA GLX rejects indirect
X11 connections from Docker. Mesa llvmpipe gives a stable OpenGL 3.1 for
Gazebo and RViz with no host GPU setup required, and MediaPipe Hands runs on
CPU anyway, so nothing here actually needs a GPU.

To attempt hardware rendering on Linux, override at runtime:

```bash
docker run -it --rm --name ros-z1-teleop --gpus all --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY -e LIBGL_ALWAYS_SOFTWARE=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix ros-z1-teleop bash
```

This needs `nvidia-container-toolkit` on the host. On Windows, `/dev/dri`
doesn't exist, so software rendering (the default) is the only supported path.

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

The default control law maps normalized image coordinates (u,v) to world
(Y,Z) with X held fixed, so no TF and no `camera_info` are needed. The
arm-tracker holds X fixed, follows Y/Z, low-pass filters the result, clamps
it to the workspace, and, once settled, points the wrist at the target.

---

## 7. Troubleshooting

### Arm doesn't move
Show an open palm to engage (`/hand/tracking_active` must read `true`), and
make sure physics is unpaused (`z1_unpause`). The arm starts and re-acquires
frozen, by design.

### No hand detected
Check `z1_cam` (`/hand/debug_image`). Improve lighting, lower
`hand/min_detection_confidence`, and make sure `hand/flip_horizontal: true`
so handedness matches. Confirm the image source is actually publishing with
`rostopic hz /camera/color/image_raw`.

### IK failures — `IK failed for target x:... y:... z:...`
The requested Cartesian position is outside the Z1's reachable workspace:
- `mapping/fixed_x` / `arm_tracker/fixed_x` above roughly 0.55 m at
  non-zero Y/Z can exceed reach; try something in the 0.25 to 0.40 range.
- Keep `mapping/y_range` / `mapping/z_range` within `arm_tracker/workspace`.
- IK runs with `checkWorkspace=True` and refuses positions near the joint
  limits.

### `unitree_legged_msgs` not found at Python import
`unitree_legged_msgs` lives in the nested submodule
`unitree_ros/unitree_ros_to_real`. Make sure you cloned with
`git submodule update --init --recursive`, then rebuild. Inside the
container, `source ~/catkin_ws/devel/setup.bash` exposes it.

### Jittery or laggy arm
Tune `smoothing/min_cutoff` and `smoothing/beta` (the OneEuro filter on
(u,v)) and `arm_tracker/smoothing_alpha` (the Cartesian target filter).
`mapping/tracked_point: palm_centroid` is steadier than `wrist`. A small
`mapping/deadzone` also helps.

### TF/clock errors in real-camera mode
Real cameras stamp images with wall-clock time, which is why the launches
set `use_sim_time:=false`. The default hand-mirror control law needs no TF,
so the clock-skew lookups that plagued the earlier ArUco real-camera path
don't come up here.

### Garbled ZED video over usbipd (Windows)
Use the `1344x376@15fps` mode, the only one we've confirmed
corruption-free, and check it with `ffplay` before trusting it. See
[DOCKER_CMDS.md](DOCKER_CMDS.md).

---

## 8. Joint-mirror mode & stretch work

- **Joint-mirror mode** (`control/mode: joint_mirror`): MediaPipe Pose drives
  six arm angles straight to the Z1 joints, bypassing IK entirely. It's
  implemented, staged (joints 1-3 by default, wrist joints 4-6 opt-in), and
  the human-to-Z1 mapping needs hand-tuning per person and camera; see the
  calibration guide at [JOINT_MIRROR.md](JOINT_MIRROR.md).
- **Depth from hand size** (`mapping/depth_from_hand_size`): the idea is to
  bound X from apparent hand size instead of holding it fixed. The hook is
  present in the code, but this isn't implemented yet.
