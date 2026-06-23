# Quick Start — Z1 Hand-Teleop Station

A full walkthrough for bringing up the station. For a commands-only version see
[MINIMAL_QUICKSTART.md](MINIMAL_QUICKSTART.md); for the ZED/USB internals see
[DOCKER_CMDS.md](DOCKER_CMDS.md).

**Operator cheat sheet:** open palm to drive, fist to freeze, show your right hand.
The arm always **starts frozen** — it won't move until you show an open palm.

---

## 1. Prerequisites

- **Docker** (Docker Desktop on Windows/macOS, native on Linux).
- **An X server** for the Gazebo/RViz GUIs:
  - Windows: **VcXsrv**, started with `-wgl` (Mesa software GL needs it).
  - macOS: **XQuartz** (allow network clients).
  - Linux: native X — usually just `-e DISPLAY=$DISPLAY` and an `xhost +local:`.
- **Only for the ZED/D435 real-camera paths on Windows:** `usbipd-win`.

---

## 2. Build the image (once)

```bash
git clone --recursive https://github.com/billisandr/ros_z1_teleop.git
cd ros_z1_teleop
docker build -t ros-z1-teleop .
```

Cloned without `--recursive`? Run `git submodule update --init --recursive` first.

The first build is ~10–15 min: it compiles the Z1 SDK and the `z1_controller`
Gazebo bridge, runs `catkin_make`, and runs a MediaPipe/OpenCV/`cv_bridge` import
smoke test that fails the build if those don't coexist.

---

## 3. Start the X server (Windows, once per session)

```powershell
& "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac -wgl
```

`start_teleop.sh` checks/repairs this for you on the ZED path.

---

## 4. Run mode A — hardware-free demo (video file)

The primary perception path is always a real image stream. With no camera, loop a
short video of a moving hand (any `.mp4` reachable inside the container).

**Terminal 1** — container + launch:

```bash
docker run -it --rm --name ros-z1-teleop -e DISPLAY=host.docker.internal:0.0 \
    ros-z1-teleop bash
# inside:
z1_teleop image_source:=video:/path/to/hand.mp4
```

**Terminal 2** — release physics (Gazebo starts paused):

```bash
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
```

**Terminal 3** — RViz (robot model, TF, hand target pose, debug overlay):

```bash
docker exec -it -e DISPLAY=host.docker.internal:0.0 ros-z1-teleop bash -ic "z1_rviz"
```

To copy a host video into a running container:
`docker cp hand.mp4 ros-z1-teleop:/home/rosuser/hand.mp4`, then launch with
`image_source:=video:/home/rosuser/hand.mp4`.

---

## 5. Run mode B — local webcam (Linux/macOS)

```bash
docker run -it --rm --name ros-z1-teleop --device /dev/video0:/dev/video0 \
    -e DISPLAY=$DISPLAY ros-z1-teleop bash -ic "z1_teleop"
```

`z1_teleop` defaults to `image_source: webcam` / `hand/webcam_device: 0`. On
Windows the built-in webcam generally can't be passed into Docker — use a video
file (mode A) or the ZED path (mode C).

---

## 6. Run mode C — ZED 2 over USB (recommended on Windows)

From **Git Bash on the Windows host** (not WSL, not inside the container):

```bash
bash start_teleop.sh           # default alias: z1_teleop_zed
# headless (no Gazebo GUI):
bash start_teleop.sh z1_teleop_zed_headless
```

The script: verifies VcXsrv (`-wgl`), binds + attaches the ZED via `usbipd`,
loads `uvcvideo` in the docker-desktop VM, starts the container, then opens RViz.
First-time `usbipd bind` needs an **Administrator** Git Bash.

Then unpause physics:

```bash
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
```

ZED device/resolution/fps/eye come from the `zed_camera:` block in
`teleop.yaml` — use `1344x376@15fps` (corruption-free over usbipd).

---

## 7. Verify the pipeline

```bash
docker exec -it ros-z1-teleop bash -ic "z1_active"   # /hand/tracking_active
docker exec -it ros-z1-teleop bash -ic "z1_target"   # /hand/target_pose
docker exec -it ros-z1-teleop bash -ic "z1_cam"      # /hand/debug_image overlay
```

Expected: with a hand visible and an **open palm**, `tracking_active` is `true`
and `target_pose` Y/Z track your hand; make a **fist** and it goes `false` and
the arm holds. The overlay border is green while driving, red while frozen.

---

## 8. Tuning

Edit [../z1_teleop/config/teleop.yaml](../z1_teleop/config/teleop.yaml) and
relaunch (or `docker cp` it into the running container). Common knobs:

- **Which hand:** `gesture/hand: right | left | either`.
- **Reach/box:** `mapping/fixed_x`, `mapping/y_range`, `mapping/z_range`
  (keep within `arm_tracker/workspace`).
- **Steadiness:** lower `smoothing/min_cutoff` for smoother (laggier) motion;
  `mapping/tracked_point: palm_centroid` is steadier than `wrist`.
- **Clutch feel:** `gesture/open_fingers`, `gesture/fist_fingers`,
  `gesture/hysteresis_frames`, `gesture/lost_frames`.
- **Gripper:** `gesture/gripper: pinch | none`; `gesture/pinch_*_dist`.

---

## 9. Cleanup

```bash
docker rm -f ros-z1-teleop
```

Mode A/B containers self-remove on exit (`--rm`); the ZED script removes a stale
container before starting, so you can switch modes without manual cleanup.

---

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| No GUI window | X server not running with `-wgl` (Windows); `DISPLAY` unset |
| Arm doesn't move | Open palm to engage; `z1_unpause`; check `z1_active` is true |
| Wrong hand | Set `gesture/hand`; keep `hand/flip_horizontal: true` |
| No detection | Better lighting; lower `hand/min_detection_confidence`; watch `z1_cam` |
| Garbled ZED video | Use `1344x376@15fps`; see [DOCKER_CMDS.md](DOCKER_CMDS.md) |
| Import error on launch | Rebuild — the in-build smoke test must pass |
