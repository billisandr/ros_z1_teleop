# Z1 Hand-Teleop Workshop Station — Implementation Plan

> **Purpose of this document.** A self-contained build plan for a *new, separate*
> ROS/Gazebo repository (`ros_z1_teleop`) that is a sibling of the existing
> `ros_z1_sim_marker-real-camera` repo (branch `feature/zed-camera-support`),
> but with **ArUco marker detection/tracking removed entirely** and replaced by
> **human hand/arm detection and following**. It is written so another AI agent
> or a fresh session can execute it without prior context. Read it top to bottom
> once before touching code.

---

## 0. TL;DR

Build a Z1-arm teleoperation station where the robot follows the operator's
**hand** seen by a camera, instead of an ArUco marker.

- **Reuse ~90% of the source repo** — the Gazebo arm, the IK-based controller
  (`z1_arm_tracker`), the camera bridge (`zed_camera_node`), the Docker build,
  the launch/world/xacro/rviz scaffolding, and all three Unitree submodules.
- **Replace only the perception node.** `aruco_detector_node.py` →
  `hand_detector_node.py` (MediaPipe Hands/Pose). It publishes the *same two
  topics* the controller already consumes, so the control side barely changes.
- **Delete** the marker animator (`marker_mover_node.py`), the ArUco Gazebo
  model, and all ArUco config/launch params.
- **Default control law:** mirror the hand in the camera image plane → arm
  follows in Y (lateral) and Z (vertical), X (depth) held fixed. This maps
  *exactly* onto the controller's existing 2D tracking mode, and needs **no
  camera calibration and no TF** (a major simplification vs. ArUco).
- **Hand-only conventions** (the "conventions" requested): track one configured
  hand; an **open-palm = engage / fist = freeze** clutch gesture; optional
  pinch → gripper. Defined precisely in §5.
- Ship it as a brand-new local + GitHub repo, re-adding the three Unitree
  submodules.

---

## 1. Source material (what already exists)

### 1.1 The repo we are adapting
`2.CodeRepos/ros_z1_sim_marker-real-camera` @ branch `feature/zed-camera-support`.
Remote: `https://github.com/billisandr/ros_z1_sim_marker.git`.

Three catkin packages + three Unitree submodules + Docker:

| Package | Role | Key files |
| --- | --- | --- |
| `z1_aruco_detector` | Perception | `aruco_detector_node.py`, `marker_mover_node.py`, `zed_camera_node.py` |
| `z1_arm_tracker` | Control | `arm_tracker_node.py` |
| `z1_aruco` | Launch/assets/config | `launch/*.launch`, `worlds/*.world`, `models/`, `xacro/z1_aruco_robot.xacro`, `rviz/`, `config/aruco_tracking.yaml`, `scripts/workshop_ui.py`, `notebooks/` |
| `sdk_z1` (submodule) | Z1 SDK (C++) — IK model via `unitree_arm_interface` | upstream `unitreerobotics/z1_sdk` |
| `z1_controller` (submodule) | Gazebo `sim_ctrl` bridge | upstream `unitreerobotics/z1_controller` |
| `unitree_ros` (submodule) | URDF/descriptions/`unitree_legged_msgs` | upstream `unitreerobotics/unitree_ros` |

**Pipeline today (data flow):**
```
camera image ─► /camera/color/image_raw
                       │
                aruco_detector_node ─► /aruco/marker_pose      (geometry_msgs/PoseStamped, world frame)
                                   └─► /aruco/marker_detected  (std_msgs/Bool)
                                            │
                                     arm_tracker_node ─► /z1_gazebo/JointXX_controller/command (MotorCmd)
```
- `arm_tracker_node` **already** does 2D Cartesian tracking: holds `fixed_x`,
  follows the target's `y` and `z`, low-pass filtered, workspace-clamped, with
  optional wrist-orientation IK once settled. IK uses the SDK arm model
  (`arm_sdk.ArmInterface(...).._ctrlComp.armModel.inverseKinematics`), no UDP.
- The image source is swappable (Gazebo plugin / D435 / ZED-2-UVC) because every
  consumer only knows the `/camera/color/image_raw` + `/camera/color/camera_info`
  topics. **We exploit the same seam.**

### 1.2 The hand-tracking example (already in this new repo)
`2.CodeRepos/ros_z1_teleop/` currently holds three standalone files:

- `hand_tracker_3d.py` — a **non-ROS** MediaPipe pipeline. Per frame: optional
  open-vocab crop (Grounding DINO / LocateAnything), **MediaPipe Hands** (21
  landmarks), optional **MediaPipe Pose** (shoulder/elbow/wrist → 6 joint
  angles), optional **Depth Anything V2**, OneEuro smoothing, and a
  matplotlib 3D skeleton render. Driven by `config.yaml`.
- `config.yaml` — toggles for all of the above.
- `requirements.txt` — `mediapipe opencv-python numpy matplotlib pyyaml` (+
  optional `torch transformers pillow accelerate`).

**Reusable gems to lift into the ROS node** (copy, don't re-derive):
- `OneEuroFilter` (low-lag landmark smoothing) — lines ~203–231.
- `mp_hands` / `mp_pose` setup and landmark topology constants (`FINGERS`,
  `TIPS`, `WRIST`, `POSE_IDX`) — lines ~53–91.
- `arm_joint_angles(...)` — full 6-DOF human-arm → robot-joint decomposition
  (lines ~482–527). **Enables the advanced "joint mirror" mode (§4.3).**
- The per-frame MediaPipe call pattern in `process_video` (lines ~843–905).

**What to drop from the example for the ROS node:** matplotlib/animation, video
file I/O, Depth Anything, Grounding DINO / LocateAnything (all heavy optional
deps). Keep them only as an offline reference under `reference/`.

---

## 2. Mission & non-goals

**Mission.** A workshop station where the Z1 (in Gazebo) follows the
participant's hand/arm seen through a real camera (webcam / ZED 2 / D435 / video
file), with clear hand-only interaction conventions, packaged as its own
repo + Docker image.

**In scope**
- New repo `ros_z1_teleop`, local + GitHub.
- MediaPipe-based `hand_detector_node` replacing the ArUco detector.
- Hand-only conventions: handedness selection, clutch gesture, optional gripper.
- Default Cartesian "hand-mirror" control (reuses existing tracker).
- Docs + launch + a one-shot start script, mirroring the source repo's polish.

**Non-goals (explicitly out for v1)**
- No ArUco anything (removed, not hidden).
- No real Z1 hardware — Gazebo only (same safety posture as source).
- No metric 3D hand pose / absolute depth in v1 (depth is fixed; see §4.4 for
  the optional stretch).
- No GPU/CUDA requirement — MediaPipe Hands runs fine on CPU.

---

## 3. Target architecture

```
 Real camera (webcam / ZED-2 UVC / D435 / video file)
        │  /camera/color/image_raw         (sensor_msgs/Image)
        ▼
 hand_detector_node  ──►  /hand/target_pose       (geometry_msgs/PoseStamped, world frame)
   (MediaPipe Hands)  ─►  /hand/tracking_active    (std_msgs/Bool)   ← clutch + detection gate
                      ─►  /hand/debug_image        (sensor_msgs/Image, overlay)
                      ─►  /hand/gripper_cmd        (std_msgs/Float64, optional 0..1)
        │
        ▼
 arm_tracker_node  ──►  /z1_gazebo/JointXX_controller/command  (MotorCmd ×6)
                   └─►  /z1_gazebo/gripper_controller/command   (optional)
        ▲
 Gazebo: Z1 arm + joint controllers + robot_state_publisher (TF)
```

### 3.1 Topic / name mapping (old → new)
| Old | New | Type | Notes |
| --- | --- | --- | --- |
| `/aruco/marker_pose` | `/hand/target_pose` | `PoseStamped` | World-frame target the arm follows |
| `/aruco/marker_detected` | `/hand/tracking_active` | `Bool` | True only when a valid hand **and** clutch engaged |
| `/aruco/debug_image` | `/hand/debug_image` | `Image` | Landmark overlay for RViz / image_view |
| — (new) | `/hand/gripper_cmd` | `Float64` | Optional pinch→gripper |
| `/camera/color/image_raw` | *unchanged* | `Image` | Same seam; camera bridge reused as-is |
| `/camera/color/camera_info` | *optional* | `CameraInfo` | **Not required** by default mapping |

### 3.2 Package layout (renames)
| Old package | New package | Action |
| --- | --- | --- |
| `z1_aruco_detector` | `z1_hand_detector` | Replace nodes; keep `zed_camera_node.py` |
| `z1_arm_tracker` | `z1_arm_tracker` | Keep name; rename subscribed topics + add gripper |
| `z1_aruco` | `z1_teleop` | Launch/world/xacro/rviz/config meta-package |

> Renaming is recommended for a clean workshop identity, but is *mechanical*.
> If time-boxed, an agent may keep the old package names and only swap node
> internals — call this out in the PR. The **default plan renames.**

---

## 4. Core design decisions

### 4.1 Why the control side barely changes
`arm_tracker_node` already implements exactly "hold X, follow a 2D target in
Y/Z, smoothed and clamped." A hand mirror *is* a 2D target. So the controller
needs only: (a) topic renames, (b) gripper passthrough (optional). Its IK,
smoothing, stability gate, and workspace clamp are reused verbatim. **Do not
rewrite the controller.**

### 4.2 Default control law — Cartesian "hand mirror" (RECOMMENDED)
Map the tracked hand point's **normalized image coordinates** directly to a
world-frame Y/Z target, fixed X:

```
u, v = normalized image coords of tracked point ∈ [0,1]   (v grows downward)
Y =  map(u, 0..1 → y_max..y_min)     # mirror so moving hand right → arm right
Z =  map(v, 0..1 → z_max..z_min)     # invert because image v is top-down
X =  fixed_x                          # depth held constant
publish PoseStamped(frame_id="world", position=(X,Y,Z))
```
Properties that make this the right v1 default:
- **No camera calibration, no TF, no depth.** The detector emits a world-frame
  target straight from a linear map. (Contrast: ArUco needed `solvePnP` +
  `tf_buffer.transform` to `world`.) Document this simplification prominently.
- Robust on any camera, any resolution, including a laptop webcam or a video.
- Feels natural to operators ("move my hand, the arm copies").
- `use_sim_time` timing issues that plagued real-camera ArUco mode **disappear**
  (no TF lookup at wall-clock-stamped images).

`tracking_active` gates publishing (clutch + detection). When false, the
controller holds last target (existing `marker_detected==false` behavior).

### 4.3 Advanced control law — joint-space "arm mirror" (STRETCH, optional)
Enable MediaPipe **Pose**, compute the operator's 6 arm angles with the
example's `arm_joint_angles()`, remap to Z1 joint conventions/limits, and
command joints directly (bypassing IK). Wow-factor for the workshop, but lower
precision and harder to bound safely. **Default OFF.** Gate behind
`control/mode: cartesian | joint_mirror` in config. Implement only after §11
Phase 1–5 are green. Requires a calibration/remap step (human ranges → Z1
`joint_limits` in the existing yaml) and per-joint clamping.

### 4.4 Depth handling (optional stretch)
Default: X fixed. Optional v2: estimate relative depth from apparent hand size
(e.g. wrist↔middle-MCP pixel distance, or palm width) and map to a bounded X
range. Avoid Depth Anything in the container (heavy torch dependency). Keep the
hook in config (`mapping/depth_from_hand_size: false`) but do not block v1.

### 4.5 Tracked point
Default = **wrist** (landmark 0). Alternative = **palm centroid**
(mean of landmarks 0,5,9,13,17) for steadier targeting. Config:
`mapping/tracked_point: wrist | palm_centroid`.

---

## 5. Hand-only tracking conventions (the "conventions" requested)

These are the interaction rules the detector enforces. All configurable; the
**defaults** are:

1. **Handedness.** Track exactly one hand. `gesture/hand: right | left | either`
   (default `right`). MediaPipe returns a handedness label per hand; if both are
   visible, pick the configured one and ignore the other. (Note: handedness
   assumes the feed is mirrored like a selfie — the example sets
   `flip_horizontal: true`; replicate that so "right" means the operator's right
   hand.)

2. **Clutch (engage/disengage).** The arm follows **only while engaged**:
   - **Open palm** (≥4 fingers extended) → **ENGAGE** (`tracking_active=true`).
   - **Closed fist** (≤1 finger extended) → **DISENGAGE / FREEZE**; arm holds
     its last commanded pose. Lets the operator reposition their hand without
     dragging the arm (classic teleop "clutch").
   - Finger-extended test: a finger is "extended" when its tip landmark is
     farther from the wrist than its PIP joint (per-finger, in normalized
     coords). Thumb handled separately (lateral test). Provide a small
     hysteresis (e.g. require the same state for N=3 frames) to avoid flicker.
   - Config: `gesture/clutch: palm_fist | always_on | pinch_hold`.

3. **Tracking gate.** `tracking_active` is true **iff** (configured hand present)
   **AND** (clutch engaged) **AND** (landmark confidence ok). If no hand for
   `gesture/lost_frames` (default 10) frames → false → arm freezes.

4. **Gripper (v1, ENABLED).** `gesture/gripper: pinch | none` (default `pinch`).
   Pinch = thumb-tip(4)↔index-tip(8) normalized distance; map [closed..open] →
   gripper command. Publish on `/hand/gripper_cmd`; the controller forwards to
   `gripper_controller`. Wired in Phase 5.

5. **Smoothing & deadzone.** OneEuro filter (lifted from the example) on the
   (u,v) target before mapping; optional small centered deadzone to kill jitter
   when the hand is nominally still.

6. **Safety convention.** On node start, before first engage, publish
   `tracking_active=false` so the arm never lurches on launch. Document that the
   arm always starts frozen until the operator shows an open palm.

Document all six in the new README as "Operator conventions," with a one-line
cheat-sheet: *Open palm to drive, fist to freeze, show your right hand.*

---

## 6. What to remove / change / keep vs. source

**Remove**
- `z1_aruco_detector/src/aruco_detector_node.py` (replaced).
- `z1_aruco_detector/src/marker_mover_node.py` (no marker to animate).
- `z1_aruco/models/aruco_marker_0/` (Gazebo marker model).
- ArUco marker object from the Gazebo world(s) (see §7.4).
- All `aruco:` config keys; all `aruco_dictionary/marker_size/tracking_id`
  launch args; the `--dictionary/--marker-size/--tracking-id` flags in the start
  script.
- `marker:` motion config block.

**Change**
- `aruco_detector_node.py` → `hand_detector_node.py` (§7.1).
- `arm_tracker_node.py`: rename subscribed topics (§3.1); optional gripper
  passthrough (§7.2). Keep all IK/smoothing/clamp logic.
- Launch files: drop marker/aruco bits, add hand detector (§7.3).
- Config yaml: new `hand:`, `mapping:`, `gesture:`, `control:` blocks; keep
  `arm_tracker:`, `joint_limits:`, `zed_camera:`, `camera:`, `simulation:` (§7.5).
- Dockerfile: add MediaPipe + deps; drop RealSense/ArUco-only bits if desired;
  update `.bashrc` aliases (§8).
- RViz config: point the image display at `/hand/debug_image`.
- README / QUICKSTART / MINIMAL_QUICKSTART / start script: rewrite for hand mode.

**Keep (reuse essentially as-is)**
- `zed_camera_node.py` (UVC bridge) — unchanged; still feeds
  `/camera/color/image_raw`.
- `z1_aruco/xacro/z1_aruco_robot.xacro` → rename to `z1_teleop_robot.xacro`; the
  end-effector camera link is still useful for real-camera-on-wrist demos, and
  the `CameraPlugin:=false` path is what the UVC/host-camera modes use. Keep both
  args.
- `worlds/aruco_tracking_real.world` (arm only, no marker) → rename
  `teleop.world`; this becomes the **primary** world (§7.4).
- All three submodules (sdk_z1, z1_controller, unitree_ros) and the
  `z1_controller` CMakeLists **sed fix** in the Dockerfile (carry it over
  verbatim — it is required to build `sim_ctrl`; see source Dockerfile lines
  ~94–98 and `[[project_ros_z1_zed_docker]]`).
- The `.gitignore` (`__pycache__/`) and `.dockerignore` patterns.

---

## 7. Detailed file-level plan

### 7.1 `z1_hand_detector/src/hand_detector_node.py` (NEW — the heart of v1)
Responsibilities:
1. `rospy.init_node('hand_detector')`; `CvBridge`; read params (`hand/*`,
   `mapping/*`, `gesture/*`).
2. Build MediaPipe Hands (`max_num_hands=2` so handedness selection works;
   `model_complexity`, confidences from config). Optionally MediaPipe Pose if
   `control/mode == joint_mirror`.
3. Subscribe `/camera/color/image_raw`. In the callback:
   - `bridge.imgmsg_to_cv2(msg, 'bgr8')`; optional horizontal flip (selfie view).
   - `hands.process(rgb)`; pick the configured handedness.
   - Compute clutch state (open-palm/fist) with hysteresis; compute tracked
     point (wrist or palm centroid) in normalized coords.
   - OneEuro-smooth (u,v); map to (X=fixed_x, Y, Z) via §4.2; publish
     `PoseStamped(frame_id='world')` on `/hand/target_pose`.
   - Publish `/hand/tracking_active` per §5.3.
   - Optional pinch → `/hand/gripper_cmd`.
   - Draw landmarks + clutch/handedness/engage HUD on a copy; publish
     `/hand/debug_image`.
4. Reuse `OneEuroFilter` and topology constants copied from `hand_tracker_3d.py`.
5. No TF, no `camera_info` needed for the default mapping.

Acceptance: with a webcam/video, `/hand/target_pose` tracks the hand,
`/hand/tracking_active` toggles with palm/fist, debug image shows the skeleton.

### 7.2 `z1_arm_tracker/src/arm_tracker_node.py` (MINIMAL EDIT)
- Change three subscriptions:
  `/aruco/marker_pose`→`/hand/target_pose`,
  `/aruco/marker_detected`→`/hand/tracking_active`.
- Rename internal `marker_detected`/`latest_pose` for clarity (optional).
- Re-read params from `arm_tracker/*` (unchanged keys).
- **Optional gripper:** subscribe `/hand/gripper_cmd` (Float64), publish to
  `/z1_gazebo/gripper_controller/command` (MotorCmd or Float64 per controller
  type — verify against `robot_control.yaml`). Gate behind a config flag.
- Everything else (IK, smoothing, stability gate, workspace clamp, wrist
  orientation) stays. `_facing_angles()` still works (it points the EE at the
  target in world frame).

Acceptance: publishing a synthetic `/hand/target_pose` moves the Gazebo arm in
Y/Z exactly as the ArUco demo did.

### 7.3 Launch files (in `z1_teleop/launch/`)
Create from the source launch files, **minus** marker/aruco:
- `z1_teleop_sim.launch` (primary): Gazebo (`teleop.world`, arm only) +
  controllers + `robot_state_publisher` + `hand_detector_node` +
  `arm_tracker_node`. Camera source = **host webcam / video file** via a param
  (`hand/image_source`) OR the simulated wrist camera. (No animated marker.)
- `z1_teleop_zed.launch`: same but add `zed_camera_node` (carry over from
  `z1_real_camera_tracking_zed.launch`, drop the aruco args).
- `z1_teleop_realsense.launch` (optional): D435 driver variant.
- Keep the `robot_path`/`dollar`/`rname` xacro plumbing and the controller
  spawners verbatim. Keep `use_sim_time:=false` for real-camera variants (wall
  clock images), `true` for pure-sim camera.

> **Camera-source nuance.** The source repo's "Variation A (full Gazebo sim)"
> relied on an *animated marker inside Gazebo*. There is no animated hand in
> Gazebo, so v1's primary perception is always a **real image stream** (webcam,
> USB cam, or a looping video file for hardware-free demos). Provide a
> `hand/image_source` param accepted by a tiny optional `image_pub` helper (or
> reuse `zed_camera_node`'s pattern) that publishes a webcam/video file to
> `/camera/color/image_raw` when no other camera node is running. A bundled demo
> `.mp4` of a moving hand makes the station work with zero camera hardware.

### 7.4 World (`z1_teleop/worlds/teleop.world`)
Start from `aruco_tracking_real.world` (arm only, no marker, no camera). Rename
models/refs. Remove any ArUco/marker includes. This is the only world needed.

### 7.5 Config (`z1_teleop/config/teleop.yaml`)
New top-level blocks (keep `arm_tracker:`, `joint_limits:`, `camera:`,
`zed_camera:`, `simulation:` from source):
```yaml
control:
  mode: cartesian            # cartesian | joint_mirror   (§4.2 / §4.3)

hand:
  model_complexity: 1
  min_detection_confidence: 0.6
  min_tracking_confidence: 0.6
  flip_horizontal: true      # selfie view → handedness labels match operator
  image_source: webcam       # webcam | video:/path.mp4 | external (another node)

mapping:
  tracked_point: wrist       # wrist | palm_centroid
  fixed_x: 0.25              # world depth held constant (mirror arm_tracker.fixed_x)
  y_range: [-0.35, 0.35]     # world Y for image u ∈ [0,1] (mirrored)
  z_range: [0.10, 0.75]      # world Z for image v ∈ [0,1] (inverted)
  deadzone: 0.0
  depth_from_hand_size: false

gesture:
  hand: right                # right | left | either
  clutch: palm_fist          # palm_fist | always_on | pinch_hold
  gripper: pinch             # pinch | none   (v1 default ON)
  lost_frames: 10            # frames without hand before freezing
  hysteresis_frames: 3
```
Smoothing reuses `arm_tracker/smoothing_alpha`; optionally add a OneEuro block.

### 7.6 RViz / aliases / docs
- RViz: image display → `/hand/debug_image`; keep robot model, TF, joint states.
- `.bashrc` aliases (Dockerfile): `z1_teleop` (sim), `z1_teleop_zed`,
  `z1_unpause`, `z1_rviz`, `z1_hand` (run detector alone), `z1_cam`
  (view `/hand/debug_image`), diagnostics (`z1_target`→echo `/hand/target_pose`,
  `z1_active`→echo `/hand/tracking_active`).
- Docs: new `README.md`, `docs/QUICKSTART.md`, `docs/MINIMAL_QUICKSTART.md`
  (model the latter on the existing one in the source repo), plus an
  **Operator conventions** section (§5 cheat-sheet).
- `start_teleop.sh`: adapt `start_zed_sim.sh` — drop ArUco flags; keep the
  VcXsrv/usbipd/uvcvideo checks for the ZED variant.

---

## 8. Docker & dependency integration (highest-risk area)

Base image and structure stay (osrf/ros:noetic-desktop-full, non-root `rosuser`,
sdk_z1 + z1_controller C++ builds, catkin_make, the **z1_controller CMakeLists
sed fix kept verbatim**). Changes:

1. **Add MediaPipe** (Python 3.8 in Noetic — supported):
   `pip3 install mediapipe`. It pulls `numpy`, `protobuf`, and depends on a
   `cv2`. **Risk:** version clashes with ROS's `cv_bridge`/`python3-opencv` and
   with `protobuf` used by ROS. Mitigations, in order of preference:
   - Pin a known-good trio and **smoke-test imports together** in the build:
     `RUN python3 -c "import cv2, mediapipe, numpy; from cv_bridge import CvBridge; print('ok')"`.
     Make the build fail loudly if this errors.
   - Prefer the system `python3-opencv` that `cv_bridge` is built against; let
     MediaPipe use it rather than dragging a conflicting `opencv-python` wheel.
     If MediaPipe insists on its own opencv, install `opencv-python` (not
     `-contrib-headless`) and verify `cv_bridge` still imports.
   - Pin `protobuf` to a version MediaPipe accepts that doesn't break ROS
     (test empirically; commonly `protobuf<4`).
2. **Pre-bake MediaPipe models.** MediaPipe ships its hand/pose model assets in
   the wheel (no runtime download for the classic `solutions` API). Confirm by
   running the import smoke test offline. If any download is triggered, warm it
   during build.
3. **Drop now-unneeded apt deps** (optional cleanup): `ros-noetic-realsense2-camera`,
   `libuvc-dev` only if you also drop the D435/ZED variants — **keep them if you
   keep `zed_camera_node`** (recommended; the ZED variant is the best Windows
   camera path — see §9).
4. **Remove ArUco-specific runtime needs** — none beyond opencv-contrib's aruco;
   you may keep `opencv-contrib-python-headless` or switch to plain
   `opencv-python` for MediaPipe compatibility. Decide via the smoke test.
5. Update `.bashrc` aliases (§7.6) and `CMD` to launch `z1_teleop_sim.launch`.

**Acceptance for Phase 1:** image builds; the import smoke test passes;
`roslaunch z1_teleop z1_teleop_sim.launch` brings up Gazebo + nodes without
import errors.

---

## 9. Camera sourcing — Windows reality check (READ THIS)

The operator's camera is the practical crux on Windows:

- **Laptop integrated webcams generally CANNOT be `usbipd`-attached** into the
  Docker Desktop WSL2 VM (they're often not exposed as detachable USB, or the
  isochronous bandwidth corrupts over usbipd — same issue documented for the ZED
  in the source repo). Do **not** assume `/dev/video0` = the built-in webcam
  inside the container.
- **Recommended Windows camera paths**, in order:
  1. **Bundled demo video file** (`hand/image_source: video:/path.mp4`) — zero
     hardware, always works, great for first bring-up and for attendees without
     cameras. Make this the default for the `sim` launch.
  2. **ZED 2 / external USB cam via usbipd** — reuse `zed_camera_node` + the
     `start_teleop.sh` usbipd/uvcvideo dance, at the lowest-bandwidth UVC mode
     (the source repo proved `1344x376@15fps` is corruption-free).
  3. **Run MediaPipe on the host, publish over ROS network** — run the detector
     natively on Windows/host Python against the laptop webcam and point
     `ROS_MASTER_URI` at the container. Heavier setup; document as advanced.
- **Linux/Mac hosts**: native `--device /dev/video0` works for the built-in or
  USB webcam, like the source repo's D435 path.

Spell this out in the README so workshop hosts pick the right path for their OS.

---

## 10. Git & GitHub setup

`ros_z1_teleop` is **not yet a git repo**. Steps:

1. Decide placement of the three example files. **Recommended:** move
   `hand_tracker_3d.py`, `config.yaml`, `requirements.txt` into `reference/`
   (offline standalone tool, kept for provenance and the joint-mirror math).
   The ROS node lifts code from it but doesn't import it.
2. Scaffold the catkin packages (§3.2) and copy/adapt files from the source repo
   per §6–7.
3. `git init`; add `.gitignore` (`__pycache__/`, build/devel/install) and
   `.dockerignore` (carry over from source).
4. **Re-add the three submodules** (the new repo needs them to build):
   ```
   git submodule add https://github.com/unitreerobotics/z1_sdk        sdk_z1
   git submodule add https://github.com/unitreerobotics/z1_controller z1_controller
   git submodule add https://github.com/unitreerobotics/unitree_ros   unitree_ros
   ```
   **Pin each to the same commit the source repo uses** (read them from the
   source repo's `git submodule status`) so the build is reproducible:
   `cd <sub> && git checkout <pinned-sha> && cd .. && git add <sub>`.
   Do **not** modify the submodules (the CMakeLists fix lives in the Dockerfile).
5. First commit. Then create the remote with `gh` (available, v2.94):
   ```
   gh repo create <owner>/ros_z1_teleop --private --source . --remote origin --push
   ```
   `--private` vs `--public` and `<owner>` (e.g. `billisandr`) are **open
   decisions** (§12).

---

## 11. Phased implementation roadmap (for the executing agent)

Each phase ends with a concrete, runnable acceptance check. Do them in order.

- **Phase 0 — Scaffold.** Copy source tree, rename packages/files (§3.2, §6),
  move example to `reference/`, `git init`, add submodules pinned to source SHAs,
  `.gitignore`/`.dockerignore`.
  *Accept:* tree builds a package list with `catkin_make` skipping (or building)
  cleanly after Phase 1’s Docker.

- **Phase 1 — Docker + MediaPipe.** Edit Dockerfile (§8); keep z1_controller sed
  fix. Build image.
  *Accept:* image builds; `python3 -c "import cv2, mediapipe; from cv_bridge import CvBridge"` passes inside the image.

- **Phase 2 — Detector.** Implement `hand_detector_node.py` (§7.1) with default
  Cartesian mapping; lift `OneEuroFilter`. Use a bundled demo video as source.
  *Accept:* `/hand/target_pose` and `/hand/tracking_active` publish sensibly;
  `/hand/debug_image` shows landmarks.

- **Phase 3 — Controller wiring.** Retopic `arm_tracker_node.py` (§7.2).
  *Accept:* with the demo video, the Gazebo arm follows the hand in Y/Z; freezes
  on fist; resumes on open palm.

- **Phase 4 — Launch/config/RViz/aliases.** `z1_teleop_sim.launch` + `teleop.yaml`
  + RViz + `.bashrc` aliases.
  *Accept:* one command (`z1_teleop`) brings up the whole station; RViz shows arm
  + debug image.

- **Phase 5 — Conventions polish.** Handedness, clutch hysteresis, lost-frame
  freeze, start-frozen safety, HUD overlay. Optional gripper via pinch.
  *Accept:* the §5 cheat-sheet behaviors all hold.

- **Phase 6 — Real camera + docs.** ZED variant + `start_teleop.sh`; README,
  QUICKSTART, MINIMAL_QUICKSTART, Operator conventions; demo media.
  *Accept:* ZED variant runs on Windows at the safe UVC mode; docs let a new host
  reproduce.

- **Phase 7 — Stretch (optional).** Joint-mirror mode (§4.3) behind
  `control/mode: joint_mirror`; optional depth-from-hand-size (§4.4).
  *Accept:* joint mirror demonstrably moves Z1 joints from arm pose, clamped to
  `joint_limits`.

---

## 12. Decisions (RESOLVED 2026-06-23)

1. **Primary control paradigm.** Cartesian hand-mirror is the default;
   joint-mirror remains an opt-in Phase-7 stretch.
2. **Package renames.** ✅ **Rename** (`z1_aruco`→`z1_teleop`,
   `z1_aruco_detector`→`z1_hand_detector`, `z1_arm_tracker` keeps its name).
3. **GitHub owner & visibility.** ✅ **`billisandr/ros_z1_teleop`, PRIVATE.**
   `gh repo create billisandr/ros_z1_teleop --private --source . --remote origin --push`.
4. **Gripper in v1.** ✅ **Yes — ship it.** `gesture/gripper: pinch` default ON;
   wire `/hand/gripper_cmd` → `gripper_controller` in `arm_tracker_node` as part
   of Phase 5 (no longer deferred).
5. **RealSense apt deps.** Keep `zed_camera_node` + ZED variant; keep
   `libuvc-dev`. RealSense2 driver apt package is optional — keep only if the
   D435 variant is wanted (low cost to keep).

### 12b. Submodule pinning (RESOLVED — reproducibility fix)
The source repo pins `z1_controller` to a **local-only** commit `eb4f40a8`
("add cmake_minimum_required and project() to sim/CMakeLists.txt") that is **not
on upstream**, so `git clone --recursive` of a fresh repo cannot fetch it. For
`ros_z1_teleop`, pin all three submodules to **genuine upstream commits** and
fold the missing 3-line preamble into the Dockerfile sed:

| Submodule | Pin (upstream) | Notes |
| --- | --- | --- |
| `sdk_z1` | `f1af2b42f2a39a5010946e049ad3fe324e2e6f06` | same as source |
| `unitree_ros` | `8ab335954190f3d46f555dfee0a8f54bc0e919d8` | same as source |
| `z1_controller` | `639bb773bd11b0e8495faf40eadc9e8eb8bc8400` | **upstream master** (= parent of source's local `eb4f40a8`) |

Dockerfile sed on the copied `sim/CMakeLists.txt` must therefore do **three**
things (the first is new vs. source):
```
# 0. (NEW) prepend the catkin preamble upstream 639bb77 lacks
grep -q 'project(z1_controller)' CMakeLists.txt || \
  sed -i '1i cmake_minimum_required(VERSION 3.0.2)\nproject(z1_controller)\n' CMakeLists.txt
# 1. normalise CRLF (file ships with Windows line endings)
sed -i 's/\r$//' CMakeLists.txt
# 2. point include dir at the full build, not the "sim" subdir
sed -i 's|^  sim$|  $ENV{HOME}/z1_controller/include|' CMakeLists.txt
# 3. add the lib dir after GAZEBO_LIBRARY_DIRS
sed -i '/${GAZEBO_LIBRARY_DIRS}/a\  $ENV{HOME}/z1_controller/lib' CMakeLists.txt
```

---

## 13. Key reference snippets to lift (file: `hand_tracker_3d.py`)

- `OneEuroFilter` class — smoothing (≈ lines 203–231).
- `mp_hands`, `mp_pose`, `FINGERS`, `TIPS`, `WRIST`, `POSE_IDX` — topology
  (≈ lines 53–91).
- MediaPipe Hands per-frame usage (`hands.process`, iterating
  `multi_hand_landmarks` / `multi_handedness`) — (≈ lines 779–905).
- `arm_joint_angles(...)` + `_unit/_angle/_signed_angle` — joint-mirror math
  (≈ lines 466–527), only for Phase 7.
- Finger-extended / pinch logic is **not** in the example — implement fresh in
  the node (simple normalized-distance tests described in §5).

---

## 14. Risks & mitigations (summary)

| Risk | Impact | Mitigation |
| --- | --- | --- |
| MediaPipe ↔ ROS opencv/protobuf conflict | Build/import breaks | Pin + in-build import smoke test (§8) |
| Webcam not reachable in Docker on Windows | No live camera | Bundled demo video default; ZED via usbipd; or host-side detector (§9) |
| Jittery/unsafe arm motion | Poor UX / risk | OneEuro + existing smoothing/clamp; start frozen; clutch (§4.2,§5) |
| Submodule build (`sim_ctrl`) fails | No Gazebo control | Carry the z1_controller CMakeLists **sed fix** in Dockerfile verbatim |
| Handedness flipped | "right" tracks wrong hand | `flip_horizontal: true` selfie view; expose `gesture/hand` (§5.1) |
| Scope creep (joint mirror/depth) | Slips v1 | Gate behind config flags; Phases 7 only after 1–6 green |

---

*End of plan. Execute Phase 0→6 for a complete v1; Phase 7 is optional polish.*
