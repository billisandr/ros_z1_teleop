# Joint-Mirror Mode — Setup & Calibration

The default control law is the **cartesian hand-mirror** (move your hand, the arm
copies its position via IK). **Joint-mirror** is an alternative mode that reads
your whole arm with MediaPipe **Pose** (shoulder → elbow → wrist) and drives the
Z1 joints **directly**, bypassing IK (PLAN.md §4.3).

It is higher "wow factor" but lower precision and harder to bound, so it is
**OFF by default** and the mapping **needs hand-tuning** for your camera and body.
This guide is that tuning manual.

---

## 1. Enable it

In [../z1_teleop/config/teleop.yaml](../z1_teleop/config/teleop.yaml):

```yaml
control:
  mode: joint_mirror      # was: cartesian
```

Relaunch, then unpause:

```bash
docker rm -f ros-z1-teleop
bash start_teleop.sh                              # ZED, or the video/webcam sim launch
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
```

---

## 2. Drive it

- **Stand back** so your **upper body — both shoulders and hips — is in frame**.
  Pose needs the torso to build its reference frame; just the hand is not enough.
- **Open palm = drive, fist = freeze** (same clutch as cartesian). The arm starts
  and re-acquires **frozen**.
- The `/hand/debug_image` overlay (`z1_cam`) shows the **pose skeleton**, the mode,
  and the live `q[1-6]` angles. Green border = driving, red = frozen.
- **Stage 1 drives joints 1–3 only** (base yaw, shoulder lift, elbow). Wrist joints
  4–6 are disabled until you tune them.

---

## 3. The mapping

The detector publishes the 6 human arm angles (degrees) on `/hand/joint_targets`:

| idx | angle | meaning |
| --- | --- | --- |
| 0 | shoulder azimuth | upper-arm rotation about the torso vertical |
| 1 | shoulder elevation | lift of the upper arm above horizontal |
| 2 | humeral rotation | forearm roll about the upper-arm axis |
| 3 | elbow flexion | 0° = straight arm |
| 4 | wrist flexion | forearm-to-hand bend |
| 5 | wrist deviation | hand roll/deviation about the forearm |

The arm tracker maps each Z1 joint from **one** of these, in the `joint_mirror:`
block. Each entry is `map_jointN: [src, sign, scale, offset_deg, enabled]`:

```yaml
#            [src, sign, scale, offset_deg, enabled]
map_joint1:  [0,  1.0,  1.0,    0.0,  true ]   # base yaw    <- shoulder azimuth
map_joint2:  [1,  1.0,  1.0,   90.0,  true ]   # shoulder    <- shoulder elevation
map_joint3:  [3, -1.0,  1.0,    0.0,  true ]   # elbow       <- elbow flexion
map_joint4:  [2,  1.0,  1.0,    0.0,  false]   # wrist roll  <- humeral rotation
map_joint5:  [4,  1.0,  1.0,    0.0,  false]   # wrist pitch <- wrist flexion
map_joint6:  [5,  1.0,  1.0,    0.0,  false]   # wrist yaw   <- wrist deviation
```

Final command per joint: `clamp( sign * scale * human_angle[src] + offset_deg )`,
converted to radians and **clamped to `joint_limits`** — a wrong mapping can never
drive the arm past its limits, so tune freely.

---

## 4. Calibration by symptom

Watch one joint at a time. Tell the symptom; apply the fix:

| What you see | Cause | Fix |
| --- | --- | --- |
| **"Raising my arm bends the elbow instead of lifting the shoulder"** | The shoulder-lift motion is routed to the wrong Z1 joint — joint2 isn't picking up elevation, and/or joint3 is. | Confirm `map_joint2` uses `src: 1` (elevation) and `map_joint3` uses `src: 3` (elbow). If joint2 is `enabled: false`, the shoulder won't lift; enable it. If an unexpected joint reacts, its `src` index is wrong. |
| A joint moves the **opposite** direction | Sign convention differs from the Z1 axis | Flip `sign` (`1.0` ↔ `-1.0`) for that joint |
| Neutral pose is **offset** (e.g. arm horizontal should sit mid-range, but the joint is near a limit) | Zero-point mismatch | Adjust `offset_deg` (e.g. joint2 uses `+90` to map elevation −90…90° → 0…180°) |
| Motion is **too small / too large** | Gain mismatch | Adjust `scale` (e.g. `0.7` to damp, `1.5` to amplify) |
| Right joint, but reacts to the **wrong body part** | Wrong source angle | Change `src` to the correct index from the table above |
| Arm is **twitchy / jittery** | Pose noise | Lower `joint_mirror/smoothing_alpha` (e.g. `0.12` → `0.08`); raise `pose_min_detection_confidence`; improve lighting |
| Arm **doesn't move at all** | Not engaged, pose not seen, or joint disabled | Show an **open palm**; stand back so shoulders+hips are visible; check the joint's `enabled: true` and `z1_active` is `true` |
| **Wrist doesn't move** | Joints 4–6 ship disabled | Tune joints 1–3 first, then set `enabled: true` on `map_joint4/5/6` |
| Tracks the **wrong arm** | Side mismatch (selfie mirror) | Set `joint_mirror/pose_side: left` or `right` |

---

## 5. Fast tuning loop (no rebuild)

**Easiest: the live control panel.** Run `z1_ui` (browser, port 8501) and use the
**Joint-mirror** tab — sliders for each joint's `src`/`sign`/`scale`/`offset`/`enabled`
apply instantly (the arm tracker re-reads the map every loop). "Save to YAML" when
you like the result. See the README "Live control panel" section.

**Manual alternative.** Edit `teleop.yaml`, push it into the running container, and
restart only the arm tracker (the detector keeps running):

```bash
docker cp z1_teleop/config/teleop.yaml \
    ros-z1-teleop:/home/rosuser/catkin_ws/src/z1_teleop/config/teleop.yaml
docker exec -it ros-z1-teleop bash -ic \
    "rosparam load ~/catkin_ws/src/z1_teleop/config/teleop.yaml; rosnode kill /arm_tracker; z1_tracker"
```

Changing `pose_side` or pose-confidence params instead? Restart the detector too
(`rosnode kill /hand_detector; z1_hand`).

---

## 6. Safety

- `joint_limits` clamping is always applied in joint-mirror — bad mappings cannot
  exceed the arm's physical range.
- The hand clutch still gates everything: **fist freezes** instantly; the arm
  **starts frozen** and **re-freezes** when the pose/hand is lost.
- Start with small, slow motions and only enable the wrist joints once the big
  joints (1–3) feel right.
