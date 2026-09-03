# Joint-Mirror Mode — Setup & Calibration

The default control law is the Cartesian hand-mirror: move your hand, the arm
copies its position through IK. Joint-mirror is an alternative that reads
your whole arm with MediaPipe Pose (shoulder, elbow, wrist) and drives the
Z1 joints directly, bypassing IK.

It has a higher "wow factor" but lower precision, and it's harder to bound,
so it ships off by default and the mapping needs hand-tuning for your camera
and body. This guide walks through that tuning.

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

- Stand back so your upper body, both shoulders and hips, is in frame. Pose
  needs the torso to build its reference frame; the hand alone isn't enough.
- Open palm drives, fist freezes, same clutch as Cartesian mode. The arm
  starts and re-acquires frozen.
- The `/hand/debug_image` overlay (`z1_cam`) shows the pose skeleton, the
  mode, and the live `q[1-6]` angles. Green border means driving, red means
  frozen.
- Stage 1 only drives joints 1-3 (base yaw, shoulder lift, elbow). Wrist
  joints 4-6 stay disabled until you tune them.

---

## 3. The mapping

The detector publishes six human arm angles, in degrees, on
`/hand/joint_targets`:

| idx | angle | meaning |
| --- | --- | --- |
| 0 | shoulder azimuth | upper-arm rotation about the torso vertical |
| 1 | shoulder elevation | lift of the upper arm above horizontal |
| 2 | humeral rotation | forearm roll about the upper-arm axis |
| 3 | elbow flexion | 0° = straight arm |
| 4 | wrist flexion | forearm-to-hand bend |
| 5 | wrist deviation | hand roll/deviation about the forearm |

The arm tracker maps each Z1 joint from one of these, in the `joint_mirror:`
block. Each entry reads `map_jointN: [src, sign, scale, offset_deg, enabled]`:

```yaml
#            [src, sign, scale, offset_deg, enabled]
map_joint1:  [0,  1.0,  1.0,    0.0,  true ]   # base yaw    <- shoulder azimuth
map_joint2:  [1,  1.0,  1.0,   90.0,  true ]   # shoulder    <- shoulder elevation
map_joint3:  [3, -1.0,  1.0,    0.0,  true ]   # elbow       <- elbow flexion
map_joint4:  [2,  1.0,  1.0,    0.0,  false]   # wrist roll  <- humeral rotation
map_joint5:  [4,  1.0,  1.0,    0.0,  false]   # wrist pitch <- wrist flexion
map_joint6:  [5,  1.0,  1.0,    0.0,  false]   # wrist yaw   <- wrist deviation
```

The final command per joint is `clamp( sign * scale * human_angle[src] +
offset_deg )`, converted to radians and clamped to `joint_limits`. A wrong
mapping can never drive the arm past its physical limits, so it's safe to
tune freely.

---

## 4. Calibration by symptom

Watch one joint at a time. Here's what we've run into and how we fixed it:

| What you see | Cause | Fix |
| --- | --- | --- |
| Raising your arm bends the elbow instead of lifting the shoulder | The shoulder-lift motion is routed to the wrong Z1 joint: joint2 isn't picking up elevation, and/or joint3 is. | Confirm `map_joint2` uses `src: 1` (elevation) and `map_joint3` uses `src: 3` (elbow). If joint2 is `enabled: false`, the shoulder won't lift; enable it. If an unexpected joint reacts, its `src` index is wrong. |
| A joint moves the opposite direction | Sign convention differs from the Z1 axis | Flip `sign` (`1.0` to `-1.0`, or back) for that joint |
| Neutral pose sits offset (say, arm horizontal should sit mid-range but the joint is near a limit) | Zero-point mismatch | Adjust `offset_deg` (joint2 uses `+90` to map elevation from -90...90° into 0...180°) |
| Motion is too small or too large | Gain mismatch | Adjust `scale` (`0.7` to damp, `1.5` to amplify) |
| Right joint, wrong body part | Wrong source angle | Change `src` to the correct index from the table above |
| Arm is twitchy or jittery | Pose noise | Lower `joint_mirror/smoothing_alpha` (e.g. `0.12` to `0.08`); raise `pose_min_detection_confidence`; improve lighting |
| Arm doesn't move at all | Not engaged, pose not seen, or joint disabled | Show an open palm; stand back so shoulders and hips are visible; check the joint's `enabled: true` and that `z1_active` is `true` |
| Wrist doesn't move | Joints 4-6 ship disabled | Tune joints 1-3 first, then set `enabled: true` on `map_joint4/5/6` |
| Tracks the wrong arm | Side mismatch (selfie mirror) | Set `joint_mirror/pose_side: left` or `right` |

---

## 5. Fast tuning loop (no rebuild)

The easiest path is the live control panel. Run `z1_ui` (browser, port 8501)
and use the Joint-mirror tab: sliders for each joint's `src`/`sign`/`scale`/
`offset`/`enabled` apply instantly, since the arm tracker re-reads the map
every loop. Hit "Save to YAML" once you like the result. See the README's
"Live control panel" section for more.

The manual alternative is to edit `teleop.yaml`, push it into the running
container, and restart only the arm tracker (leave the detector running):

```bash
docker cp z1_teleop/config/teleop.yaml \
    ros-z1-teleop:/home/rosuser/catkin_ws/src/z1_teleop/config/teleop.yaml
docker exec -it ros-z1-teleop bash -ic \
    "rosparam load ~/catkin_ws/src/z1_teleop/config/teleop.yaml; rosnode kill /arm_tracker; z1_tracker"
```

If you're changing `pose_side` or the pose-confidence params, restart the
detector too (`rosnode kill /hand_detector; z1_hand`).

---

## 6. Safety

- `joint_limits` clamping always applies in joint-mirror mode, so bad
  mappings can't push the arm past its physical range.
- The hand clutch still gates everything: a fist freezes instantly, and the
  arm starts frozen and re-freezes whenever the pose or hand is lost.
- Start with small, slow motions, and only enable the wrist joints once the
  big joints (1-3) feel right.
