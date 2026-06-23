#!/usr/bin/env python3
"""
Hand Detector Node — MediaPipe Hands -> world-frame target for the Z1 arm.

Replaces the old ArUco detector. Subscribes to /camera/color/image_raw and
publishes the SAME two topics the controller consumes (renamed for hand mode),
plus a debug overlay and an optional gripper command:

    /hand/target_pose      geometry_msgs/PoseStamped  (world frame)
    /hand/tracking_active  std_msgs/Bool              (clutch + detection gate)
    /hand/debug_image      sensor_msgs/Image          (landmark + HUD overlay)
    /hand/gripper_cmd      std_msgs/Float64           (optional pinch -> gripper, 0..1)

Default control law = Cartesian "hand mirror" (PLAN.md §4.2): the tracked hand
point's normalized image coords (u, v) map linearly to a world Y/Z target with X
held fixed. No camera calibration, no TF, no depth — the detector emits a
world-frame pose straight from a linear map.

Operator conventions (PLAN.md §5): track one configured hand; open palm ENGAGES,
fist FREEZES (clutch with frame hysteresis); optional pinch drives the gripper.
The arm starts FROZEN until the operator shows an open palm.

OneEuroFilter and the landmark-topology constants are lifted from the standalone
reference tool (reference/hand_tracker_3d.py); see PLAN.md §13.
"""

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64

try:
    import mediapipe as mp
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "[hand_detector] mediapipe is required but could not be imported (%s). "
        "Build/run inside the project Docker image (see PLAN.md §8)." % exc
    )


# ---------------------------------------------------------------------------
# Landmark topology (lifted from reference/hand_tracker_3d.py, §13)
# ---------------------------------------------------------------------------
WRIST = 0
# Long fingers: a finger is "extended" when its TIP is farther from the wrist
# than its PIP joint (in normalized image coords).
FINGER_PIPS = {'index': 6, 'middle': 10, 'ring': 14, 'pinky': 18}
FINGER_TIPS = {'index': 8, 'middle': 12, 'ring': 16, 'pinky': 20}
PALM_LMS = [0, 5, 9, 13, 17]          # wrist + the four finger MCPs (palm centroid)
THUMB_TIP, THUMB_IP, INDEX_MCP = 4, 3, 5
INDEX_TIP = 8


# ---------------------------------------------------------------------------
# One Euro filter — per-coordinate low-lag smoothing (lifted verbatim, §13)
# ---------------------------------------------------------------------------
class OneEuroFilter:
    def __init__(self, freq, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.freq = float(freq)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = None
        self.dx_prev = None

    def reset(self):
        self.x_prev = None
        self.dx_prev = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2.0 * np.pi * np.asarray(cutoff, dtype=np.float64))
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x):
        x = np.asarray(x, dtype=np.float64)
        if self.x_prev is None:
            self.x_prev = x.copy()
            self.dx_prev = np.zeros_like(x)
            return x.copy()
        dx = (x - self.x_prev) * self.freq
        a_d = self._alpha(self.d_cutoff)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = self._alpha(cutoff)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


def _lerp(t, a, b):
    return a + float(t) * (b - a)


class HandDetectorNode:
    def __init__(self):
        rospy.init_node('hand_detector', anonymous=False)
        self.bridge = CvBridge()

        # --- control mode (joint_mirror is a Phase-7 stretch; cartesian default) ---
        self.control_mode = rospy.get_param('control/mode', 'cartesian')
        if self.control_mode == 'joint_mirror':
            rospy.logwarn("[hand_detector] control/mode='joint_mirror' is a Phase-7 "
                          "stretch and is not implemented yet — using Cartesian mirror.")
            self.control_mode = 'cartesian'

        # --- MediaPipe Hands params ---
        self.flip_horizontal = bool(rospy.get_param('hand/flip_horizontal', True))
        model_complexity = int(rospy.get_param('hand/model_complexity', 1))
        min_det = float(rospy.get_param('hand/min_detection_confidence', 0.6))
        min_trk = float(rospy.get_param('hand/min_tracking_confidence', 0.6))

        # --- mapping (image -> world) ---
        self.tracked_point = rospy.get_param('mapping/tracked_point', 'wrist')
        self.fixed_x = float(rospy.get_param('mapping/fixed_x', 0.25))
        self.y_range = rospy.get_param('mapping/y_range', [-0.35, 0.35])   # [y_min, y_max]
        self.z_range = rospy.get_param('mapping/z_range', [0.10, 0.75])    # [z_min, z_max]
        self.deadzone = float(rospy.get_param('mapping/deadzone', 0.0))
        self.depth_from_hand_size = bool(rospy.get_param('mapping/depth_from_hand_size', False))
        if self.depth_from_hand_size:
            rospy.logwarn("[hand_detector] mapping/depth_from_hand_size is a Phase-7 "
                          "stretch and is not implemented yet — X stays fixed.")

        # --- gesture / conventions ---
        self.hand_pref = rospy.get_param('gesture/hand', 'right').lower()        # right|left|either
        self.clutch_mode = rospy.get_param('gesture/clutch', 'palm_fist')        # palm_fist|always_on|pinch_hold
        self.gripper_mode = rospy.get_param('gesture/gripper', 'pinch')          # pinch|none
        self.lost_frames = int(rospy.get_param('gesture/lost_frames', 10))
        self.hysteresis_frames = int(rospy.get_param('gesture/hysteresis_frames', 3))
        self.open_fingers = int(rospy.get_param('gesture/open_fingers', 4))      # >= -> open palm
        self.fist_fingers = int(rospy.get_param('gesture/fist_fingers', 1))      # <= -> closed fist
        self.pinch_hold_dist = float(rospy.get_param('gesture/pinch_hold_dist', 0.06))
        # pinch -> gripper mapping: small distance = closed (0), large = open (1)
        self.pinch_closed = float(rospy.get_param('gesture/pinch_closed_dist', 0.03))
        self.pinch_open = float(rospy.get_param('gesture/pinch_open_dist', 0.18))

        # --- smoothing (OneEuro on the normalized (u,v) target) ---
        freq = float(rospy.get_param('smoothing/freq', 30.0))
        min_cutoff = float(rospy.get_param('smoothing/min_cutoff', 1.0))
        beta = float(rospy.get_param('smoothing/beta', 0.0))
        self.smoother = OneEuroFilter(freq, min_cutoff=min_cutoff, beta=beta)

        self.show_debug = bool(rospy.get_param('camera/show_debug_image', True))

        # --- runtime state ---
        # SAFETY (§5.6): start FROZEN — never lurch on launch.
        self.engaged = False
        self._open_streak = 0
        self._fist_streak = 0
        self._frames_since_hand = self.lost_frames + 1   # treat as "lost" until first hand
        self._last_uv = None

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,                 # see both hands so handedness selection works
            model_complexity=model_complexity,
            min_detection_confidence=min_det,
            min_tracking_confidence=min_trk,
        )
        self._mp_draw = mp.solutions.drawing_utils
        self._mp_styles = mp.solutions.drawing_styles
        self._hand_conn = mp.solutions.hands.HAND_CONNECTIONS

        # --- publishers ---
        self.pose_pub = rospy.Publisher('/hand/target_pose', PoseStamped, queue_size=1)
        self.active_pub = rospy.Publisher('/hand/tracking_active', Bool, queue_size=1)
        self.debug_pub = rospy.Publisher('/hand/debug_image', Image, queue_size=1)
        self.gripper_pub = rospy.Publisher('/hand/gripper_cmd', Float64, queue_size=1)

        # Publish the safe initial state immediately so the controller knows the
        # arm must stay frozen until the operator engages.
        self.active_pub.publish(Bool(data=False))

        # --- subscriber ---
        rospy.Subscriber('/camera/color/image_raw', Image, self._image_cb,
                         queue_size=1, buff_size=2 ** 24)

        rospy.loginfo(
            "[hand_detector] Ready — tracking %s hand, clutch=%s, gripper=%s. "
            "Map: X=%.2f, Y%s, Z%s. Arm starts FROZEN (show an open palm to drive)."
            % (self.hand_pref, self.clutch_mode, self.gripper_mode, self.fixed_x,
               self.y_range, self.z_range)
        )

    # ------------------------------------------------------------------ helpers

    def _select_hand(self, result):
        """Return (landmarks, label) for the configured hand, or (None, None).

        With flip_horizontal=True the frame is mirrored (selfie view), so the
        MediaPipe handedness label already matches the operator's real hand
        (PLAN.md §5.1)."""
        if not result.multi_hand_landmarks:
            return None, None
        handed = result.multi_handedness or []
        best = None
        for i, lms in enumerate(result.multi_hand_landmarks):
            label = 'Unknown'
            score = 0.0
            if i < len(handed) and handed[i].classification:
                label = handed[i].classification[0].label   # 'Left' | 'Right'
                score = handed[i].classification[0].score
            if self.hand_pref == 'either':
                if best is None or score > best[2]:
                    best = (lms, label, score)
            elif label.lower() == self.hand_pref:
                if best is None or score > best[2]:
                    best = (lms, label, score)
        if best is None:
            return None, None
        return best[0], best[1]

    @staticmethod
    def _landmarks_to_xy(lms):
        xy = np.zeros((21, 2), dtype=np.float64)
        for i, lm in enumerate(lms.landmark):
            xy[i] = [lm.x, lm.y]
        return xy

    def _extended_fingers(self, xy):
        """Return (count, {finger: bool}) of extended fingers in normalized coords."""
        wrist = xy[WRIST]
        states = {}
        for name in ('index', 'middle', 'ring', 'pinky'):
            tip = np.linalg.norm(xy[FINGER_TIPS[name]] - wrist)
            pip = np.linalg.norm(xy[FINGER_PIPS[name]] - wrist)
            states[name] = bool(tip > pip)
        # Thumb: lateral test — extended when the tip is farther from the index
        # MCP than the thumb IP is (abducted), folded when it tucks over the palm.
        ref = xy[INDEX_MCP]
        states['thumb'] = bool(
            np.linalg.norm(xy[THUMB_TIP] - ref) > np.linalg.norm(xy[THUMB_IP] - ref)
        )
        count = sum(1 for v in states.values() if v)
        return count, states

    def _pinch_distance(self, xy):
        """Normalized thumb-tip <-> index-tip distance."""
        return float(np.linalg.norm(xy[THUMB_TIP] - xy[INDEX_TIP]))

    def _update_clutch(self, count, pinch_d):
        """Latch engage/disengage with frame hysteresis (PLAN.md §5.2)."""
        if self.clutch_mode == 'always_on':
            self.engaged = True
            return
        if self.clutch_mode == 'pinch_hold':
            want_engage = pinch_d < self.pinch_hold_dist
            if want_engage:
                self._open_streak += 1
                self._fist_streak = 0
            else:
                self._fist_streak += 1
                self._open_streak = 0
        else:  # palm_fist (default)
            if count >= self.open_fingers:
                self._open_streak += 1
                self._fist_streak = 0
            elif count <= self.fist_fingers:
                self._fist_streak += 1
                self._open_streak = 0
            else:
                # ambiguous hand shape — hold current state
                self._open_streak = 0
                self._fist_streak = 0
        if self._open_streak >= self.hysteresis_frames:
            self.engaged = True
        elif self._fist_streak >= self.hysteresis_frames:
            self.engaged = False

    def _tracked_uv(self, xy):
        if self.tracked_point == 'palm_centroid':
            uv = xy[PALM_LMS].mean(axis=0)
        else:  # wrist
            uv = xy[WRIST]
        u = float(np.clip(uv[0], 0.0, 1.0))
        v = float(np.clip(uv[1], 0.0, 1.0))
        # optional centered deadzone to kill jitter when the hand is ~still
        if self.deadzone > 0.0:
            if abs(u - 0.5) < self.deadzone:
                u = 0.5
            if abs(v - 0.5) < self.deadzone:
                v = 0.5
        return u, v

    def _uv_to_world(self, u, v):
        y_min, y_max = self.y_range[0], self.y_range[1]
        z_min, z_max = self.z_range[0], self.z_range[1]
        # u: 0..1 -> y_max..y_min (mirrored so hand-right -> arm-right)
        # v: 0(top)..1(bottom) -> z_max..z_min (inverted: image v grows downward)
        y = _lerp(u, y_max, y_min)
        z = _lerp(v, z_max, z_min)
        return self.fixed_x, y, z

    def _gripper_cmd(self, pinch_d):
        """Map pinch distance to a 0..1 gripper command (small=closed, large=open)."""
        span = max(self.pinch_open - self.pinch_closed, 1e-6)
        return float(np.clip((pinch_d - self.pinch_closed) / span, 0.0, 1.0))

    # ------------------------------------------------------------------ callback

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, "[hand_detector] cv_bridge error: %s" % e)
            return

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        result = self.hands.process(rgb)
        lms, label = self._select_hand(result)

        active = False
        target_world = None
        gripper_val = None
        count = 0
        finger_states = {}
        pinch_d = None

        if lms is not None:
            # re-acquired after a real loss -> reset the smoother to avoid a lag-jump
            if self._frames_since_hand > self.lost_frames:
                self.smoother.reset()
            self._frames_since_hand = 0

            xy = self._landmarks_to_xy(lms)
            count, finger_states = self._extended_fingers(xy)
            pinch_d = self._pinch_distance(xy)

            self._update_clutch(count, pinch_d)

            u, v = self._tracked_uv(xy)
            u_s, v_s = self.smoother(np.array([u, v]))
            self._last_uv = (float(u_s), float(v_s))
            target_world = self._uv_to_world(float(u_s), float(v_s))

            active = self.engaged

            if self.gripper_mode == 'pinch':
                gripper_val = self._gripper_cmd(pinch_d)
        else:
            self._frames_since_hand += 1

        # Tracking gate (§5.3): true iff configured hand present AND engaged AND
        # not lost for too long. Hand absence beyond lost_frames -> freeze.
        if self._frames_since_hand > self.lost_frames:
            active = False

        # Publish the target only while we have a hand; when frozen/lost the
        # controller holds its last commanded pose (it ignores pose when inactive).
        if target_world is not None:
            px, py, pz = target_world
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
            pose.header.frame_id = 'world'
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = pz
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)

        self.active_pub.publish(Bool(data=active))

        if gripper_val is not None:
            self.gripper_pub.publish(Float64(data=gripper_val))

        if self.show_debug:
            self._publish_debug(frame, lms, label, count, finger_states,
                                active, target_world, gripper_val, pinch_d)

    # ------------------------------------------------------------------ debug HUD

    def _publish_debug(self, frame, lms, label, count, finger_states,
                       active, target_world, gripper_val, pinch_d):
        if lms is not None:
            self._mp_draw.draw_landmarks(
                frame, lms, self._hand_conn,
                self._mp_styles.get_default_hand_landmarks_style(),
                self._mp_styles.get_default_hand_connections_style(),
            )

        h, w = frame.shape[:2]
        # green border when driving, red when frozen
        border_col = (0, 200, 0) if active else (0, 0, 220)
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), border_col, 8)

        clutch_txt = 'ENGAGED' if self.engaged else 'FROZEN'
        lost = self._frames_since_hand > self.lost_frames
        lines = [
            "hand: %s (want %s)" % (label or '--', self.hand_pref),
            "fingers up: %d  clutch: %s" % (count, clutch_txt),
            "tracking_active: %s%s" % (active, '  [LOST]' if lost else ''),
        ]
        if target_world is not None:
            lines.append("target  Y:% .3f  Z:% .3f  (X=%.2f)"
                         % (target_world[1], target_world[2], target_world[0]))
        if gripper_val is not None:
            lines.append("gripper: %.2f  (pinch d=%.3f)" % (gripper_val, pinch_d or 0.0))

        y0 = 28
        for i, text in enumerate(lines):
            y = y0 + i * 26
            cv2.putText(frame, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 1, cv2.LINE_AA)

        cheat = "Open palm = drive   |   Fist = freeze"
        cv2.putText(frame, cheat, (16, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, cheat, (16, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 0), 1, cv2.LINE_AA)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception:
            pass


if __name__ == '__main__':
    try:
        HandDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
