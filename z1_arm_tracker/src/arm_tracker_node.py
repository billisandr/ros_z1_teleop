#!/usr/bin/env python3
"""
Arm Tracker Node — hand-teleop control.
Follows the operator's hand target in 2D (Y lateral + Z vertical) by publishing
MotorCmd directly to the Gazebo joint controllers via ROS topics.

Subscribes to the hand detector's two topics (renamed from the old ArUco ones):
    /hand/target_pose      geometry_msgs/PoseStamped  (world frame)
    /hand/tracking_active  std_msgs/Bool              (clutch + detection gate)
    /hand/gripper_cmd      std_msgs/Float64           (optional pinch -> gripper, 0..1)

No sim_ctrl or UDP required — pure ROS control.
IK is computed using the Unitree Z1 model from the SDK (model only, no UDP).

Fixed X (forward reach) is held constant; only Y and Z follow the hand. The
pinch->gripper passthrough maps 0..1 onto the jointGripper limits.
"""

import sys
import threading
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState

SDK_LIB_PATH = '/home/rosuser/sdk_z1/lib'
if SDK_LIB_PATH not in sys.path:
    sys.path.insert(0, SDK_LIB_PATH)

try:
    import unitree_arm_interface as arm_sdk
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    rospy.logwarn("[arm_tracker] unitree_arm_interface not found — running in DRY RUN mode.")

try:
    from unitree_legged_msgs.msg import MotorCmd
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    rospy.logwarn("[arm_tracker] unitree_legged_msgs not found — running in DRY RUN mode.")


# Joint order matching IOROS.cpp publisher indices 0-5
JOINT_TOPICS = [
    '/z1_gazebo/Joint01_controller/command',
    '/z1_gazebo/Joint02_controller/command',
    '/z1_gazebo/Joint03_controller/command',
    '/z1_gazebo/Joint04_controller/command',
    '/z1_gazebo/Joint05_controller/command',
    '/z1_gazebo/Joint06_controller/command',
]
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class ArmTrackerNode:
    def __init__(self):
        rospy.init_node('arm_tracker', anonymous=False)

        self.update_rate = rospy.get_param('arm_tracker/update_rate',     50.0)
        self.enabled     = rospy.get_param('arm_tracker/enabled',         True)
        self.fixed_x     = rospy.get_param('arm_tracker/fixed_x',         0.50)
        offset           = rospy.get_param('arm_tracker/target_offset',  [0.0, 0.0, 0.0])
        self.offset_y    = offset[1]
        self.offset_z    = offset[2]

        # Low-pass filter on Cartesian target: 0.0=frozen, 1.0=instant, 0.05=slow/smooth
        self.alpha = rospy.get_param('arm_tracker/smoothing_alpha', 0.05)

        # Wrist orientation tracking
        self.track_orientation     = rospy.get_param('arm_tracker/track_orientation',   True)
        self.orientation_alpha     = rospy.get_param('arm_tracker/orientation_alpha',   0.05)
        self.stability_threshold   = rospy.get_param('arm_tracker/stability_threshold', 0.003)
        self.stability_ticks_req   = int(rospy.get_param('arm_tracker/stability_ticks', 15))

        # Joint PD gains for MotorCmd
        self.kp = rospy.get_param('arm_tracker/joint_kp', 150.0)
        self.kd = rospy.get_param('arm_tracker/joint_kd',   3.0)

        # Smoothed Cartesian target — initialised to fixed_x, arm home Y/Z
        self._tx = self.fixed_x
        self._ty = 0.0
        self._tz = 0.40
        # Smoothed orientation target (radians)
        self._tpitch = 0.0
        self._tyaw   = 0.0
        # Stability gate state
        self._stable_ticks  = 0
        self._prev_ty_raw   = 0.0
        self._prev_tz_raw   = 0.40

        ws = rospy.get_param('arm_tracker/workspace', {})
        self.y_min = ws.get('y', [-0.35, 0.35])[0]
        self.y_max = ws.get('y', [-0.35, 0.35])[1]
        self.z_min = ws.get('z', [0.10, 0.75])[0]
        self.z_max = ws.get('z', [0.10, 0.75])[1]

        self.tracking_active = False
        self.latest_pose     = None
        self.q_current       = np.zeros(6)
        self._q_lock         = threading.Lock()

        # Gripper passthrough (pinch -> gripper). jointGripper limits from the Z1
        # URDF (const.xacro): closed = 0.0 rad (Gripper_PositionMax),
        # open = -pi/2 rad (Gripper_PositionMin). Incoming cmd is 0=closed..1=open.
        self.enable_gripper   = rospy.get_param('arm_tracker/enable_gripper',       True)
        self.gripper_closed   = rospy.get_param('arm_tracker/gripper_closed_angle', 0.0)
        self.gripper_open     = rospy.get_param('arm_tracker/gripper_open_angle',  -1.5708)
        self._gripper_cmd_val = 0.0   # latest 0..1 command (start closed)

        if not self.enabled:
            rospy.loginfo("[arm_tracker] Tracking disabled in config — observe mode.")

        # Joint state subscriber — keeps q_current up to date for IK warm-start
        rospy.Subscriber('/z1_gazebo/joint_states', JointState,  self._joint_state_cb, queue_size=1)
        rospy.Subscriber('/hand/target_pose',        PoseStamped, self._pose_cb,        queue_size=1)
        rospy.Subscriber('/hand/tracking_active',    Bool,        self._active_cb,      queue_size=1)
        if self.enable_gripper:
            rospy.Subscriber('/hand/gripper_cmd',    Float64,     self._gripper_cb,     queue_size=1)

        # One publisher per joint — matches IOROS topic order
        self.joint_pubs = None
        if MSGS_AVAILABLE:
            self.joint_pubs = [
                rospy.Publisher(t, MotorCmd, queue_size=1) for t in JOINT_TOPICS
            ]

        # Gripper command publisher — same UnitreeJointController/MotorCmd as joints
        self.gripper_pub = None
        if MSGS_AVAILABLE and self.enable_gripper:
            self.gripper_pub = rospy.Publisher(
                '/z1_gazebo/gripper_controller/command', MotorCmd, queue_size=1)

        # Arm model for IK — instantiate without loopOn() (no UDP connection)
        self.arm_model = None
        if SDK_AVAILABLE:
            try:
                _iface = arm_sdk.ArmInterface(hasGripper=True)
                self.arm_model = _iface._ctrlComp.armModel
                rospy.loginfo("[arm_tracker] Z1 model loaded for IK.")
            except Exception as e:
                rospy.logwarn(f"[arm_tracker] Could not load arm model: {e}")

        rospy.loginfo(f"[arm_tracker] Ready — fixed_x={self.fixed_x:.2f} m, "
                      f"Y∈[{self.y_min:.2f},{self.y_max:.2f}], "
                      f"Z∈[{self.z_min:.2f},{self.z_max:.2f}], "
                      f"gripper={'on' if self.enable_gripper else 'off'}")

    # ------------------------------------------------------------------ callbacks

    def _joint_state_cb(self, msg):
        with self._q_lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.q_current[i] = msg.position[idx]

    def _active_cb(self, msg):
        self.tracking_active = msg.data

    def _pose_cb(self, msg):
        self.latest_pose = msg

    def _gripper_cb(self, msg):
        # incoming 0..1 (0=closed, 1=open); clamp for safety
        self._gripper_cmd_val = self._clamp(msg.data, 0.0, 1.0)

    # ------------------------------------------------------------------ helpers

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _refresh_live_params(self):
        """Re-read the control knobs the workshop UI is allowed to change at runtime.
        smoothing_alpha shapes the target filter below; kp/kd are stamped into every
        MotorCmd, so updating them here takes effect on the next published command.
        Called once per loop; cheap (local param-server cache lookup)."""
        self.alpha   = rospy.get_param('arm_tracker/smoothing_alpha', self.alpha)
        self.kp      = rospy.get_param('arm_tracker/joint_kp',        self.kp)
        self.kd      = rospy.get_param('arm_tracker/joint_kd',        self.kd)
        self.enabled = rospy.get_param('arm_tracker/enabled',         self.enabled)

    def _update_stability(self, ty_raw, tz_raw):
        """Increment stable-tick counter when position target is barely moving."""
        delta = abs(ty_raw - self._prev_ty_raw) + abs(tz_raw - self._prev_tz_raw)
        self._prev_ty_raw = ty_raw
        self._prev_tz_raw = tz_raw
        if delta < self.stability_threshold:
            self._stable_ticks = min(self._stable_ticks + 1, self.stability_ticks_req)
        else:
            self._stable_ticks = 0
        return self._stable_ticks >= self.stability_ticks_req

    def _facing_angles(self):
        """Pitch and yaw for the end-effector to face the marker in world frame.
        Computed from the arm base (world origin) to the marker — NOT from the
        end-effector target, which would give ~zero vector because the arm follows
        the marker in Y and Z.
        With Z1 postureToHomo convention (rpy=0 → end-effector points +X world):
          yaw   rotates left/right around world Z
          pitch rotates up/down around world Y
        """
        p = self.latest_pose.pose.position
        dist_xy = max(np.sqrt(p.x * p.x + p.y * p.y), 1e-6)
        yaw   =  np.arctan2(p.y, p.x)
        pitch = -np.arctan2(p.z, dist_xy)
        return pitch, yaw

    @staticmethod
    def _angle_diff(target, current):
        """Shortest signed angular difference, handles ±π wraparound."""
        return (target - current + np.pi) % (2.0 * np.pi) - np.pi

    def _send_joint_commands(self, q_target):
        for i, pub in enumerate(self.joint_pubs):
            cmd = MotorCmd()
            cmd.mode = 10        # position + velocity + torque PD control
            cmd.q    = float(q_target[i])
            cmd.dq   = 0.0
            cmd.tau  = 0.0
            cmd.Kp   = self.kp
            cmd.Kd   = self.kd
            pub.publish(cmd)

    def _send_gripper_command(self):
        """Hold the gripper at the latest pinch-derived angle every loop, so it
        keeps position even while the arm is frozen (clutch disengaged)."""
        if self.gripper_pub is None:
            return
        q = self.gripper_closed + self._gripper_cmd_val * (self.gripper_open - self.gripper_closed)
        cmd = MotorCmd()
        cmd.mode = 10
        cmd.q    = float(q)
        cmd.dq   = 0.0
        cmd.tau  = 0.0
        cmd.Kp   = self.kp
        cmd.Kd   = self.kd
        self.gripper_pub.publish(cmd)

    # ------------------------------------------------------------------ main loop

    def run(self):
        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self._refresh_live_params()
            self._send_gripper_command()
            if self.enabled and self.tracking_active and self.latest_pose is not None:
                p = self.latest_pose.pose.position

                # 2D tracking: hold X fixed, follow the hand target Y and Z only
                # Low-pass filter smooths the target to prevent jerky motion
                ty_raw = self._clamp(p.y + self.offset_y, self.y_min, self.y_max)
                tz_raw = self._clamp(p.z + self.offset_z, self.z_min, self.z_max)
                self._ty += self.alpha * (ty_raw - self._ty)
                self._tz += self.alpha * (tz_raw - self._tz)
                tx = self.fixed_x
                ty = self._ty
                tz = self._tz

                is_stable = self._update_stability(ty_raw, tz_raw)

                if self.arm_model is not None and self.joint_pubs is not None:
                    try:
                        # --- Position IK (always active, orientation fixed at zero) ---
                        pos_posture = np.array([0.0, 0.0, 0.0, tx, ty, tz])
                        T_pos = arm_sdk.postureToHomo(pos_posture)
                        with self._q_lock:
                            q_pos = self.q_current.copy()
                        pos_ok, q_pos = self.arm_model.inverseKinematics(T_pos, q_pos, True)

                        if not pos_ok:
                            rospy.logwarn_throttle(2.0,
                                f"[arm_tracker] IK failed for target "
                                f"x:{tx:.3f} y:{ty:.3f} z:{tz:.3f}")
                            rate.sleep()
                            continue

                        q_send = q_pos.copy()

                        # --- Wrist orientation IK (only when stable) ---
                        if self.track_orientation and is_stable:
                            pitch_raw, yaw_raw = self._facing_angles()
                            self._tpitch += self.orientation_alpha * self._angle_diff(pitch_raw, self._tpitch)
                            self._tyaw   += self.orientation_alpha * self._angle_diff(yaw_raw,   self._tyaw)

                            ori_posture = np.array([0.0, self._tpitch, self._tyaw, tx, ty, tz])
                            T_ori = arm_sdk.postureToHomo(ori_posture)
                            q_ori = q_pos.copy()  # warm-start from position solution
                            # checkWorkspace=False: position already validated above;
                            # workspace check rejects valid wrist-only orientation changes.
                            ori_ok, q_ori = self.arm_model.inverseKinematics(T_ori, q_ori, False)

                            if ori_ok:
                                # Joints 1-3: position from pos IK (unchanged)
                                # Joints 4-6: wrist from orientation IK
                                q_send[3:] = q_ori[3:]
                            else:
                                rospy.logdebug("[arm_tracker] Orientation IK failed — holding wrist")

                        self._send_joint_commands(q_send)

                    except Exception as e:
                        rospy.logwarn_throttle(5.0, f"[arm_tracker] Control error: {e}")
                else:
                    rospy.loginfo_throttle(1.0,
                        f"[arm_tracker] DRY RUN — x:{tx:.3f} y:{ty:.3f} z:{tz:.3f}")

            rate.sleep()


if __name__ == '__main__':
    try:
        node = ArmTrackerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
