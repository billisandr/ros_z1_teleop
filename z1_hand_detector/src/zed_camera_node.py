#!/usr/bin/env python3
"""
ZED 2 camera bridge — alternative to the RealSense D435 driver for real camera mode.

Opens the ZED 2's raw UVC side-by-side stereo feed directly via OpenCV/V4L2 (no ZED
SDK — depth/positional tracking are unused here, only one rectified-ish RGB eye),
crops it to a single eye, and republishes on /camera/color/image_raw +
/camera/color/camera_info — the same topics hand_detector_node subscribes to,
regardless of which physical camera is feeding them.

Intrinsics are a rough pinhole estimate from the ZED 2's published horizontal FOV,
not the factory per-unit calibration (which normally requires the ZED SDK to fetch).
Good enough to confirm detection/tracking end-to-end; expect some error in the
estimated marker distance/pose.

On Windows + usbipd, isochronous USB-over-IP corrupts high-bandwidth video — see
docs/DOCKER_CMDS.md#diagnosing-corruptedgarbled-video-over-usbipd-windows. Pick a
device/width/height/fps combination confirmed clean with ffplay before trusting
this node's output.
"""

import math
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class ZedCameraNode:
    def __init__(self):
        rospy.init_node('zed_camera', anonymous=False)

        self.device = rospy.get_param('zed_camera/device', '/dev/video0')
        self.width = rospy.get_param('zed_camera/width', 1344)   # combined side-by-side width (both eyes)
        self.height = rospy.get_param('zed_camera/height', 376)
        self.fps = rospy.get_param('zed_camera/fps', 15)
        self.eye = rospy.get_param('zed_camera/eye', 'left')     # 'left' or 'right'
        self.hfov_deg = rospy.get_param('zed_camera/hfov_deg', 110.0)  # ZED 2 wide-lens spec

        self.eye_width = self.width // 2

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"[zed_camera] Could not open {self.device}")

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        self.info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=1)
        self.camera_info_msg = self._build_camera_info()

        rospy.loginfo(
            f"[zed_camera] Publishing {self.eye} eye ({self.eye_width}x{self.height}) "
            f"from {self.device} @ {self.fps}fps — rough FOV-based intrinsics, "
            "not factory calibration."
        )

        self._spin()

    def _build_camera_info(self):
        hfov_rad = math.radians(self.hfov_deg)
        fx = (self.eye_width / 2.0) / math.tan(hfov_rad / 2.0)
        fy = fx
        cx = self.eye_width / 2.0
        cy = self.height / 2.0

        info = CameraInfo()
        info.width = self.eye_width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.header.frame_id = 'camera_color_optical_frame'
        return info

    def _spin(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            ok, frame = self.cap.read()
            if not ok:
                rospy.logwarn_throttle(5.0, "[zed_camera] Frame grab failed")
                rate.sleep()
                continue

            crop = frame[:, :self.eye_width] if self.eye == 'left' else frame[:, self.eye_width:]

            stamp = rospy.Time.now()
            img_msg = self.bridge.cv2_to_imgmsg(crop, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'camera_color_optical_frame'
            self.image_pub.publish(img_msg)

            self.camera_info_msg.header.stamp = stamp
            self.info_pub.publish(self.camera_info_msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        ZedCameraNode()
    except rospy.ROSInterruptException:
        pass
    except RuntimeError as e:
        rospy.logfatal(str(e))
