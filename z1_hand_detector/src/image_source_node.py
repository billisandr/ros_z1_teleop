#!/usr/bin/env python3
"""
Image Source Node — feeds /camera/color/image_raw for the hand detector.

There is no animated hand inside Gazebo, so the primary perception path is always
a real image stream. This tiny helper publishes one of:

    hand/image_source: webcam            -> open a local V4L2 device (hand/webcam_device)
    hand/image_source: video:/path.mp4   -> loop a video file (zero camera hardware)
    hand/image_source: external          -> idle (another node, e.g. the ZED bridge
                                            or RealSense driver, already publishes)

It deliberately mirrors zed_camera_node's seam: every downstream consumer only
knows the /camera/color/image_raw topic, regardless of where the frames come from.
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageSourceNode:
    def __init__(self):
        rospy.init_node('image_source', anonymous=False)

        self.source = str(rospy.get_param('hand/image_source', 'webcam'))
        self.fps = float(rospy.get_param('hand/source_fps', 30.0))
        self.webcam_device = rospy.get_param('hand/webcam_device', 0)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)

        if self.source == 'external':
            rospy.loginfo("[image_source] image_source='external' — another node "
                          "publishes /camera/color/image_raw; idling.")
            rospy.spin()
            return

        self.is_video = self.source.startswith('video:')
        if self.is_video:
            self.path = self.source.split('video:', 1)[1]
            target = self.path
        else:  # webcam
            target = self.webcam_device

        self.cap = cv2.VideoCapture(target)
        if not self.cap.isOpened():
            rospy.logfatal("[image_source] Could not open source '%s'. For a local "
                           "webcam inside Docker on Windows, see PLAN.md §9 / the "
                           "README camera-sourcing section." % self.source)
            raise RuntimeError("cannot open image source: %s" % self.source)

        rospy.loginfo("[image_source] Publishing /camera/color/image_raw from %s @ %.0f fps%s"
                      % (self.source, self.fps, " (looping)" if self.is_video else ""))
        self._spin()

    def _spin(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            ok, frame = self.cap.read()
            if not ok:
                if self.is_video:
                    # loop the file
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                rospy.logwarn_throttle(5.0, "[image_source] Frame grab failed")
                rate.sleep()
                continue

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'camera_color_optical_frame'
            self.pub.publish(msg)
            rate.sleep()

        self.cap.release()


if __name__ == '__main__':
    try:
        ImageSourceNode()
    except rospy.ROSInterruptException:
        pass
    except RuntimeError as e:
        rospy.logfatal(str(e))
