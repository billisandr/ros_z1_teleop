#!/usr/bin/env python3
"""
Hand Detector Node — STUB (Phase 0 scaffold placeholder).

Replaced in Phase 2 with the full MediaPipe Hands implementation that
subscribes to /camera/color/image_raw and publishes:
    /hand/target_pose      (geometry_msgs/PoseStamped, world frame)
    /hand/tracking_active  (std_msgs/Bool)  — clutch + detection gate
    /hand/debug_image      (sensor_msgs/Image)
    /hand/gripper_cmd      (std_msgs/Float64, optional pinch -> gripper)

See PLAN.md §7.1 for the full spec.
"""

import rospy


def main():
    rospy.init_node('hand_detector', anonymous=False)
    rospy.logwarn('[hand_detector] STUB node — full MediaPipe implementation '
                  'lands in Phase 2. Doing nothing.')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
