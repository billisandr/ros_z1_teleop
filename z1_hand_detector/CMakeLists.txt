cmake_minimum_required(VERSION 3.0)
project(z1_hand_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/hand_detector_node.py
  src/image_source_node.py
  src/zed_camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
