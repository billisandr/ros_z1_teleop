cmake_minimum_required(VERSION 3.0.2)
project(z1_teleop)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch worlds rviz xacro config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
