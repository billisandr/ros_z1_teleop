# Third-Party Notices

This project is licensed under the MIT License (see `LICENSE`). It builds on
third-party code and assets, listed below with their own licenses.

## Bundled as git submodules

These are vendored via `.gitmodules` and copied into the Docker image at
build time. Each keeps its own license file at its submodule root.

| Submodule | Upstream | License | Copyright |
| --- | --- | --- | --- |
| `sdk_z1` | https://github.com/unitreerobotics/z1_sdk | BSD 3-Clause | (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") |
| `z1_controller` | https://github.com/unitreerobotics/z1_controller | BSD 3-Clause | (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") |
| `unitree_ros` | https://github.com/unitreerobotics/unitree_ros | BSD 3-Clause | (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") |

None of these submodules were modified beyond what the build process patches
at image-build time (see the Dockerfile for the `z1_controller` CMakeLists
fix that lets `sim_ctrl` build against the copied SDK headers).

## Installed at build time (not bundled as source)

Pulled from upstream package repositories into the Docker image. Not
redistributed as source in this repo; listed here for attribution.

| Package | License |
| --- | --- |
| ROS Noetic (`osrf/ros:noetic-desktop-full` base image) | BSD |
| Gazebo, `gazebo_ros_pkgs` | Apache 2.0 |
| MediaPipe (`mediapipe==0.10.9`) | Apache 2.0 |
| OpenCV, `python3-opencv` | Apache 2.0 |
| `cv_bridge`, `realsense2_camera`, other ROS packages installed via apt | BSD |
| Streamlit (`streamlit==1.24.0`) | Apache 2.0 |

## Adapted code

The landmark-topology constants, the `OneEuroFilter` smoothing class, and the
6-DOF arm-angle decomposition in
[z1_hand_detector/src/hand_detector_node.py](z1_hand_detector/src/hand_detector_node.py)
were adapted from an earlier standalone MediaPipe prototype of ours, not a
third-party library. The One Euro Filter algorithm itself originates from
Casiez, Roussel, and Vogel, "1€ Filter: A Simple Speed-based Low-pass Filter
for Noisy Input in Interactive Systems" (CHI 2012); our implementation is an
independent reimplementation of the published algorithm, not a copy of any
particular reference implementation.

## Assets

`assets/TUC_logo.png` and `assets/senselab_logo.png` are institutional
branding for the Technical University of Crete and SenseLAB. They are
included here for workshop-branding purposes; they are not covered by this
repository's MIT license and should not be reused outside that context
without separate permission.
