# Official ROS Noetic image (Ubuntu 20.04, Python 3.8)
FROM osrf/ros:noetic-desktop-full

# Useful tools + GUI/OpenGL support + ROS control/Gazebo + perception deps
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    nano \
    git \
    tmux \
    wget \
    curl \
    iputils-ping \
    net-tools \
    psmisc \
    # OpenGL — libglvnd vendor-neutral dispatch routes GL to NVIDIA or Mesa at runtime
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    libglvnd-dev \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    # ROS GUI tools
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rviz \
    # ROS controllers
    ros-noetic-position-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-joint-state-controller \
    # Gazebo ROS integration
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-robot-state-publisher \
    ros-noetic-controller-manager \
    ros-noetic-realtime-tools \
    ros-noetic-hardware-interface \
    ros-noetic-controller-interface \
    # Dependencies for sdk_z1 and z1_controller
    libboost-all-dev \
    libeigen3-dev \
    # Perception (MediaPipe hand tracking) + camera bridge dependencies
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    # Kept for the ZED-2 (UVC) and optional D435 real-camera variants
    ros-noetic-realsense2-camera \
    ros-noetic-ddynamic-reconfigure \
    libuvc-dev \
    # MediaPipe on Python 3.8 (Noetic): 0.10.9 is the last 3.8-compatible release.
    # It pins protobuf<4 (compatible with ROS Noetic) and pulls its own opencv
    # wheel; cv_bridge keeps using the system libopencv. The smoke test below
    # verifies cv2 + mediapipe + cv_bridge import together before the build proceeds.
    && pip3 install --no-cache-dir mediapipe==0.10.9 'protobuf<4' \
    && rm -rf /var/lib/apt/lists/*

# Fail the build loudly if the MediaPipe / OpenCV / cv_bridge trio does not
# coexist (the single highest-risk integration point — see PLAN.md §8/§14).
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    python3 -c 'import cv2, mediapipe, numpy; from cv_bridge import CvBridge; \
print(\"import smoke test OK: cv2\", cv2.__version__, \"| mediapipe\", mediapipe.__version__)'"

# Initialise rosdep
RUN rosdep update

# Non-root user
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG video $USERNAME

# Copy and build sdk_z1 (provides the IK arm model used by arm_tracker_node)
COPY --chown=$USERNAME:$USERNAME sdk_z1 /home/$USERNAME/sdk_z1
RUN cd /home/$USERNAME/sdk_z1 && \
    rm -rf build && mkdir build && cd build && \
    cmake .. && make -j$(nproc)

# Copy and build z1_controller (the Gazebo sim_ctrl bridge)
COPY --chown=$USERNAME:$USERNAME z1_controller /home/$USERNAME/z1_controller
RUN cd /home/$USERNAME/z1_controller && \
    rm -rf build && mkdir build && cd build && \
    cmake .. && make -j$(nproc)

# Prepare the catkin workspace and copy the ROS packages
RUN mkdir -p /home/$USERNAME/catkin_ws/src
COPY --chown=$USERNAME:$USERNAME unitree_ros /home/$USERNAME/catkin_ws/src/unitree_ros
COPY --chown=$USERNAME:$USERNAME z1_controller/sim /home/$USERNAME/catkin_ws/src/z1_controller
# The upstream sim/CMakeLists.txt (commit 639bb77) needs three fixes to build
# here as a standalone catkin package (PLAN.md §12b):
#   0. Prepend cmake_minimum_required + project() — upstream's sim/CMakeLists.txt
#      lacks them (the source repo pinned a local-only commit that added them).
#   1. Normalise CRLF — the submodule file ships with Windows line endings, which
#      would break the $-anchored sed matches below (line ends in "sim\r").
#   2. Point the include dir at the full z1_controller build, not the "sim" subdir.
#   3. Add the matching lib dir after GAZEBO_LIBRARY_DIRS.
RUN cd /home/$USERNAME/catkin_ws/src/z1_controller && \
    ( grep -q 'project(z1_controller)' CMakeLists.txt || \
      sed -i '1i cmake_minimum_required(VERSION 3.0.2)\nproject(z1_controller)\n' CMakeLists.txt ) && \
    sed -i 's/\r$//' CMakeLists.txt && \
    sed -i 's|^  sim$|  $ENV{HOME}/z1_controller/include|' CMakeLists.txt && \
    sed -i '/${GAZEBO_LIBRARY_DIRS}/a\  $ENV{HOME}/z1_controller/lib' CMakeLists.txt
COPY --chown=$USERNAME:$USERNAME z1_hand_detector /home/$USERNAME/catkin_ws/src/z1_hand_detector
COPY --chown=$USERNAME:$USERNAME z1_arm_tracker /home/$USERNAME/catkin_ws/src/z1_arm_tracker
COPY --chown=$USERNAME:$USERNAME z1_teleop /home/$USERNAME/catkin_ws/src/z1_teleop

# unitree_legged_msgs ships inside unitree_ros/unitree_ros_to_real — no separate clone needed
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/catkin_ws

# Switch to the non-root user
USER $USERNAME
ENV HOME=/home/$USERNAME

# Build the catkin workspace (unitree_ros + unitree_legged_msgs + the three packages)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd $HOME/catkin_ws && \
    catkin_make"

# Shell setup: auto-source ROS + workshop aliases (PLAN.md §7.6)
RUN echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc && \
    echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc && \
    echo 'export PS1="\[\033[01;36m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Hand-teleop station (Gazebo arm + MediaPipe hand follower) ---' >> $HOME/.bashrc && \
    echo 'alias z1_teleop="roslaunch z1_teleop z1_teleop_sim.launch"' >> $HOME/.bashrc && \
    echo 'alias z1_teleop_headless="roslaunch z1_teleop z1_teleop_sim.launch headless:=true paused:=false"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Hand-teleop with a physical ZED 2 (UVC, no ZED SDK) ---' >> $HOME/.bashrc && \
    echo 'alias z1_teleop_zed="roslaunch z1_teleop z1_teleop_zed.launch"' >> $HOME/.bashrc && \
    echo 'alias z1_teleop_zed_headless="roslaunch z1_teleop z1_teleop_zed.launch headless:=true"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Unpause Gazebo physics ---' >> $HOME/.bashrc && \
    echo 'alias z1_unpause="rosservice call /gazebo/unpause_physics"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Individual nodes (restart without full relaunch) ---' >> $HOME/.bashrc && \
    echo 'alias z1_hand="rosrun z1_hand_detector hand_detector_node.py"' >> $HOME/.bashrc && \
    echo 'alias z1_tracker="rosrun z1_arm_tracker arm_tracker_node.py"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Visualisation ---' >> $HOME/.bashrc && \
    echo 'alias z1_rviz="rviz -d ~/catkin_ws/src/z1_teleop/rviz/z1_teleop.rviz"' >> $HOME/.bashrc && \
    echo 'alias z1_cam="rosrun image_view image_view image:=/hand/debug_image"' >> $HOME/.bashrc && \
    echo 'alias z1_cam_raw="rosrun image_view image_view image:=/camera/color/image_raw"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Diagnostics ---' >> $HOME/.bashrc && \
    echo 'alias z1_nodes="rosnode list"' >> $HOME/.bashrc && \
    echo 'alias z1_topics="rostopic list"' >> $HOME/.bashrc && \
    echo 'alias z1_target="rostopic echo /hand/target_pose"' >> $HOME/.bashrc && \
    echo 'alias z1_active="rostopic echo /hand/tracking_active"' >> $HOME/.bashrc && \
    echo 'alias z1_gripper="rostopic echo /hand/gripper_cmd"' >> $HOME/.bashrc && \
    echo 'alias z1_joints="rostopic echo /z1_gazebo/joint_states"' >> $HOME/.bashrc

# GUI / GPU environment
ENV LD_LIBRARY_PATH=/home/rosuser/sdk_z1/lib
ENV QT_X11_NO_MITSHM=1
# Tell the NVIDIA container runtime to expose the GPU and its GL libraries
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility
# Force the Mesa software renderer (llvmpipe).
# The Intel GPU (Meteor Lake / 0x7d67) is too new for Mesa 21 in Ubuntu 20.04,
# and NVIDIA GLX rejects indirect X11 connections from Docker.
# llvmpipe provides stable OpenGL 3.1 for Gazebo and RViz.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3

WORKDIR $HOME

# Launch the hand-teleop station on container start
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source $HOME/catkin_ws/devel/setup.bash && roslaunch z1_teleop z1_teleop_sim.launch"]
