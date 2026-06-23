# Utilisation de l'image officielle ROS Noetic
FROM osrf/ros:noetic-desktop-full

# Installation de quelques outils utiles + support GUI
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
    # Outils ROS GUI
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
    # ArUco detection dependencies
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-realsense2-camera \
    ros-noetic-ddynamic-reconfigure \
    libuvc-dev \
    && pip3 install opencv-contrib-python-headless \
    && pip3 install streamlit \
    && pip3 install jupyterlab \
    && rm -rf /var/lib/apt/lists/*

# Initialisation de rosdep
RUN rosdep update

# Création d'un utilisateur non-root
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG video $USERNAME

# Copy and build sdk_z1
COPY --chown=$USERNAME:$USERNAME sdk_z1 /home/$USERNAME/sdk_z1
RUN cd /home/$USERNAME/sdk_z1 && \
    rm -rf build && mkdir build && cd build && \
    cmake .. && make -j$(nproc)

# Copy and build z1_controller
COPY --chown=$USERNAME:$USERNAME z1_controller /home/$USERNAME/z1_controller
RUN cd /home/$USERNAME/z1_controller && \
    rm -rf build && mkdir build && cd build && \
    cmake .. && make -j$(nproc)

# Préparer le workspace catkin et copier unitree_ros (encore en root pour COPY --chown)
RUN mkdir -p /home/$USERNAME/catkin_ws/src
COPY --chown=$USERNAME:$USERNAME unitree_ros /home/$USERNAME/catkin_ws/src/unitree_ros
COPY --chown=$USERNAME:$USERNAME z1_controller/sim /home/$USERNAME/catkin_ws/src/z1_controller
# Upstream sim/CMakeLists.txt assumes z1_controller's headers/libs live under
# a "sim" subdir of itself; here they're the separate full copy built above,
# so point include/link dirs at $HOME/z1_controller instead.
# (Normalize CRLF first — the submodule file has Windows line endings, which
# breaks a $-anchored sed match since the line ends in "sim\r", not "sim".)
RUN sed -i 's/\r$//' /home/$USERNAME/catkin_ws/src/z1_controller/CMakeLists.txt && \
    sed -i 's|^  sim$|  $ENV{HOME}/z1_controller/include|' /home/$USERNAME/catkin_ws/src/z1_controller/CMakeLists.txt && \
    sed -i '/${GAZEBO_LIBRARY_DIRS}/a\  $ENV{HOME}/z1_controller/lib' /home/$USERNAME/catkin_ws/src/z1_controller/CMakeLists.txt
COPY --chown=$USERNAME:$USERNAME z1_aruco_detector /home/$USERNAME/catkin_ws/src/z1_aruco_detector
COPY --chown=$USERNAME:$USERNAME z1_arm_tracker /home/$USERNAME/catkin_ws/src/z1_arm_tracker
COPY --chown=$USERNAME:$USERNAME z1_aruco /home/$USERNAME/catkin_ws/src/z1_aruco

# unitree_legged_msgs is already included in unitree_ros/unitree_ros_to_real — no separate clone needed
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/catkin_ws

# Passer à l'utilisateur non-root
USER $USERNAME
ENV HOME=/home/$USERNAME

# Construire le workspace catkin (inclut unitree_ros + unitree_legged_msgs)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd $HOME/catkin_ws && \
    catkin_make"

# Configuration du shell pour charger ROS automatiquement
RUN echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc && \
    echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc && \
    echo 'export PS1="\[\033[01;36m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- ArUco tracking simulation (full Gazebo + simulated camera + marker) ---' >> $HOME/.bashrc && \
    echo 'alias z1_sim="roslaunch z1_aruco z1_aruco_tracking.launch"' >> $HOME/.bashrc && \
    echo 'alias z1_sim_headless="roslaunch z1_aruco z1_aruco_tracking.launch headless:=true paused:=false"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Real camera tracking (Gazebo arm + physical D435) ---' >> $HOME/.bashrc && \
    echo 'alias z1_real="roslaunch z1_aruco z1_real_camera_tracking.launch"' >> $HOME/.bashrc && \
    echo 'alias z1_real_headless="roslaunch z1_aruco z1_real_camera_tracking.launch headless:=true"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Real camera tracking (Gazebo arm + physical ZED 2, no ZED SDK) ---' >> $HOME/.bashrc && \
    echo 'alias z1_real_zed="roslaunch z1_aruco z1_real_camera_tracking_zed.launch"' >> $HOME/.bashrc && \
    echo 'alias z1_real_zed_headless="roslaunch z1_aruco z1_real_camera_tracking_zed.launch headless:=true"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Unpause Gazebo physics ---' >> $HOME/.bashrc && \
    echo 'alias z1_unpause="rosservice call /gazebo/unpause_physics"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Individual nodes (restart without full relaunch) ---' >> $HOME/.bashrc && \
    echo 'alias z1_detector="rosrun z1_aruco_detector aruco_detector_node.py"' >> $HOME/.bashrc && \
    echo 'alias z1_tracker="rosrun z1_arm_tracker arm_tracker_node.py"' >> $HOME/.bashrc && \
    echo 'alias z1_mover="rosrun z1_aruco_detector marker_mover_node.py"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Workshop live-knob UI (browser, http://localhost:8501) ---' >> $HOME/.bashrc && \
    echo 'alias z1_ui="streamlit run ~/catkin_ws/src/z1_aruco/scripts/workshop_ui.py --server.headless true --server.port 8501 --server.address 0.0.0.0"' >> $HOME/.bashrc && \
    echo '# --- Workshop companion notebook (browser, http://localhost:8888) ---' >> $HOME/.bashrc && \
    echo 'alias z1_nb="jupyter lab ~/catkin_ws/src/z1_aruco/notebooks/Z1_Workshop_Companion.ipynb --ip=0.0.0.0 --port=8888 --no-browser --IdentityProvider.token= --allow-root"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Visualisation ---' >> $HOME/.bashrc && \
    echo 'alias z1_rviz="rviz -d ~/catkin_ws/src/z1_aruco/rviz/z1_aruco_tracking.rviz"' >> $HOME/.bashrc && \
    echo 'alias z1_camera="rosrun image_view image_view image:=/aruco/debug_image"' >> $HOME/.bashrc && \
    echo 'alias z1_camera_raw="rosrun image_view image_view image:=/camera/color/image_raw"' >> $HOME/.bashrc && \
    echo '' >> $HOME/.bashrc && \
    echo '# --- Diagnostics ---' >> $HOME/.bashrc && \
    echo 'alias z1_nodes="rosnode list"' >> $HOME/.bashrc && \
    echo 'alias z1_topics="rostopic list"' >> $HOME/.bashrc && \
    echo 'alias z1_pose="rostopic echo /aruco/marker_pose"' >> $HOME/.bashrc && \
    echo 'alias z1_detected="rostopic echo /aruco/marker_detected"' >> $HOME/.bashrc && \
    echo 'alias z1_joints="rostopic echo /z1_gazebo/joint_states"' >> $HOME/.bashrc

# GUI / GPU environment
ENV LD_LIBRARY_PATH=/home/rosuser/sdk_z1/lib
ENV QT_X11_NO_MITSHM=1
# Tell NVIDIA container runtime to expose the GPU and its GL libraries
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility
# Force Mesa software renderer (llvmpipe).
# The Intel GPU (Meteor Lake / 0x7d67) is too new for Mesa 21 in Ubuntu 20.04,
# and NVIDIA GLX rejects indirect X11 connections from Docker.
# llvmpipe provides stable OpenGL 3.1 for Gazebo and RViz.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3

WORKDIR $HOME

# Launch ArUco tracking simulation on container start
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source $HOME/catkin_ws/devel/setup.bash && roslaunch z1_aruco z1_aruco_tracking.launch"]
