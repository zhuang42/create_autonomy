FROM create_ubuntu_bionic
LABEL maintainer="Emiliano Borghi"

ENV ROS1_DISTRO=melodic
ENV USER=create

USER root

# Setup sources.list for ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list

# Setup keys for ROS
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install ROS packages
RUN apt-get update && \
    apt-get install -y \
    ros-$ROS1_DISTRO-xacro \
    ros-$ROS1_DISTRO-desktop && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Initialize rosdep
RUN rosdep init

# Install camera dependencies
RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev

RUN apt-get update && \
    apt-get install -y \
    ros-$ROS1_DISTRO-amcl \
    ros-$ROS1_DISTRO-camera-calibration \
    ros-$ROS1_DISTRO-controller-manager \
    ros-$ROS1_DISTRO-ecl-threads \
    ros-$ROS1_DISTRO-hardware-interface \
    ros-$ROS1_DISTRO-joint-limits-interface \
    ros-$ROS1_DISTRO-move-base \
    ros-$ROS1_DISTRO-pcl-ros \
    ros-$ROS1_DISTRO-robot-localization \
    ros-$ROS1_DISTRO-transmission-interface \
    ros-$ROS1_DISTRO-yocs-controllers \
    && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

RUN pip install --upgrade pip

USER $USER
RUN echo ". /opt/ros/$ROS1_DISTRO/setup.bash" >> /home/$USER/.bashrc
USER root

# Workspace
ENV WS_DIR=/create_ws
RUN mkdir -p $WS_DIR/src && \
    chown -R $USER $WS_DIR
USER $USER
WORKDIR $WS_DIR
RUN rosdep update

CMD [ "tmux" ]
