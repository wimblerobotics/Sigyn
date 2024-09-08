##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image
###########################################
FROM osrf/ros:jazzy-desktop-full-noble AS base

ENV DEBIAN_FRONTEND=noninteractive

###########################################
#  Develop image
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  nano \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  vim \
  xterm \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

######
# Install matplotlib
######
FROM ubuntu:20.10
RUN apt-get update && apt-get install -y python3 python3-pip
RUN pip3 install matplotlib

COPY matplotlibcpp.h /usr/local/include

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/* 

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#  Full image
###########################################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-laser-geometry \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-nav2-* \
  ros-jazzy-pcl-ros \
  nlohmann-json3-dev \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

###########################################
#  Full+Gazebo image
###########################################
FROM full AS gazebo

ENV DEBIAN_FRONTEND=noninteractive
# Install gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-jazzy-gazebo* \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

RUN apt-get update -y || true && \
DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata libx11-dev && \
apt-get install -y --no-install-recommends libopencv-dev && \
     rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean

COPY . /home/ros

RUN cat <<EOT >> /home/ros/.bashrc
alias cb='colcon build --symlink-install'
alias fr='ros2 run tf2_tools view_frames'
alias nav='clear;ros2 launch base navigation.launch.py'
alias pm='ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'
alias rd='rosdep install --from-paths src --ignore-src -r -y'
alias rvs='rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz'
alias save_map='ros2 run nav2_map_server map_saver_cli -f ~/map_`date +"%Y_%m_%d_%H%M"`'
alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
source /opt/ros/jazzy/setup.bash
EOT

ENV HOME=/home/ros
ENV SHELL /bin/bash
#CMD source /home/ros/.bashrc
# Force Python stdout and stderr streams to be unbuffered.
ENV PYTHONUNBUFFERED 1
# USER $USERNAME
# CMD ["/bin/bash"]