FROM osrf/ros:jazzy-desktop-full
ARG USERNAME=USERNAME
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-laser-geometry \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-nav2-* \
  ros-jazzy-pcl-ros \
  nlohmann-json3-dev \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
RUN apt-get update && apt-get install -y git-core bash-completion \
&& echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
&& echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
&& rm -rf /var/lib/apt/lists/* 

RUN cat <<EOT >> /home/ros/.bashrc
alias cb='colcon build --symlink-install'
alias cgcm='ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap'
alias clcm='ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap'
alias fr='ros2 run tf2_tools view_frames'
alias pm='ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'
alias rd='rosdep install --from-paths src --ignore-src -r -y'
alias record='rm -r ~/bag;~/sigyn_ws/src/Sigyn/base/scripts/bag_record.sh'
alias rvs='rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz'
alias sim='clear;ros2 launch base sigyn.launch.py use_sim_time:=true do_rviz:=true'
alias sr='ssh -YC sigyn7900'
alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias teles='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel_unstamped'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
EOT

ENV PYTHONUNBUFFERED 1

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]