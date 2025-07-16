FROM rosjazzybase


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    && rm -rf /var/lib/apt/lists/*


# Example of copying a file
# COPY config/ /site_config/


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# RUN groupadd --gid $USER_GID $USERNAME \
#   && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#   && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# # Set up sudo
# RUN apt-get update \
#   && apt-get install -y sudo \
#   && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
#   && chmod 0440 /etc/sudoers.d/$USERNAME \
#   && rm -rf /var/lib/apt/lists/*


# # Copy the entrypoint and bashrc scripts so we have 
# # our container's environment set up correctly
# COPY entrypoint.sh /entrypoint.sh
# COPY bashrc /home/${USERNAME}/.bashrc


# # Set up entrypoint and default command
# ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
# CMD ["bash"]

# Create a non-root user
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/ros\
  && chmod 0440 /etc/sudoers.d/ros \
  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
&& echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
&& echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
&& rm -rf /var/lib/apt/lists/* 

USER ros
RUN cat <<EOT >> /home/ros/.bashrc
alias cb='colcon build --symlink-install'
alias fr='ros2 run tf2_tools view_frames'
alias pm='ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'
alias rd='rosdep install --from-paths src --ignore-src -r -y'
alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
source /opt/ros/jazzy/setup.bash
source ~/sigyn_ws/install/setup.bash
EOT

# RUN chown -R ros:ros /home/ros
ENV PYTHONUNBUFFERED=1
# ENV HOME=/home/ros
# ENV SHELL /bin/bash
# CMD source /home/ros/.bashrc
# Force Python stdout and stderr streams to be unbuffered.
USER ros
WORKDIR /home/ros