# Dockerfile for Sigyn Robot Development Environment
# 
# This Dockerfile creates a comprehensive development environment for the Sigyn house patroller robot.
# It includes ROS2 Jazzy, Gazebo Harmonic, Arduino/Teensyduino, Visual Studio Code, and all necessary
# dependencies for robot development, simulation, and Teensy programming.
#
# Author: Sigyn Development Team
# Version: 2.0
# Updated: 2025-01-27

FROM osrf/ros:jazzy-desktop-full

# Dockerfile metadata
LABEL maintainer="sigyn-dev@example.com"
LABEL description="Sigyn Robot Development Environment with ROS2 Jazzy, Gazebo, Arduino IDE, and VSCode"
LABEL version="2.0"

# Build arguments for user configuration
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV PYTHONUNBUFFERED=1

# GUI and graphics environment variables for X11 forwarding
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV LIBGL_ALWAYS_INDIRECT=0
ENV LIBGL_ALWAYS_SOFTWARE=0

# ROS2 DDS configuration for better network performance
ENV RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
ENV CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='*' /></Interfaces></General></Domain></CycloneDDS>"

# Arduino IDE configuration
ENV ARDUINO_IDE_VERSION=2.3.2
ENV ARDUINO_CLI_VERSION=0.35.3
ENV TEENSYDUINO_VERSION=1.59

# ==============================================================================
# SYSTEM SETUP AND SECURITY
# ==============================================================================

# Remove snap packages to prevent conflicts
RUN if dpkg -l | grep -q snapd; then \
        apt-get update && \
        apt-get purge -y snapd && \
        apt-get autoremove -y && \
        rm -rf /var/lib/snapd /snap /var/snap; \
    fi && \
    echo 'Package: snapd' > /etc/apt/preferences.d/nosnap.pref && \
    echo 'Pin: release a=*' >> /etc/apt/preferences.d/nosnap.pref && \
    echo 'Pin-Priority: -10' >> /etc/apt/preferences.d/nosnap.pref && \
    rm -rf /var/lib/apt/lists/*

# Configure DNS resolution and package manager for Docker build environment
RUN echo 'nameserver 8.8.8.8' > /etc/resolv.conf && \
    echo 'nameserver 8.8.4.4' >> /etc/resolv.conf && \
    echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4 && \
    echo 'Acquire::Retries "3";' >> /etc/apt/apt.conf.d/99force-ipv4 && \
    echo 'Acquire::http::Timeout "30";' >> /etc/apt/apt.conf.d/99force-ipv4 && \
    echo 'Acquire::ftp::Timeout "30";' >> /etc/apt/apt.conf.d/99force-ipv4

# Update package lists and install essential system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Development tools
    build-essential \
    cmake \
    git \
    curl \
    sudo \
    pkg-config \
    # System libraries and hardware support
    libudev-dev \
    # Additional tools
    ca-certificates \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install packages that may not be available in base image
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    vim \
    nano \
    bash-completion \
    tree \
    htop \
    tmux \
    screen \
    gdb \
    valgrind \
    strace \
    udev \
    nlohmann-json3-dev \
    usbutils \
    pciutils \
    iputils-ping \
    net-tools \
    iproute2 \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    xvfb \
    mesa-utils \
    software-properties-common \
    apt-transport-https \
    python3-serial \
    minicom \
    picocom \
    || echo "Some packages may not be available - continuing..." \
    && rm -rf /var/lib/apt/lists/*

# Install graphics libraries separately to handle dependency issues
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libglu1-mesa \
    libglx-mesa0 \
    libegl-dev \
    libxext6 \
    libx11-6 \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libxi6 \
    libxtst6 \
    libxrender1 \
    libxfixes3 \
    libxdamage1 \
    libasound2t64 \
    libgl1 \
    libglvnd0 \
    libglx0 \
    libglvnd-dev \
    || echo "Some graphics packages may not be available - continuing..." \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# PYTHON DEVELOPMENT ENVIRONMENT
# ==============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools \
    python3-wheel \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for robot development
RUN pip3 install --no-cache-dir --break-system-packages \
    # DepthAI for OAK-D camera
    depthai \
    # Computer vision
    opencv-python \
    numpy \
    # Data science and visualization
    matplotlib \
    scipy \
    pandas \
    # Machine learning
    ultralytics \
    # Serial communication
    pyserial \
    # Arduino/PlatformIO support
    platformio \
    # Development tools
    black \
    flake8 \
    pytest

# ==============================================================================
# ROS2 PACKAGES AND NAVIGATION STACK
# ==============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core ROS2 development
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-index-python \
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-action \
    ros-jazzy-rclcpp-components \
    ros-jazzy-rclpy \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    # Message types
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-action-msgs \
    ros-jazzy-diagnostic-msgs \
    # Navigation stack
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-map-server \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-robot-localization \
    ros-jazzy-laser-filters \
    ros-jazzy-laser-geometry \
    ros-jazzy-pointcloud-to-laserscan \
    # Robot description and visualization
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-rendering \
    # Hardware interfaces
    ros-jazzy-controller-manager \
    ros-jazzy-hardware-interface \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    # Vision and perception
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-pcl-ros \
    # Behavior trees
    ros-jazzy-behaviortree-cpp \
    # TF2
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-geometry-msgs \
    # Launch system
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-launch-testing \
    # Teleop
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    # Testing and linting
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    # Additional utilities
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-twist-mux \
    # MicroROS for Teensy communication
    ros-jazzy-micro-ros-agent \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# GAZEBO HARMONIC SIMULATION
# ==============================================================================

# Install Gazebo Harmonic from official repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# OAK-D CAMERA SUPPORT
# ==============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-depthai \
    ros-jazzy-depthai-ros-driver \
    ros-jazzy-depthai-examples \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# USER SETUP
# ==============================================================================

# Create ros user with proper UID/GID
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} --shell /bin/bash --create-home ${USERNAME} && \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ==============================================================================
# ARDUINO IDE AND TEENSYDUINO SETUP
# ==============================================================================

# Switch to ros user for Arduino installation
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Create directories
RUN mkdir -p ~/Downloads ~/Arduino/libraries ~/Arduino/hardware

# Download and install Arduino IDE 2.0
RUN cd ~/Downloads && \
    wget https://downloads.arduino.cc/arduino-ide/arduino-ide_${ARDUINO_IDE_VERSION}_Linux_64bit.AppImage && \
    chmod +x arduino-ide_${ARDUINO_IDE_VERSION}_Linux_64bit.AppImage && \
    mkdir -p ~/.local/bin && \
    mv arduino-ide_${ARDUINO_IDE_VERSION}_Linux_64bit.AppImage ~/.local/bin/arduino-ide && \
    # Create desktop entry
    mkdir -p ~/.local/share/applications && \
    echo "[Desktop Entry]" > ~/.local/share/applications/arduino-ide.desktop && \
    echo "Version=1.0" >> ~/.local/share/applications/arduino-ide.desktop && \
    echo "Type=Application" >> ~/.local/share/applications/arduino-ide.desktop && \
    echo "Name=Arduino IDE" >> ~/.local/share/applications/arduino-ide.desktop && \
    echo "Exec=/home/${USERNAME}/.local/bin/arduino-ide %F" >> ~/.local/share/applications/arduino-ide.desktop && \
    echo "Icon=arduino-ide" >> ~/.local/share/applications/arduino-ide.desktop && \
    echo "Categories=Development;Electronics;" >> ~/.local/share/applications/arduino-ide.desktop

# Download and install Arduino CLI
RUN cd ~/Downloads && \
    wget https://downloads.arduino.cc/arduino-cli/arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz && \
    tar -xzf arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz && \
    mv arduino-cli ~/.local/bin/ && \
    chmod +x ~/.local/bin/arduino-cli

# Install Teensy support (Teensyduino)
RUN ~/.local/bin/arduino-cli core update-index && \
    ~/.local/bin/arduino-cli core install teensy:avr

# Download and install Teensy Loader CLI
RUN cd ~/Downloads && \
    git clone https://github.com/PaulStoffregen/teensy_loader_cli.git && \
    cd teensy_loader_cli && \
    make && \
    sudo make install

# Switch back to root for final system setup
USER root

# Add ros user to dialout group for serial access
RUN usermod -a -G dialout ${USERNAME}

# ==============================================================================
# VISUAL STUDIO CODE INSTALLATION
# ==============================================================================

# Install Visual Studio Code
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/ && \
    echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list && \
    apt-get update && \
    apt-get install -y code && \
    rm -rf /var/lib/apt/lists/*

# ==============================================================================
# BASH CONFIGURATION AND ALIASES
# ==============================================================================

# Switch back to ros user for environment setup
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Set up comprehensive .bashrc with ROS2 environment and useful aliases
RUN echo "# Sigyn Robot Development Environment" > ~/.bashrc && \
    echo "# Auto-generated bashrc for Docker container" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Standard Ubuntu bashrc settings" >> ~/.bashrc && \
    echo "case \$- in" >> ~/.bashrc && \
    echo "    *i*) ;;" >> ~/.bashrc && \
    echo "      *) return;;" >> ~/.bashrc && \
    echo "esac" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "HISTCONTROL=ignoreboth" >> ~/.bashrc && \
    echo "shopt -s histappend" >> ~/.bashrc && \
    echo "HISTSIZE=1000" >> ~/.bashrc && \
    echo "HISTFILESIZE=2000" >> ~/.bashrc && \
    echo "shopt -s checkwinsize" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Enable color support" >> ~/.bashrc && \
    echo "if [ -x /usr/bin/dircolors ]; then" >> ~/.bashrc && \
    echo "    test -r ~/.dircolors && eval \"\$(dircolors -b ~/.dircolors)\" || eval \"\$(dircolors -b)\"" >> ~/.bashrc && \
    echo "    alias ls='ls --color=auto'" >> ~/.bashrc && \
    echo "    alias grep='grep --color=auto'" >> ~/.bashrc && \
    echo "    alias fgrep='fgrep --color=auto'" >> ~/.bashrc && \
    echo "    alias egrep='egrep --color=auto'" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Standard aliases" >> ~/.bashrc && \
    echo "alias ll='ls -alF'" >> ~/.bashrc && \
    echo "alias la='ls -A'" >> ~/.bashrc && \
    echo "alias l='ls -CF'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# ROS2 Environment Setup" >> ~/.bashrc && \
    echo "source /opt/ros/\${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Workspace setup (dynamic - will source any workspace if available)" >> ~/.bashrc && \
    echo "# Check for workspace setup in current directory" >> ~/.bashrc && \
    echo "if [ -f ./install/setup.bash ]; then" >> ~/.bashrc && \
    echo "  source ./install/setup.bash" >> ~/.bashrc && \
    echo "elif [ -f ~/workspace/install/setup.bash ]; then" >> ~/.bashrc && \
    echo "  source ~/workspace/install/setup.bash" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# ROS2 DDS Configuration" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc && \
    echo "export CYCLONEDDS_URI=\"<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='*' /></Interfaces></General></Domain></CycloneDDS>\"" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Arduino and development tools" >> ~/.bashrc && \
    echo "export PATH=\"\$PATH:/home/${USERNAME}/.local/bin\"" >> ~/.bashrc && \
    echo "" >> ~/.bashrc

# Add comprehensive aliases for Sigyn development
RUN echo "# ==============================================================================" >> ~/.bashrc && \
    echo "# SIGYN ROBOT DEVELOPMENT ALIASES" >> ~/.bashrc && \
    echo "# ==============================================================================" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Colcon build aliases" >> ~/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias cbt='colcon build --symlink-install --packages-select'" >> ~/.bashrc && \
    echo "alias cbu='colcon build --symlink-install --packages-up-to'" >> ~/.bashrc && \
    echo "alias cbf='colcon build --symlink-install --continue-on-error'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# ROS2 utility aliases" >> ~/.bashrc && \
    echo "alias rd='rosdep install --from-paths src --ignore-src -r -y'" >> ~/.bashrc && \
    echo "alias fr='ros2 run tf2_tools view_frames'" >> ~/.bashrc && \
    echo "alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.25 -p turn:=0.3 -p repeat_rate:=10.0'" >> ~/.bashrc && \
    echo "alias stele='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.25 -p turn:=0.3 -p repeat_rate:=10.0 --remap cmd_vel:=/cmd_vel_keyboard'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Robot control aliases" >> ~/.bashrc && \
    echo "alias pm='ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}}\"'" >> ~/.bashrc && \
    echo "alias cgcm='ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap'" >> ~/.bashrc && \
    echo "alias clcm='ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Sigyn launch aliases" >> ~/.bashrc && \
    echo "alias sim='clear;ros2 launch base sigyn.launch.py use_sim_time:=true do_rviz:=true'" >> ~/.bashrc && \
    echo "alias nav='clear;ros2 launch base sigyn.launch.py use_sim_time:=false do_rviz:=true'" >> ~/.bashrc && \
    echo "alias map='clear;ros2 launch base sigyn.launch.py use_sim_time:=false do_rviz:=true make_map:=true'" >> ~/.bashrc && \
    echo "alias sensor='ros2 launch sigyn_to_sensor sigyn_to_sensor.launch.py'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# RViz and visualization (workspace-agnostic)" >> ~/.bashrc && \
    echo "alias rvs='if [ -f ./rviz/config/config.rviz ]; then rviz2 -d ./rviz/config/config.rviz; elif [ -f ./src/Sigyn/rviz/config/config.rviz ]; then rviz2 -d ./src/Sigyn/rviz/config/config.rviz; else rviz2; fi'" >> ~/.bashrc && \
    echo "alias rviz='ros2 run rviz2 rviz2'" >> ~/.bashrc && \
    echo "alias gazebo='gz sim --verbose'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Development tools" >> ~/.bashrc && \
    echo "alias code='code --no-sandbox'" >> ~/.bashrc && \
    echo "alias arduino='~/.local/bin/arduino-ide'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Teensy development aliases (workspace-agnostic)" >> ~/.bashrc && \
    echo "alias setup_venv='if [ -f ./oakd_detector/venv/bin/activate ]; then source ./oakd_detector/venv/bin/activate; else echo \"Virtual env not found in current directory\"; fi'" >> ~/.bashrc && \
    echo "alias compileBoard1='if [ -f platformio.ini ]; then pio run -e board1; else echo \"No platformio.ini found in current directory\"; fi'" >> ~/.bashrc && \
    echo "alias compileBoard2='if [ -f platformio.ini ]; then pio run -e board2; else echo \"No platformio.ini found in current directory\"; fi'" >> ~/.bashrc && \
    echo "alias buildBoard1='if [ -f platformio.ini ]; then pio run -e board1 && teensy_loader_cli --mcu=TEENSY41 -w -v .pio/build/board1/firmware.hex; else echo \"No platformio.ini found in current directory\"; fi'" >> ~/.bashrc && \
    echo "alias buildBoard2='if [ -f platformio.ini ]; then pio run -e board2 && teensy_loader_cli --mcu=TEENSY41 -w -v .pio/build/board2/firmware.hex; else echo \"No platformio.ini found in current directory\"; fi'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Hardware access aliases" >> ~/.bashrc && \
    echo "alias gripper='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_gripper'" >> ~/.bashrc && \
    echo "alias redoudev='sudo service udev restart;sudo udevadm control --reload-rules;sudo udevadm trigger'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Map saving" >> ~/.bashrc && \
    echo "alias rms='ros2 launch nav2_map_server map_saver_server.launch.py'" >> ~/.bashrc && \
    echo "alias savem='ros2 run nav2_map_server map_saver_cli -f my_map'" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Default working directory (container starts in home, user can cd to mounted workspace)" >> ~/.bashrc

# Create workspace directory
# Note: No workspace created in container - user will mount their own workspace

# ==============================================================================
# UDEV RULES AND HARDWARE ACCESS
# ==============================================================================

# Switch back to root for final system configuration
USER root

# Set up udev rules directory
RUN mkdir -p /etc/udev/rules.d

# Copy udev rules for Teensy and other hardware (if mounted)
# Note: These will be copied from the host workspace when the container is run
RUN echo "# Udev rules will be copied from host workspace during container startup" > /etc/udev/rules.d/README

# ==============================================================================
# ENTRYPOINT AND SCRIPTS SETUP
# ==============================================================================

# Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Handle user ID mapping if specified' >> /entrypoint.sh && \
    echo 'if [ -n "${USER_ID}" ] && [ -n "${GROUP_ID}" ]; then' >> /entrypoint.sh && \
    echo '    echo "Mapping container user to host UID:${USER_ID} GID:${GROUP_ID}"' >> /entrypoint.sh && \
    echo '    ' >> /entrypoint.sh && \
    echo '    # Update ros user UID/GID to match host' >> /entrypoint.sh && \
    echo '    groupmod -g ${GROUP_ID} ros 2>/dev/null || true' >> /entrypoint.sh && \
    echo '    usermod -u ${USER_ID} -g ${GROUP_ID} ros 2>/dev/null || true' >> /entrypoint.sh && \
    echo '    ' >> /entrypoint.sh && \
    echo '    # Fix ownership of home directory' >> /entrypoint.sh && \
    echo '    chown -R ${USER_ID}:${GROUP_ID} /home/ros 2>/dev/null || true' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Copy udev rules from workspace if available (check common mount locations)' >> /entrypoint.sh && \
    echo 'for ws_path in "/workspace" "/home/ros/workspace" "/home/ros/sigyn_ws" "$(pwd)"; do' >> /entrypoint.sh && \
    echo '  if [ -d "${ws_path}/src/Sigyn/udev" ]; then' >> /entrypoint.sh && \
    echo '    echo "Copying udev rules from ${ws_path}/src/Sigyn/udev..."' >> /entrypoint.sh && \
    echo '    cp ${ws_path}/src/Sigyn/udev/*.rules /etc/udev/rules.d/ 2>/dev/null || true' >> /entrypoint.sh && \
    echo '    service udev restart 2>/dev/null || true' >> /entrypoint.sh && \
    echo '    break' >> /entrypoint.sh && \
    echo '  fi' >> /entrypoint.sh && \
    echo 'done' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source ROS2 environment' >> /entrypoint.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source workspace if available (check common locations)' >> /entrypoint.sh && \
    echo 'for ws_path in "/workspace" "/home/ros/workspace" "/home/ros/sigyn_ws" "$(pwd)"; do' >> /entrypoint.sh && \
    echo '  if [ -f "${ws_path}/install/setup.bash" ]; then' >> /entrypoint.sh && \
    echo '    echo "Sourcing workspace from ${ws_path}/install/setup.bash"' >> /entrypoint.sh && \
    echo '    source ${ws_path}/install/setup.bash' >> /entrypoint.sh && \
    echo '    break' >> /entrypoint.sh && \
    echo '  fi' >> /entrypoint.sh && \
    echo 'done' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# If no command specified, start interactive bash as ros user' >> /entrypoint.sh && \
    echo 'if [ $# -eq 0 ]; then' >> /entrypoint.sh && \
    echo '    exec su - ros' >> /entrypoint.sh && \
    echo 'else' >> /entrypoint.sh && \
    echo '    # Execute specified command as ros user' >> /entrypoint.sh && \
    echo '    exec su - ros -c "$*"' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh

RUN chmod +x /entrypoint.sh

# ==============================================================================
# FINAL SETUP
# ==============================================================================

# Switch back to ros user for final setup
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Set proper permissions
USER root
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Final switch to ros user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Set default entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

# ==============================================================================
# HEALTH CHECK
# ==============================================================================

HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD ros2 topic list > /dev/null || exit 1

# ==============================================================================
# CONTAINER METADATA
# ==============================================================================

# Add build date and version info
ARG BUILD_DATE
ARG VCS_REF
LABEL org.label-schema.build-date=${BUILD_DATE}
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.schema-version="1.0"
