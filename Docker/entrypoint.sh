#!/bin/bash
# Docker entrypoint script to handle user ID mapping for mounted volumes

# If USER_ID and GROUP_ID are provided, create/modify the ros user to match
if [ -n "$USER_ID" ] && [ -n "$GROUP_ID" ]; then
    # Create group with matching GID if it doesn't exist
    if ! getent group "$GROUP_ID" > /dev/null 2>&1; then
        groupadd -g "$GROUP_ID" hostgroup
    fi
    
    # Create user with matching UID/GID if it doesn't exist
    if ! getent passwd "$USER_ID" > /dev/null 2>&1; then
        useradd -u "$USER_ID" -g "$GROUP_ID" -m -s /bin/bash hostuser
        # Copy ros user's environment and permissions
        usermod -aG sudo hostuser
        echo "hostuser:ros" | chpasswd
        # Set up ROS environment for the new user
        echo "source /opt/ros/jazzy/setup.bash" >> /home/hostuser/.bashrc
        echo "if [ -f /home/ros/sigyn_ws/install/setup.bash ]; then" >> /home/hostuser/.bashrc
        echo "    source /home/ros/sigyn_ws/install/setup.bash" >> /home/hostuser/.bashrc
        echo "fi" >> /home/hostuser/.bashrc
        # Set working directory to workspace
        echo "cd /home/ros/sigyn_ws" >> /home/hostuser/.bashrc
        export HOME=/home/hostuser
        cd /home/hostuser
    fi
    
    # Fix ownership of workspace if it exists and is mounted
    if [ -d "/home/ros/sigyn_ws" ] && [ "$(stat -c %u /home/ros/sigyn_ws)" != "$USER_ID" ]; then
        echo "Fixing workspace ownership for user mapping..."
        chown -R "$USER_ID:$GROUP_ID" /home/ros/sigyn_ws
    fi
    
    # Switch to the host-matching user
    exec gosu "$USER_ID:$GROUP_ID" "$@"
else
    # Run as normal ros user, but check if we need to adjust ros user UID for mounted volumes
    if [ -d "/home/ros/sigyn_ws" ] && [ "$(stat -c %u /home/ros/sigyn_ws)" = "1000" ] && [ "$(id -u ros)" = "1001" ]; then
        echo "Detected mounted workspace with UID 1000, adjusting ros user UID to match..."
        # Stop any processes that might be using the ros user
        pkill -u ros || true
        # Change ros user UID from 1001 to 1000 to match host
        usermod -u 1000 ros
        # Fix home directory ownership
        chown -R ros:ros /home/ros
        # Fix workspace ownership if needed
        chown -R ros:ros /home/ros/sigyn_ws
        echo "ros user UID adjusted to 1000 to match host system"
    elif [ -d "/home/ros/sigyn_ws" ] && [ "$(whoami)" = "ros" ] && [ "$(stat -c %u /home/ros/sigyn_ws)" != "$(id -u ros)" ]; then
        echo "Fixing workspace ownership for ros user..."
        chown -R ros:ros /home/ros/sigyn_ws
    fi
    exec "$@"
fi
