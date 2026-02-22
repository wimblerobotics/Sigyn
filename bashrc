# =============================================================================
# SIGYN ROBOT FLEET - Bash Configuration Reference
# =============================================================================
# This file documents the Sigyn Robot Fleet bash configuration approach.
#
# ARCHITECTURE (Ubuntu Best Practice):
#   - ~/.bash_aliases: Fully managed by SIGYN (deployed from config/bash_aliases_managed)
#   - ~/.bashrc: Pristine Ubuntu default + private user settings (NOT managed by SIGYN)
#
# DEPLOYMENT:
#   Local:  python3 setup_robot.py --update-all
#   Remote: scripts/update_bash_aliases.sh [--machine HOSTNAME]
#
# WHAT GETS DEPLOYED:
#   1. ~/.bash_aliases is completely replaced with managed template
#   2. ~/.bashrc is cleaned (removes old SIGYN sections and duplicates)
#   3. Private settings in ~/.bashrc are preserved (e.g., ROBOFLOW_API_KEY)
#
# PRIVATE SETTINGS:
#   Keep in ~/.bashrc (outside managed sections):
#     export ROBOFLOW_API_KEY="your_key_here"
#     export OTHER_PRIVATE_VAR="value"
#
# MANAGED CONTENT (in ~/.bash_aliases):
#   - SSH aliases: sa, sr, sv, sn1, sm, sp
#   - ROS2 environment: ROS_DOMAIN_ID, RMW, cyclonedds.xml
#   - ROS2 aliases: cb, nav, sim, restart-gazebo, etc.
#   - PlatformIO functions: buildBoard1, buildBoard2, buildElevator
#
# =============================================================================
# Sigyn Robot Fleet - Managed Bash Aliases Configuration
# This file is managed by setup_robot.py and deployed as ~/.bash_aliases
# Do not edit manually - changes will be overwritten
# Last updated: 2026-02-17
#
# This file is automatically sourced by ~/.bashrc
# Private settings (like API keys) should be kept in ~/.bashrc, not here

# ==================== ROS2 ENVIRONMENT ====================
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Source ROS2 and workspace
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "$HOME/sigyn_ws/install/setup.bash" ]; then
    source "$HOME/sigyn_ws/install/setup.bash"
fi

# Activate sigyn-venv if not already active
if [ -z "$VIRTUAL_ENV" ] && [ -f "$HOME/sigyn-venv/bin/activate" ]; then
    source "$HOME/sigyn-venv/bin/activate"
fi

# Add ~/.local/bin to PATH for local Python packages
export PATH="$HOME/.local/bin:$PATH"

# ==================== SSH ALIASES ====================
alias sa='ssh -YC amdc'
alias sr='ssh -YC sigyn7900a'
alias sv='ssh -YC sigynVision'
alias sn1='ssh -YC sigynNvidia1'
alias sm='ssh -YC MiniMe4'
alias sp='ssh -YC sigynPiservo'

# ==================== ROS2 ALIASES ====================
alias cb='colcon build --symlink-install'
alias cgcm='ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap'
alias clcm='ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap'
alias dla="ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator"
alias fr='ros2 run tf2_tools view_frames'
alias map='clear;ros2 launch base sigyn.launch.py use_sim_time:=false do_rviz:=true make_map:=true'
alias mr='micro-ros-agent serial --dev /dev/ttyACM0 -b 115200'
alias nav='clear;ros2 launch base sigyn.launch.py use_sim_time:=false do_rviz:=true do_oakd:=true'
alias pm='ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'
alias rd='rosdep install --from-paths src --ignore-src -r -y'
alias redoudev='sudo service udev restart;sudo udevadm control --reload-rules;sudo udevadm trigger'
alias rms='ros2 launch nav2_map_server map_saver_server.launch.py'
alias rvs='rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz'
alias savem='ros2 run nav2_map_server map_saver_cli -f my_map'
alias sim='clear;ros2 launch base sigyn.launch.py use_sim_time:=true do_rviz:=true'
alias teensy='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_sensor'
alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias toggle='ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'

# ==================== ROBOT-SPECIFIC ALIASES ====================
# These are useful on sigyn7900a (SR) - main robot computer
alias gripper='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_gripper'
alias patrol='ros2 launch perimeter_roamer_v3 patrol_using_waypoints_launch.py'
alias record='/home/ros/sigyn_ws/src/Sigyn/scripts/bag_record_sim.sh'
alias s2s='ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py'
alias stele='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_nav'
alias teles='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel_unstamped'

# ==================== PLATFORMIO ALIASES ====================
alias compileBoard1='platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2'
alias compileBoard2='platformio run -e board2 -d ~/sigyn_ws/src/Sigyn/TeensyV2'
alias compileElevator='platformio run -e elevator_board -d ~/sigyn_ws/src/Sigyn/TeensyV2'

# ==================== PLATFORMIO UPLOAD FUNCTIONS ====================
# Safer upload helpers: refuse to auto-detect a board if the expected udev symlink isn't present.
# Note: aliases expand before functions in interactive shells, so remove any older aliases.
unalias buildBoard1 2>/dev/null || true
unalias buildBoard2 2>/dev/null || true
unalias buildElevator 2>/dev/null || true

function buildBoard1 {
    local symlink="/dev/teensy_sensor"
    echo "[DEBUG] Starting buildBoard1..."
    
    if [ ! -e "$symlink" ]; then
        echo "[ERROR] Symlink $symlink not found."
        # Check if we can recover a board already in Bootloader mode
        if lsusb -d 16c0:0478 > /dev/null 2>&1; then
            echo "[WARN] Found a Teensy in Bootloader mode (HalfKay). Attempting blind upload..."
            platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port auto
            return $?
        fi
        echo "       (Is the board plugged in? Check 'ls -l /dev/teensy*' and 'lsusb')"
        return 1
    fi

    local port=$(realpath "$symlink")
    echo "[DEBUG] Resolved symlink $symlink -> $port"
    
    echo "[DEBUG] Attempting Upload (Attempt 1) to $port..."
    platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port "$port"
    local ret=$?

    if [ $ret -ne 0 ]; then
        echo "[WARN] Upload failed (code $ret). Checking device state..."
        
        if [ ! -e "$port" ]; then
            echo "[INFO] Port $port is gone. Assuming board entered Bootloader mode."
            echo "[DEBUG] Attempting Upload (Attempt 2 - Blind/Auto)..."
            platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port auto
        else
            echo "[INFO] Port $port still exists. Retrying upload to explicit port..."
            sleep 2
            platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port "$port"
        fi
    fi
}

function buildBoard2 {
    local symlink="/dev/teensy_sensor2"
    if [ ! -e "$symlink" ]; then
        echo "[ERROR] $symlink not present. Refusing to upload board2 firmware." >&2
        echo "[HINT] Power the power/sensor Teensy or check udev rules (try: ls -l /dev/teensy_*)." >&2
        return 2
    fi
    local port=$(realpath "$symlink")
    platformio run -e board2 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port "$port"
}

function buildElevator {
    local symlink="/dev/teensy_gripper"
    if [ ! -e "$symlink" ]; then
        echo "[ERROR] $symlink not present. Refusing to upload elevator firmware." >&2
        echo "[HINT] Power the elevator/gripper Teensy or check udev rules (try: ls -l /dev/teensy_*)." >&2
        return 2
    fi
    local port=$(realpath "$symlink")
    platformio run -e elevator_board -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port "$port"
}

# ==================== GROOT ALIASES ====================
# Different machines may have Groot in different locations
if [ -f "$HOME/Groot2/bin/groot2" ]; then
    alias groot='~/Groot2/bin/groot2'
elif [ -f "$HOME/Groot/build/Groot" ]; then
    alias groot='~/Groot/build/Groot'
fi

# ==================== TEST ALIASES ====================
alias test_teensy='cd ~/sigyn_ws/src/Sigyn/TeensyV2 && pio test -e test && cd ~/sigyn_ws'
