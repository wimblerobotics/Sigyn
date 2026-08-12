#!/bin/bash
# Comprehensive docking debug data capture
# Records ALL relevant topics for post-mortem analysis

OUTPUT_DIR="${1:-$HOME/docking_debug_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUTPUT_DIR"

echo "=========================================="
echo "  COMPREHENSIVE DOCKING DEBUG CAPTURE"
echo "  Started: $(date)"
echo "  Output: $OUTPUT_DIR"
echo "=========================================="
echo ""

# Create metadata file
cat > "$OUTPUT_DIR/metadata.txt" << EOF
Docking Debug Session
Started: $(date)
Hostname: $(hostname)
ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}
ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-0}

EOF

# Capture initial state
echo "📋 Capturing initial system state..."

# Capture all parameters
ros2 param dump /docking_server > "$OUTPUT_DIR/docking_server_params.yaml" 2>/dev/null || echo "docking_server not ready"
ros2 param dump /planner_server > "$OUTPUT_DIR/planner_server_params.yaml" 2>/dev/null || echo "planner_server not ready"
ros2 param dump /controller_server > "$OUTPUT_DIR/controller_server_params.yaml" 2>/dev/null || echo "controller_server not ready"
ros2 param dump /amcl > "$OUTPUT_DIR/amcl_params.yaml" 2>/dev/null || echo "amcl not ready"

# Capture dock database
cp ~/sigyn_ws/src/Sigyn/sigyn_bringup/config/docking_stations.yaml "$OUTPUT_DIR/" 2>/dev/null

# Capture TF tree
ros2 run tf2_tools view_frames --ros-args -r __ns:=/tf2_frames 2>&1 | head -50 > "$OUTPUT_DIR/tf_tree.txt" &
TF_PID=$!

# List all active topics
ros2 topic list > "$OUTPUT_DIR/topics.txt"

# List all active nodes
ros2 node list > "$OUTPUT_DIR/nodes.txt"

# Start rosbag recording in background
echo ""
echo "🎥 Starting rosbag recording..."
echo "   This will capture: localization, navigation, sensing, control, transforms"
echo ""

# Critical topics for docking diagnosis
TOPICS=(
    # Localization
    /amcl_pose
    /particle_cloud
    /initialpose
    
    # Odometry & Motion
    /odom
    /cmd_vel
    /cmd_vel_nav
    /cmd_vel_keyboard
    /cmd_vel_joystick
    
    # Navigation Goals & Plans
    /goal_pose
    /plan
    /local_plan
    /navigate_to_pose/_action/feedback
    /navigate_to_pose/_action/status
    
    # Docking-specific
    /detected_dock_pose
    /staging_pose
    /dock_robot/_action/feedback
    /dock_robot/_action/status
    
    # Sensing
    /scan
    /scan_filtered
    
    # AprilTag Detection
    /oakd_apriltag_node/detections
    
    # Costmaps (commented out by default - very large)
    # /local_costmap/costmap
    # /global_costmap/costmap
    
    # Transforms (CRITICAL)
    /tf
    /tf_static
    
    # System
    /rosout
)

# Build topic list string
TOPIC_STR=""
for topic in "${TOPICS[@]}"; do
    TOPIC_STR="$TOPIC_STR $topic"
done

cd "$OUTPUT_DIR"
ros2 bag record --topics $TOPIC_STR \
    --storage sqlite3 \
    --max-cache-size 500000000 \
    -o docking_debug &

BAG_PID=$!
echo "   Rosbag PID: $BAG_PID"
echo "   Recording to: $OUTPUT_DIR/docking_debug"
echo ""

# Wait for tf tree capture to finish
wait $TF_PID 2>/dev/null

# Monitor robot state
echo "📍 Current robot state (will update every 2 seconds):"
echo "   Press Ctrl+C when docking test is complete"
echo ""

# Create monitoring loop
trap "echo ''; echo '🛑 Stopping recording...'; kill $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null; echo '✅ Recording stopped'; echo 'Data saved to: $OUTPUT_DIR'; exit 0" SIGINT SIGTERM

COUNT=0
while kill -0 $BAG_PID 2>/dev/null; do
    sleep 2
    COUNT=$((COUNT + 1))
    
    # Every 10 seconds, show robot pose
    if [ $((COUNT % 5)) -eq 0 ]; then
        echo "--- Time: $((COUNT * 2))s ---"
        timeout 1 ros2 topic echo /amcl_pose --once 2>/dev/null | grep -A 3 "pose:" | head -4 || echo "No pose data"
    fi
done

echo ""
echo "✅ Recording completed"
echo "Data saved to: $OUTPUT_DIR"
