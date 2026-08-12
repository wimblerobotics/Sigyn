#!/bin/bash
# =============================================================================
# UNIFIED DOCKING TEST ORCHESTRATOR
# =============================================================================
# Launches all required terminals for a docking test in the correct order,
# captures comprehensive debug data, and provides interactive control.
#
# Usage:
#   ./run_docking_test.sh [test_name]
#
# Where test_name is optional (defaults to test_YYYYMMDD_HHMMSS)
# =============================================================================

set -e

# Configuration
TEST_NAME="${1:-test_$(date +%Y%m%d_%H%M%S)}"
TEST_DIR="$HOME/$TEST_NAME"
WORKSPACE="$HOME/sigyn_ws"
VENV="$HOME/sigyn-venv/bin/activate"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}"
echo "============================================================================="
echo "  DOCKING TEST ORCHESTRATOR"
echo "  Test: $TEST_NAME"
echo "  Output: $TEST_DIR"
echo "============================================================================="
echo -e "${NC}"

# Clear out any previous run's data before starting. This matters even though
# launch.log/nav2_monitor.log are truncated (>) on each run: `ros2 bag record
# -o docking_debug` REFUSES to write into an already-existing bag directory
# and fails silently (background process just dies), which left a stale
# docking_debug bag from the very first run being (mis)reported as "current"
# on every subsequent reuse of the same test name.
if [ -d "$TEST_DIR" ]; then
    echo -e "${YELLOW}Clearing previous test data in $TEST_DIR...${NC}"
    rm -rf "$TEST_DIR"
fi

# Create test directory
mkdir -p "$TEST_DIR"
cd "$TEST_DIR"

# Function to check if a process is running
is_running() {
    kill -0 "$1" 2>/dev/null
}

# Function to wait for a topic to have at least one publisher.
# Uses 'ros2 topic info' (a graph query, not a subscription) so it works
# instantly without needing DDS participant discovery or data delivery.
# Usage: wait_for_topic <topic> [timeout_sec]
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    local elapsed=0

    echo -ne "${YELLOW}Waiting for $topic..."
    while ! ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ $elapsed -ge $timeout ]; then
            echo -e " ${RED}TIMEOUT${NC}"
            return 1
        fi
    done
    echo -e " ${GREEN}OK${NC}"
    return 0
}

# Function to wait for a node
wait_for_node() {
    local node=$1
    local timeout=${2:-30}
    local elapsed=0
    
    echo -ne "${YELLOW}Waiting for node $node..."
    while ! ros2 node list 2>/dev/null | grep -q "$node"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ $elapsed -ge $timeout ]; then
            echo -e " ${RED}TIMEOUT${NC}"
            return 1
        fi
    done
    echo -e " ${GREEN}OK${NC}"
    return 0
}

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}🛑 Shutting down...${NC}"
    
    # Stop bag recorder and monitors first so they flush data cleanly
    [ -n "$BAG_PID" ]     && kill $BAG_PID 2>/dev/null     && echo "  Stopped bag recorder"
    [ -n "$MONITOR_PID" ] && kill $MONITOR_PID 2>/dev/null && echo "  Stopped debug monitor"
    [ -n "$HELPER_PID" ]  && kill $HELPER_PID 2>/dev/null  && echo "  Stopped docking helper"

    # Send SIGINT (not SIGTERM) to ros2 launch — this triggers its shutdown
    # sequence and properly stops all managed nodes in reverse dependency order.
    if [ -n "$LAUNCH_PID" ] && kill -0 $LAUNCH_PID 2>/dev/null; then
        echo "  Sending SIGINT to launch process ($LAUNCH_PID)..."
        kill -SIGINT $LAUNCH_PID 2>/dev/null
        # Wait up to 15 seconds for a clean shutdown
        local waited=0
        while kill -0 $LAUNCH_PID 2>/dev/null && [ $waited -lt 15 ]; do
            sleep 1
            waited=$((waited + 1))
        done
        # Force-kill anything still alive
        if kill -0 $LAUNCH_PID 2>/dev/null; then
            echo "  Force-killing launch process and children..."
            kill -SIGKILL -- -$(ps -o pgid= $LAUNCH_PID 2>/dev/null | tr -d ' ') 2>/dev/null || true
            kill -SIGKILL $LAUNCH_PID 2>/dev/null || true
        fi
        echo "  Stopped launch file"
    fi

    # Wait briefly for processes to fully release resources
    sleep 2
    
    echo -e "${GREEN}✅ Test complete!${NC}"
    echo -e "${CYAN}Data saved to: $TEST_DIR${NC}"
    
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# =============================================================================
# PHASE 1: PRE-TEST CHECKS
# =============================================================================
echo -e "${BLUE}[1/5] Pre-test checks${NC}"

# Check that we're in the right environment
if [ ! -f "$VENV" ]; then
    echo -e "${RED}ERROR: Virtual environment not found at $VENV${NC}"
    exit 1
fi

# Source environment
source "$VENV"
source "$WORKSPACE/install/setup.bash"

# Check required scripts exist
REQUIRED_SCRIPTS=(
    "$WORKSPACE/src/Sigyn/scripts/capture_docking_data.sh"
    "$WORKSPACE/src/Sigyn/sigyn_bringup/scripts/nav2_debug_monitor.py"
    "$WORKSPACE/src/Sigyn/sigyn_bringup/scripts/docking_helper.py"
)

for script in "${REQUIRED_SCRIPTS[@]}"; do
    if [ ! -f "$script" ]; then
        echo -e "${RED}ERROR: Required script not found: $script${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✓ Environment ready${NC}"

# =============================================================================
# PHASE 2: LAUNCH NAVIGATION STACK
# =============================================================================
echo ""
echo -e "${BLUE}[2/5] Launching navigation stack with DEBUG logging${NC}"

# Check if already running
if ros2 node list 2>/dev/null | grep -q docking_server; then
    echo -e "${YELLOW}⚠ Navigation stack appears to be already running${NC}"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Launch in background, save output
nohup ros2 launch sigyn_bringup sigyn.launch.py do_rviz:=false do_oakd:=true > "$TEST_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# Wait for critical nodes
echo "  Waiting for nodes to come online (this may take 30-60 seconds)..."
wait_for_node "/docking_server" 60 || { echo -e "${RED}Failed to start docking_server${NC}"; cleanup; }
wait_for_node "/controller_server" 60 || { echo -e "${RED}Failed to start controller_server${NC}"; cleanup; }
wait_for_node "/amcl" 60 || { echo -e "${RED}Failed to start amcl${NC}"; cleanup; }

echo -e "${GREEN}✓ Navigation stack online${NC}"

# =============================================================================
# PHASE 3: START MONITORING
# =============================================================================
echo ""
echo -e "${BLUE}[3/5] Starting monitoring and data capture${NC}"

# Start nav2_debug_monitor
echo "  Starting Nav2 debug monitor..."
nohup python3 "$WORKSPACE/src/Sigyn/sigyn_bringup/scripts/nav2_debug_monitor.py" > "$TEST_DIR/nav2_monitor.log" 2>&1 &
MONITOR_PID=$!
echo "  Monitor PID: $MONITOR_PID"

# Wait for monitor to initialize
sleep 2

# Start bag recording
echo "  Starting rosbag recording..."
"$WORKSPACE/src/Sigyn/scripts/capture_docking_data.sh" "$TEST_DIR/bag_data" > "$TEST_DIR/bag_recorder.log" 2>&1 &
BAG_PID=$!
echo "  Bag recorder PID: $BAG_PID"

# Wait for bag recorder to initialize
sleep 3

# Verify bag recorder started
if ! is_running $BAG_PID; then
    echo -e "${RED}ERROR: Bag recorder failed to start. Check $TEST_DIR/bag_recorder.log${NC}"
    cleanup
fi

echo -e "${GREEN}✓ Monitoring active${NC}"

# =============================================================================
# PHASE 4: WAIT FOR READINESS
# =============================================================================
echo ""
echo -e "${BLUE}[4/5] Waiting for sensor data${NC}"

# Wait for hardware topics first (these don't need user action)
wait_for_topic "/scan" 30 || { echo -e "${RED}No laser scan${NC}"; cleanup; }
wait_for_topic "/odom" 30 || { echo -e "${RED}No odometry${NC}"; cleanup; }
wait_for_topic "/amcl_pose" 30 || { echo -e "${RED}AMCL publisher not found${NC}"; cleanup; }

echo ""
echo -e "${CYAN}=================================================================${NC}"
echo -e "${CYAN}  ACTION REQUIRED: Set the robot's initial pose in RViz2${NC}"
echo -e "${CYAN}=================================================================${NC}"
echo -e "  1. In RViz2 on amdc, click ${YELLOW}\"2D Pose Estimate\"${NC}"
echo -e "  2. Click and drag on the map at the robot's current location"
echo -e "  3. Press Enter here once the pose is set and particles have converged"
echo -e "${CYAN}=================================================================${NC}"
read -r -p "  Press Enter when pose estimate is set... "
echo ""

# NOTE: No check for /oakd_apriltag_node/detections here — the robot won't see
# the dock AprilTags until it approaches during the docking manoeuvre itself.
# The docking_server handles acquisition during the approach.

# Capture current pose (TRANSIENT_LOCAL — must match QoS to get cached value)
echo ""
echo -e "${CYAN}Current robot pose:${NC}"
timeout 5 ros2 topic echo /amcl_pose --once --qos-durability transient_local --qos-reliability reliable 2>/dev/null | grep -A 6 "pose:" | head -7 || echo "  (not available)"

echo ""
echo -e "${GREEN}✓ System ready for docking test${NC}"

# =============================================================================
# PHASE 5: INTERACTIVE DOCKING
# =============================================================================
echo ""
echo -e "${BLUE}[5/5] Interactive docking control${NC}"
echo ""
echo -e "${CYAN}=================================================================${NC}"
echo -e "${CYAN}  SYSTEM IS READY - YOU HAVE TWO OPTIONS:${NC}"
echo -e "${CYAN}=================================================================${NC}"
echo ""
echo -e "${GREEN}OPTION 1: Use docking_helper.py (RECOMMENDED)${NC}"
echo "  This runs in this terminal with full monitoring"
echo "  Command: python3 $WORKSPACE/src/Sigyn/sigyn_bringup/scripts/docking_helper.py dock home_charging_dock"
echo ""
echo -e "${GREEN}OPTION 2: Manual ROS 2 action call${NC}"
echo "  Run in a separate terminal:"
echo "  ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot \"{use_dock_id: true, dock_id: 'home_charging_dock'}\""
echo ""
echo -e "${YELLOW}When test is complete, press Ctrl+C to stop recording and shutdown${NC}"
echo ""
echo -e "${CYAN}=================================================================${NC}"
echo ""

# Ask user which option
while true; do
    read -p "Choose option [1=helper, 2=manual, q=quit]: " -n 1 -r
    echo
    case $REPLY in
        1)
            echo ""
            echo -e "${GREEN}Starting docking_helper.py...${NC}"
            echo ""
            python3 "$WORKSPACE/src/Sigyn/sigyn_bringup/scripts/docking_helper.py" dock home_charging_dock
            HELPER_EXIT=$?
            echo ""
            echo -e "${CYAN}Docking helper exited with code: $HELPER_EXIT${NC}"
            echo ""
            read -p "Run again? [y=yes, n=stop test]: " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                break
            fi
            ;;
        2)
            echo ""
            echo -e "${YELLOW}Waiting for manual docking action...${NC}"
            echo "  Monitoring logs in background"
            echo "  Press Ctrl+C when done"
            echo ""
            # Just wait until user presses Ctrl+C
            while true; do
                sleep 1
            done
            ;;
        [Qq])
            break
            ;;
        *)
            echo "Invalid option"
            ;;
    esac
done

# Cleanup will be called by trap
cleanup
