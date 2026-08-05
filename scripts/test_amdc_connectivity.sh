#!/bin/bash
# =============================================================================
# Quick DDS Connectivity Test - Run on AMDC
# =============================================================================
# This script tests if amdc can discover and receive data from sigyn7900a
# =============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  DDS Connectivity Test for AMDC${NC}"
echo -e "${CYAN}  Testing connection to sigyn7900a${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Check hostname
HOSTNAME=$(hostname)
echo -e "${BLUE}[1] System Info${NC}"
echo "Hostname: $HOSTNAME"
echo "Date: $(date)"
echo ""

# Check environment
echo -e "${BLUE}[2] ROS 2 Environment${NC}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NOT SET}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-NOT SET}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-NOT SET}"
echo "CYCLONEDDS_URI: ${CYCLONEDDS_URI:-NOT SET}"
echo ""

# Check if config file exists and what it says
if [ -n "$CYCLONEDDS_URI" ]; then
    CONFIG_FILE="${CYCLONEDDS_URI#file://}"
    if [ -f "$CONFIG_FILE" ]; then
        echo -e "${GREEN}✓ Config file exists: $CONFIG_FILE${NC}"
        echo "  MaxMessageSize: $(grep MaxMessageSize "$CONFIG_FILE" | sed 's/.*>\(.*\)<.*/\1/' | tr -d ' ')"
    else
        echo -e "${RED}✗ Config file not found: $CONFIG_FILE${NC}"
    fi
elif [ -f "/etc/cyclonedds.xml" ]; then
    echo -e "${YELLOW}⚠ Using system config: /etc/cyclonedds.xml${NC}"
    echo "  MaxMessageSize: $(grep MaxMessageSize /etc/cyclonedds.xml | sed 's/.*>\(.*\)<.*/\1/' | tr -d ' ')"
elif [ -f "$HOME/.ros/cyclonedds.xml" ]; then
    echo -e "${YELLOW}⚠ Found ~/.ros/cyclonedds.xml but CYCLONEDDS_URI not set${NC}"
    echo "  MaxMessageSize: $(grep MaxMessageSize ~/.ros/cyclonedds.xml | sed 's/.*>\(.*\)<.*/\1/' | tr -d ' ')"
else
    echo -e "${YELLOW}⚠ No CycloneDDS config found${NC}"
fi
echo ""

# Check network connectivity
echo -e "${BLUE}[3] Network Connectivity${NC}"
echo -n "Ping sigyn7900a (192.168.86.109) ... "
if ping -c 1 -W 1 192.168.86.109 > /dev/null 2>&1; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    echo "  Cannot reach sigyn7900a - check network connection!"
fi
echo ""

# Check ROS 2 daemon
echo -e "${BLUE}[4] ROS 2 Daemon${NC}"
if pgrep -f "ros2.*daemon" > /dev/null; then
    echo -e "${GREEN}✓ Daemon is running${NC}"
    DAEMON_PID=$(pgrep -f "ros2.*daemon" | head -1)
    if [ -f "/proc/$DAEMON_PID/environ" ]; then
        DAEMON_CYCLONE=$(cat /proc/$DAEMON_PID/environ | tr '\0' '\n' | grep CYCLONEDDS_URI || echo "NOT SET")
        echo "  Daemon CYCLONEDDS_URI: $DAEMON_CYCLONE"
    fi
else
    echo -e "${RED}✗ Daemon not running${NC}"
fi
echo ""

# Try to list topics
echo -e "${BLUE}[5] Topic Discovery${NC}"
echo "Attempting to discover topics from sigyn7900a..."
echo ""

TOPICS=$(timeout 5 ros2 topic list 2>&1)
if [ $? -eq 0 ]; then
    TOPIC_COUNT=$(echo "$TOPICS" | wc -l)
    echo -e "${GREEN}✓ Discovered $TOPIC_COUNT topics${NC}"
    
    # Check for AprilTag topic specifically
    if echo "$TOPICS" | grep -q "oakd_apriltag_node/annotated_image"; then
        echo -e "${GREEN}✓ Found /oakd_apriltag_node/annotated_image${NC}"
        
        # Check publisher info
        echo ""
        echo "Publisher details:"
        timeout 3 ros2 topic info /oakd_apriltag_node/annotated_image --verbose 2>&1 | head -30
    else
        echo -e "${RED}✗ AprilTag image topic NOT found${NC}"
        echo ""
        echo "Available topics:"
        echo "$TOPICS" | head -20
    fi
else
    echo -e "${RED}✗ Failed to discover topics${NC}"
    echo "Error: $TOPICS"
fi
echo ""

# Try to echo a simple topic
echo -e "${BLUE}[6] Data Reception Test${NC}"
echo "Testing if we can receive /rosout (simple topic)..."
if timeout 2 ros2 topic echo /rosout --once > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Can receive /rosout${NC}"
else
    echo -e "${RED}✗ Cannot receive /rosout${NC}"
fi
echo ""

# Summary
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}  DIAGNOSIS${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"

if [ -z "$CYCLONEDDS_URI" ]; then
    echo -e "${RED}⚠ PROBLEM: CYCLONEDDS_URI not set${NC}"
    echo ""
    echo "Solution:"
    echo "  1. Run the fix script: ./fix_cyclonedds_amdc.sh"
    echo "  2. Or manually: export CYCLONEDDS_URI=file://\$HOME/.ros/cyclonedds.xml"
    echo "  3. Restart daemon: ros2 daemon stop && ros2 daemon start"
    echo "  4. Restart this terminal or source ~/.bashrc"
fi

if ! timeout 3 ros2 topic list 2>/dev/null | grep -q "oakd"; then
    echo -e "${RED}⚠ PROBLEM: Cannot discover topics from sigyn7900a${NC}"
    echo ""
    echo "Possible causes:"
    echo "  1. CycloneDDS config issue (MaxMessageSize too small)"
    echo "  2. Wrong network interface configured"
    echo "  3. Firewall blocking DDS discovery"
    echo "  4. ROS_DOMAIN_ID mismatch"
    echo ""
    echo "Solution: Run ./fix_cyclonedds_amdc.sh"
fi

echo ""
echo "For full diagnostics, run: ~/sigyn_ws/src/Sigyn/scripts/diagnose_cyclonedds.sh"
echo ""
