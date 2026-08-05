#!/bin/bash
# =============================================================================
# CycloneDDS Configuration Diagnostic Tool
# =============================================================================
# Diagnoses configuration conflicts and network issues between ROS 2 machines
# =============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  CycloneDDS Configuration Diagnostic${NC}"
echo -e "${CYAN}  Hostname: $(hostname)${NC}"
echo -e "${CYAN}  Date: $(date)${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Check environment variables
echo -e "${BLUE}[1] Environment Variables${NC}"
echo "----------------------------------------"
echo "CYCLONEDDS_URI: ${CYCLONEDDS_URI:-NOT SET}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NOT SET}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-NOT SET}"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE: ${ROS_AUTOMATIC_DISCOVERY_RANGE:-NOT SET}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-NOT SET}"
echo ""

# Check which config files exist
echo -e "${BLUE}[2] Configuration Files${NC}"
echo "----------------------------------------"
for config in /etc/cyclonedds.xml ~/.ros/cyclonedds.xml; do
    if [ -f "$config" ]; then
        echo -e "${GREEN}✓ EXISTS:${NC} $config"
        echo "  Modified: $(stat -c %y "$config" | cut -d'.' -f1)"
        echo "  Size: $(stat -c %s "$config") bytes"
        
        # Extract key settings
        if grep -q "MaxMessageSize" "$config"; then
            size=$(grep "MaxMessageSize" "$config" | sed 's/.*>\(.*\)<.*/\1/' | tr -d ' ')
            echo "  MaxMessageSize: $size"
        fi
        if grep -q "FragmentSize" "$config"; then
            frag=$(grep "FragmentSize" "$config" | sed 's/.*>\(.*\)<.*/\1/' | tr -d ' ')
            echo "  FragmentSize: $frag"
        fi
        if grep -q "Peer" "$config"; then
            peers=$(grep -o "address=\"[^\"]*\"" "$config" | wc -l)
            echo "  Peer count: $peers"
            grep "Peer" "$config" | sed 's/^/    /'
        fi
    else
        echo -e "${YELLOW}✗ NOT FOUND:${NC} $config"
    fi
    echo ""
done

# Check active config
echo -e "${BLUE}[3] Active Configuration${NC}"
echo "----------------------------------------"
if [ -n "$CYCLONEDDS_URI" ]; then
    active_config="${CYCLONEDDS_URI#file://}"
    echo -e "Using: ${GREEN}$active_config${NC}"
    if [ ! -f "$active_config" ]; then
        echo -e "${RED}ERROR: Specified config file does not exist!${NC}"
    fi
else
    echo -e "${YELLOW}WARNING: CYCLONEDDS_URI not set${NC}"
    echo "CycloneDDS will use default settings or /etc/cyclonedds.xml if it exists"
fi
echo ""

# Check network interfaces
echo -e "${BLUE}[4] Network Interfaces${NC}"
echo "----------------------------------------"
ip addr show | grep -E "^[0-9]+:|inet " | while read line; do
    if [[ $line =~ ^[0-9]+ ]]; then
        echo "$line"
    else
        echo "  $line"
    fi
done
echo ""

# Check ROS 2 daemon
echo -e "${BLUE}[5] ROS 2 Daemon${NC}"
echo "----------------------------------------"
if pgrep -f "ros2.*daemon" > /dev/null; then
    echo -e "${GREEN}✓ Daemon is running${NC}"
    daemon_pid=$(pgrep -f "ros2.*daemon" | head -1)
    echo "  PID: $daemon_pid"
    
    # Check daemon's environment
    if [ -f "/proc/$daemon_pid/environ" ]; then
        daemon_cyclone=$(cat /proc/$daemon_pid/environ | tr '\0' '\n' | grep CYCLONEDDS_URI || echo "NOT SET")
        echo "  Daemon CYCLONEDDS_URI: $daemon_cyclone"
    fi
else
    echo -e "${YELLOW}✗ Daemon is not running${NC}"
fi
echo ""

# Test connectivity to peers
echo -e "${BLUE}[6] Network Connectivity to Peers${NC}"
echo "----------------------------------------"
for peer in 192.168.86.28 192.168.86.109; do
    if [ "$peer" != "$(hostname -I | awk '{print $1}')" ]; then
        echo -n "Testing $peer ... "
        if ping -c 1 -W 1 "$peer" > /dev/null 2>&1; then
            echo -e "${GREEN}OK${NC}"
        else
            echo -e "${RED}UNREACHABLE${NC}"
        fi
    fi
done
echo ""

# Check for large message topics
echo -e "${BLUE}[7] Topics with Large Messages (potential fragmentation issues)${NC}"
echo "----------------------------------------"
timeout 3 ros2 topic list -t 2>/dev/null | grep -E "Image|PointCloud|CompressedImage|LaserScan" || echo "No topics found (timeout or no ROS 2 nodes running)"
echo ""

# Summary and recommendations
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}  RECOMMENDATIONS${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"

if [ -f "/etc/cyclonedds.xml" ] && [ -f "$HOME/.ros/cyclonedds.xml" ]; then
    if ! diff /etc/cyclonedds.xml ~/.ros/cyclonedds.xml > /dev/null 2>&1; then
        echo -e "${RED}⚠ CONFLICT: Two different CycloneDDS configs exist${NC}"
        echo "  /etc/cyclonedds.xml (system-wide)"
        echo "  ~/.ros/cyclonedds.xml (user-specific)"
        echo ""
        echo "SOLUTION: Remove or unify the configs"
        echo "  sudo mv /etc/cyclonedds.xml /etc/cyclonedds.xml.backup"
        echo "  OR: Ensure CYCLONEDDS_URI is set in ALL shell sessions"
    fi
fi

if [ -z "$CYCLONEDDS_URI" ]; then
    echo -e "${YELLOW}⚠ CYCLONEDDS_URI not set in current shell${NC}"
    echo "  Add to ~/.bashrc: export CYCLONEDDS_URI=file://\$HOME/.ros/cyclonedds.xml"
fi

echo ""
echo "For image transmission over WiFi, ensure:"
echo "  • MaxMessageSize below the path MTU (e.g. 1470 B), NOT megabytes"
echo "  • FragmentSize = 1200B (optimal for WiFi)"
echo "  • Explicit peer addresses listed"
echo "  • Same config on ALL machines"
echo ""
