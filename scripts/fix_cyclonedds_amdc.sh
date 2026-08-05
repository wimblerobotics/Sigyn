#!/bin/bash
# =============================================================================
# FIX CYCLONEDDS CONFIGURATION ON AMDC
# =============================================================================
# This script fixes the CycloneDDS configuration conflict on the amdc desktop
# Run this ON THE AMDC MACHINE (192.168.86.28)
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  CycloneDDS Configuration Fix for AMDC${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# 1. Check current hostname
HOSTNAME=$(hostname)
echo -e "${BLUE}[1] Checking hostname...${NC}"
if [ "$HOSTNAME" != "amdc" ]; then
    echo -e "${YELLOW}WARNING: This script is intended for amdc, but hostname is: $HOSTNAME${NC}"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 2. Get the correct network interface name
echo ""
echo -e "${BLUE}[2] Detecting network interface...${NC}"
echo "Current interfaces:"
ip addr show | grep -E "^[0-9]+: " | awk '{print $2}' | tr -d ':'

# Try to detect ethernet interface
ETH_INTERFACE=$(ip addr show | grep -E "^[0-9]+: e" | head -1 | awk '{print $2}' | tr -d ':')
if [ -z "$ETH_INTERFACE" ]; then
    echo -e "${YELLOW}Could not auto-detect ethernet interface${NC}"
    read -p "Enter your network interface name (e.g., enp6s0, eth0): " ETH_INTERFACE
fi

echo -e "Using interface: ${GREEN}$ETH_INTERFACE${NC}"
IP_ADDR=$(ip addr show "$ETH_INTERFACE" | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
echo -e "IP Address: ${GREEN}${IP_ADDR:-NOT FOUND}${NC}"

# 3. Backup existing configs
echo ""
echo -e "${BLUE}[3] Backing up existing configurations...${NC}"
BACKUP_DATE=$(date +%Y%m%d_%H%M%S)

if [ -f /etc/cyclonedds.xml ]; then
    sudo mv /etc/cyclonedds.xml /etc/cyclonedds.xml.backup_$BACKUP_DATE
    echo -e "${GREEN}✓ Backed up /etc/cyclonedds.xml${NC}"
fi

if [ -f ~/.ros/cyclonedds.xml ]; then
    mv ~/.ros/cyclonedds.xml ~/.ros/cyclonedds.xml.backup_$BACKUP_DATE
    echo -e "${GREEN}✓ Backed up ~/.ros/cyclonedds.xml${NC}"
fi

# 4. Create new unified config
echo ""
echo -e "${BLUE}[4] Creating new CycloneDDS configuration...${NC}"

mkdir -p ~/.ros

cat > ~/.ros/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
    xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <!-- Enable multicast for discovery, but rely on unicast peers for reliability -->
      <AllowMulticast>true</AllowMulticast>
      
      <!-- IMPORTANT: keep MaxMessageSize BELOW the real network MTU (1500).
           A too-large value (e.g. MB-scale) causes EMSGSIZE
           (DDS_RETCODE_NOT_ENOUGH_SPACE / retcode -58) on every large message,
           or forces fragile OS-level IP fragmentation that WiFi tends to drop.
           FragmentSize already chops large samples (images/pointclouds) into
           small pieces; do not raise MaxMessageSize to "fix" image delivery.
           See CYCLONEDDS_ISSUE_ANALYSIS.md for the full writeup. -->
      <MaxMessageSize>1470 B</MaxMessageSize>
      
      <!-- Fragment size optimized for Ethernet -->
      <FragmentSize>1200 B</FragmentSize>
      
      <!-- Network interface configuration -->
      <Interfaces>
        <NetworkInterface name="INTERFACE_PLACEHOLDER" priority="default" multicast="true"/>
      </Interfaces>
    </General>
    
    <Discovery>
      <!-- Explicit unicast peers for reliable discovery -->
      <Peers>
        <Peer address="192.168.86.28"/>   <!-- amdc desktop -->
        <Peer address="192.168.86.109"/>  <!-- sigyn7900a robot -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    
    <Internal>
      <!-- Large socket buffers to handle burst traffic -->
      <SocketReceiveBufferSize min="10MB"/>
      <SocketSendBufferSize min="10MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
EOF

# Replace interface placeholder
sed -i "s/INTERFACE_PLACEHOLDER/$ETH_INTERFACE/g" ~/.ros/cyclonedds.xml

echo -e "${GREEN}✓ Created ~/.ros/cyclonedds.xml${NC}"

# 5. Update bashrc if needed
echo ""
echo -e "${BLUE}[5] Checking bash configuration...${NC}"

if ! grep -q "CYCLONEDDS_URI" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# CycloneDDS configuration" >> ~/.bashrc
    echo "export CYCLONEDDS_URI=file://\$HOME/.ros/cyclonedds.xml" >> ~/.bashrc
    echo -e "${GREEN}✓ Added CYCLONEDDS_URI to ~/.bashrc${NC}"
else
    echo -e "${GREEN}✓ CYCLONEDDS_URI already in ~/.bashrc${NC}"
fi

# Export for current session
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml

# 6. Restart ROS 2 daemon
echo ""
echo -e "${BLUE}[6] Restarting ROS 2 daemon...${NC}"

if command -v ros2 &> /dev/null; then
    ros2 daemon stop 2>/dev/null || true
    sleep 2
    ros2 daemon start
    echo -e "${GREEN}✓ ROS 2 daemon restarted${NC}"
else
    echo -e "${YELLOW}⚠ ros2 command not found - daemon not restarted${NC}"
    echo "  You'll need to restart it manually after sourcing ROS 2"
fi

# 7. Summary
echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  CONFIGURATION COMPLETE!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo "Changes made:"
echo "  ✓ Backed up old configs (with timestamp)"
echo "  ✓ Created new unified config in ~/.ros/cyclonedds.xml"
echo "  ✓ Set MaxMessageSize to 1470B (MTU-safe; was 4KB/4MB/16MB)"
echo "  ✓ Added CYCLONEDDS_URI to ~/.bashrc"
echo "  ✓ Restarted ROS 2 daemon"
echo ""
echo -e "${YELLOW}IMPORTANT: Restart any ROS 2 nodes (including rviz2) for changes to take effect${NC}"
echo ""
echo "To verify configuration:"
echo "  ~/sigyn_ws/src/Sigyn/scripts/diagnose_cyclonedds.sh"
echo ""
