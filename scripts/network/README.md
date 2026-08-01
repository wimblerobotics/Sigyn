# Network Configuration

This directory contains network-related configuration files and scripts for the Sigyn Robot.

## CycloneDDS Configuration

### Files
- `cyclonedds.xml` - Generic CycloneDDS configuration (fallback)
- `cyclonedds_sigyn7900a.xml` - Configuration for sigyn7900a robot (WiFi wlp8s0)
- `cyclonedds_amdc.xml` - Configuration for amdc desktop (Ethernet eno1)
- `setup_cyclonedds.sh` - Installation script that auto-detects hostname

### Setup

To install the CycloneDDS configuration on a machine:

```bash
cd ~/sigyn_ws/src/Sigyn/scripts/network
./setup_cyclonedds.sh
```

This will:
1. Detect the hostname (sigyn7900a, amdc, or other)
2. Select the appropriate machine-specific configuration
3. Create `~/.ros/` directory if it doesn't exist
4. Copy the selected config to `~/.ros/cyclonedds.xml`
5. Verify environment variables are configured

### Machine-Specific Configurations

**sigyn7900a (robot):**
- Network: WiFi interface `wlp8s0`
- IP: 192.168.86.109
- Peers: amdc (192.168.86.28)

**amdc (desktop):**
- Network: Ethernet interface `eno1`
- IP: 192.168.86.28
- Peers: sigyn7900a (192.168.86.109)

**Other machines:**
- Uses generic configuration (no interface or peer restrictions)
- Relies on multicast discovery only

### Environment Variable

The bashrc configuration (deployed via the Sigyn setup) sets:
```bash
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
```

### Configuration Details

The CycloneDDS configuration includes:
- **AllowMulticast**: Enabled for ROS 2 discovery
- **MaxMessageSize**: 4096 bytes
- **FragmentSize**: 1200 bytes (optimized for network MTU)
- **SocketReceiveBufferSize**: 10MB minimum
- **SocketSendBufferSize**: 10MB minimum

These settings optimize DDS for the Sigyn robot's network environment and reduce UDP communication errors.

## WiFi Configuration

### Files
- `99-bgscan.sh` - WiFi background scanning configuration

See the main Sigyn documentation for WiFi setup details.
