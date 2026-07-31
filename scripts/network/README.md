# Network Configuration

This directory contains network-related configuration files and scripts for the Sigyn Robot.

## CycloneDDS Configuration

### Files
- `cyclonedds.xml` - CycloneDDS configuration template
- `setup_cyclonedds.sh` - Installation script for CycloneDDS configuration

### Setup

To install the CycloneDDS configuration on a new system:

```bash
cd ~/sigyn_ws/src/Sigyn/scripts/network
./setup_cyclonedds.sh
```

This will:
1. Create `~/.ros/` directory if it doesn't exist
2. Copy `cyclonedds.xml` to `~/.ros/cyclonedds.xml`
3. Verify environment variables are configured

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
