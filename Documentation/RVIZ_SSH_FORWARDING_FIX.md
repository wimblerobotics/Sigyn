# RViz2 SSH Forwarding Fix Guide

## Problem
SSH-forwarded `rviz2` from sigyn7900a to amdc crashes the user session due to NVIDIA/GLX issues.

## Root Causes
1. **On sigyn7900a**: Forced Mesa software rendering (`LIBGL_ALWAYS_SOFTWARE=1`) in bash aliases
2. **On amdc**: Outdated NVIDIA driver (535) incompatible with indirect GLX rendering

## Solution Overview

### Phase 1: Remove Mesa/GLX Forcing on Sigyn7900a ✅
**Status**: COMPLETED

The following aliases in `bashrc` have been fixed:
- `map` - removed `LIBGL_ALWAYS_SOFTWARE=1`
- `nav` - removed `LIBGL_ALWAYS_SOFTWARE=1`

**Deployment Steps**:
1. Copy the updated bashrc to sigyn7900a:
   ```bash
   scp /home/ros/sigyn_ws/src/Sigyn/bashrc sigyn7900a:~/.bash_aliases
   ```

2. On sigyn7900a, reload the configuration:
   ```bash
   source ~/.bash_aliases
   ```

3. Verify the fix:
   ```bash
   alias map | grep LIBGL  # Should return nothing
   alias nav | grep LIBGL  # Should return nothing
   ```

### Phase 2: Investigate and Fix NVIDIA Driver on amdc

#### Step 1: Check Current Driver Version
On **amdc**, run:
```bash
nvidia-smi --query-gpu=driver_version --format=csv,noheader
```

Expected output: `535.xxx` (current outdated version)

#### Step 2: Check Available Drivers
```bash
ubuntu-drivers list
```

Look for newer stable drivers (550, 555, 560, etc.)

#### Step 3: Upgrade NVIDIA Driver

**Option A: Automatic (Recommended)**
```bash
sudo ubuntu-drivers install
```
This installs the recommended driver for your GPU.

**Option B: Manual (Specific Version)**
```bash
# Check available versions
apt-cache search '^nvidia-driver-[0-9]+$' | sort -V

# Install specific version (e.g., 550)
sudo apt-get update
sudo apt-get install nvidia-driver-550

# Alternatively, for latest production branch
sudo apt-get install nvidia-driver-560
```

#### Step 4: Reboot
```bash
sudo reboot
```

#### Step 5: Validate Installation
After reboot, on **amdc**:
```bash
# Check driver version
nvidia-smi

# Verify GLX is using NVIDIA (not Mesa)
glxinfo | grep -E "vendor|renderer|version"
```

Expected output should show:
- `OpenGL vendor string: NVIDIA Corporation`
- `OpenGL renderer string: NVIDIA GeForce XXX`
- `direct rendering: Yes`

### Phase 3: Test SSH X Forwarding

#### Test 1: GLX Info
From **sigyn7900a**:
```bash
ssh -Y amdc 'glxinfo | grep -E "vendor|renderer|version"'
```

Should show NVIDIA, not Mesa/llvmpipe.

#### Test 2: GLXGears
```bash
ssh -Y amdc glxgears
```

Should display spinning gears without crashes.

#### Test 3: RViz2
```bash
ssh -Y amdc 'source ~/.bash_aliases && rviz2'
```

Should launch without session crashes.

### Phase 4: VirtualGL + TurboVNC Fallback (If X11 Forwarding Remains Unstable)

If direct X11 forwarding still causes issues, use VirtualGL + TurboVNC for hardware-accelerated remote rendering.

#### Installation on amdc

1. **Install VirtualGL**:
   ```bash
   wget https://sourceforge.net/projects/virtualgl/files/3.1/virtualgl_3.1_amd64.deb
   sudo dpkg -i virtualgl_3.1_amd64.deb
   sudo apt-get install -f  # Fix any dependencies
   ```

2. **Install TurboVNC**:
   ```bash
   wget https://sourceforge.net/projects/turbovnc/files/3.1.1/turbovnc_3.1.1_amd64.deb
   sudo dpkg -i turbovnc_3.1.1_amd64.deb
   sudo apt-get install -f
   ```

3. **Configure VirtualGL**:
   ```bash
   sudo /opt/VirtualGL/bin/vglserver_config
   ```
   - Select option `1` (Configure server for use with VirtualGL)
   - Answer `Yes` to all prompts
   - Restart X server or reboot if prompted

#### Usage

**On amdc**:
1. Start TurboVNC server:
   ```bash
   /opt/TurboVNC/bin/vncserver :1 -geometry 1920x1080
   ```
   Note: Set your VNC password when prompted (first time only)

2. Check running VNC sessions:
   ```bash
   /opt/TurboVNC/bin/vncserver -list
   ```

**On sigyn7900a**:
1. Create SSH tunnel:
   ```bash
   ssh -L 5901:localhost:5901 amdc
   ```
   Keep this terminal open.

2. Connect with VNC client:
   ```bash
   # Install viewer if needed
   sudo apt-get install tigervnc-viewer
   
   # Connect
   vncviewer localhost:5901
   ```

3. In the VNC session, launch rviz2 with VirtualGL:
   ```bash
   source ~/.bash_aliases
   vglrun rviz2
   ```

**Stop VNC session** (on amdc):
```bash
/opt/TurboVNC/bin/vncserver -kill :1
```

## Helper Script

A convenience script has been created at:
```
/home/ros/sigyn_ws/src/Sigyn/scripts/fix_rviz_ssh_forwarding.sh
```

**Usage**:
```bash
# Interactive menu
./scripts/fix_rviz_ssh_forwarding.sh

# Or specific commands:
./scripts/fix_rviz_ssh_forwarding.sh check        # Check current driver
./scripts/fix_rviz_ssh_forwarding.sh available   # List available drivers
./scripts/fix_rviz_ssh_forwarding.sh upgrade 550 # Upgrade to driver 550
./scripts/fix_rviz_ssh_forwarding.sh validate    # Validate GLX config
./scripts/fix_rviz_ssh_forwarding.sh test        # Test rviz2 locally
./scripts/fix_rviz_ssh_forwarding.sh virtualgl   # Show VirtualGL setup
./scripts/fix_rviz_ssh_forwarding.sh ssh-test    # Show SSH test commands
```

## Quick Reference Commands

### Deploy bashrc changes to sigyn7900a:
```bash
scp ~/sigyn_ws/src/Sigyn/bashrc sigyn7900a:~/.bash_aliases
ssh sigyn7900a 'source ~/.bash_aliases'
```

### On amdc - Check and upgrade NVIDIA:
```bash
nvidia-smi                    # Check current version
ubuntu-drivers list           # Check available versions
sudo ubuntu-drivers install   # Install recommended driver
sudo reboot                   # Reboot required
```

### Test SSH X forwarding:
```bash
ssh -Y amdc 'glxinfo | grep renderer'
ssh -Y amdc glxgears
ssh -Y amdc 'source ~/.bash_aliases && rviz2'
```

### VirtualGL method (if X11 forwarding fails):
```bash
# On amdc:
/opt/TurboVNC/bin/vncserver :1

# On sigyn7900a:
ssh -L 5901:localhost:5901 amdc
vncviewer localhost:5901

# In VNC session:
vglrun rviz2
```

## Troubleshooting

### Issue: "Cannot open display"
- Ensure `ssh -Y` (not just `ssh -X`) is used for trusted X11 forwarding
- Check `echo $DISPLAY` in SSH session - should show something like `localhost:10.0`

### Issue: GLX still shows Mesa/llvmpipe
- Verify NVIDIA driver is active: `nvidia-smi`
- Check Xorg is using nvidia driver: `grep -i nvidia /var/log/Xorg.0.log`
- May need to disable Wayland and use X11 session

### Issue: rviz2 crashes immediately
- Check for conflicting OpenGL libraries: `ldconfig -p | grep libGL`
- Try forcing NVIDIA: `__GLX_VENDOR_LIBRARY_NAME=nvidia rviz2`
- Consider VirtualGL+TurboVNC as more stable alternative

### Issue: VNC connection refused
- Check VNC server is running: `/opt/TurboVNC/bin/vncserver -list`
- Verify port forwarding: `netstat -tln | grep 5901`
- Check firewall rules allow local port 5901

## Status Checklist

- [x] Phase 1: Remove Mesa forcing from sigyn7900a bashrc
- [ ] Phase 2: Update bashrc on sigyn7900a (deploy required)
- [ ] Phase 3: Check NVIDIA driver version on amdc
- [ ] Phase 4: Upgrade NVIDIA driver on amdc (if < 550)
- [ ] Phase 5: Reboot amdc
- [ ] Phase 6: Validate GLX with glxinfo
- [ ] Phase 7: Test SSH X forwarding with rviz2
- [ ] Phase 8: (Optional) Setup VirtualGL+TurboVNC if needed

## References

- [VirtualGL Documentation](https://virtualgl.org/Documentation/Documentation)
- [TurboVNC User Guide](https://turbovnc.org/Documentation/Documentation)
- [NVIDIA Linux Driver Archive](https://www.nvidia.com/en-us/drivers/unix/)
- [Ubuntu NVIDIA Driver Installation](https://ubuntu.com/server/docs/nvidia-drivers-installation)

---
**Last Updated**: 2026-02-21  
**Author**: GitHub Copilot  
**Related Files**: 
- [bashrc](../bashrc)
- [fix_rviz_ssh_forwarding.sh](../scripts/fix_rviz_ssh_forwarding.sh)
