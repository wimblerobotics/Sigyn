# CycloneDDS Configuration Issue - Root Cause Analysis

**Date:** 2026-08-04  
**Issue:** Intermittent image transmission failures between sigyn7900a and amdc  
**Status:** FIXED on sigyn7900a, NEEDS FIX on amdc  

---

> ## ⚠️ CORRECTION (later session, same date range) — the 16MB fix below was WRONG
>
> The `MaxMessageSize=16 MB` fix documented in this file was itself a bug that
> caused **total, 100% failure** of every large topic (rgb_image,
> annotated_image, depth_image, points) — not just intermittent failures —
> even on the **same machine** with no network involved. It was diagnosed via
> `ddsi_udp_conn_write ... failed with retcode -58` in the node log.
>
> `-58` = `DDS_RETCODE_NOT_ENOUGH_SPACE`, CycloneDDS's mapping of the POSIX
> `EMSGSIZE` errno. `General/MaxMessageSize` is **not** "max sample/image
> size" — per CycloneDDS's own docs it's *"the maximum size of the **UDP
> payload** Cyclone DDS will generate"*, i.e. how much it will try to pack
> into a single `sendmsg()` call. Setting it to 16MB made Cyclone try to
> write single UDP datagrams far beyond what a real UDP/IP stack allows,
> which the kernel rejects outright with EMSGSIZE for every large message.
>
> The correct fix is the opposite: keep `MaxMessageSize` **below the path
> MTU** (1500 on this Ethernet/WiFi network), e.g. **`1470 B`**, so Cyclone
> never triggers OS-level IP fragmentation (which WiFi drops reliably) and
> instead relies entirely on its own DDSI-level `FragmentSize` (1200B)
> fragmentation with proper reassembly/retransmission. `Internal/MaxSampleSize`
> (default effectively unlimited, ~2GB) is the setting that actually governs
> the max size of a sample CycloneDDS will forward/reassemble — that one was
> already fine at its default and never needed to change.
>
> Applied and verified on both `sigyn7900a` and `amdc`
> (`~/.ros/cyclonedds.xml`, `MaxMessageSize` = `1470 B`): confirmed
> `ros2 topic hz` on `rgb_image`/`annotated_image`/`depth_image` works both
> locally and cross-machine over WiFi with zero retcode -58 errors. See
> `/memories/repo/cyclonedds_maxmessagesize_root_cause.md` for the full
> investigation notes. `scripts/fix_cyclonedds_amdc.sh` and
> `scripts/diagnose_cyclonedds.sh` have been updated to reflect this.
>
> The rest of this document (below) is kept for historical context but its
> "16MB" recommendation should NOT be followed.

---

## Root Cause

**Configuration conflict with critically undersized MaxMessageSize parameter.**

### The Problem

Two CycloneDDS configuration files existed on sigyn7900a:

1. **`/etc/cyclonedds.xml`** (system-wide, older)
   - MaxMessageSize: **4MB**
   - 6 peer addresses (including obsolete 192.168.12.x)
   - Last modified: July 26

2. **`~/.ros/cyclonedds.xml`** (user-specific, active)
   - MaxMessageSize: **4KB** ← **CRITICAL BUG**
   - 2 peer addresses (192.168.86.28, 192.168.86.109)
   - Last modified: July 31

### Why This Caused Failures

**Image data size:** 1920×1080×3 = **6,220,800 bytes (~6MB)**

With `MaxMessageSize=4096B`:
- Requires **~1,500 fragments** per image
- Each fragment must be:
  - Transmitted successfully over WiFi
  - Received in order
  - Reassembled correctly
- **One lost fragment = entire image lost**
- WiFi packet loss (even 0.1%) makes this extremely unreliable

### Symptoms Explained

1. **Intermittent failures** - WiFi conditions vary, sometimes 1500 fragments succeed, sometimes they don't
2. **Works on local machine** - No network, no fragmentation issues
3. **Starts working then stops** - Network congestion or DDS reassembly buffer exhaustion
4. **QoS changes had no effect** - Problem was message size, not QoS policy
5. **Message count freezes at 5** - rviz2 received 5 images then hit fragment reassembly failures

---

## The Fix

### On sigyn7900a (COMPLETED)

```bash
# 1. Updated MaxMessageSize in ~/.ros/cyclonedds.xml
#    Changed: 4096 B → 16 MB

# 2. Removed conflicting system-wide config
sudo mv /etc/cyclonedds.xml /etc/cyclonedds.xml.backup_20260804

# 3. Restarted ROS 2 daemon
ros2 daemon stop
ros2 daemon start
```

**Result:** Image topic publishing at 8Hz, local reception confirmed working.

### On amdc (TODO)

**You must apply the same fix on amdc.** Run this script on the amdc machine:

```bash
# Copy the script to amdc (if needed)
scp ~/sigyn_ws/src/Sigyn/scripts/fix_cyclonedds_amdc.sh ros@amdc:~/

# On amdc, run:
cd ~
./fix_cyclonedds_amdc.sh
```

The script will:
- Auto-detect your Ethernet interface
- Backup any existing configs
- Create new config with MaxMessageSize=16MB
- Update ~/.bashrc to set CYCLONEDDS_URI
- Restart ROS 2 daemon

**IMPORTANT:** After running the script, restart rviz2 for changes to take effect.

---

## Verification

### Run Diagnostics (on both machines)

```bash
~/sigyn_ws/src/Sigyn/scripts/diagnose_cyclonedds.sh
```

### Expected Output

```
MaxMessageSize: 16MB  ← Should be 16MB, not 4KB or 4MB
FragmentSize: 1200B
Peer count: 2
  Peer address="192.168.86.28"   (amdc)
  Peer address="192.168.86.109"  (sigyn7900a)

⚠ NO conflicts should be reported
```

### Test Image Reception on amdc

After applying fix and restarting rviz2:

```bash
# Should show steady 8Hz rate
ros2 topic hz /oakd_apriltag_node/annotated_image

# Should receive images continuously
ros2 run rqt_image_view rqt_image_view /oakd_apriltag_node/annotated_image
```

---

## Why Two Configs Existed

Likely timeline:
1. Original setup used `/etc/cyclonedds.xml` (system-wide)
2. Later, per-robot configs created in `~/.ros/cyclonedds.xml`
3. `CYCLONEDDS_URI` env var pointed to user config
4. User config accidentally set 4KB instead of 4MB (typo: `4096 B` vs `4 MB`)
5. System config wasn't removed, causing confusion

---

## Related Issues

### SSH and VS Code Remote

You mentioned ROS 2 commands don't work in:
- SSH sessions with X forwarding (`ssh -YC ros@sigyn7900a`)
- VS Code remote terminals

**Root cause (likely):** These sessions may not inherit environment variables from `.bashrc` properly:
- `CYCLONEDDS_URI` not set → uses system default
- `ROS_DOMAIN_ID` not set → wrong domain
- `RMW_IMPLEMENTATION` not set → might not use CycloneDDS

**Solution:**
1. Check environment in problematic sessions:
   ```bash
   env | grep -E "ROS_|CYCLONE|RMW"
   ```

2. If variables missing, explicitly source:
   ```bash
   source ~/.bashrc
   # Or directly:
   source ~/.sigyn_sigyn7900a
   ```

3. For VS Code remote, ensure `.bashrc` is sourced for non-interactive shells:
   ```bash
   # Add to very top of ~/.bashrc
   # Source global definitions
   if [ -f /etc/bashrc ]; then
       . /etc/bashrc
   fi
   ```

### ROS 2 Daemon Not Working in SSH

The daemon runs per-user, per-display. SSH sessions might:
- Not see the daemon from local terminal
- Try to start their own daemon with wrong env vars

**Solution:**
```bash
# In SSH session, explicitly use existing daemon
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Test
ros2 topic list
```

---

## Technical Details

### DDS Fragmentation Math

**Original (broken) config:**
- Message: 6,220,800 bytes
- MaxMessageSize: 4,096 bytes
- Fragments per message: 6,220,800 / 1,200 ≈ **5,184 fragments**
- UDP packets required: 5,184+
- Probability of success (assuming 0.1% packet loss): (0.999)^5184 ≈ **0.5%**

**Fixed config:**
- Message: 6,220,800 bytes  
- MaxMessageSize: 16,777,216 bytes
- Message fits in **1 logical unit**
- Fragments per message: 6,220,800 / 1,200 ≈ **5,184** (same physical packets)
- But DDS handles reassembly with 16MB buffer, much more reliable

The key difference: DDS internal fragmentation handling vs trying to fit through a 4KB aperture.

---

## Files Created

1. **`/home/ros/sigyn_ws/src/Sigyn/scripts/diagnose_cyclonedds.sh`**
   - Diagnostic tool for CycloneDDS configuration
   - Run on both machines to verify setup

2. **`/home/ros/sigyn_ws/src/Sigyn/scripts/fix_cyclonedds_amdc.sh`**
   - Automated fix script for amdc
   - Auto-detects network interface
   - Creates unified configuration

3. **`/home/ros/sigyn_ws/src/Sigyn/config/cyclonedds_unified.xml`**
   - Template configuration
   - Reference for future machines

4. **`~/.ros/cyclonedds.xml`** (updated on sigyn7900a)
   - MaxMessageSize: 16MB
   - Optimized for WiFi image transmission

---

## Lessons Learned

1. **MaxMessageSize must exceed largest expected message**
   - Images: 6MB minimum
   - Use 16MB for safety margin

2. **One configuration source**
   - Remove `/etc/cyclonedds.xml` to avoid conflicts
   - Use user-specific `~/.ros/cyclonedds.xml`
   - Always set `CYCLONEDDS_URI` explicitly

3. **Fragmentation is expensive**
   - Keep FragmentSize at 1200B for WiFi MTU
   - But ensure MaxMessageSize allows efficient reassembly

4. **Test tools are essential**
   - `diagnose_cyclonedds.sh` catches configuration issues
   - `ros2 topic hz` reveals intermittent failures
   - `ros2 topic info --verbose` shows QoS mismatches

5. **Environment variable propagation**
   - SSH sessions need explicit sourcing
   - VS Code remote needs proper `.bashrc` setup
   - Daemons inherit env from parent shell

---

## Next Steps

1. **Apply fix on amdc** (CRITICAL)
   ```bash
   # On amdc:
   ./fix_cyclonedds_amdc.sh
   ```

2. **Restart rviz2 on amdc**
   ```bash
   killall rviz2
   ros2 run rviz2 rviz2
   ```

3. **Verify image reception**
   ```bash
   ros2 topic hz /oakd_apriltag_node/annotated_image
   # Should show stable 8Hz
   ```

4. **Run Test 3**
   - Network issues should be resolved
   - Image data will be reliably captured in rosbag
   - rviz2 monitoring will work continuously

5. **Document for future machines**
   - Use `diagnose_cyclonedds.sh` on new setup
   - Apply unified config from template
   - Set `CYCLONEDDS_URI` in robot setup scripts
