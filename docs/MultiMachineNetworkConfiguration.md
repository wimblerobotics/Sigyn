# Multi-Machine Network Configuration for ROS 2 Jazzy on Ubuntu 24.04

This tutorial documents how the Sigyn robot network is configured across its three primary
computers and explains *why* each decision was made. It is written for ROS 2 developers who
are comfortable with Linux but may not have deep networking experience.

---

## 1. The Network Topology

```
Internet
   │
192.168.12.1  ← Google Nest mesh router (primary) + 4 x mesh repeaters
   │
   ├── wired Ethernet
   │      └── amdc  (192.168.12.140)   AMD Ryzen 9 7900X development desktop
   │
   └── Wi-Fi 802.11ax (Wi-Fi 6), 5 GHz band
          ├── sigyn7900a  (192.168.12.210)   Robot main computer — AMD Ryzen 9 7900X
          └── sigynVision (192.168.12.120)   Robot vision processor — Raspberry Pi 5
```

All three machines share the `192.168.12.0/24` subnet, served by the Google Nest mesh at
gateway `192.168.12.1`. There is no NAT between them — they communicate directly at LAN
speeds.

---

## 2. Why Static IPs Instead of DHCP

ROS 2 does not use zeroconf (mDNS) for inter-machine discovery in this setup. All hostnames
are resolved through `/etc/hosts` on each machine, and SSH configurations reference machines
by their hostnames. This means every machine **must** have a predictable, fixed IP address.

If a machine receives a different DHCP lease after a reboot, every other robot computer loses
the ability to reach it until `/etc/hosts` is manually updated everywhere — which is
impractical in a fleet.

**Rule:** every robot computer has a static IP assigned directly in its NetworkManager
connection profile (`ipv4.method = manual`). No DHCP reservations are used because that would
create a hidden dependency on the router's config surviving power cycles, firmware updates, or
router replacement.

---

## 3. The `/etc/hosts` File (All Machines)

Each machine carries an identical `/etc/hosts` file covering all robot computers:

```
127.0.0.1       localhost
127.0.1.1       <this-machine-hostname>

192.168.12.136  MiniMe4     MiniMe4    # development/test system
192.168.12.140  amdc        amdc       # AMD Ryzen 9 7900X desktop
192.168.12.210  sigyn7900a  SR         # Robot main computer
192.168.12.120  sigynVision SV         # Robot vision processor (RPi 5)
192.168.12.219  sigynNvidia1 SN1       # Jetson Orin Nano dev board
```

The short aliases (`SR`, `SV`, etc.) allow terse usage in scripts. When you add a new robot
computer, add its entry to **all** machines' `/etc/hosts`.

---

## 4. Machine-by-Machine Configuration

### 4.1 `amdc` — Development Desktop (Wired)

| Property | Value |
|---|---|
| Interface | `wlp9s0` (now **wired** via Ethernet) |
| IP | `192.168.12.140/24` |
| Gateway | `192.168.12.1` |
| Method | Static (`ipv4.method = manual`) |
| Role | iperf3 server, ROS 2 visualization, bag recording |

`amdc` is a desktop: it never moves, so Wi-Fi adds no benefit. Connecting it via Ethernet to
the nearest Google Nest point eliminates one wireless hop from the robot→desktop video path.
See §7 for why this matters.

### 4.2 `sigyn7900a` — Robot Main Computer (Wi-Fi)

| Property | Value |
|---|---|
| Interface | `wlp8s0` |
| Wi-Fi chip | MediaTek MT7922 (PCIe, `mt7921e` driver) |
| IP | `192.168.12.210/24` |
| Gateway | `192.168.12.1` |
| Band | 5 GHz only (`802-11-wireless.band = a`) |
| Channel | Any (`802-11-wireless.channel = 0`) |
| Powersave | Disabled (`802-11-wireless.powersave = 2`) |
| PMF | Optional on WPA2 mesh profiles (`802-11-wireless-security.pmf = 2`) |
| MAC randomization | Off (`mac-address-randomization = never`) |
| Background scan | `bgscan="simple:30:-65:8"` applied on reconnect via NM dispatcher |
| Route metric | 50 (lowest = preferred default route) |

### 4.3 `sigynVision` — Robot Vision Processor (Wi-Fi)

| Property | Value |
|---|---|
| Interface | `wlan0` |
| Wi-Fi chip | Broadcom CYW43 on-board (SDIO, `brcmfmac` driver) |
| IP | `192.168.12.120/24` |
| Gateway | `192.168.12.1` |
| Band | 5 GHz only (`802-11-wireless.band = a`) |
| Channel | Any (`802-11-wireless.channel = 0`) |
| Powersave | Disabled |
| PMF | Optional on the production mesh profile |
| MAC randomization | Off |
| Background scan | `bgscan="simple:30:-65:8"` applied on reconnect via NM dispatcher |
| Route metric | 50 |

`sigynVision` is a Raspberry Pi 5 mounted on the robot's gripper assembly. It is always
co-located with `sigyn7900a` and shares the same roaming requirements.

---

## 5. What Was Wrong (and Why It Had to Be Fixed)

The out-of-the-box Ubuntu 24.04 Wi-Fi configuration has several defaults that are fine for a
laptop but harmful for a robot:

### 5.1 Wi-Fi Powersave Was On

Ubuntu installs `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf` with
`wifi.powersave = 3`, which tells the driver to aggressively suspend the radio between packets.

**Effect on robots:** a sleeping radio introduces 50–200 ms of wake-up latency on every burst.
ROS 2 DDS (UDP) transport times out nodes, causes dropped TF frames, and makes rviz2
visualization stutter. Under high-throughput video streaming, the radio could not sustain the
required duty cycle and the connection dropped entirely.

**Fix:** `/etc/NetworkManager/conf.d/99-wifi-powersave-off.conf`
```ini
[connection]
wifi.powersave = 2
```
The `99-` prefix ensures this file is processed last and wins over the conflicting default.
The conflicting `default-wifi-powersave-on.conf` was deleted.

Additionally, each connection profile sets `802-11-wireless.powersave = 2` so the setting
survives even if the global conf.d files are reset.

### 5.2 PCIe ASPM Was Causing Link Resets (MT7922 Only)

The MT7922 chip on `sigyn7900a` uses a PCIe bus. Linux's PCIe Active State Power Management
(ASPM) was allowed to clock-gate the PCIe link between packets, which caused the driver to
lose sync with the firmware and reset — appearing as brief Wi-Fi disconnections.

**Fix:** `/etc/modprobe.d/mt7921e.conf`
```
options mt7921e disable_aspm=Y
```
This is applied at module load time. The Raspberry Pi's `brcmfmac` is SDIO-attached, not
PCIe, so this fix is not needed on `sigynVision`.

### 5.3 The Radio Was Sticking to 2.4 GHz

Without explicit band configuration, the NetworkManager profile had no band preference. The
device would associate with whichever BSSID responded first — often a 2.4 GHz radio (which
responds more aggressively). The 2.4 GHz band tops out at ~130 Mbit/s link rate versus
720–960 Mbit/s on 5 GHz with the same AP.

**Fix:** `802-11-wireless.band = a` in both robot profiles. This forces the supplicant to
only scan and associate on 5 GHz. The channel is left unpinned (`channel = 0`) so the robot
can roam to any 5 GHz channel the mesh uses.

### 5.4 MAC Address Randomization Was Breaking Roaming

Ubuntu randomizes the MAC address used during Wi-Fi scanning by default. Some firmware builds
do not distinguish scan MACs from association MACs cleanly; the mesh access points saw a
different MAC arriving than the one from the previous association and treated it as a new
client, resetting the session and dropping in-flight packets.

**Fix:** `mac-address-randomization = never` in both robot profiles and globally via
`/etc/NetworkManager/conf.d/20-wifi-scan-rand-off.conf`:
```ini
[device]
wifi.scan-rand-mac-address = no
```

### 5.5 Internet Traffic Was Routing Via the TP-Link Adapter, Not Wi-Fi

The robot previously used a USB/PCIe TP-Link Ethernet adapter (`eno1`) as its primary
network interface because the built-in Wi-Fi had poor throughput (caused by problems §5.1–5.4
above). After fixes were applied, `eno1` still held the default route (metric 100) while
`wlp8s0` had no default route at all, meaning all traffic was still going through the old path.

**Fix:** set `ipv4.route-metric = 50` on the Wi-Fi profile and `ipv4.never-default = yes` on
the `eno1` profile, then remove `eno1` from service. The Wi-Fi profile now owns the default
route.

### 5.6 The Desktop Was on Wi-Fi

`amdc` was connected to the mesh over Wi-Fi. Video streaming from `sigyn7900a` to `amdc` went:

```
sigyn7900a → (Wi-Fi) → Nest AP → (5 GHz mesh backhaul) → Nest AP → (Wi-Fi) → amdc
```

This means *two* over-the-air hops plus mesh backhaul contention. Measured throughput:
**126 Mbps** with 101 TCP retransmits in 10 seconds.

After connecting `amdc` via Ethernet:
```
sigyn7900a → (Wi-Fi) → Nest AP → (wired) → amdc
```

Measured throughput: **340 Mbps** — 2.7× improvement, retransmits dropped proportionally.
For ROS 2 DDS/UDP video topics, this translates directly to higher sustainable image rates and
lower latency.

---

## 6. Global NetworkManager Tuning Files

These files exist on **all Wi-Fi machines** in the fleet (`sigyn7900a`, `sigynVision`):

### `/etc/NetworkManager/conf.d/99-wifi-powersave-off.conf`
```ini
[connection]
wifi.powersave = 2
```

### `/etc/NetworkManager/conf.d/20-wifi-scan-rand-off.conf`
```ini
[device]
wifi.scan-rand-mac-address = no
```

### `/etc/NetworkManager/dispatcher.d/99-bgscan.sh`
This NetworkManager dispatcher script reapplies the `bgscan` policy each time a Wi-Fi
connection comes up or is reapplied. The apply happens after a short delay because
NetworkManager can still be rewriting supplicant state during the first few seconds of a
reconnect:

```sh
#!/bin/sh
set -eu

PATH=/usr/sbin:/usr/bin:/sbin:/bin
IFACE="${1:-}"
ACTION="${2:-}"

case "$ACTION" in
  up|dhcp4-change|reapply|connectivity-change)
    ;;
  *)
    exit 0
    ;;
esac

[ -n "$IFACE" ] || exit 0

(
   sleep 5
   network_id=$(wpa_cli -i "$IFACE" status 2>/dev/null | awk -F= '/^id=/{print $2; exit}')
   [ -n "$network_id" ] || exit 0

   wpa_cli -i "$IFACE" set_network "$network_id" bgscan '"simple:30:-65:8"' >/dev/null 2>&1 || exit 0
   wpa_cli -i "$IFACE" reassociate >/dev/null 2>&1 || true
   logger -t nm-bgscan "Applied bgscan simple:30:-65:8 to $IFACE network $network_id after $ACTION"
) &

exit 0
```

After creating or changing these files, reload NetworkManager:
```bash
sudo systemctl reload NetworkManager
```

---

## 7. Wi-Fi Roaming on Google Mesh

The house mesh uses Google Nest Wi-Fi points that already support 802.11v BSS Transition
Management. That means the mesh can tell the client which node it should move to when the
robot is moving through the house, as long as the client keeps its BSS table fresh and can
accept protected transition-management frames.

### The problem with the previous fix

An earlier AI-generated solution installed `wifi-roam-agent.service`, a systemd unit that ran
`/usr/local/sbin/wifi-roam-agent.sh` in a loop. That script called `wpa_cli scan` every
6 seconds to try to force faster roaming between mesh nodes.

That configuration was wrong for this environment for two reasons:

1. Each full scan takes the radio briefly off-channel, which drops in-flight packets and can
   create measurable LAN packet loss.
2. It fights Google Mesh's own 802.11v steering logic instead of cooperating with it.

On `sigynVision`, that aggressive scan loop was measured to cause about **10% packet loss**.
The service and script are therefore kept **disabled** and must not be re-enabled unless the
roaming design is revisited from first principles.

### Correct solution

The correct configuration for Google Mesh on Ubuntu + NetworkManager is:

1. Disable Wi-Fi powersave globally with `/etc/NetworkManager/conf.d/99-wifi-powersave-off.conf`
   using `wifi.powersave = 2`.
2. Disable scan MAC randomisation globally with
   `/etc/NetworkManager/conf.d/20-wifi-scan-rand-off.conf` using
   `wifi.scan-rand-mac-address = no`.
3. Set PMF (802.11w) to optional on the production Wi-Fi connection profiles. Google Mesh uses
   PMF-protected 802.11v BSS Transition frames, so the client must allow them. Some test
   profiles that use SAE may legitimately require PMF instead of allowing `optional`; that is
   acceptable and still compatible with the mesh.
4. Apply `bgscan "simple:30:-65:8"` via a NetworkManager dispatcher script
   (`/etc/NetworkManager/dispatcher.d/99-bgscan.sh`) each time the Wi-Fi connection comes up.
5. Apply the same `bgscan` setting immediately to the live session after installing the script.

`bgscan "simple:30:-65:8"` tells `wpa_supplicant` to perform low-impact background scanning on
a longer cadence during healthy signal conditions and more frequently when RSSI drops below
`-65 dBm`. Unlike an explicit `wpa_cli scan` loop, this keeps roaming information current
without repeatedly forcing disruptive full scans.

### Required disabled components

These legacy components may still exist on disk, but they must remain disabled:

```
/usr/local/sbin/wifi-roam-agent.sh
/etc/systemd/system/wifi-roam-agent.service
```

Check the disabled state with:

```bash
systemctl status wifi-roam-agent.service
```

---

## 8. Setting Up a New Robot Machine

When adding a computer to the fleet:

1. **Set a static IP** in the NetworkManager profile:
   ```bash
   nmcli con modify <profile-name> \
     ipv4.method manual \
     ipv4.addresses <NEW_IP>/24 \
     ipv4.gateway 192.168.12.1 \
     ipv4.dns "8.8.8.8 8.8.4.4" \
     ipv4.route-metric 50 \
     ipv4.never-default no
   ```

2. **Force 5 GHz, disable powersave and MAC randomization:**
   ```bash
   nmcli con modify <profile-name> \
     802-11-wireless.band a \
     802-11-wireless.powersave 2 \
     802-11-wireless.mac-address-randomization never \
     802-11-wireless-security.pmf 2
   ```

3. **Add the global NM conf.d files** (see §6).

4. **Delete the conflicting default powersave file** if present:
   ```bash
   sudo rm -f /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
   sudo systemctl reload NetworkManager
   ```

5. **Install the bgscan dispatcher script** and apply it to the live session:
   ```bash
   sudo install -m 755 99-bgscan.sh /etc/NetworkManager/dispatcher.d/99-bgscan.sh
   sudo /etc/NetworkManager/dispatcher.d/99-bgscan.sh <iface> reapply
   ```

6. **Ensure the legacy roaming service stays disabled:**
   ```bash
   sudo systemctl disable --now wifi-roam-agent.service
   ```

7. **Add the PCIe ASPM fix** if the machine uses the MT7922 or similar MediaTek PCIe chip:
   ```bash
   echo "options mt7921e disable_aspm=Y" | sudo tee /etc/modprobe.d/mt7921e.conf
   sudo update-initramfs -u
   ```

8. **Update `/etc/hosts`** on every machine in the fleet with the new entry.

9. **Apply and reconnect if needed:**
   ```bash
   nmcli con up <profile-name>
   ```

---

## 9. Benchmarking

Verify LAN throughput after setting up a machine. Run on `amdc` (wired):
```bash
iperf3 -s
```

Run on the robot:
```bash
iperf3 -c amdc -t 10 -P 4
```

Expected results at good signal (−50 dBm, MCS 8, 2×2 MIMO, 80 MHz):
- **Before fixes (typical):** 10–30 Mbps, many retransmits
- **After fixes:** 300–400 Mbps, few retransmits

If you see unexpectedly low throughput, check:
```bash
iw dev <iface> link          # look at tx/rx bitrate and MCS index
journalctl -t nm-bgscan -n 20   # recent bgscan applications
sudo wpa_cli -i <iface> get_network 0 bgscan
nmcli -f ACTIVE,BSSID,SSID,FREQ,SIGNAL,RATE dev wifi   # current AP
```

---

## 10. Quick Reference

```bash
# Check current Wi-Fi link quality
iw dev wlp8s0 link

# See all visible APs and their signal/band
nmcli dev wifi list ifname wlp8s0

# Show active connection profile
nmcli con show --active

# Force reconnect (picks up any profile changes)
nmcli con up livingroom

# Verify bgscan and legacy roam-agent state
sudo wpa_cli -i wlp8s0 get_network 0 bgscan
journalctl -t nm-bgscan -n 30
systemctl status wifi-roam-agent

# Verify static IP is applied
ip -4 addr show wlp8s0

# Verify default route goes via Wi-Fi
ip -4 route get 1.1.1.1
```
