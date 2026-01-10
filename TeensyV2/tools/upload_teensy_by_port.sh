#!/usr/bin/env bash
set -euo pipefail

# Deterministic Teensy upload helper for multi-board setups.
#
# PlatformIO's default Teensy flow (teensy_reboot without an explicit port,
# then teensy_loader_cli) can reboot the wrong board when multiple Teensys are
# connected. This script forces the reboot to a specific serial port and then
# uploads, while refusing to run when the HalfKay state is ambiguous.

debug="${TEENSY_UPLOAD_DEBUG:-0}"
halfkay_settle_s="${TEENSY_HALFKAY_SETTLE_S:-0.5}"

print_port_debug() {
  local port_raw="$1"
  local port_resolved="$2"

  echo "[INFO] upload_port raw:     $port_raw"
  echo "[INFO] upload_port real:    $port_resolved"

  if [[ -e "$port_raw" ]]; then
    ls -l "$port_raw" || true
  fi
  if [[ "$port_resolved" != "$port_raw" && -e "$port_resolved" ]]; then
    ls -l "$port_resolved" || true
  fi

  if command -v udevadm >/dev/null 2>&1 && [[ -e "$port_resolved" ]]; then
    # Keep this concise; full dumps are noisy during PIO builds.
    udevadm info -q property -n "$port_resolved" 2>/dev/null | grep -E '^(DEVPATH|ID_PATH|ID_SERIAL_SHORT|ID_VENDOR_ID|ID_MODEL_ID)=' || true
  fi
}

wait_for_halfkay_permissions() {
  # HalfKay enumerates as USB HID 16c0:0478. Immediately after reboot, udev may
  # not have applied the MODE rules yet; wait a short time for device-node perms.
  local timeout_s="${1:-3}"
  local deadline
  deadline=$((SECONDS + timeout_s))

  local hidraw_ok=false
  local usbnode_ok=false

  while (( SECONDS <= deadline )); do
    hidraw_ok=false
    usbnode_ok=false

    # Prefer checking the actual hidraw node by walking sysfs. This avoids
    # depending on udev property names, which can vary.
    for dev in /dev/hidraw*; do
      [[ -e "$dev" ]] || continue
      local hidbase sysdev
      hidbase="$(basename "$dev")"
      sysdev="/sys/class/hidraw/${hidbase}/device"
      if [[ -e "$sysdev" ]]; then
        local cur vid pid
        cur="$(readlink -f "$sysdev" 2>/dev/null || true)"
        vid=""
        pid=""

        # Walk up to find idVendor/idProduct.
        while [[ -n "$cur" && "$cur" != "/" ]]; do
          if [[ -z "$vid" && -r "$cur/idVendor" ]]; then
            vid="$(tr '[:upper:]' '[:lower:]' <"$cur/idVendor" 2>/dev/null || true)"
          fi
          if [[ -z "$pid" && -r "$cur/idProduct" ]]; then
            pid="$(tr '[:upper:]' '[:lower:]' <"$cur/idProduct" 2>/dev/null || true)"
          fi
          if [[ -n "$vid" && -n "$pid" ]]; then
            break
          fi
          cur="$(dirname "$cur")"
        done

        if [[ "$vid" == "16c0" && "$pid" == "0478" ]]; then
          if [[ -r "$dev" && -w "$dev" ]]; then
            hidraw_ok=true
            break
          fi
        fi
      fi
    done

    # Also check /dev/bus/usb permissions for the exact Bus/Device of HalfKay.
    # teensy_loader_cli may use libusb access depending on build.
    local bus_dev
    bus_dev="$(lsusb -d 16c0:0478 2>/dev/null | head -n 1 | sed -n 's/^Bus \([0-9]\+\) Device \([0-9]\+\):.*/\1 \2/p')"
    if [[ -n "$bus_dev" ]]; then
      local bus devnum node
      bus="$(awk '{print $1}' <<<"$bus_dev")"
      devnum="$(awk '{print $2}' <<<"$bus_dev")"

      # lsusb emits zero-padded numbers (e.g. "059"). In bash arithmetic/printf,
      # leading zeros can be treated as octal, so force base-10.
      bus="$((10#$bus))"
      devnum="$((10#$devnum))"

      node="/dev/bus/usb/$(printf '%03d' "$bus")/$(printf '%03d' "$devnum")"
      if [[ -e "$node" && -r "$node" && -w "$node" ]]; then
        usbnode_ok=true
      fi
    fi

    if [[ "$hidraw_ok" == true || "$usbnode_ok" == true ]]; then
      return 0
    fi

    sleep 0.05
  done

  return 1
}

usage() {
  cat <<'EOF'
Usage:
  upload_teensy_by_port.sh <upload_port> <mcu> <firmware.hex>

Arguments:
  upload_port   A device path (e.g. /dev/teensy_sensor2 or /dev/ttyACM3)
  mcu           Teensy MCU id used by teensy_loader_cli (e.g. IMXRT1062)
  firmware.hex  Path to the firmware hex to upload
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" || $# -ne 3 ]]; then
  usage
  exit 2
fi

upload_port_raw="$1"
mcu="$2"
hex="$3"

if [[ ! -e "$hex" ]]; then
  echo "[ERROR] Firmware file not found: $hex" >&2
  exit 2
fi

# Count HalfKay devices currently present. When multiple Teensys are connected,
# teensy_loader_cli cannot choose which HalfKay to program.
halfkay_count="$(lsusb -d 16c0:0478 2>/dev/null | wc -l | tr -d ' ' || true)"

# Resolve symlinks so the reboot is tied to a concrete ttyACM* (if present).
upload_port_resolved="$upload_port_raw"
if [[ -e "$upload_port_raw" ]]; then
  upload_port_resolved="$(readlink -f "$upload_port_raw" 2>/dev/null || echo "$upload_port_raw")"
fi

port_exists=false
if [[ -e "$upload_port_resolved" ]]; then
  port_exists=true
fi

echo "[INFO] Target upload_port: $upload_port_raw"
echo "[INFO] Resolved port:      $upload_port_resolved"
echo "[INFO] HalfKay count:      $halfkay_count"

print_port_debug "$upload_port_raw" "$upload_port_resolved"

if [[ "$debug" == "1" ]] && command -v teensy_ports >/dev/null 2>&1; then
  teensy_ports || true
fi

if [[ "$halfkay_count" -gt 1 ]]; then
  echo "[ERROR] Multiple HalfKay bootloaders present (16c0:0478)." >&2
  echo "[ERROR] Refusing to upload because teensy_loader_cli cannot target a specific HalfKay." >&2
  echo "[HINT] Let all but the target Teensy boot normally (or unplug others), then retry." >&2
  exit 3
fi

if [[ "$halfkay_count" -eq 1 && "$port_exists" == true ]]; then
  echo "[ERROR] A HalfKay bootloader is already present, but the target serial port still exists." >&2
  echo "[ERROR] Upload would be ambiguous and could hit the wrong board." >&2
  echo "[HINT] Reset the other Teensy out of HalfKay (or unplug it), then retry." >&2
  exit 3
fi

if [[ "$halfkay_count" -eq 0 && "$port_exists" == false ]]; then
  echo "[ERROR] Target serial port not present and no HalfKay found." >&2
  echo "[HINT] Is the board unplugged or in bootloader without permissions?" >&2
  exit 2
fi

if [[ "$halfkay_count" -eq 0 ]]; then
  echo "[INFO] Rebooting Teensy via: $upload_port_resolved"
  # IMPORTANT: passing the port makes the reboot deterministic.
  teensy_reboot -s "$upload_port_resolved" || {
    echo "[ERROR] teensy_reboot failed for port: $upload_port_resolved" >&2
    exit 4
  }

  # Wait briefly for HalfKay to enumerate.
  for _ in $(seq 1 60); do
    if lsusb -d 16c0:0478 >/dev/null 2>&1; then
      break
    fi
    sleep 0.05
  done

  if ! lsusb -d 16c0:0478 >/dev/null 2>&1; then
    echo "[ERROR] Timed out waiting for HalfKay bootloader after reboot." >&2
    exit 5
  fi
else
  echo "[INFO] HalfKay already present; skipping reboot step."
fi

if ! wait_for_halfkay_permissions 10; then
  echo "[ERROR] HalfKay present, but device-node permissions never became ready." >&2
  echo "[HINT] This is usually udev timing or missing udev rules for 16c0:0478 (hidraw/usb)." >&2
  echo "[HINT] Repo rules exist at: /home/ros/sigyn_ws/src/Sigyn/udev/00-teensy.rules" >&2
  echo "[HINT] If you just installed rules, run: sudo udevadm control --reload-rules && sudo udevadm trigger" >&2
  exit 6
fi

# Even after nodes are writable, HalfKay can be briefly present-but-not-ready.
# A short settle delay significantly reduces first-attempt "error writing" races.
if [[ "$halfkay_settle_s" != "0" ]]; then
  if [[ "$debug" == "1" ]]; then
    echo "[INFO] Settling HalfKay for ${halfkay_settle_s}s..." >&2
  fi
  sleep "$halfkay_settle_s"
fi

# Upload. Retry once to absorb udev/permission timing races.
set +e
teensy_loader_cli -mmcu="$mcu" -w -v "$hex"
rc=$?
if [[ $rc -ne 0 ]]; then
  echo "[WARN] Upload failed (rc=$rc). Retrying once after short delay..." >&2
  sleep 0.5
  teensy_loader_cli -mmcu="$mcu" -w -v "$hex"
  rc=$?
fi
set -e

exit $rc
