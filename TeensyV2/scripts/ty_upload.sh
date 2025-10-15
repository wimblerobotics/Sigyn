#!/usr/bin/env bash
# Deterministic Teensy uploader using TyTools (tycmd)
# Usage: ty_upload.sh <usb-serial> <firmware.hex>
set -euo pipefail

SERIAL="$1"
HEX="$2"

if ! command -v tycmd >/dev/null 2>&1; then
  echo "ERROR: tycmd not found. Install TyTools (tycmd). See: https://koromix.dev/tytools#install" >&2
  exit 1
fi

if [ ! -f "$HEX" ]; then
  echo "ERROR: Firmware file not found: $HEX" >&2
  exit 1
fi

# Show current devices for debugging
echo "[ty_upload] Target serial: $SERIAL"
 echo "[ty_upload] Connected Teensy devices:"
 tycmd list --output json || true

# Try a soft reboot to bootloader for the specific board
# If the board isn't enumerated, this will fail silently; we rely on --wait below
set +e
 tycmd reset -b --board "$SERIAL" >/dev/null 2>&1
set -e

# Upload deterministically to the specific serial, waiting for bootloader
# tycmd ensures the .hex model matches the board and will not upload to others
exec tycmd upload --board "$SERIAL" --wait "$HEX"
