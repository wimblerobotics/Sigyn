# TeensyV2 PlatformIO Setup Guide

This guide explains how to build and upload the TeensyV2 firmware using PlatformIO, specifically configured for the Sigyn robot's three-board architecture:
- **Board 1 (board1)**: Navigation & Safety
- **Board 2 (board2)**: Power & Sensors
- **Board 3 (elevator_board)**: Elevator/Gripper

## Prerequisites

1. **Install PlatformIO Extension for VS Code:**
   - Open VS Code
   - Go to Extensions (Ctrl+Shift+X)
   - Search for "PlatformIO IDE"
   - Install the official PlatformIO IDE extension

2. **Install Teensy CLI Tools:**
   ```bash
   sudo apt-get update
   sudo apt-get install teensy-loader-cli
   ```

3. **Install Udev Rules:**
   Ensure the custom udev rules are installed to distinguish between the multiple Teensy boards.

## Multi-Board Udev Configuration

Since Sigyn uses multiple Teensy 4.1 boards connected simultaneously, we use udev rules to assign persistent symlinks based on the unique serial number of each Teensy.

**File:** `/etc/udev/rules.d/00-teensy.rules` (or linked from `src/Sigyn/udev/00-teensy.rules`)

```udev
# Rule for Teensy HalfKay Bootloader (Program Mode)
SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0478", MODE:="0666"

# Rules for Teensy 4.1 devices (Serial Mode)
# Board 1: Navigation & Safety
KERNEL=="ttyACM*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="17055770", MODE:="0666", SYMLINK+="teensy_sensor", RUN+="/bin/stty -F /dev/%k raw -echo"

# Board 2: Power & Sensors
KERNEL=="ttyACM*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="16482570", MODE:="0666", SYMLINK+="teensy_sensor2", RUN+="/bin/stty -F /dev/%k raw -echo"

# Elevator/Gripper Board
KERNEL=="ttyACM*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="16483110", MODE:="0666", SYMLINK+="teensy_gripper", RUN+="/bin/stty -F /dev/%k raw -echo"
```

**To apply changes:**
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Command Line Workflow & Aliases

We use custom bash functions to safely upload to the correct board. These functions prevent accidentally uploading firmware to the wrong Teensy by verifying the device path before uploading.

### Setup
Add the following to your `~/.bashrc`:

```bash
function buildBoard1 {
    local port="/dev/teensy_sensor"
    if [ -e "$port" ]; then
        platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port "$port"
    elif lsusb -d 16c0:0478 > /dev/null; then
        echo "WARN: $port not found, but Teensy Bootloader detected. Attempting upload via auto-detect..."
        platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port auto
    else
        echo "ERROR: $port not present and no Teensy Bootloader found." >&2
        echo "Hint: power the navigation/safety Teensy or check udev rules." >&2
        return 2
    fi
}

function buildBoard2 {
    local port="/dev/teensy_sensor2"
    if [ -e "$port" ]; then
        platformio run -e board2 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port "$port"
    elif lsusb -d 16c0:0478 > /dev/null; then
        echo "WARN: $port not found, but Teensy Bootloader detected. Attempting upload via auto-detect..."
        platformio run -e board2 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port auto
    else
        echo "ERROR: $port not present and no Teensy Bootloader found." >&2
        return 2
    fi
}

function buildElevator {
    local port="/dev/teensy_gripper"
    if [ -e "$port" ]; then
        platformio run -e elevator_board -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port "$port"
    elif lsusb -d 16c0:0478 > /dev/null; then
        echo "WARN: $port not found, but Teensy Bootloader detected. Attempting upload via auto-detect..."
        platformio run -e elevator_board -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload --upload-port auto
    else
        echo "ERROR: $port not present and no Teensy Bootloader found." >&2
        return 2
    fi
}
```

### Usage

```bash
# Build and upload to Board 1
buildBoard1

# Build and upload to Board 2
buildBoard2

# Build and upload to Elevator Board
buildElevator
```

## Known Behavior: The "Double Upload"

You may observe that the **first upload attempt fails**, but the **second attempt succeeds**. This is expected behavior in some scenarios.

**Why this happens:**
1.  **Attempt 1:** The script finds `/dev/teensy_sensor` and tells PlatformIO to upload to that specific port. The Teensy loader sends a reboot command. The Teensy reboots into **Bootloader Mode**.
    *   *Crucially:* In Bootloader Mode, the device ID changes (to `16c0:0478`), and the `/dev/teensy_sensor` symlink **disappears**.
    *   The upload tool tries to write to the port that just vanished, causing a "Permission denied" or "Error opening USB device" error.
2.  **State:** The Teensy is now sitting in Bootloader Mode (waiting for firmware).
3.  **Attempt 2:** You run `buildBoard1` again.
    *   The script sees that `/dev/teensy_sensor` is missing.
    *   However, it detects the Bootloader device (`16c0:0478`) via `lsusb`.
    *   It switches to `--upload-port auto`, which allows the Teensy Loader to find the waiting device and successfully upload the firmware.

**Resolution:**
Simply run the command again.

## Troubleshooting

**Device not found / "Hung" in Program Mode:**
If a board is stuck in bootloader mode (Red LED might be pulsing or off, Orange LED not blinking) and the symlink is gone:
1.  Run the build command again (it should catch it in bootloader mode).
2.  If that fails, hold the button on the Teensy for 15 seconds to perform a factory reset (restores the Blink program).

**USB Hub Issues:**
If devices disappear completely from `lsusb`, the USB hub might need resetting:
```bash
sudo ~/sigyn_ws/src/Sigyn/scripts/reset_teensy_hubs.sh
```
