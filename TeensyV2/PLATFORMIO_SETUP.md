# TeensyV2 PlatformIO Setup Guide

This guide explains how to build and upload the TeensyV2 firmware using PlatformIO instead of the Arduino IDE.

## Prerequisites

1. **Install PlatformIO Extension for VS Code:**
   - Open VS Code
   - Go to Extensions (Ctrl+Shift+X)
   - Search for "PlatformIO IDE"
   - Install the official PlatformIO IDE extension

2. **Install Teensy CLI Tools:**
   ```bash
   # Download and install teensy_loader_cli
   sudo apt-get update
   sudo apt-get install teensy-loader-cli
   ```

## Project Structure

```
TeensyV2/
├── platformio.ini          # PlatformIO configuration
├── library.json            # Library metadata
├── common/                  # Core system components
│   └── core/
│       ├── module.h/.cpp
│       └── serial_manager.h/.cpp
├── modules/                 # Feature modules
│   ├── performance/
│   ├── battery/
│   └── safety/
└── platform/               # Board-specific implementations
    ├── board1_main/
    │   └── board1_main.ino
    └── board2_main/
        └── board2_main.ino
```

## Building and Uploading

### Using VS Code PlatformIO Extension:

1. **Open Project:**
   - File → Open Folder → Select `TeensyV2/` directory
   - PlatformIO should automatically detect the project

2. **Build Options:**
   - **Board 1 (Navigation/Safety):** Click "Build" for `env:board1`
   - **Board 2 (Power/Sensors):** Click "Build" for `env:board2`
   - **Debug Builds:** Use `env:board1_debug` or `env:board2_debug`

3. **Upload:**
   - Connect Teensy 4.1 board via USB
   - Click "Upload" for the appropriate environment
   - Press the reset button on Teensy when prompted

### Using Command Line:

```bash
cd /home/ros/sigyn_ws/src/Sigyn/TeensyV2

# Build board 1 firmware
pio run -e board1

# Build board 2 firmware  
pio run -e board2

# Upload to board 1 (with auto-detect)
pio run -e board1 -t upload

# Upload to specific port
pio run -e board1 -t upload --upload-port /dev/ttyACM0

# Build debug version
pio run -e board1_debug

# Clean build
pio run -t clean

# Run unit tests
pio test -e test
```

## Environment Configurations

- **`board1`**: Production build for navigation/safety board
- **`board2`**: Production build for power/sensor board  
- **`board1_debug`**: Debug build with symbols and verbose logging
- **`board2_debug`**: Debug build with symbols and verbose logging
- **`test`**: Native unit testing environment

## Build Flags

Each board has specific feature flags enabled:

### Board 1 (Navigation/Safety):
- `BOARD_ID=1`
- `ENABLE_PERFORMANCE_MONITOR`
- `ENABLE_SAFETY_COORDINATOR`
- `ENABLE_MOTOR_CONTROL`
- `ENABLE_VL53L0X_SENSORS`

### Board 2 (Power/Sensors):
- `BOARD_ID=2`
- `ENABLE_PERFORMANCE_MONITOR`
- `ENABLE_BATTERY_MONITOR`
- `ENABLE_IMU_SENSORS`
- `ENABLE_TEMPERATURE_SENSORS`

## Serial Monitor

```bash
# Monitor serial output
pio device monitor

# Monitor with specific port and baud rate
pio device monitor --port /dev/ttyACM0 --baud 115200
```

## Debugging

PlatformIO supports hardware debugging with compatible debug probes:

```bash
# Start debug session
pio debug -e board1_debug
```

## Troubleshooting

1. **Include Path Issues:** PlatformIO automatically handles include paths defined in `platformio.ini`
2. **Library Dependencies:** Listed in `lib_deps` are automatically downloaded
3. **Board Detection:** Use `pio device list` to see connected devices
4. **Permission Issues:** Add user to `dialout` group: `sudo usermod -a -G dialout $USER`

## Advantages over Arduino IDE

- ✅ Proper dependency management
- ✅ Multiple build configurations
- ✅ Advanced debugging support
- ✅ Unit testing framework
- ✅ Better IntelliSense/code completion
- ✅ Version control friendly
- ✅ Command-line automation
- ✅ Professional development workflow
