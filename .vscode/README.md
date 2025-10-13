# VS Code Configuration for Sigyn

This directory contains VS Code workspace configuration for the Sigyn project.

## Machine-Specific Files (Local Only)

The following files are **NOT** tracked in git and should be created locally on each development machine:

### `settings.json`
Your local VS Code settings. Copy from one of the example files below and modify as needed.

### `tasks.json`
Build and run tasks with machine-specific workspace paths. Copy from `tasks.json.template` and customize:
- Update workspace paths to match your system
- Update ROS version paths (/opt/ros/jazzy vs /opt/ros/humble)
- Add any custom tasks you need

### `c_cpp_properties.json`
IntelliSense configuration with include paths. Should contain:
- **ROS2** configuration using your local `compile_commands.json`
- **TeensyV2** configuration with Arduino/Teensy include paths

### `c_cpp_properties.json`
C++ IntelliSense configuration specific to your system architecture and ROS installation.

### `settings_*.json` 
Architecture-specific settings files (if you want to maintain multiple configurations).

### `switch_settings.sh`
Script to switch between different settings configurations (optional).

## Setting Up IntelliSense

### Steps to Set Up IntelliSense:

1. **Build the workspace to generate compile_commands.json:**
   ```bash
   cd /home/ros/sigyn_ws
   source /opt/ros/[YOUR_ROS_DISTRO]/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
   ```

2. **For PlatformIO/Arduino (TeensyV2) development:**
   ```bash
   cd TeensyV2
   pio run  # This generates TeensyV2/compile_commands.json
   ```

3. **Create your local `c_cpp_properties.json`:**

   **For AMD64/x86_64 systems:**
   ```json
   {
       "configurations": [
           {
               "name": "Arduino",
               "includePath": [
                   "${workspaceFolder}/TeensyV2/**",
                   "~/.platformio/packages/framework-arduinoteensy/**",
                   "~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/arm-none-eabi/include/**"
               ],
               "defines": [
                   "ARDUINO=10819",
                   "TEENSY41",
                   "__IMXRT1062__"
               ],
               "compilerPath": "~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-gcc",
               "intelliSenseMode": "gcc-arm",
               "compileCommands": "${workspaceFolder}/TeensyV2/compile_commands.json"
           },
           {
               "name": "ROS2",
               "includePath": [
                   "${workspaceFolder}/**",
                   "/opt/ros/jazzy/include/**",
                   "${workspaceFolder}/../install/*/include/**"
               ],
               "compilerPath": "/usr/bin/gcc",
               "intelliSenseMode": "linux-gcc-x64",
               "compileCommands": "${workspaceFolder}/build/compile_commands.json"
           }
       ]
   }
   ```

   **For ARM64 systems:** Change `intelliSenseMode` to `"linux-gcc-arm64"` and verify ROS paths.

4. **Create your local `settings.json`:**
   ```json
   {
       "C_Cpp.intelliSenseEngine": "default",
       "C_Cpp.configurationWarnings": "disabled",
       "files.associations": {
           "*.ino": "cpp"
       },
       "cmake.configureOnOpen": false
   }
   ```

2. **For ARM64 systems:** Change `intelliSenseMode` to `"linux-gcc-arm64"`

3. **For different ROS distributions:** Update include paths and defines accordingly

## Building for IntelliSense

To generate the `compile_commands.json` file needed for IntelliSense:

```bash
cd /home/ros/sigyn_ws
source /opt/ros/jazzy/setup.bash  # or your ROS distro
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Troubleshooting IntelliSense

If IntelliSense isn't working:

1. Verify `compile_commands.json` exists in `build/` directory
2. Check that ROS include paths match your installation
3. Make sure VS Code C++ extension (`ms-vscode.cpptools`) is installed
4. Reload VS Code window: Ctrl+Shift+P → "Developer: Reload Window"

## Files Tracked in Git

- `extensions.json` - Recommended extensions
- `keybindings.json` - Custom key bindings
- `launch.json` - Debug configurations  
- `tasks.json` - Build and run tasks
- `requirements.txt` - Python dependencies