# VS Code Configuration for Sigyn

This directory contains VS Code workspace configuration for the Sigyn project.

## Machine-Specific Files (Local Only)

The following files are **NOT** tracked in git and should be created locally on each development machine:

### `settings.json`
Your local VS Code settings. Copy from one of the example files below and modify as needed.

### `c_cpp_properties.json`
C++ IntelliSense configuration specific to your system architecture and ROS installation.

### `settings_*.json` 
Architecture-specific settings files (if you want to maintain multiple configurations).

### `switch_settings.sh`
Script to switch between different settings configurations (optional).

## Setting Up IntelliSense

1. **For AMD64/x86_64 systems:**
   ```json
   // c_cpp_properties.json
   {
       "configurations": [
           {
               "name": "ROS2",
               "includePath": [
                   "${workspaceFolder}/**",
                   "/opt/ros/jazzy/include/**",
                   "/home/ros/sigyn_ws/install/*/include/**",
                   "/usr/include/**"
               ],
               "defines": [
                   "ROS_DISTRO_JAZZY",
                   "__linux__"
               ],
               "compilerPath": "/usr/bin/gcc",
               "cStandard": "c17",
               "cppStandard": "c++17",
               "intelliSenseMode": "linux-gcc-x64",
               "compileCommands": "${workspaceFolder}/build/compile_commands.json",
               "configurationProvider": "ms-vscode.cmake-tools"
           }
       ],
       "version": 4
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