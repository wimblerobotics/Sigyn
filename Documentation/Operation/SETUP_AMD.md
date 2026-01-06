# Instructions for Setting Up VS Code on an AMD64 Machine

Follow these steps on your AMD64-based Linux machines to ensure VS Code and IntelliSense work correctly with this workspace.

This process configures your environment to use a settings file specific to the AMD64 architecture, resolving issues with hard-coded paths for Python and C++ libraries.

## Step 1: Create the `settings_linux_amd.json` File

The `settings_macbook.json` file is configured for the ARM-based virtual machine. You need to create a parallel file for your AMD machines.

1.  Create a new file named `settings_linux_amd.json` inside the `.vscode` directory:
    ```bash
    touch .vscode/settings_linux_amd.json
    ```

2.  Copy the contents of `settings_macbook.json` into this new file.

3.  **Crucially**, you must now edit `settings_linux_amd.json` and adjust any paths to match your AMD machine's environment. The most important settings to check are:
    *   `"python.autoComplete.extraPaths"`
    *   `"python.analysis.extraPaths"`
    *   `"C_Cpp.default.includePath"`

    These paths should point to your ROS 2 installation (e.g., `/opt/ros/jazzy/...`) and your workspace's `install` directory (e.g., `/home/ros/sigyn_ws/install/...`). Make sure these paths are correct for your AMD machine's setup.

## Step 2: Make the Switch Script Executable

A script named `switch_settings.sh` exists in the `.vscode` directory. This script automatically detects your machine's architecture and links the correct settings file.

Make it executable by running this command from the root of the `Sigyn` repository:

```bash
chmod +x .vscode/switch_settings.sh
```

You only need to do this once on each machine.

## Step 3: Run the Switch Script

Execute the script to configure VS Code:

```bash
./.vscode/switch_settings.sh
```

The script will detect that you are on an `x86_64` machine and create a symbolic link from `settings_linux_amd.json` to `settings.json`.

## Step 4: Restart VS Code

Close and reopen VS Code to ensure it loads the new settings. Your IntelliSense errors should now be resolved.

You can now pull changes from Git without worrying about overwriting machine-specific settings, as the `settings.json` file is now a symbolic link managed by the script.
