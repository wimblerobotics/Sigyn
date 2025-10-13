#!/bin/bash
# This script switches the VS Code settings based on the machine's architecture.

# The directory where this script and the settings files are located
VSCODE_DIR="$(dirname "$0")"
SETTINGS_FILE="$VSCODE_DIR/settings.json"

# Determine the machine's architecture
ARCH=$(uname -m)

echo "Detected architecture: $ARCH"

# Remove existing symlink to avoid errors
if [ -L "$SETTINGS_FILE" ]; then
    echo "Removing existing settings.json symlink."
    rm "$SETTINGS_FILE"
elif [ -f "$SETTINGS_FILE" ]; then
    echo "Backing up existing settings.json file to settings.json.bak"
    mv "$SETTINGS_FILE" "$SETTINGS_FILE.bak"
fi

# Create a new symlink based on the architecture
if [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == "arm64" ]]; then
    echo "This is an ARM-based machine (like your virtual Linux environment on Apple Silicon)."
    echo "Linking settings_macbook.json to settings.json"
    ln -s "$VSCODE_DIR/settings_macbook.json" "$SETTINGS_FILE"
elif [[ "$ARCH" == "x86_64" ]]; then
    echo "This is an AMD64/x86_64 machine."
    echo "Linking settings_linux_amd.json to settings.json"
    # You would create a settings_linux_amd.json file for your other machines
    if [ -f "$VSCODE_DOR/settings_linux_amd.json" ]; then
        ln -s "$VSCODE_DIR/settings_linux_amd.json" "$SETTINGS_FILE"
    else
        echo "Warning: settings_linux_amd.json not found. No settings linked."
    fi
else
    echo "Unknown architecture: $ARCH. No settings linked."
fi

echo "Done."
