#!/bin/bash
# Setup DDS configuration for Sigyn Robot
# This script installs the CycloneDDS configuration file

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONEDDS_XML="$SCRIPT_DIR/cyclonedds.xml"
TARGET_DIR="$HOME/.ros"
TARGET_FILE="$TARGET_DIR/cyclonedds.xml"

echo "Setting up CycloneDDS configuration..."

# Create ~/.ros directory if it doesn't exist
if [ ! -d "$TARGET_DIR" ]; then
    echo "Creating $TARGET_DIR directory..."
    mkdir -p "$TARGET_DIR"
fi

# Copy the cyclonedds.xml file
if [ -f "$CYCLONEDDS_XML" ]; then
    echo "Copying cyclonedds.xml to $TARGET_FILE..."
    cp "$CYCLONEDDS_XML" "$TARGET_FILE"
    echo "CycloneDDS configuration installed successfully."
else
    echo "ERROR: Source file $CYCLONEDDS_XML not found!"
    exit 1
fi

# Verify the environment variable is set
if grep -q "CYCLONEDDS_URI" ~/.bash_aliases 2>/dev/null || grep -q "CYCLONEDDS_URI" ~/.sigyn_* 2>/dev/null; then
    echo "CYCLONEDDS_URI environment variable is already configured."
else
    echo ""
    echo "WARNING: CYCLONEDDS_URI environment variable not found in bash configuration."
    echo "Please ensure your .bashrc or .bash_aliases includes:"
    echo "  export CYCLONEDDS_URI=file://\$HOME/.ros/cyclonedds.xml"
fi

echo ""
echo "Setup complete! Please restart your shell or run:"
echo "  source ~/.bashrc"
