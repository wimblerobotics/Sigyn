#!/bin/bash
# Resets USB hubs where Teensy might be connected

# Function to reset a device by ID
reset_by_id() {
    local id=$1
    # Handle multiple instances of the same hub
    lsusb | grep "$id" | while read -r line ; do
        local bus=$(echo "$line" | awk '{print $2}')
        local dev=$(echo "$line" | awk '{print $4}' | sed 's/://')
        local path="/dev/bus/usb/$bus/$dev"
        
        if [ -e "$path" ]; then
            echo "Resetting $id at $path"
            sudo /usr/bin/usbreset "$path"
        fi
    done
}

echo "Resetting USB Hubs..."

# Realtek Hubs (often used in expansion boards)
reset_by_id "0bda:5411"
reset_by_id "0bda:0411"

# Genesys Logic Hub
reset_by_id "05e3:0608"

echo "Done resetting hubs. Please check 'lsusb' to see if the Teensy has reappeared."
