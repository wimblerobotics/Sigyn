# Teensy SD Card Directory Service

## Overview
The `teensy_sd_getdir` service allows you to request directory listings from the Teensy's SD card storage.

## Service Definition
**Service Name:** `teensy_sd_getdir`
**Service Type:** `sigyn_interfaces/srv/TeensySdGetDir`

### Request
- `directory_path` (string): The directory path on the Teensy SD card to list (e.g., "/", "/logs", etc.)

### Response
- `directory_listing` (string): The directory listing result from the Teensy (pipe-delimited format)
- `success` (bool): Whether the operation was successful
- `error_message` (string): Error message if the operation failed

## Usage Examples

### Using ros2 service call
```bash
# List root directory
ros2 service call /teensy_sd_getdir sigyn_interfaces/srv/TeensySdGetDir "{directory_path: '/'}"

# List a specific directory
ros2 service call /teensy_sd_getdir sigyn_interfaces/srv/TeensySdGetDir "{directory_path: '/logs'}"
```

### Expected Response Format
The `directory_listing` field will contain a pipe-delimited string of filenames, like:
```
SDIR: | LOG00053.TXT | .Trashes | Total files: 57
```

## Prerequisites
1. **Teensy Bridge Node**: The `teensy_bridge` node must be running
2. **Serial Connection**: The Teensy must be connected via USB serial (/dev/teensy_sensor)
3. **SD Card**: The Teensy must have an SD card inserted and properly initialized

## Starting the Service
```bash
# Start the teensy bridge node (which provides the service)
ros2 run sigyn_to_teensy teensy_bridge
```

## Implementation Details
- The service sends an `SDDIR:<directory_path>` command to the Teensy
- The Teensy responds with a `DIAG:SDIR:<listing>` message
- The service waits up to 5 seconds for a response
- Filenames in the response are separated by " | " for readability

## Error Conditions
- **Serial connection unavailable**: If the Teensy is not connected
- **Timeout**: If the Teensy doesn't respond within 5 seconds
- **Invalid directory**: If the specified directory doesn't exist on the SD card
