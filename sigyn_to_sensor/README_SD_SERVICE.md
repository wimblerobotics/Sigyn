# Teensy SD Card Services

## Overview
The Teensy bridge provides two services for interacting with the Teensy's SD card storage:
1. `teensy_sd_getdir` - Get directory listings from the SD card
2. `teensy_sd_getfile` - Dump the contents of files from the SD card

## Service Definitions

### Directory Listing Service
**Service Name:** `teensy_sd_getdir`
**Service Type:** `sigyn_interfaces/srv/TeensySdGetDir`

#### Request
- No parameters required (always returns root directory listing)

#### Response
- `directory_listing` (string): The directory listing result from the Teensy (tab-separated format)
- `success` (bool): Whether the operation was successful
- `error_message` (string): Error message if the operation failed

### File Dump Service
**Service Name:** `teensy_sd_getfile`
**Service Type:** `sigyn_interfaces/srv/TeensySdGetFile`

#### Request
- `filename` (string): The filename on the Teensy SD card to dump (e.g., "LOG00067.TXT")

#### Response
- `file_contents` (string): The complete contents of the requested file
- `success` (bool): Whether the operation was successful
- `error_message` (string): Error message if the operation failed

## Usage Examples

### Directory Listing Service
```bash
# Get directory listing (root directory)
ros2 service call /teensy_sd_getdir sigyn_interfaces/srv/TeensySdGetDir "{}"
```

Example response shows each file in the root directory followed by a comma then the size of the file. The last file in the list
is the current log file and it has no size showing. The directory list is computed once at startup is that cached result
is what is shown for each request.
```code
requester: making request: sigyn_interfaces.srv.TeensySdGetDir_Request()

response:
sigyn_interfaces.srv.TeensySdGetDir_Response(directory_listing='.Spotlight-V100,0\tLOG00001.TXT,622209\tLOG00002.TXT,0\tLOG00003.TXT,0\tLOG00004.TXT,163489\tLOG00005.TXT,15206097\tLOG00006.TXT,491379\t.Trashes,0\t.fseventsd,0\tLOG00007.TXT', success=True, error_message='')
```

### File Dump Service
```bash
# Dump contents of a specific log file
ros2 service call /teensy_sd_getfile sigyn_interfaces/srv/TeensySdGetFile "{filename: 'LOG00067.TXT'}"

# Dump contents of another file
ros2 service call /teensy_sd_getfile sigyn_interfaces/srv/TeensySdGetFile "{filename: 'LOG00044.TXT'}"
```

Example response:
```
success: true
file_contents: "[0000000.329] [SdModule::SdModule] Compiled on: Jun 11 2025, 21:53:12
[0000001.234] Battery voltage: 12.45V
[0000002.456] Motor command received: linear=0.5, angular=0.0
[0000003.789] Sensor reading: temperature=25.3C"
error_message: ""
```

### Expected Response Formats

#### Directory Listing
The `directory_listing` field will contain tab-separated filenames:
- **Format**: `filename1\tfilename2\tfilename3...`
- **Example**: `.Spotlight-V100\tLOG00044.TXT\tLOG00045.TXT\tLOG00067.TXT`

#### File Contents  
The `file_contents` field will contain the complete file with newline-separated lines:
- **Format**: Multi-line text with original line breaks preserved
- **Example**: Log entries with timestamps, sensor readings, diagnostic messages

## Timing Considerations

### Directory Listing Service
- **First request**: May take 15+ seconds due to SD card initialization and file scanning
- **Subsequent requests**: Should complete within 1-2 seconds (uses cached listing)
- **Service timeout**: 10 seconds

### File Dump Service
- **Small files (<1KB)**: Complete within 1-2 seconds
- **Medium files (1KB-100KB)**: Complete within 10-30 seconds
- **Large files (>100KB)**: May take several minutes depending on file size
- **Activity timeout**: 10 seconds of inactivity (resets on each line received)
- **Maximum timeout**: 5 minutes (configurable via `MAX_FILE_DUMP_TIMEOUT` constant)
- **Progress logging**: Shows character count and activity status every 5 seconds

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

### Directory Listing Service
- The service sends an `SDDIR:` command to the Teensy
- The Teensy responds with a `DIAG:SDIR:<listing>` message
- Uses cached directory listing for fast subsequent requests
- Filenames are separated by tabs (`\t`) in the response

### File Dump Service  
- The service sends an `SDFILE:<filename>` command to the Teensy
- The Teensy uses a state machine to read the file line by line
- Each line is sent as `DIAG:SDLINE:<line_content>\r`
- When complete, sends `DIAG:SDEOF:` to signal end of file
- Uses activity-based timeout that resets on each received line
- Accumulates all lines into a single response string

### Serial Communication Protocol
Both services use the single-threaded serial communication loop with:
- 10ms timer for all serial I/O operations
- Thread-safe message queuing for outgoing commands
- Multi-threaded executor with separate callback group for service handlers

## Error Conditions

### Common Errors
- **Serial connection unavailable**: If the Teensy is not connected
- **SD card not initialized**: If the SD card is not properly mounted

### Directory Listing Errors
- **Timeout**: If the Teensy doesn't respond within 10 seconds

### File Dump Errors
- **File not found**: If the specified file doesn't exist on the SD card
- **Activity timeout**: If no SDLINE responses received for 10 seconds
- **Maximum timeout**: If the operation exceeds the 5-minute limit
- **Empty filename**: If no filename is provided in the request

## Performance Notes
- **Large log files**: Files with hundreds of thousands of lines may take several minutes to transfer
- **Memory usage**: Large files are accumulated in memory during transfer
- **Concurrent requests**: Only one file dump can be active at a time
- **Interruption**: File dumps cannot be cancelled once started (will timeout or complete)
