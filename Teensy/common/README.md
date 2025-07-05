# Common Teensy Modules

This directory contains shared code modules used by both Teensy boards in the Sigyn robot system. These modules provide common functionality and ensure consistency between the two boards.

## Shared Modules

### Module Base Class (`module.h`, `module.cpp`)
The core framework for all sensor and actuator modules. Provides:
- **Automatic Registration**: Modules register themselves when instantiated
- **Performance Monitoring**: Tracks execution time statistics for each module
- **Safety Interface**: Standard safety monitoring and emergency stop capabilities
- **Real-time Loop Management**: Coordinated execution of all registered modules

#### Key Features:
- **Singleton Pattern**: All modules implement singleton pattern for single instantiation
- **Statistics Tracking**: Min/max/average execution times for performance monitoring
- **Safety System**: `isUnsafe()` and `resetSafetyFlags()` for emergency handling
- **Real-time Constraints**: Enforces ≤2ms loop time requirements

### Serial Manager (`serial_manager.h`, `serial_manager.cpp`)
Centralized USB serial communication with the main computer/ROS2 system. Provides:
- **Diagnostic Messaging**: Standardized error and status reporting
- **Battery Status**: Battery voltage and health reporting
- **RoboClaw Status**: Motor controller status and odometry data
- **Module Statistics**: Performance data for debugging and monitoring

#### Communication Format:
All outgoing messages use structured format for easy parsing by ROS2 nodes.

### SD Card Module (`sd_module.h`, `sd_module.cpp`)  
Data logging functionality for both boards. Provides:
- **Automatic File Management**: Creates numbered log files (LOG00001.TXT, etc.)
- **Buffered Writing**: Efficient chunk-based writes to minimize loop delays
- **Directory Operations**: File listing and content retrieval
- **State Machine**: Non-blocking file operations for real-time performance

#### Key Features:
- **Performance Optimized**: Uses 4KB buffer to minimize write operations
- **Robust Error Handling**: Graceful degradation on SD card errors
- **Remote Access**: Directory listing and file dump via serial commands

### Configuration (`config.h`)
Shared configuration constants and hardware definitions:
- **Pin Assignments**: Standard pin definitions used across boards
- **Timing Constants**: Update rates and timeout values
- **Hardware Configuration**: I2C addresses, sensor parameters
- **Safety Limits**: Threshold values for safety systems

## Usage Guidelines

### Including Common Modules:
The common modules are available in each board directory via symbolic links:
```cpp
#include "module.h"
#include "serial_manager.h"
#include "config.h"
#include "sd_module.h"
```

**Note**: The common files are maintained in the `common/` directory and linked to each board directory using symbolic links. This ensures a single source of truth while maintaining Arduino IDE compatibility.

### Creating New Modules:
1. Inherit from `Module` base class
2. Implement required virtual functions:
   - `loop()` - Must complete in ≤2ms
   - `setup()` - One-time initialization (can be slow)
   - `name()` - Return module name string
3. Implement singleton pattern with automatic registration
4. Add safety monitoring if applicable

### Example Module Structure:
```cpp
class MyModule : public Module {
 public:
  static MyModule& singleton() {
    if (!instance_) instance_ = new MyModule();
    return *instance_;
  }

 protected:
  void loop() override { /* Fast operations only */ }
  void setup() override { /* Initialization */ }
  const char* name() override { return "MyModule"; }

 private:
  MyModule() : Module() { /* Constructor auto-registers */ }
  static MyModule* instance_;
};
```

## Performance Requirements

### Real-time Constraints:
- **Module Loop Time**: ≤2ms per module
- **Total System Loop**: Target 100Hz (10ms total)
- **Critical Modules**: Motor control, safety systems prioritized
- **Non-critical**: Logging, diagnostics can be slower

### Memory Management:
- **Static Allocation**: Avoid dynamic memory allocation in loop()
- **Buffer Management**: Use fixed-size buffers for performance
- **Stack Usage**: Keep local variables minimal in loop functions

## Safety Integration

### Safety Interface:
All modules can implement safety monitoring:
```cpp
bool isUnsafe() override {
  // Return true if module detects unsafe condition
  return sensor_failed_ || communication_timeout_;
}

void resetSafetyFlags() override {
  // Clear any safety flags after manual intervention
  sensor_failed_ = false;
  communication_timeout_ = false;
}
```

### Emergency Stop:
The Module system checks all registered modules for unsafe conditions and can trigger emergency stops.

## Debugging and Monitoring

### Performance Statistics:
Enable module statistics via Serial output. Times are in milliseconds (ms):
```json
{
  "loops": 1000,
  "Ms": 5.0,
  "mdls": [
    {"n": "VL53L0X", "MnMxAv": [0.5, 1.0, 0.7]},
    {"n": "RoboClaw", "MnMxAv": [1.2, 1.8, 1.5]}
  ]
}
```json
{
  "loops": 1523,
  "Ms": 8.2,
  "mdls": [
    {"n": "Battery", "MnMxAv": [0.8, 1.2, 0.9]},
    {"n": "Motor", "MnMxAv": [1.5, 2.1, 1.8]}
  ]
}
```

### Diagnostic Messages:
Use SerialManager for consistent logging:
```cpp
SerialManager::singleton().SendDiagnosticMessage("Sensor initialized");
```

## Maintenance

### Code Synchronization:
- Common modules are stored in the `common/` directory
- Each board directory contains symbolic links to the common files
- Changes to common functionality should be made in the `common/` directory
- The symbolic links automatically reflect changes in both board directories
- This maintains Arduino IDE compatibility while preventing code drift

### Symbolic Link Structure:
```
teensy_board1/
├── module.h -> ../common/module.h
├── module.cpp -> ../common/module.cpp  
├── config.h -> ../common/config.h
├── serial_manager.h -> ../common/serial_manager.h
├── serial_manager.cpp -> ../common/serial_manager.cpp
└── sd_module.h -> ../common/sd_module.h

teensy_board2/
└── (same symbolic links)
```

### Version Control:
- The `common/` directory contains the authoritative versions
- Board-specific modules should only contain board-specific code
- Test changes on both boards before deployment
