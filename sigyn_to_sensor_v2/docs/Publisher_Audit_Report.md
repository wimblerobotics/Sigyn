# ROS2 Topic Publisher Audit Report
**sigyn_to_sensor_v2 Package**  
**Date:** 2025-01-22  
**Auditor:** GitHub Copilot

## Executive Summary

This audit identified **20+ ROS2 topic publishers** across 6 nodes in the sigyn_to_sensor_v2 package. **Only 5 publishers are actively functioning**, while **15+ publishers are orphaned** and never publish data. The orphaned publishers create misleading topic structures and waste system resources.

## Active Publishers (✅ FUNCTIONAL)

### teensy_bridge_node - Main Communication Hub
All functional publishers are centralized in `teensy_bridge.cpp`, which handles serial communication with TeensyV2 hardware:

| Topic | Message Type | Publisher | Status | Description |
|-------|-------------|-----------|---------|-------------|
| `~/teensy_bridge/battery_state` | `sensor_msgs/BatteryState` | `battery_pub_` | ✅ **ACTIVE** | Battery voltage, current, temperature |
| `~/teensy_bridge/battery_diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_pub_` | ✅ **ACTIVE** | Battery health diagnostics |
| `~/teensy_bridge/safety/estop_status` | `std_msgs/Bool` | `estop_status_pub_` | ✅ **ACTIVE** | Emergency stop status |
| `~/teensy_bridge/imu/sensor_0` | `sensor_msgs/Imu` | `imu_sensor0_pub_` | ✅ **ACTIVE** | IMU data from BNO055 sensor 0 |
| `~/teensy_bridge/imu/sensor_1` | `sensor_msgs/Imu` | `imu_sensor1_pub_` | ✅ **ACTIVE** | IMU data from BNO055 sensor 1 |

**Total Active Publishers: 5**

## Orphaned Publishers (❌ NON-FUNCTIONAL)

### performance_monitor_node - No Data Source
| Topic | Message Type | Publisher | Issue | Recommendation |
|-------|-------------|-----------|-------|---------------|
| `teensy_v2/performance_diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_pub_` | Timer-only, no real data | **REMOVE** |
| `teensy_v2/loop_frequency` | `std_msgs/Float32` | `frequency_pub_` | `ProcessPerformanceData()` never called | **REMOVE** |
| `teensy_v2/execution_time` | `std_msgs/Int32` | `execution_time_pub_` | `ProcessPerformanceData()` never called | **REMOVED** ✅ |
| `teensy_v2/memory_usage` | `std_msgs/Int32` | `memory_usage_pub_` | `ProcessPerformanceData()` never called | **REMOVED** ✅ |

### battery_monitor_node - No Data Source  
| Topic | Message Type | Publisher | Issue | Recommendation |
|-------|-------------|-----------|-------|---------------|
| `teensy_v2/battery_state` | `sensor_msgs/BatteryState` | `battery_pub_` | `ProcessBatteryData()` never called | **REMOVE** |
| `teensy_v2/battery_diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_pub_` | Timer-only, no real data | **REMOVE** |

### imu_monitor_node - No Data Source
| Topic | Message Type | Publisher | Issue | Recommendation |
|-------|-------------|-----------|-------|---------------|
| `imu/sensor_0` | `sensor_msgs/Imu` | `imu_publishers_[0]` | `ProcessImuData()` never called | **REMOVE** |
| `imu/sensor_1` | `sensor_msgs/Imu` | `imu_publishers_[1]` | `ProcessImuData()` never called | **REMOVE** |
| `imu/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostic_pub_` | Timer-only, no real data | **REMOVE** |

### safety_coordinator_node - Potentially Orphaned
| Topic | Message Type | Publisher | Issue | Recommendation |
|-------|-------------|-----------|-------|---------------|
| `safety/global_estop` | `std_msgs/Bool` | `global_estop_pub_` | Complex logic, unclear if triggered | **AUDIT** |
| `safety/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `safety_diagnostics_pub_` | Timer-only callback | **AUDIT** |
| `safety/status` | `std_msgs/String` | `safety_status_pub_` | Complex logic, unclear if triggered | **AUDIT** |

### safety_publisher.cpp - Utility Class (Likely Orphaned)
| Topic | Message Type | Publisher | Issue | Recommendation |
|-------|-------------|-----------|-------|---------------|
| `safety/estop` | `std_msgs/Bool` | `estop_pub_` | Utility class, unclear if instantiated | **REMOVE** |
| `safety/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_pub_` | Utility class, unclear if instantiated | **REMOVE** |
| `safety/coordination` | `std_msgs/String` | `safety_coordination_pub_` | Utility class, unclear if instantiated | **REMOVE** |

**Total Orphaned Publishers: 15+**

## Root Cause Analysis

### 1. **Architectural Mismatch**
- **Issue**: Standalone nodes (`battery_monitor`, `performance_monitor`, `imu_monitor`) have no data input mechanisms
- **Pattern**: Nodes declare publishers and implement `ProcessXxxData()` methods, but these methods are never called
- **Root Cause**: Missing subscription or timer logic to fetch data from `teensy_bridge`

### 2. **Launch File Misconfiguration**  
- **Issue**: Launch file starts all nodes regardless of functionality
- **Pattern**: All nodes launched with `respawn=True`, creating persistent orphaned processes
- **Root Cause**: Launch configuration doesn't match actual data flow

### 3. **Missing Integration**
- **Issue**: No communication between `teensy_bridge` and specialized monitor nodes
- **Pattern**: `teensy_bridge` handles all data internally instead of publishing to other nodes
- **Root Cause**: Centralized vs. distributed architecture mismatch

## Immediate Actions Taken

### ✅ Completed
1. **Removed** `execution_time_pub_` from performance_monitor.cpp/.h
2. **Removed** `memory_usage_pub_` from performance_monitor.cpp/.h  
3. **Added documentation** comments marking orphaned publishers
4. **Fixed syntax error** in teensy_bridge.cpp estop publisher declaration
5. **Verified successful build** after changes

## Recommended Actions

### Phase 1: Cleanup (Immediate)
1. **Remove orphaned nodes** from CMakeLists.txt:
   - `battery_monitor_node`
   - `performance_monitor_node` 
   - `imu_monitor_node`
   - `safety_publisher.cpp` (if unused)

2. **Update launch file** to only start `teensy_bridge_node` and `safety_coordinator_node`

3. **Remove orphaned publisher declarations** from header files

### Phase 2: Architecture Decision (Short-term)
Choose one of two approaches:

#### Option A: Centralized (Recommended)
- **Keep**: Only `teensy_bridge_node` for all data publishing
- **Benefit**: Simpler, matches current data flow
- **Change**: Update topic names to match ROS2 conventions

#### Option B: Distributed  
- **Add**: Inter-node communication from `teensy_bridge` to specialized nodes
- **Benefit**: Better separation of concerns
- **Change**: Implement subscription/publishing pipeline between nodes

### Phase 3: Topic Naming (Medium-term)
Standardize topic names following ROS2 conventions:
- Current: Mixed naming (`teensy_v2/battery_state`, `~/imu/sensor_0`)
- Target: Consistent namespace (`~/teensy_v2/battery/state`, `~/teensy_v2/imu/sensor_0`)

## Impact Assessment

### Resource Savings
- **CPU**: Remove 4+ unnecessary node processes
- **Memory**: Eliminate orphaned publisher objects and timers
- **Network**: Reduce empty topic registrations

### System Clarity  
- **Topic List**: Clean `/ros2 topic list` output
- **Debugging**: Easier to identify actual data sources
- **Monitoring**: Focus on functional topics only

### Development Efficiency
- **Build Time**: Faster compilation with fewer files
- **Launch Time**: Quicker startup with fewer nodes
- **Maintenance**: Less code to maintain and debug

## Testing Recommendations

### Before Cleanup
```bash
# Capture current topic list for comparison
ros2 topic list > /tmp/topics_before.txt
ros2 topic hz /sigyn/teensy_bridge/battery_state
```

### After Cleanup
```bash
# Verify only functional topics remain
ros2 topic list > /tmp/topics_after.txt
diff /tmp/topics_before.txt /tmp/topics_after.txt

# Confirm data flow still works
ros2 topic hz /sigyn/teensy_bridge/battery_state
ros2 topic echo /sigyn/teensy_bridge/imu/sensor_0 --once
```

## Conclusion

This audit reveals a **75% publisher orphan rate** (15+ orphaned vs. 5 functional). The current architecture has a fundamental mismatch between declared capabilities and actual data flow. **Immediate cleanup** of orphaned publishers will significantly improve system clarity and resource efficiency.

**Priority 1**: Remove orphaned nodes from build and launch  
**Priority 2**: Decide on centralized vs. distributed architecture  
**Priority 3**: Standardize topic naming conventions  

The `teensy_bridge_node` is the **only functional data source** and should be the focus of future development efforts.
