# Release 2025_10_14

**Release Date:** October 14, 2025  
**Tag:** `2025_10_14`  
**Commit:** `842a792`

## Overview

This release focuses on battery monitoring visualization, navigation improvements with range sensors, comprehensive documentation updates, and significant codebase cleanup.

---

## 🎯 Major Features

### Battery Overlay Visualization
- **Real-time battery display in RViz** with color-coded status indicators
  - Green (>50%): Normal operation
  - Yellow (20-50%): Low battery warning  
  - Red (<20%): Critical battery level
- **Multi-battery support** with filtering for specific battery (36VLIPO)
- **Automatic launch** integrated into main system launch file
- Uses standard `rviz_2d_overlay_plugins` for reliable rendering
- Implementation: `scripts/py_scripts/battery_overlay_publisher.py`

### VL53L0X/SONAR Range Sensor Integration
- **Enhanced navigation** with Time-of-Flight sensors
- **RangeSensorLayer configuration** for Nav2 costmap integration
- **Comprehensive documentation** in `Documentation/Articles/HowToConfigureSonar.md`
- Proper URDF integration with tilt compensation
- Fixed costmap saturation issues with marking/clearing thresholds

### OAK-D Camera Improvements
- **Compressed image republisher** for bandwidth optimization
- Launch file: `base/launch/sub_launch/oakd_compressed_republisher.launch.py`
- Bandwidth monitoring scripts added to `scripts/`

---

## 🔧 Improvements

### Navigation Configuration
- **Updated `navigation_sim.yaml`**:
  - Configured RangeSensorLayer with proper parameters
  - Set marking threshold to 0.50 (prevents single-sensor saturation)
  - Enabled proper sensor integration with costmaps
  - Optimized for VL53L0X sensors

### Documentation
- **Completely rewritten README.md**:
  - All 25+ packages documented and categorized
  - Build instructions with rosdep usage
  - Clear launch options (single command vs. manual)
  - Monitoring section explaining overlays
  - Recent updates section with links to detailed docs

- **New Documentation**:
  - `Documentation/Articles/HowToConfigureSonar.md` - Comprehensive range sensor guide
  - Inline battery overlay documentation in README

### RViz Configuration
- Added OverlayTextDisplay for battery status
- Updated display configurations
- Improved visualization layouts

### Launch Files
- **Battery overlay integrated** into `sigyn.launch.py`
- Removed outdated micro_ros_agent manual launch instructions
- Note added that Teensy bridge launches automatically

---

## 🧹 Cleanup

### Removed Packages
- **`this_to_that`**: Generic message transformer no longer needed
- **`pi_servo1`**: Old servo control package removed
- **`min_max_curr_rviz_overlay`**: Git submodule removed (non-functional plugin)

### Removed Files
- `.gitmodules` (no more submodules)
- `Analysis.txt` (moved to proper documentation)
- `teensy_sensor.txt`, `teensy_sensor2.txt` (outdated)
- `requirements.txt` (Python dependencies managed elsewhere)
- Various unused VS Code task configurations

### Updated References
- Removed all `micro_ros` package references
- Removed `micro_ros_agent` manual launch instructions
- Updated to reflect `sigyn_to_sensor_v2` as current Teensy bridge
- Cleaned up broken documentation links

### VS Code Configuration
- **Machine-specific settings** now properly managed
- Added to `.gitignore`:
  - `.vscode/settings_linux_amd.json`
  - `.vscode/settings_macbook.json`
  - `.vscode/switch_settings.sh`
- Prevents merge conflicts across different development machines

---

## 📦 Package Updates

### New Dependencies
- `sensor_msgs` - For BatteryState messages
- `rviz_2d_overlay_msgs` - For overlay text display

### Updated Package Metadata
- `scripts/package.xml` - Added overlay dependencies
- `base/package.xml` - Added py_scripts exec_depend

---

## 🔄 Migration Notes

### For Existing Installations

1. **Update repository**:
   ```bash
   cd ~/sigyn_ws/src/Sigyn
   git pull
   git checkout 2025_10_14
   ```

2. **Install new dependencies**:
   ```bash
   cd ~/sigyn_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Rebuild workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Clean up removed packages**:
   ```bash
   rm -rf build/this_to_that install/this_to_that
   rm -rf build/pi_servo1 install/pi_servo1
   rm -rf build/min_max_curr_rviz_overlay install/min_max_curr_rviz_overlay
   ```

### Breaking Changes
- **Removed packages**: If you were using `this_to_that` or `pi_servo1`, you'll need to update your dependencies
- **micro_ros_agent**: No longer launched separately; `sigyn_to_sensor_v2` handles Teensy communication

---

## 📊 Statistics

- **Files Changed**: 100+ files
- **Packages Added**: 1 (battery_overlay_publisher in scripts)
- **Packages Removed**: 3 (this_to_that, pi_servo1, min_max_curr_rviz_overlay)
- **Documentation**: 2 new guides added
- **Lines Added**: ~800 in README alone

---

## 🔗 Quick Links

- **GitHub Release**: https://github.com/wimblerobotics/Sigyn/releases/tag/2025_10_14
- **Repository**: https://github.com/wimblerobotics/Sigyn
- **Commit**: https://github.com/wimblerobotics/Sigyn/commit/842a792

---

## 👥 Contributors

- Michael Wimble (@wimblerobotics)

---

## 📝 Additional Notes

### Testing Recommendations
1. Verify battery overlay appears in RViz
2. Test range sensor integration in navigation
3. Check OAK-D compressed images publish correctly
4. Ensure all launch files work as documented

### Known Issues
- None reported for this release

### Future Considerations
- Consider adding voltage overlay in addition to combined display
- Possible integration of additional battery metrics
- Further navigation tuning with range sensors

---

## 🎉 Thank You

This release represents significant progress in battery monitoring, sensor integration, and documentation. The codebase is now cleaner and more maintainable.

For questions or issues, please open an issue on GitHub.
