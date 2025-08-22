/**
 * @file lidar_base.hpp
 * @author Sigyn Robotics
 * @brief Base interface for modular LIDAR drivers
 * @version 1.0.0
 * @date 2025-08-21
 * 
 * @copyright Copyright (c) 2025 Sigyn Robotics. All rights reserved.
 * Licensed under the MIT License.
 */

#pragma once

#include "lidar_types.hpp"
#include <memory>
#include <functional>

namespace sigyn_lidar_v2 {

// Base interface for all LIDAR drivers
class LidarBase {
public:
  virtual ~LidarBase() = default;
  
  // Configuration and lifecycle
  virtual bool configure(const LidarConfig& config) = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool is_connected() const = 0;
  
  // Data processing
  virtual bool process_data(const uint8_t* data, size_t length) = 0;
  virtual bool has_complete_scan() const = 0;
  virtual LidarScan get_scan() = 0;
  
  // Status and diagnostics
  virtual std::string get_device_info() const = 0;
  virtual std::string get_status_string() const = 0;
  
  // Callback for when a complete scan is ready
  using ScanCallback = std::function<void(const LidarScan&)>;
  virtual void set_scan_callback(ScanCallback callback) = 0;

protected:
  LidarConfig config_;
  ScanCallback scan_callback_;
};

// Factory for creating LIDAR drivers
class LidarDriverFactory {
public:
  static std::unique_ptr<LidarBase> create_driver(const std::string& device_type);
  static std::vector<std::string> get_supported_types();
};

} // namespace sigyn_lidar_v2
