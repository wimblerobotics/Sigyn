// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file sd_logger.h
 * @brief SD Card logging system for TeensyV2
 *
 * Provides comprehensive data logging capabilities to SD card with automatic
 * file management, buffered writes for performance, and diagnostic capabilities.
 * Designed to minimize impact on real-time performance while providing
 * reliable data storage.
 *
 * Features:
 * - Automatic log file creation with sequential numbering
 * - Buffered writes to minimize SD card access frequency
 * - Periodic flush to ensure data persistence
 * - Directory listing and file dumping capabilities
 * - Error handling and recovery for SD card failures
 * - Performance monitoring and statistics
 *
 * File Management:
 * - Log files use format: LOG#####.TXT (5-digit sequential numbers)
 * - Automatically finds next available number on startup
 * - Periodic directory caching for performance
 * - Graceful handling of card removal/insertion
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>
#include <SdFat.h>
#include <cstdint>
#include "sdios.h"

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

/**
 * @brief Configuration parameters for SD card logging system.
 */
struct SDLoggerConfig {
  // File management
  const char* log_prefix = "LOG";           ///< Log file prefix
  const char* log_extension = ".TXT";       ///< Log file extension
  uint32_t max_log_files = 99999;           ///< Maximum log file number
  
  // Buffer management
  uint32_t buffer_size = 4096;              ///< Write buffer size (bytes)
  uint32_t chunk_size = 1024;               ///< Write chunk size (bytes)
  uint32_t flush_interval_ms = 10000;       ///< Interval for forcing physical writes to SD card (data safety)
  uint32_t max_write_slice_ms = 2;          ///< Max milliseconds to spend writing per loop to reduce blocking
  
  // Performance settings
  uint32_t card_detect_interval_ms = 5000;  ///< Card detection retry interval
  uint32_t directory_cache_interval_ms = 30000; ///< Directory cache refresh interval
  
  // Safety settings
  uint32_t max_file_size_mb = 100;          ///< Maximum log file size (MB)
  uint32_t low_space_threshold_mb = 50;     ///< Low space warning threshold (MB)
  bool rotate_on_size_limit = true;         ///< Create new file when size limit reached
};

/**
 * @brief SD card and logging system status.
 */
struct SDLoggerStatus {
  // Card status
  bool card_present = false;                ///< SD card detected
  bool card_initialized = false;            ///< SD card successfully initialized
  bool file_open = false;                   ///< Log file currently open
  
  // Current file information
  String current_filename;                  ///< Current log filename
  uint32_t current_file_number = 0;         ///< Current file number
  uint32_t current_file_size = 0;           ///< Current file size (bytes)
  uint32_t total_files_created = 0;         ///< Total files created this session
  
  // Buffer status
  uint32_t buffer_usage_bytes = 0;          ///< Current buffer usage
  float buffer_usage_percent = 0.0f;        ///< Buffer usage percentage
  
  // Performance statistics
  uint32_t total_writes = 0;                ///< Total write operations
  uint32_t total_bytes_written = 0;         ///< Total bytes written
  uint32_t write_errors = 0;                ///< Number of write errors
  uint32_t flush_count = 0;                 ///< Number of flush operations
  float write_rate_bps = 0.0f;              ///< Current write rate (bytes/second)
  
  // Timing
  uint32_t last_write_time_ms = 0;          ///< Time of last write
  uint32_t last_flush_time_ms = 0;          ///< Time of last flush
  uint32_t session_start_time_ms = 0;       ///< Session start time
};

/**
 * @brief SD Card data logging module.
 * 
 * This module provides comprehensive SD card logging capabilities with
 * automatic file management, buffered writes, and real-time performance
 * monitoring. It follows the TeensyV2 module architecture with minimal
 * impact on real-time performance.
 */
class SDLogger : public Module {
public:
  static SDLogger& getInstance();
  
    // Logging operations
  void log(const String& message);
  void logFormatted(const char* format, ...);
  void flush();
  void forceFlush();
  
  // File management
  bool createNewLogFile();
  void closeCurrentFile();
  String getCurrentFilename() const { return status_.current_filename; }
  
  // Directory and file operations
  void refreshDirectoryCache();
  String getDirectoryListing();
  bool dumpFile(const String& filename);
  bool deleteFile(const String& filename);
  
  // Status and configuration
  bool isSDAvailable() const;
  uint32_t getBufferUsagePercent() const;
  bool hasPendingWrites() const { return write_buffer_.length() > 0; }  ///< Check if buffer has data
  const SDLoggerStatus& getStatus() const { return status_; }
  void updateConfig(const SDLoggerConfig& config) { config_ = config; }
  const SDLoggerConfig& getConfig() const { return config_; }
  
  // Message handling (for ROS2 integration)
  void handleDirMessage(const String& message);
  void handleFileDumpMessage(const String& message);
  void handleDeleteMessage(const String& message);

protected:
  // Module interface implementation
  void setup() override;
  void loop() override;
  const char* name() const override { return "SDLogger"; }
  
  // Safety interface
  bool isUnsafe() override;
  void resetSafetyFlags() override;

private:
  // Singleton constructor
  SDLogger();
  
  // State machine for file operations
  enum class LoggerState {
    UNINITIALIZED,
    INITIALIZING,
    READY,
    WRITING,
    FLUSHING,
    ERROR_RECOVERY
  };
  
  // File dump state machine - removed unused FileDumpState enum
  
  // Core functionality
  void initializeSDCard();
  void updateCardStatus();
  void performPeriodicMaintenance();
  void updatePerformanceStatistics();
  
  // File management helpers
  uint32_t findNextLogFileNumber();
  String generateLogFilename(uint32_t file_number);
  bool openLogFile(const String& filename);
  
  // Buffer management
  void addToBuffer(const String& data);
  void writeBufferToFile();
  void drainWriteBufferWithBudget(uint32_t max_ms);
  
  // Directory operations
  void updateDirectoryCache();
  
  // Configuration and state
  SDLoggerConfig config_;
  SDLoggerStatus status_;
  LoggerState logger_state_;
  
  // SD card interface
  SdFs sd_card_;
  FsFile log_file_;
  
  // Write buffer
  String write_buffer_;
  uint32_t last_buffer_add_time_ms_;
  
  // File dumping state
  enum class DumpState {
    IDLE,
    DUMPING,
    COMPLETE,
    ERROR_STATE
  };
  
  DumpState dump_state_;
  FsFile dump_file_;
  String dump_filename_;
  uint32_t dump_bytes_sent_;
  
  // State machine processing
  void processDumpStateMachine();
  
  // Directory caching
  String cached_directory_listing_;
  uint32_t last_directory_cache_time_ms_;
  
  // Timing for periodic operations
  uint32_t last_flush_time_ms_;
  uint32_t last_maintenance_time_ms_;
  
  // Performance tracking
  uint32_t session_start_time_ms_;
  uint32_t bytes_written_this_second_;
  uint32_t last_rate_calculation_time_ms_;
  
  // Constants
  static constexpr uint8_t SD_CS_PIN = 10;  // Use standard SD CS pin
  static constexpr uint32_t SD_SPI_SPEED = 25000000;  // 25MHz
  static constexpr uint32_t MAX_FILENAME_LENGTH = 32;
  static constexpr uint32_t DUMP_CHUNK_SIZE = 512;
};

} // namespace sigyn_teensy
