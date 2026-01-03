// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file sd_logger.cpp
 * @brief SD card logging implementation for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "sd_logger.h"
#include <cstdarg>

namespace sigyn_teensy {

    static constexpr bool kVerboseDirScan = false; // Set true to debug per-file during SD init

    SDLogger& SDLogger::getInstance() {
        static SDLogger instance;
        return instance;
    }

    SDLogger::SDLogger()
        : config_(),
        status_(),
        logger_state_(LoggerState::UNINITIALIZED),
        last_buffer_add_time_ms_(0),
        dump_state_(DumpState::IDLE),
        dump_bytes_sent_(0),
        last_directory_cache_time_ms_(0),
        last_flush_time_ms_(0),
        last_maintenance_time_ms_(0),
        session_start_time_ms_(0),
        bytes_written_this_second_(0),
        last_rate_calculation_time_ms_(0) {

        // Reserve buffer space for performance
        write_buffer_.reserve(config_.buffer_size);
        cached_directory_listing_.reserve(2048);
    }

    void SDLogger::setup() {
        session_start_time_ms_ = millis();
        status_.session_start_time_ms = session_start_time_ms_;

        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Initializing SD card");
        initializeSDCard();

        if (status_.card_initialized) {
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Setup complete");
        }
        else {
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "SD card initialization failed");
        }
    }

    void SDLogger::loop() {
        uint32_t current_time = millis();

        // Check SD commands from SerialManager
        SerialManager& serial_mgr = SerialManager::getInstance();

        if (serial_mgr.hasNewSDDirCommand()) {
            String dir_command = serial_mgr.getLatestSDDirCommand();
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), ("SDDIR command received: " + dir_command).c_str());
            handleDirMessage(dir_command);
        }

        if (serial_mgr.hasNewSDFileCommand()) {
            String file_command = serial_mgr.getLatestSDFileCommand();
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), ("SDFILE command received: " + file_command).c_str());
            handleFileDumpMessage(file_command);
        }

        // Handle file dumping state machine
        processDumpStateMachine();

        // Periodic maintenance
        if (current_time - last_maintenance_time_ms_ > 1000) {
            performPeriodicMaintenance(); // Usually takes 1 ms, but I've seen as hight as 9 ms.
            last_maintenance_time_ms_ = current_time;
        }

        // Cooperatively drain buffer within a small time budget to reduce blocking
        if (write_buffer_.length() > 0) {
            drainWriteBufferWithBudget(config_.max_write_slice_ms);
        }

        // Update performance statistics
        updatePerformanceStatistics();
    }

    void SDLogger::processDumpStateMachine() {
        if (dump_state_ == DumpState::DUMPING && dump_file_) {
            if (dump_file_.available()) {
                // Read one line from the file
                String line = dump_file_.readStringUntil('\n');

                // Remove carriage return if present
                if (line.endsWith("\r")) {
                    line.remove(line.length() - 1);
                }

                // Send the line with SDLINE prefix
                SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), line.c_str());

                dump_bytes_sent_ += line.length();
            }
            else {
                // End of file reached
                SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
                dump_file_.close();
                dump_state_ = DumpState::COMPLETE;

                String msg = "Completed dumping file: " + dump_filename_;
                SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg.c_str());

                dump_filename_ = "";
                dump_bytes_sent_ = 0;
                dump_state_ = DumpState::IDLE;
            }
        }
    }

    bool SDLogger::isUnsafe() {
        return status_.write_errors > 10 || (!status_.card_present && logger_state_ != LoggerState::UNINITIALIZED);
    }

    void SDLogger::resetSafetyFlags() {
        status_.write_errors = 0;
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Safety flags reset");
    }

    void SDLogger::log(const String& message) {
        if (!status_.card_initialized || !status_.file_open) {
            return;
        }

        uint32_t current_time = millis();

        // Create timestamped log entry
        String log_entry = "[";
        log_entry += String(current_time / 1000);
        log_entry += ".";
        log_entry += String(current_time % 1000, DEC);
        log_entry += "] ";
        log_entry += message;
        log_entry += "\n";

        addToBuffer(log_entry);

        // Flush if buffer is getting full
        if (write_buffer_.length() >= config_.chunk_size) {
            writeBufferToFile();
        }
    }

    void SDLogger::logFormatted(const char* format, ...) {
        // If SD isn't ready or file not open, skip formatting to reduce overhead during init
        if (!status_.card_initialized || !status_.file_open) {
            return;
        }
        static char formatted_message[3072]; // Increased to reduce risk of truncation
        va_list args;
        va_start(args, format);
        vsnprintf(formatted_message, sizeof(formatted_message), format, args);
        va_end(args);

        log(String(formatted_message));
    }

    void SDLogger::flush() {
        // Best-effort non-blocking flush: drain with budget this cycle
        if (write_buffer_.length() > 0) {
            drainWriteBufferWithBudget(config_.max_write_slice_ms);
        }
    }

    void SDLogger::forceFlush() {
        // Forcefully drain entire buffer and sync to the card; can block
        while (write_buffer_.length() > 0) {
            writeBufferToFile();
        }
        if (status_.file_open && log_file_) {
            log_file_.flush();
            status_.last_flush_time_ms = millis();
            status_.flush_count++;
        }
    }

    bool SDLogger::createNewLogFile() {
        if (!status_.card_initialized) {
            return false;
        }

        closeCurrentFile();

        uint32_t next_file_number = findNextLogFileNumber();
        String filename = generateLogFilename(next_file_number);

        SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), ("Creating log file: " + filename).c_str());

        if (openLogFile(filename)) {
            status_.current_file_number = next_file_number;
            status_.current_filename = filename;
            status_.current_file_size = 0;
            status_.total_files_created++;

            // Add the new log file to the cached directory listing (without size, as it's current)
            if (cached_directory_listing_.length() > 0) {
                cached_directory_listing_ += "\t";
            }
            cached_directory_listing_ += filename;

            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), ("Created log file: " + filename).c_str());

            // Log startup message
            log("SDLogger started");

            return true;
        }

        SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), ("Failed to create log file: " + filename).c_str());
        return false;
    }

    void SDLogger::closeCurrentFile() {
        if (status_.file_open && log_file_) {
            forceFlush();
            log_file_.close();
            status_.file_open = false;
            status_.current_filename = "";
        }
    }

    void SDLogger::refreshDirectoryCache() {
        updateDirectoryCache();
    }

    String SDLogger::getDirectoryListing() {
        if (millis() - last_directory_cache_time_ms_ > config_.directory_cache_interval_ms) {
            updateDirectoryCache();
        }
        return cached_directory_listing_;
    }

    bool SDLogger::dumpFile(const String& filename) {
        if (!status_.card_initialized) {
            SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), "ERROR: SD card not initialized");
            SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
            return false;
        }

        if (dump_state_ != DumpState::IDLE) {
            SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), "ERROR: File dump already in progress");
            SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
            return false;
        }

        if (filename.length() == 0) {
            SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), "ERROR: No filename specified");
            SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
            return false;
        }

        // Try to open the file
        dump_file_ = sd_card_.open(filename.c_str(), O_RDONLY);
        if (!dump_file_) {
            String error_msg = "ERROR: Could not open file '" + filename + "'";
            SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), error_msg.c_str());
            SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
            return false;
        }

        // Set state to start dumping
        dump_state_ = DumpState::DUMPING;
        dump_filename_ = filename;
        dump_bytes_sent_ = 0;

        String msg = "Started dumping file: " + filename;
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg.c_str());

        return true;
    }

    bool SDLogger::deleteFile(const String& filename) {
        if (!status_.card_initialized) {
            return false;
        }

        if (sd_card_.remove(filename.c_str())) {
            updateDirectoryCache();
            return true;
        }

        return false;
    }

    void SDLogger::handleDirMessage(const String& message) {
        if (!status_.card_initialized) {
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "SD card not initialized");
            return;
        }

        // Get current directory listing
        String dir_listing = getDirectoryListing();

        // Send directory listing with SDIR prefix - format exactly like legacy
        SerialManager::getInstance().sendDiagnosticMessage("SDIR", name(), dir_listing.c_str());
    }

    void SDLogger::handleFileDumpMessage(const String& message) {
        String msg = "Received file dump request: " + message;
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg.c_str());

        // Extract filename from message (should be just the filename)
        String filename = message;
        filename.trim();

        dumpFile(filename);
    }

    void SDLogger::handleDeleteMessage(const String& message) {
        String filename = message;
        filename.trim();

        if (deleteFile(filename)) {
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), ("Deleted file: " + filename).c_str());
        }
        else {
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), ("Failed to delete file: " + filename).c_str());
        }
    }

    bool SDLogger::isSDAvailable() const {
        return status_.card_present && status_.card_initialized;
    }

    uint32_t SDLogger::getBufferUsagePercent() const {
        if (config_.buffer_size == 0) return 0;
        return (write_buffer_.length() * 100) / config_.buffer_size;
    }

    // Private methods implementation

    void SDLogger::initializeSDCard() {
        logger_state_ = LoggerState::INITIALIZING;

        SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "Attempting SD card initialization...");

        // Reset state - don't mark as initialized until everything succeeds
        status_.card_present = false;
        status_.card_initialized = false;
        status_.file_open = false;

        if (!sd_card_.begin(SdioConfig(FIFO_SDIO))) {
            logger_state_ = LoggerState::ERROR_RECOVERY;
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "SD card initialization failed");
            return;
        }

        SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "SD card begin() succeeded");
        status_.card_present = true;
        status_.card_initialized = true;  // Set this before creating log file

        // Build directory cache first (quickly)
        updateDirectoryCache();
        // Brief yield to avoid monopolizing loop during init
        delay(1);

        // Create initial log file
        if (!createNewLogFile()) {
            status_.card_initialized = false;  // Reset on failure
            logger_state_ = LoggerState::ERROR_RECOVERY;
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Failed to create initial log file");
            return;
        }

        // Check if log file was created successfully - only mark as initialized if everything worked
        if (status_.file_open && log_file_) {
            logger_state_ = LoggerState::READY;
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Ready for logging");

            // Log startup message exactly like legacy code with board info
            String board_info = "Board ";
#ifdef BOARD_ID
            board_info += String(BOARD_ID);
#else
            board_info += "Unknown";
#endif
#if defined(BOARD_ID) && BOARD_ID == 1
            board_info += " (Navigation_Safety)";
#elif defined(BOARD_ID) && BOARD_ID == 2
            board_info += " (Power_Sensors)";
#endif

            String startup_msg = "[SDLogger::SDLogger] " + board_info + " - Compiled on: " __DATE__ ", " __TIME__;
            log(startup_msg);
        }
        else {
            status_.card_initialized = false;  // Ensure it remains false on failure
            logger_state_ = LoggerState::ERROR_RECOVERY;
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Failed to create log file");
        }
    }

    void SDLogger::updateCardStatus() {
        // Simple card presence check - could be enhanced with actual detection
        status_.card_present = status_.card_initialized;
    }

    void SDLogger::performPeriodicMaintenance() {
        updateCardStatus();

        // Flush any pending buffer data - writeBufferToFile handles physical writes
        if (write_buffer_.length() > 0) {
            flush();
        }
    }

    void SDLogger::updatePerformanceStatistics() {
        uint32_t current_time = millis();

        if (current_time - last_rate_calculation_time_ms_ >= 1000) {
            status_.write_rate_bps = bytes_written_this_second_;
            bytes_written_this_second_ = 0;
            last_rate_calculation_time_ms_ = current_time;

            status_.buffer_usage_bytes = write_buffer_.length();
            status_.buffer_usage_percent = getBufferUsagePercent();
            char msg[256];
            snprintf(msg, sizeof(msg), "SDLogger: Performance stats: Buffer usage: %lu bytes (%u%%), Write rate: %.2f B/s, Total writes: %lu",
                (unsigned long)status_.buffer_usage_bytes,
                (unsigned)status_.buffer_usage_percent,
                (double)status_.write_rate_bps,
                (unsigned long)status_.total_writes);
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
        }
    }

    uint32_t SDLogger::findNextLogFileNumber() {
        uint32_t highest_number = 0;

        FsFile root = sd_card_.open("/");
        if (!root) {
            return 1;
        }

        while (true) {
            FsFile entry = root.openNextFile();
            if (!entry) {
                break;
            }

            char filename[256];
            entry.getName(filename, sizeof(filename));
            entry.close();

            // Check if this is a log file (LOG#####.TXT pattern)
            String filename_str = String(filename);
            if (filename_str.startsWith("LOG") && filename_str.endsWith(".TXT") && filename_str.length() == 12) {
                String number_str = filename_str.substring(3, 8);
                uint32_t file_number = number_str.toInt();
                if (file_number > highest_number) {
                    highest_number = file_number;
                }
            }
        }

        root.close();
        return highest_number + 1;
    }

    String SDLogger::generateLogFilename(uint32_t file_number) {
        char filename[16];
        snprintf(filename, sizeof(filename), "LOG%05lu.TXT", (unsigned long)file_number);
        return String(filename);
    }

    bool SDLogger::openLogFile(const String& filename) {
        log_file_ = sd_card_.open(filename.c_str(), O_WRONLY | O_CREAT | O_APPEND);
        if (log_file_) {
            status_.file_open = true;
            status_.current_filename = filename;
            return true;
        }
        return false;
    }

    void SDLogger::addToBuffer(const String& data) {
        write_buffer_ += data;
        last_buffer_add_time_ms_ = millis();

        // Count bytes as soon as they're added to buffer for accurate rate calculation
        bytes_written_this_second_ += data.length();
    }

    void SDLogger::writeBufferToFile() {
        if (!status_.file_open || write_buffer_.length() == 0) {
            return;
        }

        // Write at most a chunk to bound latency
        size_t bytes_to_write = write_buffer_.length();
        if (bytes_to_write > config_.chunk_size) {
            bytes_to_write = config_.chunk_size;
        }
        size_t bytes_written = log_file_.write(write_buffer_.c_str(), bytes_to_write);

        if (bytes_written > 0) {
            // Update file statistics (don't double-count bytes - they were counted in addToBuffer)
            status_.current_file_size += bytes_written;
            status_.total_bytes_written += bytes_written;
            status_.total_writes++;
            status_.last_write_time_ms = millis();

            // Clear the written portion from buffer
            write_buffer_.remove(0, bytes_written);

            // Opportunistic physical write if enough time has passed since last flush
            uint32_t current_time = millis();
            if (current_time - last_flush_time_ms_ > config_.flush_interval_ms) {
                // This flush can block; keep it infrequent by interval
                log_file_.flush();  // Ensure data safety periodically
                last_flush_time_ms_ = current_time;
                status_.last_flush_time_ms = current_time;
                status_.flush_count++;
            }
        }
        else {
            // Write failed - assume card issue
            status_.write_errors++;
            status_.card_initialized = false;
            logger_state_ = LoggerState::ERROR_RECOVERY;
        }
    }

    void SDLogger::drainWriteBufferWithBudget(uint32_t max_ms) {
        uint32_t start = millis();
        if (max_ms == 0) return;
        // Write chunks until we hit time budget or buffer is empty
        while (write_buffer_.length() > 0) {
            writeBufferToFile();
            if (millis() - start >= max_ms) {
                break;
            }
        }
    }

    void SDLogger::updateDirectoryCache() {
        if (!status_.card_present) {  // Check card_present instead of card_initialized
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "updateDirectoryCache: Card not present");
            return;
        }

        cached_directory_listing_ = "";

        // Open root directory
        FsFile rootDirectory = sd_card_.open("/");
        if (!rootDirectory) {
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Could not open root directory");
            return;
        }

        if (kVerboseDirScan) {
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "updateDirectoryCache: Scanning directory...");
        }

        // Build cached directory listing only - no file creation
        uint32_t fileCount = 0;
        while (true) {
            FsFile nextFileInDirectory = rootDirectory.openNextFile();
            if (!nextFileInDirectory) {
                break;
            }
            char fileName[256];
            nextFileInDirectory.getName(fileName, sizeof(fileName));

            // Get file size before closing
            uint32_t fileSize = nextFileInDirectory.size();
            nextFileInDirectory.close();

            // Add to cached directory listing with filename,size format (legacy format)
            if (cached_directory_listing_.length() > 0) {
                cached_directory_listing_ += "\t";
            }
            cached_directory_listing_ += fileName;
            cached_directory_listing_ += ",";
            cached_directory_listing_ += String(fileSize);

            if (kVerboseDirScan) {
                String dbg = String("Found file: ") + String(fileName) + String(" size: ") + String(fileSize);
                SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), dbg.c_str());
            }
            fileCount++;
        }

        rootDirectory.close();

        if (kVerboseDirScan) {
            // Potentially very long, only emit when verbose
            String dbg = String("Directory listing: '") + cached_directory_listing_ + String("'");
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), dbg.c_str());
        }
        else {
            char msg[96];
            snprintf(msg, sizeof(msg), "Directory cached: %lu entries", (unsigned long)fileCount);
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
        }

        last_directory_cache_time_ms_ = millis();
    }

} // namespace sigyn_teensy
