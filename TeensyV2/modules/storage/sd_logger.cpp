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

SDLogger& SDLogger::getInstance() {
    static SDLogger instance;
    return instance;
}

SDLogger::SDLogger() 
  : config_(),
    status_(),
    logger_state_(LoggerState::UNINITIALIZED),
    card_detection_enabled_(true),
    buffer_write_count_(0),
    last_buffer_add_time_ms_(0),
    dump_state_(DumpState::IDLE),
    dump_position_(0),
    dump_bytes_sent_(0),
    last_directory_cache_time_ms_(0),
    last_flush_time_ms_(0),
    last_maintenance_time_ms_(0),
    last_card_check_time_ms_(0),
    last_performance_update_ms_(0),
    session_start_time_ms_(0),
    last_write_size_(0),
    bytes_written_this_second_(0),
    last_rate_calculation_time_ms_(0) {
    
    // Reserve buffer space for performance
    write_buffer_.reserve(config_.buffer_size);
    cached_directory_listing_.reserve(2048);
}

void SDLogger::setup() {
    session_start_time_ms_ = millis();
    status_.session_start_time_ms = session_start_time_ms_;
    
    SerialManager::getInstance().sendMessage("INFO", "SDLogger: Initializing SD card");
    initializeSDCard();
    
    if (status_.card_initialized) {
        SerialManager::getInstance().sendMessage("INFO", "SDLogger: Setup complete");
    } else {
        SerialManager::getInstance().sendMessage("ERROR", "SDLogger: SD card initialization failed");
    }
}

void SDLogger::loop() {
    uint32_t current_time = millis();
    
    // Check SD commands from SerialManager
    SerialManager& serial_mgr = SerialManager::getInstance();
    
    if (serial_mgr.hasNewSDDirCommand()) {
        String dir_command = serial_mgr.getLatestSDDirCommand();
        SerialManager::getInstance().sendMessage("DEBUG", ("SDDIR command received: " + dir_command).c_str());
        handleDirMessage(dir_command);
    }
    
    if (serial_mgr.hasNewSDFileCommand()) {
        String file_command = serial_mgr.getLatestSDFileCommand();
        SerialManager::getInstance().sendMessage("DEBUG", ("SDFILE command received: " + file_command).c_str());
        handleFileDumpMessage(file_command);
    }
    
    // Handle file dumping state machine
    processDumpStateMachine();
    
    // Periodic maintenance
    if (current_time - last_maintenance_time_ms_ > 1000) {
        performPeriodicMaintenance();
        last_maintenance_time_ms_ = current_time;
    }
    
    // Auto-flush buffer if needed
    if (write_buffer_.length() > 0 && 
        (current_time - last_buffer_add_time_ms_ > config_.flush_interval_ms ||
         write_buffer_.length() >= config_.buffer_size)) {
        flush();
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
            SerialManager::getInstance().sendMessage("SDLINE", line.c_str());
            
            dump_bytes_sent_ += line.length();
        } else {
            // End of file reached
            SerialManager::getInstance().sendMessage("SDEOF", "");
            dump_file_.close();
            dump_state_ = DumpState::COMPLETE;
            
            String msg = "Completed dumping file: " + dump_filename_;
            SerialManager::getInstance().sendMessage("INFO", msg.c_str());
            
            dump_filename_ = "";
            dump_position_ = 0;
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
    SerialManager::getInstance().sendMessage("INFO", "SDLogger: Safety flags reset");
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
    char formatted_message[512];
    va_list args;
    va_start(args, format);
    vsnprintf(formatted_message, sizeof(formatted_message), format, args);
    va_end(args);
    
    log(String(formatted_message));
}

void SDLogger::flush() {
    if (write_buffer_.length() > 0) {
        writeBufferToFile();
    }
}

void SDLogger::forceFlush() {
    flush();
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
    
    if (openLogFile(filename)) {
        status_.current_file_number = next_file_number;
        status_.current_filename = filename;
        status_.current_file_size = 0;
        status_.total_files_created++;
        
        // Log startup message
        log("SDLogger started - TeensyV2 Board1");
        
        return true;
    }
    
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
        SerialManager::getInstance().sendMessage("SDLINE", "ERROR: SD card not initialized");
        SerialManager::getInstance().sendMessage("SDEOF", "");
        return false;
    }
    
    if (dump_state_ != DumpState::IDLE) {
        SerialManager::getInstance().sendMessage("SDLINE", "ERROR: File dump already in progress");
        SerialManager::getInstance().sendMessage("SDEOF", "");
        return false;
    }
    
    if (filename.length() == 0) {
        SerialManager::getInstance().sendMessage("SDLINE", "ERROR: No filename specified");
        SerialManager::getInstance().sendMessage("SDEOF", "");
        return false;
    }
    
    // Try to open the file
    dump_file_ = sd_card_.open(filename.c_str(), O_RDONLY);
    if (!dump_file_) {
        String error_msg = "ERROR: Could not open file '" + filename + "'";
        SerialManager::getInstance().sendMessage("SDLINE", error_msg.c_str());
        SerialManager::getInstance().sendMessage("SDEOF", "");
        return false;
    }
    
    // Set state to start dumping
    dump_state_ = DumpState::DUMPING;
    dump_filename_ = filename;
    dump_position_ = 0;
    dump_bytes_sent_ = 0;
    
    String msg = "Started dumping file: " + filename;
    SerialManager::getInstance().sendMessage("INFO", msg.c_str());
    
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
        SerialManager::getInstance().sendMessage("ERROR", "SD card not initialized");
        return;
    }
    
    // Get current directory listing
    String dir_listing = getDirectoryListing();
    
    // Send directory listing with SDIR prefix - format exactly like legacy
    SerialManager::getInstance().sendMessage("SDIR", dir_listing.c_str());
}

void SDLogger::handleFileDumpMessage(const String& message) {
    String msg = "Received file dump request: " + message;
    SerialManager::getInstance().sendMessage("INFO", msg.c_str());
    
    // Extract filename from message (should be just the filename)
    String filename = message;
    filename.trim();
    
    dumpFile(filename);
}

void SDLogger::handleDeleteMessage(const String& message) {
    String filename = message;
    filename.trim();
    
    if (deleteFile(filename)) {
        SerialManager::getInstance().sendMessage("INFO", ("Deleted file: " + filename).c_str());
    } else {
        SerialManager::getInstance().sendMessage("ERROR", ("Failed to delete file: " + filename).c_str());
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
    
    SerialManager::getInstance().sendMessage("DEBUG", "SDLogger: Attempting SD card initialization...");
    
    // Reset state - don't mark as initialized until everything succeeds
    status_.card_present = false;
    status_.card_initialized = false;
    status_.file_open = false;
    
    if (!sd_card_.begin(SdioConfig(FIFO_SDIO))) {
        logger_state_ = LoggerState::ERROR_RECOVERY;
        SerialManager::getInstance().sendMessage("ERROR", "SDLogger: SD card initialization failed");
        return;
    }
    
    SerialManager::getInstance().sendMessage("DEBUG", "SDLogger: SD card begin() succeeded");
    status_.card_present = true;
    
    // Build directory cache and create log file exactly like legacy code
    updateDirectoryCache();
    
    // Check if log file was created successfully - only mark as initialized if everything worked
    if (status_.file_open && log_file_) {
        status_.card_initialized = true;  // Only set true after ALL steps succeed
        logger_state_ = LoggerState::READY;
        SerialManager::getInstance().sendMessage("INFO", "SDLogger: Ready for logging");
        
        // Log startup message exactly like legacy code
        log("[SDLogger::SDLogger] Compiled on: " __DATE__ ", " __TIME__);
    } else {
        status_.card_initialized = false;  // Ensure it remains false on failure
        logger_state_ = LoggerState::ERROR_RECOVERY;
        SerialManager::getInstance().sendMessage("ERROR", "SDLogger: Failed to create log file");
    }
}

void SDLogger::updateCardStatus() {
    // Simple card presence check - could be enhanced with actual detection
    status_.card_present = status_.card_initialized;
}

void SDLogger::processWriteBuffer() {
    if (write_buffer_.length() > 0 && status_.file_open) {
        writeBufferToFile();
    }
}

void SDLogger::performPeriodicMaintenance() {
    updateCardStatus();
    
    // Flush buffer if it has data
    if (write_buffer_.length() > 0) {
        flush();
    }
    
    // Update directory cache periodically
    if (millis() - last_directory_cache_time_ms_ > config_.directory_cache_interval_ms) {
        updateDirectoryCache();
    }
}

void SDLogger::updatePerformanceStatistics() {
    uint32_t current_time = millis();
    
    if (current_time - last_rate_calculation_time_ms_ >= 1000) {
        status_.write_rate_bps = bytes_written_this_second_;
        bytes_written_this_second_ = 0;
        last_rate_calculation_time_ms_ = current_time;
    }
    
    status_.buffer_usage_bytes = write_buffer_.length();
    status_.buffer_usage_percent = getBufferUsagePercent();
}

void SDLogger::checkSpaceAndRotate() {
    // Could implement file size checking and rotation here
    if (status_.current_file_size > config_.max_file_size_mb * 1024 * 1024) {
        if (config_.rotate_on_size_limit) {
            rotateLogFile();
        }
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
    snprintf(filename, sizeof(filename), "LOG%05lu.TXT", file_number);
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

void SDLogger::rotateLogFile() {
    closeCurrentFile();
    createNewLogFile();
}

void SDLogger::addToBuffer(const String& data) {
    write_buffer_ += data;
    last_buffer_add_time_ms_ = millis();
    buffer_write_count_++;
}

void SDLogger::writeBufferToFile() {
    if (!status_.file_open || write_buffer_.length() == 0) {
        return;
    }
    
    size_t bytes_to_write = write_buffer_.length();
    size_t bytes_written = log_file_.write(write_buffer_.c_str(), bytes_to_write);
    
    if (bytes_written > 0) {
        log_file_.flush();
        status_.current_file_size += bytes_written;
        status_.total_bytes_written += bytes_written;
        status_.total_writes++;
        status_.last_write_time_ms = millis();
        last_flush_time_ms_ = status_.last_write_time_ms;
        bytes_written_this_second_ += bytes_written;
        
        // Clear the written portion from buffer
        write_buffer_.remove(0, bytes_written);
    } else {
        // Write failed - assume card issue
        status_.write_errors++;
        status_.card_initialized = false;
        logger_state_ = LoggerState::ERROR_RECOVERY;
    }
}

void SDLogger::clearBuffer() {
    write_buffer_ = "";
    buffer_write_count_ = 0;
}

void SDLogger::handleCardError() {
    status_.card_initialized = false;
    status_.file_open = false;
    logger_state_ = LoggerState::ERROR_RECOVERY;
}

void SDLogger::attemptRecovery() {
    if (millis() - last_card_check_time_ms_ > config_.card_detect_interval_ms) {
        initializeSDCard();
        last_card_check_time_ms_ = millis();
    }
}

bool SDLogger::testCardPresence() {
    return sd_card_.begin(SdioConfig(FIFO_SDIO));
}

void SDLogger::updateDirectoryCache() {
    if (!status_.card_present) {  // Check card_present instead of card_initialized
        SerialManager::getInstance().sendMessage("DEBUG", "updateDirectoryCache: Card not present");
        return;
    }
    
    cached_directory_listing_ = "";
    static uint32_t highest_log_number = 0;
    
    // Open root directory exactly like legacy code
    FsFile rootDirectory = sd_card_.open("/");
    if (!rootDirectory) {
        SerialManager::getInstance().sendMessage("ERROR", "Could not open root directory");
        return;
    }
    
    SerialManager::getInstance().sendMessage("DEBUG", "updateDirectoryCache: Scanning directory...");
    
    // Scan for highest log file number AND build cached directory listing
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

        SerialManager::getInstance().sendMessage("DEBUG", ("Found file: " + String(fileName) + " size: " + String(fileSize)).c_str());

        // Check if this is a log file for numbering (simple pattern matching)
        if (strncmp(fileName, "LOG", 3) == 0 && strstr(fileName, ".TXT")) {
            // Extract number from LOG#####.TXT format
            char *num_start = fileName + 3;
            char *num_end = strstr(fileName, ".TXT");
            if (num_end) {
                *num_end = '\0';  // Temporarily null-terminate
                uint32_t logNumber = atoi(num_start);
                if (logNumber > highest_log_number) {
                    highest_log_number = logNumber;
                }
                *num_end = '.';  // Restore the dot
            }
        }
    }

    rootDirectory.close();

    // Create new log file exactly like legacy code
    char newLogFileName[20];
    sprintf(newLogFileName, "LOG%05lu.TXT", ++highest_log_number);
    SerialManager::getInstance().sendMessage("DEBUG", ("Attempting to create log file: " + String(newLogFileName)).c_str());
    
    log_file_ = sd_card_.open(newLogFileName, FILE_WRITE);
    
    if (log_file_) {
        status_.file_open = true;
        status_.current_filename = newLogFileName;
        status_.current_file_number = highest_log_number;
        
        // Add the new log file to the cached directory listing WITHOUT size (legacy behavior)
        if (cached_directory_listing_.length() > 0) {
            cached_directory_listing_ += "\t";
        }
        cached_directory_listing_ += newLogFileName;
        // Note: Legacy code does NOT add a size for the current log file
        
        SerialManager::getInstance().sendMessage("DEBUG", ("Final directory listing: '" + cached_directory_listing_ + "'").c_str());
        
        SerialManager::getInstance().sendMessage("INFO", ("Created log file: " + String(newLogFileName)).c_str());
    } else {
        status_.file_open = false;
        SerialManager::getInstance().sendMessage("ERROR", ("Failed to create log file: " + String(newLogFileName)).c_str());
    }
    
    last_directory_cache_time_ms_ = millis();
}

void SDLogger::parseDirectoryListing() {
    // This method could be used for processing the directory listing if needed
}

} // namespace sigyn_teensy
