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
#include <cctype>
#include <cstring>
#include <cstdlib>

namespace sigyn_teensy {

    static constexpr bool kVerboseDirScan = false; // Set true to debug per-file during SD init

    namespace {
    bool parse_log_filename_number_(const char* filename, uint32_t& out_number) {
        if (!filename) {
            return false;
        }
        // LOG#####.TXT pattern
        const size_t len = strlen(filename);
        if (len != 12) {
            return false;
        }
        if (strncmp(filename, "LOG", 3) != 0) {
            return false;
        }
        if (strcmp(filename + 8, ".TXT") != 0) {
            return false;
        }
        for (size_t i = 3; i < 8; i++) {
            if (!isdigit(static_cast<unsigned char>(filename[i]))) {
                return false;
            }
        }
        char number_str[6] = {0};
        memcpy(number_str, filename + 3, 5);
        const unsigned long file_number = strtoul(number_str, nullptr, 10);
        out_number = static_cast<uint32_t>(file_number);
        return true;
    }

    bool snappend(char* buf, size_t buf_size, size_t& offset, const char* fmt, ...) {
        if (!buf || buf_size == 0 || offset >= buf_size) {
            return true;
        }

        va_list args;
        va_start(args, fmt);
        const int written = vsnprintf(buf + offset, buf_size - offset, fmt, args);
        va_end(args);

        if (written <= 0) {
            return false;
        }

        const size_t remaining = buf_size - offset;
        const size_t advance = (static_cast<size_t>(written) < remaining) ? static_cast<size_t>(written) : (remaining - 1);
        offset += advance;

        return static_cast<size_t>(written) >= remaining;
    }

    void copy_cstr(char* dest, size_t dest_size, const char* src) {
        if (!dest || dest_size == 0) {
            return;
        }
        if (!src) {
            dest[0] = '\0';
            return;
        }
        strncpy(dest, src, dest_size - 1);
        dest[dest_size - 1] = '\0';
    }

    const char* trim_in_place(char* s) {
        if (!s) return "";
        while (*s && isspace(static_cast<unsigned char>(*s))) {
            ++s;
        }
        if (*s == '\0') {
            return s;
        }
        char* end = s + strlen(s) - 1;
        while (end >= s && isspace(static_cast<unsigned char>(*end))) {
            *end = '\0';
            --end;
        }
        return s;
    }
    } // namespace

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

        status_.current_filename[0] = '\0';
        dump_filename_[0] = '\0';
        cached_directory_listing_[0] = '\0';
        dir_stream_listing_[0] = '\0';
        cached_directory_len_ = 0;
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
            const char* dir_command = serial_mgr.getLatestSDDirCommand();
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "SDDIR command received: %s", dir_command ? dir_command : "");
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
            handleDirMessage(dir_command);
        }

        if (serial_mgr.hasNewSDFileCommand()) {
            const char* file_command = serial_mgr.getLatestSDFileCommand();
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "SDFILE command received: %s", file_command ? file_command : "");
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
            handleFileDumpMessage(file_command);
        }

        if (serial_mgr.hasNewSDPruneCommand()) {
            const char* prune_command = serial_mgr.getLatestSDPruneCommand();
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "SDPRUNE command received: %s", prune_command ? prune_command : "");
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
            handlePruneMessage(prune_command);
        }

        // Handle file dumping state machine
        processDumpStateMachine();

        // Periodic maintenance
        if (current_time - last_maintenance_time_ms_ > 1000) {
            performPeriodicMaintenance(); // Usually takes 1 ms, but I've seen as hight as 9 ms.
            last_maintenance_time_ms_ = current_time;
        }

        // Cooperatively drain buffer within a small time budget to reduce blocking
        if (write_offset_ > 0) {
            drainWriteBufferWithBudget(config_.max_write_slice_ms);
        }

        // Update performance statistics
        updatePerformanceStatistics();
    }

    void SDLogger::processDumpStateMachine() {
        // Enforce: at most one SDLINE per loop().
        // If a file dump is active, it has priority and directory streaming pauses.
        if (dump_state_ == DumpState::DUMPING && dump_file_) {
            if (dump_file_.available()) {
                // Read one line from the file without heap-backed String
                char line_buf[DUMP_CHUNK_SIZE] = {0};
                size_t n = 0;
                while (n < sizeof(line_buf) - 1) {
                    const int c = dump_file_.read();
                    if (c < 0) {
                        break;
                    }
                    if (c == '\n') {
                        break;
                    }
                    line_buf[n++] = static_cast<char>(c);
                }
                line_buf[n] = '\0';

                // Remove carriage return if present
                if (n > 0 && line_buf[n - 1] == '\r') {
                    line_buf[n - 1] = '\0';
                    --n;
                }

                SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), line_buf);
                dump_bytes_sent_ += static_cast<uint32_t>(n);
            }
            else {
                // End of file reached
                SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
                dump_file_.close();
                dump_state_ = DumpState::COMPLETE;

                char msg[192] = {0};
                snprintf(msg, sizeof(msg), "Completed dumping file: %s", dump_filename_);
                SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);

                dump_filename_[0] = '\0';
                dump_bytes_sent_ = 0;
                dump_state_ = DumpState::IDLE;
            }
            return;
        }

        // Stream directory listing entries one token per loop.
        if (dir_dump_active_ && dir_dump_ptr_ && *dir_dump_ptr_) {
            const char* p = dir_dump_ptr_;
            // Skip any accidental consecutive delimiters.
            while (*p == '\t') {
                ++p;
            }

            if (*p == '\0') {
                dir_dump_active_ = false;
                dir_dump_ptr_ = nullptr;
                SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "DIR");
                return;
            }

            const char* next_tab = strchr(p, '\t');
            const size_t token_len = next_tab ? static_cast<size_t>(next_tab - p) : strlen(p);
            if (token_len > 0) {
                char token[128] = {0};
                const size_t copy_len = (token_len < (sizeof(token) - 1U)) ? token_len : (sizeof(token) - 1U);
                memcpy(token, p, copy_len);
                token[copy_len] = '\0';
                SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), token);
            }

            if (next_tab) {
                dir_dump_ptr_ = next_tab + 1;
            } else {
                dir_dump_active_ = false;
                dir_dump_ptr_ = nullptr;
                SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "DIR");
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

    void SDLogger::log(const char* message) {
        if (!status_.card_initialized || !status_.file_open) {
            return;
        }

        uint32_t current_time = millis();

        // Create timestamped log entry using fixed buffers
        char entry[1024] = {0};
        const unsigned long secs = static_cast<unsigned long>(current_time / 1000);
        const unsigned long ms = static_cast<unsigned long>(current_time % 1000);
        snprintf(entry, sizeof(entry), "[%lu.%03lu] %s\n", secs, ms, message ? message : "");

        addToBuffer(entry);

        // Flush if buffer is getting full
        if (write_offset_ >= config_.chunk_size) {
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

        log(formatted_message);
    }

    void SDLogger::flush() {
        // Best-effort non-blocking flush: drain with budget this cycle
        if (write_offset_ > 0) {
            drainWriteBufferWithBudget(config_.max_write_slice_ms);
        }
    }

    void SDLogger::forceFlush() {
        // Forcefully drain entire buffer and sync to the card; can block
        while (write_offset_ > 0) {
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

        uint32_t next_file_number = getNextLogFileNumber();
        char filename[MAX_FILENAME_LENGTH] = {0};
        generateLogFilename(filename, sizeof(filename), next_file_number);

        {
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "Creating log file: %s", filename);
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
        }

        if (openLogFile(filename)) {
            status_.current_file_number = next_file_number;
            copy_cstr(status_.current_filename, sizeof(status_.current_filename), filename);
            status_.current_file_size = 0;
            status_.total_files_created++;

            // Persist last-used log number so numbering stays monotonic even if logs are deleted.
            writeLastLogNumberToSequenceFile(next_file_number);

            // Refresh directory listing cache (bounded) so callers see the new file.
            updateDirectoryCache();

            {
                char msg[192] = {0};
                snprintf(msg, sizeof(msg), "Created log file: %s", filename);
                SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
            }

            // Log startup message
            log("SDLogger started");

            return true;
        }

        {
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "Failed to create log file: %s", filename);
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        }
        return false;
    }

    void SDLogger::closeCurrentFile() {
        if (status_.file_open && log_file_) {
            forceFlush();
            log_file_.close();
            status_.file_open = false;
            status_.current_filename[0] = '\0';
        }
    }

    void SDLogger::refreshDirectoryCache() {
        updateDirectoryCache();
    }

    const char* SDLogger::getDirectoryListing() {
        if (millis() - last_directory_cache_time_ms_ > config_.directory_cache_interval_ms) {
            updateDirectoryCache();
        }
        return cached_directory_listing_;
    }

    bool SDLogger::dumpFile(const char* filename) {
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

        if (!filename || filename[0] == '\0') {
            SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), "ERROR: No filename specified");
            SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
            return false;
        }

        // Try to open the file
        dump_file_ = sd_card_.open(filename, O_RDONLY);
        if (!dump_file_) {
            char error_msg[192] = {0};
            snprintf(error_msg, sizeof(error_msg), "ERROR: Could not open file '%s'", filename);
            SerialManager::getInstance().sendDiagnosticMessage("SDLINE", name(), error_msg);
            SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "");
            return false;
        }

        // Set state to start dumping
        dump_state_ = DumpState::DUMPING;
        copy_cstr(dump_filename_, sizeof(dump_filename_), filename);
        dump_bytes_sent_ = 0;

        {
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "Started dumping file: %s", filename);
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
        }

        return true;
    }

    bool SDLogger::deleteFile(const char* filename) {
        if (!status_.card_initialized) {
            return false;
        }

        if (filename && sd_card_.remove(filename)) {
            updateDirectoryCache();
            return true;
        }

        return false;
    }

    bool SDLogger::deleteAllButLastLogs(uint16_t preservation_count) {
        if (!status_.card_initialized) {
            return false;
        }

        // Never delete the current log file.
        const char* current_name = status_.current_filename;

        // Clamp and normalize preserve count.
        static constexpr size_t kMaxPreserveCount = 64;
        size_t preserve = static_cast<size_t>(preservation_count);
        if (preserve == 0) {
            preserve = 1;
        }
        if (preserve > kMaxPreserveCount) {
            preserve = kMaxPreserveCount;
        }

        // First pass: find the smallest log number among the largest `preserve` logs.
        uint32_t top_numbers[kMaxPreserveCount] = {0};
        size_t top_count = 0;
        uint32_t total_logs = 0;

        FsFile root = sd_card_.open("/");
        if (!root) {
            return false;
        }

        while (true) {
            FsFile entry = root.openNextFile();
            if (!entry) {
                break;
            }

            char filename[MAX_FILENAME_LENGTH] = {0};
            entry.getName(filename, sizeof(filename));
            entry.close();

            uint32_t log_num = 0;
            if (!parse_log_filename_number_(filename, log_num)) {
                continue;
            }

            total_logs++;

            if (top_count < preserve) {
                // Insert into sorted ascending list.
                size_t insert_pos = top_count;
                top_numbers[top_count++] = log_num;
                while (insert_pos > 0 && top_numbers[insert_pos - 1] > top_numbers[insert_pos]) {
                    const uint32_t tmp = top_numbers[insert_pos - 1];
                    top_numbers[insert_pos - 1] = top_numbers[insert_pos];
                    top_numbers[insert_pos] = tmp;
                    insert_pos--;
                }
            } else {
                // Replace the smallest if this is larger.
                if (log_num > top_numbers[0]) {
                    top_numbers[0] = log_num;
                    // Bubble up to keep ascending order.
                    size_t pos = 0;
                    while ((pos + 1) < top_count && top_numbers[pos] > top_numbers[pos + 1]) {
                        const uint32_t tmp = top_numbers[pos];
                        top_numbers[pos] = top_numbers[pos + 1];
                        top_numbers[pos + 1] = tmp;
                        pos++;
                    }
                }
            }
        }
        root.close();

        if (total_logs <= preserve || top_count == 0) {
            // Nothing to delete.
            return true;
        }

        const uint32_t min_preserved = top_numbers[0];

        // Second pass: delete logs with number < min_preserved (except current file).
        uint32_t deleted = 0;
        uint32_t failed = 0;
        FsFile root2 = sd_card_.open("/");
        if (!root2) {
            return false;
        }

        while (true) {
            FsFile entry = root2.openNextFile();
            if (!entry) {
                break;
            }

            char filename[MAX_FILENAME_LENGTH] = {0};
            entry.getName(filename, sizeof(filename));
            entry.close();

            uint32_t log_num = 0;
            if (!parse_log_filename_number_(filename, log_num)) {
                continue;
            }

            if (log_num < min_preserved) {
                if (current_name[0] != '\0' && strcmp(filename, current_name) == 0) {
                    // Safety: never delete the active log.
                    continue;
                }
                if (sd_card_.remove(filename)) {
                    deleted++;
                } else {
                    failed++;
                }
            }
        }

        root2.close();

        updateDirectoryCache();

        {
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "Pruned logs: kept_last=%u, deleted=%lu, failed=%lu", (unsigned)preservation_count,
                     (unsigned long)deleted, (unsigned long)failed);
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
        }

        return failed == 0;
    }

    void SDLogger::handleDirMessage(const char* message) {
        if (!status_.card_initialized) {
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "SD card not initialized");
            return;
        }

        // Cancel any in-progress directory streaming and restart.
        dir_dump_active_ = false;
        dir_dump_ptr_ = nullptr;

        // Get current directory listing
        const char* dir_listing = getDirectoryListing();

        // Stream directory listing as:
        // - SDIR: header line (counts)
        // - SDLINE: one entry per line
        // - SDEOF: end marker
        // This avoids overflowing SerialManager::kMaxMessageLength once the DIAG JSON envelope is added.
        // Snapshot into a stable buffer so directory-cache refreshes during streaming
        // (e.g., log rotation calling updateDirectoryCache()) can't invalidate our token pointer.
        copy_cstr(dir_stream_listing_, sizeof(dir_stream_listing_), dir_listing ? dir_listing : "");
        const char* listing = dir_stream_listing_;

        // Header is everything before the first tab delimiter.
        const char* first_tab = strchr(listing, '\t');
        {
            char header[192] = {0};
            if (first_tab && first_tab > listing) {
                const size_t header_len = static_cast<size_t>(first_tab - listing);
                const size_t copy_len = (header_len < (sizeof(header) - 1U)) ? header_len : (sizeof(header) - 1U);
                memcpy(header, listing, copy_len);
                header[copy_len] = '\0';
            } else {
                copy_cstr(header, sizeof(header), listing);
            }
            SerialManager::getInstance().sendDiagnosticMessage("SDIR", name(), header);
        }

        // Entries are tab-delimited tokens after the header.
        // Stream these one SDLINE per loop() to avoid bursty serial output.
        if (first_tab && *(first_tab + 1) != '\0') {
            dir_dump_ptr_ = first_tab + 1;
            dir_dump_active_ = true;
        } else {
            SerialManager::getInstance().sendDiagnosticMessage("SDEOF", name(), "DIR");
        }
    }

    void SDLogger::handleFileDumpMessage(const char* message) {
        char msg[256] = {0};
        snprintf(msg, sizeof(msg), "Received file dump request: %s", message ? message : "");
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);

        // Extract filename from message (should be just the filename)
        char filename_buf[MAX_FILENAME_LENGTH] = {0};
        copy_cstr(filename_buf, sizeof(filename_buf), message ? message : "");
        const char* trimmed = trim_in_place(filename_buf);
        dumpFile(trimmed);
    }

    void SDLogger::handleDeleteMessage(const char* message) {
        char filename_buf[MAX_FILENAME_LENGTH] = {0};
        copy_cstr(filename_buf, sizeof(filename_buf), message ? message : "");
        const char* trimmed = trim_in_place(filename_buf);

        if (deleteFile(trimmed)) {
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "Deleted file: %s", trimmed);
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
        }
        else {
            char msg[192] = {0};
            snprintf(msg, sizeof(msg), "Failed to delete file: %s", trimmed);
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        }
    }

    void SDLogger::handlePruneMessage(const char* message) {
        // Expect: "<count>" (uint16) or empty
        char buf[32] = {0};
        copy_cstr(buf, sizeof(buf), message ? message : "");
        const char* trimmed = trim_in_place(buf);

        char* endptr = nullptr;
        unsigned long keep = 0;
        if (trimmed && trimmed[0] != '\0') {
            keep = strtoul(trimmed, &endptr, 10);
            if (endptr == trimmed) {
                SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "SDPRUNE: invalid count");
                return;
            }
        }

        if (status_.current_filename[0] == '\0') {
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "SDPRUNE: no current log file");
            return;
        }

        (void)deleteAllButLastLogs(static_cast<uint16_t>(keep));
    }

    bool SDLogger::isSDAvailable() const {
        return status_.card_present && status_.card_initialized;
    }

    uint32_t SDLogger::getBufferUsagePercent() const {
        if (config_.buffer_size == 0) return 0;
        const uint32_t denom = (config_.buffer_size > kWriteBufferSize) ? static_cast<uint32_t>(kWriteBufferSize) : config_.buffer_size;
        if (denom == 0) return 0;
        return (static_cast<uint32_t>(write_offset_) * 100) / denom;
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
            char board_info[64] = {0};
#ifdef BOARD_ID
            snprintf(board_info, sizeof(board_info), "Board %d", BOARD_ID);
#else
            snprintf(board_info, sizeof(board_info), "Board Unknown");
#endif

            const char* role = "";
#if defined(BOARD_ID) && BOARD_ID == 1
            role = " (Navigation_Safety)";
#elif defined(BOARD_ID) && BOARD_ID == 2
            role = " (Power_Sensors)";
#endif

            char startup_msg[192] = {0};
            snprintf(startup_msg, sizeof(startup_msg), "[SDLogger::SDLogger] %s%s - Compiled on: " __DATE__ ", " __TIME__,
                     board_info, role);
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
        if (write_offset_ > 0) {
            flush();
        }
    }

    void SDLogger::updatePerformanceStatistics() {
        uint32_t current_time = millis();

        if (current_time - last_rate_calculation_time_ms_ >= 1000) {
            status_.write_rate_bps = bytes_written_this_second_;
            bytes_written_this_second_ = 0;
            last_rate_calculation_time_ms_ = current_time;

            status_.buffer_usage_bytes = static_cast<uint32_t>(write_offset_);
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

    uint32_t SDLogger::findHighestLogFileNumber() {
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

            uint32_t log_num = 0;
            if (parse_log_filename_number_(filename, log_num)) {
                if (log_num > highest_number) {
                    highest_number = log_num;
                }
            }
        }

        root.close();
        return highest_number;
    }

    uint32_t SDLogger::readLastLogNumberFromSequenceFile() {
        if (!status_.card_initialized) {
            return 0;
        }

        FsFile f = sd_card_.open(kLogSequenceFilename, O_RDONLY);
        if (!f) {
            return 0;
        }

        char buf[32] = {0};
        size_t n = 0;
        while (n < sizeof(buf) - 1) {
            const int c = f.read();
            if (c < 0) {
                break;
            }
            if (c == '\n' || c == '\r') {
                break;
            }
            buf[n++] = static_cast<char>(c);
        }
        buf[n] = '\0';
        f.close();

        const char* trimmed = trim_in_place(buf);
        if (!trimmed || trimmed[0] == '\0') {
            return 0;
        }

        char* endptr = nullptr;
        const unsigned long v = strtoul(trimmed, &endptr, 10);
        if (endptr == trimmed) {
            return 0;
        }
        return static_cast<uint32_t>(v);
    }

    void SDLogger::writeLastLogNumberToSequenceFile(uint32_t last_number) {
        if (!status_.card_initialized) {
            return;
        }

        FsFile f = sd_card_.open(kLogSequenceFilename, O_WRONLY | O_CREAT | O_TRUNC);
        if (!f) {
            return;
        }

        char buf[32] = {0};
        const int n = snprintf(buf, sizeof(buf), "%lu\n", (unsigned long)last_number);
        if (n > 0) {
            f.write(buf, static_cast<size_t>(n));
        }
        f.close();
    }

    uint32_t SDLogger::getNextLogFileNumber() {
        const uint32_t seq_last = readLastLogNumberFromSequenceFile();
        const uint32_t scanned_highest = findHighestLogFileNumber();
        const uint32_t last = (seq_last > scanned_highest) ? seq_last : scanned_highest;
        return last + 1;
    }

    void SDLogger::generateLogFilename(char* out, size_t out_size, uint32_t file_number) {
        if (!out || out_size == 0) {
            return;
        }
        snprintf(out, out_size, "LOG%05lu.TXT", (unsigned long)file_number);
    }

    bool SDLogger::openLogFile(const char* filename) {
        if (!filename || filename[0] == '\0') {
            return false;
        }
        log_file_ = sd_card_.open(filename, O_WRONLY | O_CREAT | O_APPEND);
        if (log_file_) {
            status_.file_open = true;
            copy_cstr(status_.current_filename, sizeof(status_.current_filename), filename);
            return true;
        }
        return false;
    }

    void SDLogger::addToBuffer(const char* data) {
        if (!data) {
            return;
        }
        const size_t data_len = strlen(data);
        if (data_len == 0) {
            return;
        }

        last_buffer_add_time_ms_ = millis();
        bytes_written_this_second_ += static_cast<uint32_t>(data_len);

        const uint32_t effective_capacity = (config_.buffer_size > kWriteBufferSize) ? static_cast<uint32_t>(kWriteBufferSize) : config_.buffer_size;
        const size_t capacity = (effective_capacity == 0) ? kWriteBufferSize : static_cast<size_t>(effective_capacity);

        // If the message won't fit, drain and/or truncate.
        if (data_len > capacity) {
            // Flush existing, then keep the tail end that fits (preserves newline-terminated records better than head).
            while (write_offset_ > 0) {
                writeBufferToFile();
            }
            const char* start = data + (data_len - (capacity - 1));
            copy_cstr(write_buffer_, capacity, start);
            write_offset_ = strlen(write_buffer_);
            return;
        }

        // Ensure we have room; if not, write out some.
        while (write_offset_ + data_len >= capacity) {
            writeBufferToFile();
            if (!status_.file_open) {
                return;
            }
            if (write_offset_ == 0) {
                break;
            }
        }

        const size_t copy_len = ((write_offset_ + data_len) < capacity) ? data_len : (capacity - write_offset_ - 1);
        if (copy_len > 0) {
            memcpy(write_buffer_ + write_offset_, data, copy_len);
            write_offset_ += copy_len;
            write_buffer_[write_offset_] = '\0';
        }
    }

    void SDLogger::writeBufferToFile() {
        if (!status_.file_open || write_offset_ == 0) {
            return;
        }

        // Write at most a chunk to bound latency
        size_t bytes_to_write = write_offset_;
        if (bytes_to_write > config_.chunk_size) {
            bytes_to_write = config_.chunk_size;
        }
        size_t bytes_written = log_file_.write(write_buffer_, bytes_to_write);

        if (bytes_written > 0) {
            // Update file statistics (don't double-count bytes - they were counted in addToBuffer)
            status_.current_file_size += bytes_written;
            status_.total_bytes_written += bytes_written;
            status_.total_writes++;
            status_.last_write_time_ms = millis();

            // Clear the written portion from buffer
            if (bytes_written < write_offset_) {
                memmove(write_buffer_, write_buffer_ + bytes_written, write_offset_ - bytes_written);
                write_offset_ -= bytes_written;
                write_buffer_[write_offset_] = '\0';
            } else {
                write_offset_ = 0;
                write_buffer_[0] = '\0';
            }

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
        while (write_offset_ > 0) {
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

        cached_directory_listing_[0] = '\0';
        cached_directory_len_ = 0;

        // Keep the last N log files (LOG#####.TXT) in numeric order so output reflects
        // creation/build order and remains stable even after deletions.
        static constexpr size_t kKeepCount = SDLogger::kDirectoryListingKeepCount;
        static constexpr size_t kOtherEntryBufSize = 64;
        struct LogEntry {
            uint32_t number;
            uint32_t size;
        };
        LogEntry kept[kKeepCount] = {};
        size_t kept_count = 0;
        uint32_t total_log_files = 0;
        uint32_t total_files = 0;

        // Tail buffer for non-log files (in directory iteration order).
        char other_tail[kKeepCount][kOtherEntryBufSize] = {{0}};
        size_t other_tail_count = 0;
        size_t other_tail_write_idx = 0;

        // Open root directory
        FsFile rootDirectory = sd_card_.open("/");
        if (!rootDirectory) {
            SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Could not open root directory");
            return;
        }

        if (kVerboseDirScan) {
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "updateDirectoryCache: Scanning directory...");
        }

        // Build cached directory listing for log files only - no file creation
        while (true) {
            FsFile nextFileInDirectory = rootDirectory.openNextFile();
            if (!nextFileInDirectory) {
                break;
            }
            char fileName[MAX_FILENAME_LENGTH] = {0};
            nextFileInDirectory.getName(fileName, sizeof(fileName));

            // Skip directories.
            if (nextFileInDirectory.isDirectory()) {
                nextFileInDirectory.close();
                continue;
            }

            // Skip internal bookkeeping file
            if (strcmp(fileName, kLogSequenceFilename) == 0) {
                nextFileInDirectory.close();
                continue;
            }

            // Get file size before closing (used for both logs and non-logs)
            const uint32_t fileSize = nextFileInDirectory.size();

            uint32_t log_num = 0;
            const bool is_log = parse_log_filename_number_(fileName, log_num);
            nextFileInDirectory.close();

            total_files++;

            if (!is_log) {
                // Keep a tail of non-log files (bounded, in iteration order).
                char entry_buf[kOtherEntryBufSize] = {0};
                snprintf(entry_buf, sizeof(entry_buf), "%s,%lu", fileName, (unsigned long)fileSize);

                copy_cstr(other_tail[other_tail_write_idx], sizeof(other_tail[other_tail_write_idx]), entry_buf);
                other_tail_write_idx = (other_tail_write_idx + 1) % kKeepCount;
                if (other_tail_count < kKeepCount) {
                    other_tail_count++;
                }

                if (kVerboseDirScan) {
                    char dbg[192] = {0};
                    snprintf(dbg, sizeof(dbg), "Found file: %s size: %lu", fileName, (unsigned long)fileSize);
                    SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), dbg);
                }
                continue;
            }

            total_log_files++;

            // Maintain a sorted-ascending set of the largest kKeepCount log numbers.
            if (kept_count < kKeepCount) {
                size_t insert_pos = kept_count;
                kept[kept_count++] = {log_num, fileSize};
                while (insert_pos > 0 && kept[insert_pos - 1].number > kept[insert_pos].number) {
                    const LogEntry tmp = kept[insert_pos - 1];
                    kept[insert_pos - 1] = kept[insert_pos];
                    kept[insert_pos] = tmp;
                    insert_pos--;
                }
            } else {
                if (log_num > kept[0].number) {
                    kept[0] = {log_num, fileSize};
                    size_t pos = 0;
                    while ((pos + 1) < kept_count && kept[pos].number > kept[pos + 1].number) {
                        const LogEntry tmp = kept[pos];
                        kept[pos] = kept[pos + 1];
                        kept[pos + 1] = tmp;
                        pos++;
                    }
                }
            }

            if (kVerboseDirScan) {
                char dbg[192] = {0};
                snprintf(dbg, sizeof(dbg), "Found log: %s size: %lu", fileName, (unsigned long)fileSize);
                SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), dbg);
            }
        }

        rootDirectory.close();

        // Always include the header line, then include the last N entries.
        cached_directory_listing_[0] = '\0';
        cached_directory_len_ = 0;

        const size_t other_output_max = (kKeepCount > kept_count) ? (kKeepCount - kept_count) : 0;
        const size_t other_output_count = (other_tail_count > other_output_max) ? other_output_max : other_tail_count;

        const uint32_t omitted = (total_files > static_cast<uint32_t>(kept_count + other_output_count))
                         ? (total_files - static_cast<uint32_t>(kept_count + other_output_count))
                         : 0;
        (void)snappend(cached_directory_listing_, sizeof(cached_directory_listing_), cached_directory_len_,
                   "TOTAL FILES: %lu, OMITTED_FILES: %lu", (unsigned long)total_files,
                       (unsigned long)omitted);

        for (size_t i = 0; i < kept_count; i++) {
            char fname[MAX_FILENAME_LENGTH] = {0};
            generateLogFilename(fname, sizeof(fname), kept[i].number);
            (void)snappend(cached_directory_listing_, sizeof(cached_directory_listing_), cached_directory_len_, "\t%s,%lu",
                           fname, (unsigned long)kept[i].size);
        }

        if (other_output_count > 0) {
            const size_t start_idx = (other_tail_write_idx + kKeepCount - other_tail_count) % kKeepCount;
            const size_t first = other_tail_count - other_output_count;
            for (size_t i = 0; i < other_output_count; i++) {
                const size_t idx = (start_idx + first + i) % kKeepCount;
                (void)snappend(cached_directory_listing_, sizeof(cached_directory_listing_), cached_directory_len_, "\t%s",
                               other_tail[idx]);
            }
        }

        if (kVerboseDirScan) {
            // Potentially very long, only emit when verbose
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "Directory listing:");
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), cached_directory_listing_);
        }
        else {
            char msg[96];
            snprintf(msg, sizeof(msg), "Directory cached: %lu files", (unsigned long)total_files);
            SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
        }

        last_directory_cache_time_ms_ = millis();
    }

} // namespace sigyn_teensy
