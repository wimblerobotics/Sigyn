#pragma once

#include "module.h"
#include "serial_manager.h"
#include <stdint.h>
#include <Arduino.h>
#include <IntervalTimer.h>

/*
 * SonarModule - Ultrasonic Distance Sensor Module
 * 
 * Manages multiple HC-SR04 ultrasonic sensors using interrupt-driven measurement
 * with timer-based sensor staggering to prevent interference.
 * 
 * Features:
 * - Supports 4 ultrasonic sensors (Front, Right, Back, Left)
 * - Interrupt-driven echo measurement for precise timing
 * - Timer-based sensor triggering to prevent interference
 * - Non-blocking operation with round-robin sensor polling
 * - Automatic safety threshold monitoring
 * - Serial output for distance data via SerialManager
 * 
 * Hardware Requirements:
 * - HC-SR04 ultrasonic sensors (4 sensors)
 * - Teensy 4.1 microcontroller
 * 
 * Wiring:
 * - Sensor 0 (Front): Echo->Pin 25, Trigger->Pin 24
 * - Sensor 1 (Right): Echo->Pin 27, Trigger->Pin 26  
 * - Sensor 2 (Back):  Echo->Pin 29, Trigger->Pin 28
 * - Sensor 3 (Left):  Echo->Pin 37, Trigger->Pin 36
 * - All VCC to 5V, all GND to GND
 */
class SonarModule : public Module {
public:
    // Sonar sensor positions
    typedef enum {
        FRONT = 0,
        RIGHT = 1,
        BACK = 2,
        LEFT = 3,
        NUMBER_SONARS = 4
    } SonarPosition;

    // Get singleton instance
    static SonarModule& singleton();
    
    // Get distance from specific sensor (returns -1 if sensor failed or invalid)
    int getDistanceMm(SonarPosition sensor);
    
    // Check if any sensor detects obstacle within safety threshold
    bool isObstacleDetected(SonarPosition sensor = NUMBER_SONARS) const;

protected:
    // Module interface implementation
    void setup() override;
    void loop() override;
    const char* name() override { return "Sonar"; }
    
    // Safety interface (inherited from Module)
    bool isUnsafe() override;
    void resetSafetyFlags() override;

private:
    // Private constructor for singleton
    SonarModule();
    
    // Singleton instance
    static SonarModule* g_instance_;
    
    // Safety threshold in millimeters
    static const int SAFETY_THRESHOLD_MM = 76; // 3 inches (3 * 25.4mm)
    
    // GPIO pins for controlling the sonar sensors
    static const uint8_t PIN_ECHO0 = 25;
    static const uint8_t PIN_TRIGGER0 = 24;
    static const uint8_t PIN_ECHO1 = 27;
    static const uint8_t PIN_TRIGGER1 = 26;
    static const uint8_t PIN_ECHO2 = 29;
    static const uint8_t PIN_TRIGGER2 = 28;
    static const uint8_t PIN_ECHO3 = 37;
    static const uint8_t PIN_TRIGGER3 = 36;

    // Timer configuration
    static const uint16_t TIMER_PERIOD_USEC = 20;              // 20µs timer period
    static const uint16_t SAMPLING_PERIOD_MSEC = 10;           // 10ms between sensor triggers
    static const uint16_t TIMER_COUNTS_PER_SAMPLING = (SAMPLING_PERIOD_MSEC * 1000) / TIMER_PERIOD_USEC;
    
    // Data transmission timing
    uint32_t last_data_send_time_;
    static const uint32_t DATA_SEND_INTERVAL_MS = 100;         // Send data every 100ms
    
    // Sensor state
    static volatile int sensor_distances_mm_[NUMBER_SONARS];   // Last measured distances
    static volatile uint8_t next_sensor_index_;                // Next sensor to trigger
    static IntervalTimer sonar_timer_;                         // Timer for sensor triggering
    static bool timer_running_;                                // Track timer state manually
    
    // Conversion factor: time to distance
    // Speed of sound = 343 m/s = 0.343 mm/µs
    // Distance = (time/2) * 0.343, simplified to time * 0.1715
    static constexpr float TIME_TO_MM_SCALER = 0.1715f;
    
    // Setup completed flag
    bool setup_completed_;
    
    // Interrupt handlers for echo pins
    static void echo0InterruptHandler();
    static void echo1InterruptHandler();
    static void echo2InterruptHandler();
    static void echo3InterruptHandler();
    
    // Timer interrupt handler for sensor triggering
    static void timerInterruptHandler();
    
    // Send distance data via SerialManager
    void sendDistanceData();
    
    // Convert sensor position to string for diagnostics
    const char* sensorPositionToString(SonarPosition pos) const;

    // Get current timer status
    bool isTimerRunning() const { return timer_running_; }
    
    // Stop timer (for debugging or shutdown)
    void stopTimer() { 
        sonar_timer_.end(); 
        timer_running_ = false;
    }
    
    // Start timer (for debugging or restart)
    void startTimer() { 
        sonar_timer_.begin(timerInterruptHandler, TIMER_PERIOD_USEC); 
        timer_running_ = true;
    }
};
