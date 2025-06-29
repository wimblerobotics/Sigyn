#include "sonar_module.h"
#include <Arduino.h>
#include <IntervalTimer.h>

// Static member definitions
SonarModule* SonarModule::g_instance_ = nullptr;
volatile int SonarModule::sensor_distances_mm_[NUMBER_SONARS] = {-1, -1, -1, -1};
volatile uint8_t SonarModule::next_sensor_index_ = 0;
IntervalTimer SonarModule::sonar_timer_;
bool SonarModule::timer_running_ = false;

SonarModule& SonarModule::singleton() {
    if (g_instance_ == nullptr) {
        g_instance_ = new SonarModule();
    }
    return *g_instance_;
}

SonarModule::SonarModule() : Module() {
    last_data_send_time_ = 0;
    setup_completed_ = false;
    
    SerialManager::singleton().SendDiagnosticMessage(
        "SonarModule: Configured for " + String(NUMBER_SONARS) + " sensors");
}

void SonarModule::setup() {
    if (setup_completed_) {
        return;  // Already completed setup
    }
    
    SerialManager::singleton().SendDiagnosticMessage(
        "SonarModule: Starting sensor initialization...");
    
    // Configure GPIO pins
    pinMode(PIN_ECHO0, INPUT);
    pinMode(PIN_TRIGGER0, OUTPUT);
    pinMode(PIN_ECHO1, INPUT);
    pinMode(PIN_TRIGGER1, OUTPUT);
    pinMode(PIN_ECHO2, INPUT);
    pinMode(PIN_TRIGGER2, OUTPUT);
    pinMode(PIN_ECHO3, INPUT);
    pinMode(PIN_TRIGGER3, OUTPUT);
    
    // Ensure all trigger pins start LOW
    digitalWrite(PIN_TRIGGER0, LOW);
    digitalWrite(PIN_TRIGGER1, LOW);
    digitalWrite(PIN_TRIGGER2, LOW);
    digitalWrite(PIN_TRIGGER3, LOW);
    
    // Setup timer interrupt for sensor triggering
    sonar_timer_.begin(timerInterruptHandler, TIMER_PERIOD_USEC);
    timer_running_ = true;
    
    // Attach interrupt handlers for echo pins
    attachInterrupt(digitalPinToInterrupt(PIN_ECHO0), echo0InterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ECHO1), echo1InterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ECHO2), echo2InterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ECHO3), echo3InterruptHandler, CHANGE);
    
    SerialManager::singleton().SendDiagnosticMessage(
        "SonarModule: All sensors initialized successfully");
    
    setup_completed_ = true;
}

void SonarModule::loop() {
    if (!setup_completed_) {
        return;  // Skip if setup not completed
    }
    
    uint32_t current_time = millis();
    
    // Send data periodically (non-blocking)
    if (current_time - last_data_send_time_ >= DATA_SEND_INTERVAL_MS) {
        sendDistanceData();
        last_data_send_time_ = current_time;
    }
}

bool SonarModule::isUnsafe() {
    // Check if any sensor detects an obstacle within safety threshold
    for (int i = 0; i < NUMBER_SONARS; i++) {
        int distance = sensor_distances_mm_[i];
        if (distance > 0 && distance < SAFETY_THRESHOLD_MM) {
            return true;  // Obstacle detected within safety threshold
        }
    }
    return false;
}

void SonarModule::resetSafetyFlags() {
    // Reset any safety flags related to the sonar sensors
    // For now, this is just a placeholder as the safety state is derived
    // from current distance measurements
    SerialManager::singleton().SendDiagnosticMessage(
        "SonarModule: Safety flags reset");
}

int SonarModule::getDistanceMm(SonarPosition sensor) {
    if (sensor >= NUMBER_SONARS) {
        return -1;
    }
    return sensor_distances_mm_[sensor];
}

bool SonarModule::isObstacleDetected(SonarPosition sensor) const {
    if (sensor == NUMBER_SONARS) {
        // Check all sensors
        for (int i = 0; i < NUMBER_SONARS; i++) {
            int distance = sensor_distances_mm_[i];
            if (distance > 0 && distance < SAFETY_THRESHOLD_MM) {
                return true;
            }
        }
        return false;
    } else {
        // Check specific sensor
        int distance = sensor_distances_mm_[sensor];
        return (distance > 0 && distance < SAFETY_THRESHOLD_MM);
    }
}

void SonarModule::echo0InterruptHandler() {
    static uint32_t start_time = 0;
    
    if (digitalRead(PIN_ECHO0) == HIGH) {
        // Rising edge - start timing
        start_time = micros();
    } else {
        // Falling edge - calculate distance
        uint32_t end_time = micros();
        uint32_t duration = end_time - start_time;
        sensor_distances_mm_[FRONT] = static_cast<int>(duration * TIME_TO_MM_SCALER);
    }
}

void SonarModule::echo1InterruptHandler() {
    static uint32_t start_time = 0;
    
    if (digitalRead(PIN_ECHO1) == HIGH) {
        // Rising edge - start timing
        start_time = micros();
    } else {
        // Falling edge - calculate distance
        uint32_t end_time = micros();
        uint32_t duration = end_time - start_time;
        sensor_distances_mm_[RIGHT] = static_cast<int>(duration * TIME_TO_MM_SCALER);
    }
}

void SonarModule::echo2InterruptHandler() {
    static uint32_t start_time = 0;
    
    if (digitalRead(PIN_ECHO2) == HIGH) {
        // Rising edge - start timing
        start_time = micros();
    } else {
        // Falling edge - calculate distance
        uint32_t end_time = micros();
        uint32_t duration = end_time - start_time;
        sensor_distances_mm_[BACK] = static_cast<int>(duration * TIME_TO_MM_SCALER);
    }
}

void SonarModule::echo3InterruptHandler() {
    static uint32_t start_time = 0;
    
    if (digitalRead(PIN_ECHO3) == HIGH) {
        // Rising edge - start timing
        start_time = micros();
    } else {
        // Falling edge - calculate distance
        uint32_t end_time = micros();
        uint32_t duration = end_time - start_time;
        sensor_distances_mm_[LEFT] = static_cast<int>(duration * TIME_TO_MM_SCALER);
    }
}

void SonarModule::timerInterruptHandler() {
    typedef enum { 
        COUNTDOWN, 
        PULSE_HIGH, 
        PULSE_LOW 
    } TimerState;
    
    static volatile TimerState state = COUNTDOWN;
    static volatile uint16_t countdown = TIMER_COUNTS_PER_SAMPLING;
    
    // Decrement countdown
    if (--countdown == 0) {
        state = PULSE_HIGH;
        countdown = TIMER_COUNTS_PER_SAMPLING;
    }
    
    switch (state) {
        case COUNTDOWN:
            // Just waiting
            break;
            
        case PULSE_HIGH:
            // Send 10µs trigger pulse to current sensor
            switch (next_sensor_index_ % NUMBER_SONARS) {
                case FRONT:
                    digitalWrite(PIN_TRIGGER0, HIGH);
                    break;
                case RIGHT:
                    digitalWrite(PIN_TRIGGER1, HIGH);
                    break;
                case BACK:
                    digitalWrite(PIN_TRIGGER2, HIGH);
                    break;
                case LEFT:
                    digitalWrite(PIN_TRIGGER3, HIGH);
                    break;
            }
            state = PULSE_LOW;
            break;
            
        case PULSE_LOW:
            // End trigger pulse and move to next sensor
            switch (next_sensor_index_ % NUMBER_SONARS) {
                case FRONT:
                    digitalWrite(PIN_TRIGGER0, LOW);
                    break;
                case RIGHT:
                    digitalWrite(PIN_TRIGGER1, LOW);
                    break;
                case BACK:
                    digitalWrite(PIN_TRIGGER2, LOW);
                    break;
                case LEFT:
                    digitalWrite(PIN_TRIGGER3, LOW);
                    break;
            }
            
            next_sensor_index_++;
            state = COUNTDOWN;
            break;
    }
}

void SonarModule::sendDistanceData() {
    // Send distance data for all sensors
    String message = "SONAR_DISTANCES:";
    
    uint8_t valid_sensors = 0;
    for (uint8_t i = 0; i < NUMBER_SONARS; i++) {
        int distance = sensor_distances_mm_[i];
        
        // Count valid sensors for debugging
        if (distance > 0) {
            valid_sensors++;
        }
        
        // Append sensor data to message
        message += sensorPositionToString(static_cast<SonarPosition>(i));
        message += ":";
        if (distance < 0) {
            message += "INVALID";
        } else {
            message += String(distance);
        }
        
        // Add separator if not the last sensor
        if (i < NUMBER_SONARS - 1) {
            message += ",";
        }
    }
    
    // Add valid sensor count for debugging
    message += ",[" + String(valid_sensors) + "/" + String(NUMBER_SONARS) + " valid]";
    
    SerialManager::singleton().SendDiagnosticMessage(message);
}

const char* SonarModule::sensorPositionToString(SonarPosition pos) const {
    switch (pos) {
        case FRONT: return "FRONT";
        case RIGHT: return "RIGHT";
        case BACK:  return "BACK";
        case LEFT:  return "LEFT";
        default:    return "UNKNOWN";
    }
}
