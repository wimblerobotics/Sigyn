#ifndef CONFIG_H
#define CONFIG_H

// Serial Communication
#define SERIAL_BAUD_RATE 115200
#define SERIAL_TIMEOUT_MS 10 // Timeout for reading serial commands

// Robot Physical Parameters
#define WHEEL_DIAMETER_M 0.102224144529039f       // Wheel diameter in meters (e.g., 10 cm)
#define WHEEL_RADIUS_M (WHEEL_DIAMETER_M / 2.0f)
#define WHEEL_BASE_M 0.3906f         // Distance between wheel centers in meters (e.g., 30 cm)
#define QUADRATURE_PULSES_PER_REVOLUTION 1000  // Encoder pulses per full wheel revolution
#define MAX_SECONDS_COMMANDED_TRAVEL 0.05f
#define MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS 200 // Max time to wait for a new command before stopping motors

// For a maximum speed of 1 mph (0.44704 m/s) and wheel diameter of 0.102224144529039 meters,
// and the encoder pulses per revolution of 1000, the maximum encoder pulses
// per second (QPPS) can be calculated as follows:
// wheel_circumference_m = M_PI * WHEEL_DIAMETER_M = 0.32114635
// revolutions_per_second = 0.44704 / wheel_circumference_m = 1.3920
// ticks_per_second = revolutions_per_second * QUADRATURE_PULSES_PER_REVOLUTION = 1392f
#define MAX_MOTOR_SPEED_QPPS 1392

#define MAX_ACCELERATION_QPPS2 3000

// #define MAX_MOTOR_RPM 300          // Max safe motor RPM
// #define MAX_LINEAR_SPEED_MPS 1.0f  // Max safe linear speed in m/s
// #define MAX_ANGULAR_SPEED_RPS 1.57f // Max safe angular speed in rad/s (approx 90 deg/s)

// RoboClaw Configuration
#define ROBOCLAW_ADDRESS 0x80      // RoboClaw address (default is 128)
#define ROBOCLAW_TIMEOUT_US 10'000  // RoboClaw communication timeout (10ms)
#define ROBOCLAW_BAUD_RATE 38400U
#define ROBOCLAW_SOFTWARE_VERSION "USB Roboclaw 2x15a v4.2.8\n"
#define ROBOCLAW_PUBLISH_STATUS_INTERVAL_MS 1000 // Publish status every 1000ms (1 second)
// #define ROBOCLAW_M1_MAX_CURRENT_A 5.0f // Max current for motor 1
// #define ROBOCLAW_M2_MAX_CURRENT_A 5.0f // Max current for motor 2
// // QPPS = Quarter Pulses Per Second for RoboClaw speed commands
// // Calculate QPPS from RPM: (RPM / 60) * QUADRATURE_PULSES_PER_REVOLUTION


// Safety and E-Stop
#define ROBOCLAW_SERIAL Serial6
#define E_STOP_PIN 5              // Digital pin connected to RoboClaw E-Stop or relay
// #define MOTOR_RUNAWAY_SPEED_FACTOR 1.5f // Actual speed > commanded * factor = runaway
// #define MOTOR_STALL_DETECTION_MS 500 // Time (ms) to detect stall if encoders don't change with command
// #define MOTOR_STALL_ENCODER_THRESHOLD 10 // Minimal encoder change to consider not stalled

// Battery Monitoring
#define MAIN_BATTERY_PIN 27             // Analog pin for battery voltage sensing
#define MAIN_BATTERY_READ_INTERVAL_MS 500 // Interval to read battery voltage (in ms)
#define MAIN_BATTERY_REPORT_INTERVAL_MS 1000   // Report battery every 100 main loops (~1Hz)
#define MAIN_BATTERY_MAX_VOLTAGE 42.0f // Max battery voltage (e.g., 3S LiPo fully charged)
#define MAIN_BATTERY_MIN_VOLTAGE 32.0f  // Min battery voltage (e.g., 3S LiPo empty)
#define MAIN_BATTERY_LIPO_CELLS 10       // Number of LiPo cells in series
#define MAIN_BATTERY_CRITICAL_VOLTAGE_THRESHOLD 33.0f // Critical low battery voltage threshold

// Loop Frequencies (Number of main loops per action)
// These define how often certain messages are sent or actions are performed.
// Example: If main loop is ~10ms (100Hz), ODOMETRY_SEND_INTERVAL = 5 means 100Hz/5 = 20Hz odometry.
#define ODOMETRY_UPDATE_PERIOD_MS 33 // Update odometry every 10ms (100Hz)
#define ROBOCLAW_STATUS_INTERVAL 100  // Report RoboClaw status every 100 main loops (~1Hz)
#define MODULE_STATS_INTERVAL 100     // Report module stats every 100 main loops (~1Hz)

// Math Constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#endif // CONFIG_H
