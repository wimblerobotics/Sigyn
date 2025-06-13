#ifndef ROBOCLAWMODULE_H
#define ROBOCLAWMODULE_H

#include "RoboClaw.h"  // External RoboClaw library
#include "TModule.h"
#include "config.h"
// #include "utils.h"  // For Pose2D, Twist

// Define RoboClaw serial port if not using default HardwareSerial
// For Teensy 4.x, Serial1, Serial2, etc. are HardwareSerial objects
// Example: #define ROBOCLAW_SERIAL Serial1
// If ROBOCLAW_SERIAL is not defined, RoboClaw library might default to
// SoftwareSerial or expect a Stream object. For this example, we'll assume it's
// configured to use a HardwareSerial port passed to constructor.

class RoboClawModule : public TModule {
 public:
  static RoboClawModule& singleton();
  void handleTwistMessage(const String& data);
  bool IsUnsafe() override;
  void ResetSafetyFlags() override;

  // void SetTargetSpeed(const Twist& cmd_vel);
  // Pose2D GetCurrentOdometryPose() const; // Returns current accumulated pose
  // Twist GetCurrentVelocity() const;     // Returns current calculated
  // velocity

  // // For status reporting
  // uint16_t GetRoboClawErrorStatus();
  // float GetMotor1Current();
  // float GetMotor2Current();
  // // Add methods for temperatures if RoboClaw model supports and library
  // provides

  // bool IsEStopActive() const;

 protected:
  void loop() override;
  const char* name() override { return "Robo"; }
  void setup() override;

 private:
  struct Pose2D {
    float x;
    float y;
    float theta;
  };

  // Simple struct for Twist data
  struct Twist {
    float linear_x;   // m/s
    float angular_z;  // rad/s
  };

  typedef enum State { kReconnect, kLoop } State;

  State current_state_;  // Current state of the module for state machine

  RoboClaw roboclaw_;  // RoboClaw object initialized with serial port

  RoboClawModule();

  void doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                             int32_t m1_quad_pulses_per_second,
                             uint32_t m1_max_distance,
                             int32_t m2_quad_pulses_per_second,
                             uint32_t m2_max_distance);
  uint32_t getError();
  float getLogicBatteryVoltage();
  void getMotorCurrents(float& current_m1, float& current_m2);
  uint32_t getM1Encoder();
  uint32_t getM2Encoder();
  int32_t getM1Speed();
  int32_t getM2Speed();
  float getMainBatteryVoltage();

  // Twist message data.
  float angular_z_;
  float linear_x_;
  unsigned long last_twist_received_time_ms_;

  // Helper to constrain a value within a min and max
  template <typename T>
  T constrain_value(T val, T min_val, T max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
  }

  inline const void EulerToQuaternion(float roll, float pitch, float yaw,
                                      float* q) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
  }

  // Is the RoboClaw version compatible with this module?
  bool isVersionOk();

  // Normalize angle to be within -PI to PI
  inline float normalize_angle(float angle_rad) {
    while (angle_rad > M_PI) angle_rad -= 2.0f * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0f * M_PI;
    return angle_rad;
  }

  // Publish status of the RoboClaw device.
  void publishStatus();

  // Reconnect to RoboClaw if communication fails or version mismatch
  void reconnect();

  void setM1PID(float p, float i, float d, uint32_t qpps);
  void setM2PID(float p, float i, float d, uint32_t qpps);
  
  float ticksToMeters(long quadrature_pulses);

  void updateMotorCommands();
  void updateOdometry();

  Pose2D current_pose_;
  Twist current_velocity_;

  // Motor runaway detection variables
  int32_t last_commanded_qpps_m1_;
  int32_t last_commanded_qpps_m2_;
  unsigned long last_command_time_ms_;

  // Runaway detection state
  bool runaway_detection_initialized_;
  long last_runaway_encoder_m1_;
  long last_runaway_encoder_m2_;
  unsigned long last_runaway_check_time_ms_;

  // Runaway fault flags
  bool motor_runaway_fault_m1_;
  bool motor_runaway_fault_m2_;

  // Motor runaway detection function
  void checkForRunaway();

  //   long prev_encoder_m1_;
  //   long prev_encoder_m2_;
  //   unsigned long last_odom_update_time_us_;

  //   Twist last_commanded_twist_;
  //   int32_t last_commanded_qpps_m1_;
  //   int32_t last_commanded_qpps_m2_;
  //   unsigned long last_nonzero_command_time_ms_;
  //   unsigned long motor_stall_start_time_m1_ms_;
  //   unsigned long motor_stall_start_time_m2_ms_;

  //   // Safety flags
  //   bool e_stop_active_;
  //   bool motor_runaway_detected_;
  //   bool motor_overcurrent_detected_;
  //   bool motor_stall_detected_m1_;
  //   bool motor_stall_detected_m2_;

  //   // Internal state machine for tasks
  //   enum class RoboClawState {
  //     IDLE,
  //     READ_ENCODERS,
  //     CHECK_SAFETY,
  //     // Add more states for gathering other status data if needed
  //   };
  //   RoboClawState current_roboclaw_state_;
  //   unsigned long last_status_check_time_ms_;

  //   void updateOdometry();
  //   void applyMotorSpeeds(float linear_x, float angular_z);
  //   void checkMotorSafety();
  //   void triggerEStop(const char* reason);
  //   void resetEStop();

  static RoboClawModule* g_singleton_;
};

#endif  // ROBOCLAWMODULE_H
