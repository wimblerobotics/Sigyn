/**
 * MIT License
 * Copyright 2021 by Michael Wimble
 *
 * Interface to the RoboClaw controller.
 *
 * This module alks over serial bus and gathers several values of
 * interest, such as the motor encoder values, motor speeds, and
 * motor currents. Also, a check is made of the software version.
 *
 * If any operation fails, the serial port is dropped and restarted.
 * The average loop duration is 1.05 ms with a min of 0.54 ms and
 * a max of 3.17 ms.
 *
 * To minimum loop duration, during each call to loop(), the next
 * parameter is read from the RoboClaw via state machine, so each
 * loop() call results in one parameter being read. That means, for
 * an average loop duration 1.05 ms, and there being 8 parameters
 * read, the total duration to read all parameters is 8.4 ms on
 * average, or just under 12 frame updates per second.
 */

#pragma once

#include "RoboClaw.h"
#include "kinematics.h"
#include "tmodule.h"

class TRoboClaw : TModule {
 public:
  void DoMixedSpeedDist(int32_t m1_quad_pulses_per_second,
                        int32_t m1_max_distance,
                        int32_t m2_quad_pulses_per_second,
                        int32_t m2_max_distance);

  void DoMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                             int32_t m1_quad_pulses_per_second,
                             uint32_t m1_max_distance,
                             int32_t m2_quad_pulses_per_second,
                             uint32_t m2_max_distance);

  // Convert roll, pitch, yaw angles to a quaternion
  const void EulerToQuaternion(float roll, float pitch, float yaw, float* q);

  // Get the logic battery voltage.
  float GetBatteryLogic();

  // Get the main battery voltage.
  float GetBatteryMain();

  uint32_t getError();

  // Get the motor current (amps) for motor 1. This is
  // always a positive number.
  float GetM1Current();

  // Get the encoder counts for motor 1.
  int32_t GetM1Encoder();

  // Get the speed of motor 1 in encoder pulses/second.
  int32_t GetM1Speed();

  // Get the motor current (amps) for motor 2. This is
  // always a positive number.
  float GetM2Current();

  // Get the encoder counts for motor 1.
  int32_t GetM2Encoder();

  // Get the speed of motor 2 in encoder pulses/second.
  int32_t GetM2Speed();

  // Handle a twist message.
  void handleTwistMessage(const String& data);

  void ResetEncoders();
  void SetM1PID(float p, float i, float d, uint32_t qpps);
  void SetM2PID(float p, float i, float d, uint32_t qpps);

  // Singleton constructor.
  static TRoboClaw& singleton();

  typedef enum State {
    kCurrents,
    kEncoderM1,
    kEncoderM2,
    kLogicBattery,
    kMainBattery,
    kSpeedM1,
    kSpeedM2,
    kVersion
  } State;

 protected:
  // From TModule.
  void loop();

  // From TModule.
  virtual const char* name() { return "ROBO"; }

  // From TModule.
  void setup();

 private:
  enum WhichMotor { kLeftMotor, kRightMotor };

  static const int kDeviceAddress = 0x80;

  static const char* kDeviceVersion;

  // Private constructor.
  TRoboClaw();

  // Check for runaway condition and shutdown motors.
  void CheckForRunaway(WhichMotor which_motor);

  // Check for motor stall.
  void CheckForMotorStall();

  // Get current for motor 1 and 2;
  bool GetCurrents();

  // Get encoder value for motor 1;
  bool GetEncoderM1();

  // Get encoder value for motor 2;
  bool GetEncoderM2();

  // Get logic battery voltage;
  bool GetLogicBattery();

  // Get main battery voltage;
  bool GetMainBattery();

  // Get speed for motor 1;
  bool GetSpeedM1();

  // Get speed for motor 2;
  bool GetSpeedM2();

  // Get device version string;
  bool GetVersion();

  // Publish odometry information.
  void MoveRobotAndPublishOdometry();

  // Calculate and send motor commands based on twist message
  void UpdateMotorCommands(const Kinematics& kinematics,
                           float delta_time_minutes);

  // Calculate odometry from encoder readings
  void CalculateOdometry(const Kinematics& kinematics, float delta_time_usec);

  // Publish odometry data to serial
  void PublishOdometryData(float x_pos, float y_pos, float heading,
                           const Kinematics::velocities& current_vel);

  void PublishRoboClawStats();

  // Reestablish connection to device.
  void Reconnect();

  // Motor currents.
  int16_t g_current_m1_10ma_;
  int16_t g_current_m2_10ma_;

  // Motor encoders.
  int32_t g_encoder_m1_;
  int32_t g_encoder_m2_;

  // Battery voltages;
  int16_t g_logic_battery_;
  int16_t g_main_battery_;

  static RoboClaw g_roboclaw_;

  // Singleton instance.
  static TRoboClaw* g_singleton_;

  // Motor speeds.
  int32_t g_speed_m1_;
  int32_t g_speed_m2_;

  // State machine state.
  static State g_state_;

  // Twist message data.
  float linear_x_;
  float angular_z_;
  unsigned long last_twist_received_time_ms_;

  // Odometry state
  float x_pos_;
  float y_pos_;
  float heading_;
  bool odometry_initialized_;
  uint32_t prev_m1_ticks_;
  uint32_t prev_m2_ticks_;

  static const uint32_t kBaudRate = 38'400;
};