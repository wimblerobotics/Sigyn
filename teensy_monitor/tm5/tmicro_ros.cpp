#include "tmicro_ros.h"

#include <Arduino.h>
#include <Wire.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "kinematics.h"
#include "tconfiguration.h"
#include "tmodule.h"
#include "troboclaw.h"
#if USE_TSD
#include "tsd.h"
#endif

Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, SuperDroid_1831.max_rpm, 1.0,
                      SuperDroid_1831.motor_operating_voltage,
                      SuperDroid_1831.motor_power_max_voltage, Sigyn_specs.wheel_diameter,
                      Sigyn_specs.wheels_separation);

#define ignore_result(x) \
  if (x) {               \
  }

void TMicroRos::SyncTime() {
  await_time_sync_ = true;  // Disable motor commands except for stop.
  unsigned long long ros_time_ms = 0;
  float sync_duration_ms = 0;
  bool unexpected_sync_duration = false;

  // Get the current time from the agent.
  unsigned long millis_val = millis();
  rmw_ret_t sync_result = rmw_uros_sync_session(10);  // Atttempt synchronization.
  if (sync_result == RMW_RET_OK) {
    ros_time_ms = rmw_uros_epoch_millis();
    unsigned long now_ms = millis();
    sync_duration_ms = now_ms - millis_val;
    unexpected_sync_duration = (sync_duration_ms > 100);

    // Now we can find the difference between ROS time and uC time.
    time_offset_ = ros_time_ms - millis_val;
  } else {
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "[TMicroRos(teensy)::SyncTime] rmw_uros_sync_session FAILED sync_result: %ld",
             sync_result);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  await_time_sync_ = false;  // Renable motor commands.

  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::SyncTime] ros_time_ms: %lld, millis_val: %lu, time_offset: "
           "%lld, ms to sync: %f %s",
           ros_time_ms, millis_val, time_offset_, sync_duration_ms,
           unexpected_sync_duration ? "###" : "");
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
}

unsigned long long TMicroRos::GetTimeMs() {
  // Add time difference between uC time and ROS time to synchronize time with ROS
  unsigned long millis_val = millis();
  unsigned long long now = millis_val + time_offset_;

  return now;
}

void TMicroRos::loop() {
  static bool showedWaitingAgent = false;
  static bool showedAgentAvailable = false;
  static bool showedAgentConnected = false;
  switch (state_) {
    case kWaitingAgent: {
#if USE_TSD
      if (!showedWaitingAgent) {
        TSd::singleton().log("INFO [TMicroRos(teensy)::loop] kWaitingAgent");
      }
#endif
      showedWaitingAgent = true;
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 500) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? kAgentAvailable : kWaitingAgent;
        last_time = uxr_millis();
      }
      break;
    }

    case kAgentAvailable: {
#if USE_TSD
      if (!showedAgentAvailable) {
        TSd::singleton().log("INFO [TMicroRos(teensy)::loop] kAgentAvailable");
      }
#endif
      showedWaitingAgent = false;
      showedAgentAvailable = true;
      if (CreateEntities()) {
#if USE_TSD
        TSd::singleton().log(
            "INFO [TMicroRos(teensy)::loop] kAgentAvailable "
            "successful CreateEntities");
#endif
        state_ = kAgentConnected;
      } else {
#if USE_TSD
        TSd::singleton().log(
            "ERROR [TMicroRos(teensy)::loop] kAgentAvailable "
            "FAILED CreateEntities");
#endif
        state_ = kWaitingAgent;
        DestroyEntities();
      }
      break;
    }

    case kAgentConnected: {
#if USE_TSD
      if (!showedAgentConnected) {
        TSd::singleton().log("INFO [TMicroRos(teensy)::loop] kAgentConnected");
      }
#endif
      showedWaitingAgent = false;
      showedAgentAvailable = false;
      showedAgentConnected = true;
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 10) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? kAgentConnected : kAgentDisconnected;
        last_time = uxr_millis();
        if (TMicroRos::singleton().state_ == kAgentConnected) {
          rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
        }
      }
      break;
    }

    case kAgentDisconnected: {
#if USE_TSD
      TSd::singleton().log("INFO [TMicroRos(teensy)::loop] kAgentDisconnected");
#endif
      showedWaitingAgent = false;
      showedAgentAvailable = false;
      showedAgentConnected = false;
      DestroyEntities();
      state_ = kWaitingAgent;
      break;
    }

    default:
      break;
  }

  {
#define LED_PIN 13
    static int32_t blink_counter = 0;
    if ((blink_counter++ % 100) == 0) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}

void TMicroRos::PublishDiagnostic(const char *msg) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    snprintf(g_singleton_->string_msg_.data.data, g_singleton_->string_msg_.data.capacity, "%s",
             msg);
    g_singleton_->string_msg_.data.size = strlen(g_singleton_->string_msg_.data.data);
    ignore_result(
        rcl_publish(&g_singleton_->diagnostics_publisher_, &g_singleton_->string_msg_, nullptr));
#if USE_TSD
    TSd::singleton().log(msg);
#endif
  }
}

void TMicroRos::PublishOdometry(float vel_dt, float linear_vel_x, float linear_vel_y,
                                float angular_vel_z) {
  static bool first_time = true;

  static float x_pos_(0.0);
  static float y_pos_(0.0);
  static float heading_(0.0);
  static uint32_t call_count = 0;

  call_count++;
  if (first_time) { //###
    first_time = false;
    x_pos_ = 0.0;
    y_pos_ = 0.0;
    heading_ = 0.0;
  }

  if (TMicroRos::singleton().state_ == kAgentConnected) {
    unsigned long long time_stamp = GetTimeMs();
    if (time_stamp == 0) {
      return;
    }

    g_singleton_->odom_msg_.header.stamp.sec = (time_stamp / 1000LL);
    g_singleton_->odom_msg_.header.stamp.nanosec = (time_stamp % 1000LL) * 1'000'000L;

    float delta_heading = angular_vel_z * vel_dt;  // radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt;  // m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt;  // m

    // calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // calculate robot's heading in quaternion angle
    // ROS has a function to calculate yaw in quaternion angle
    float q[4];
    TRoboClaw::singleton().EulerToQuaternion(0, 0, heading_, q);

    // robot's position in x,y, and z
    g_singleton_->odom_msg_.pose.pose.position.x = x_pos_;
    g_singleton_->odom_msg_.pose.pose.position.y = y_pos_;
    g_singleton_->odom_msg_.pose.pose.position.z = 0.0;

    // robot's heading in quaternion
    g_singleton_->odom_msg_.pose.pose.orientation.x = (double)q[1];
    g_singleton_->odom_msg_.pose.pose.orientation.y = (double)q[2];
    g_singleton_->odom_msg_.pose.pose.orientation.z = (double)q[3];
    g_singleton_->odom_msg_.pose.pose.orientation.w = (double)q[0];

    g_singleton_->odom_msg_.pose.covariance[0] = 0.001;
    g_singleton_->odom_msg_.pose.covariance[7] = 0.001;
    g_singleton_->odom_msg_.pose.covariance[35] = 0.001;

    // linear speed from encoders
    g_singleton_->odom_msg_.twist.twist.linear.x = linear_vel_x;
    g_singleton_->odom_msg_.twist.twist.linear.y = linear_vel_y;
    g_singleton_->odom_msg_.twist.twist.linear.z = 0.0;

    // angular speed from encoders
    g_singleton_->odom_msg_.twist.twist.angular.x = 0.0;
    g_singleton_->odom_msg_.twist.twist.angular.y = 0.0;
    g_singleton_->odom_msg_.twist.twist.angular.z = angular_vel_z;

    ignore_result(rcl_publish(&g_singleton_->odom_publisher_, &g_singleton_->odom_msg_, nullptr));
  }
}

void TMicroRos::PublishBattery(const char *frame_id, float voltage) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    // --- Update dynamic fields before publishing ---
    g_singleton_->battery_msg_.voltage = voltage;
    
    // --- Calculate Percentage (State of Charge - SoC) ---
    // This is a simple linear approximation. 
    // IMPORTANT: LiPo discharge is non-linear. For better accuracy, use a lookup table 
    // or polynomial fit based on your specific battery's discharge curve.
    const float min_voltage = 30.0; // 10S * 3.0V/cell (adjust if your cutoff is different)
    const float max_voltage = 42.0; // 10S * 4.2V/cell (fully charged)
    float percentage = (voltage - min_voltage) / (max_voltage - min_voltage);
    // Clamp the value between 0.0 and 1.0
    g_singleton_->battery_msg_.percentage = fmaxf(0.0, fminf(1.0, percentage)); 

    // --- Determine Power Supply Status ---
    // TODO: Refine this logic if you have current sensing.
    // If current > 0, it's charging. If current < 0, discharging. If near 0 and voltage is high, it's full.
    if (voltage >= 41.8) { // Close to full charge voltage
        // Assuming no current sensing, guess based on voltage.
        // If current was near zero, this would be POWER_SUPPLY_STATUS_FULL
        g_singleton_->battery_msg_.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING; // Or FULL if current is negligible
    } else if (voltage > min_voltage) { // Normal operating range
        g_singleton_->battery_msg_.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    } else { // Below minimum voltage
        g_singleton_->battery_msg_.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING; // Still discharging, but critically low
    }

    // TODO: Update current if measured.
    // g_singleton_->battery_msg_.current = measured_current; 

    // TODO: Update cell voltages if BMS data is available
    // for (int i = 0; i < 10; ++i) {
    //   g_singleton_->battery_msg_.cell_voltage.data[i] = bms_cell_voltages[i];
    // }
    
    // Update timestamp
    unsigned long long time_stamp = GetTimeMs();
    g_singleton_->battery_msg_.header.stamp.sec = (time_stamp / 1000LL);
    g_singleton_->battery_msg_.header.stamp.nanosec = (time_stamp % 1000LL) * 1'000'000L;

    ignore_result(
        rcl_publish(&g_singleton_->battery_publisher_, &g_singleton_->battery_msg_, nullptr));
  }
}

void TMicroRos::setup() {
  static bool is_setup = false;
  if (!is_setup) {
    Wire.begin();
    pinMode(13, OUTPUT);
    set_microros_transports();
    state_ = kWaitingAgent;
    while (state_ != kAgentConnected) {
      loop();
    }

    TRoboClaw::singleton().SetM1PID(7.26239, 2.43, 00, 2437);
    TRoboClaw::singleton().SetM2PID(7.26239, 2.43, 00, 2437);
    is_setup = true;
  }
}

void TMicroRos::MotorTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;

  // NOTE !!! When checked on 2024-12-28, the loop that gathered wheel encoder data
  // in troboclaw.cpp was called at  136/2 hz (68 hz).
  // This function is called at 50 hz and relies on the wheel encoder data being up to date.

  static unsigned long prev_update_time = micros();
  unsigned long current_time_us = micros();
  unsigned long delta_time_us = current_time_us - prev_update_time;
  double delta_time_minutes = ((double)delta_time_us) / 60'000'000;
  prev_update_time = current_time_us;

  if (delta_time_minutes == 0) {
    return;
  }

  if ((timer != NULL) && (TMicroRos::singleton().state_ == kAgentConnected)) {
    if (delta_time_us > 200'000) {
      // No command in a while, or first command.
      TMicroRos::singleton().twist_msg_.linear.x = 0;
      TMicroRos::singleton().twist_msg_.linear.y = 0;
      TMicroRos::singleton().twist_msg_.angular.z = 0;
    }

    Kinematics::rpm rpm = kinematics.getRPM(TMicroRos::singleton().twist_msg_.linear.x,
                                            TMicroRos::singleton().twist_msg_.linear.y,
                                            TMicroRos::singleton().twist_msg_.angular.z);

    const int32_t m1_quad_pulses_per_second =
        rpm.motor1 * SuperDroid_1831.quad_pulses_per_revolution / 60.0;
    const int32_t m2_quad_pulses_per_second =
        rpm.motor2 * SuperDroid_1831.quad_pulses_per_revolution / 60.0;
    const int32_t m1_max_distance =
        fabs(m1_quad_pulses_per_second * g_singleton_->max_seconds_uncommanded_travel_);
    const int32_t m2_max_distance =
        fabs(m2_quad_pulses_per_second * g_singleton_->max_seconds_uncommanded_travel_);

    TRoboClaw::singleton().DoMixedSpeedAccelDist(g_singleton_->accel_quad_pulses_per_second_,
                                                 m1_quad_pulses_per_second, m1_max_distance,
                                                 m2_quad_pulses_per_second, m2_max_distance);

    // Get the actual  RPM values for each motor.
    static unsigned long prev_m1_ticks = TRoboClaw::singleton().GetM1Encoder();
    static unsigned long prev_m2_ticks = TRoboClaw::singleton().GetM2Encoder();
    int32_t current_m1_ticks = TRoboClaw::singleton().GetM1Encoder();
    int32_t current_m2_ticks = TRoboClaw::singleton().GetM2Encoder();
    int32_t delta_ticks_m1 = current_m1_ticks - prev_m1_ticks;
    int32_t delta_ticks_m2 = current_m2_ticks - prev_m2_ticks;

    float current_rpm1 =
        (((float)delta_ticks_m1 / (float)SuperDroid_1831.quad_pulses_per_revolution) /
         delta_time_minutes);
    float current_rpm2 =
        (((float)delta_ticks_m2 / (float)SuperDroid_1831.quad_pulses_per_revolution) /
         delta_time_minutes);
    Kinematics::velocities current_vel =
        kinematics.getVelocities(current_rpm1, current_rpm2, 0.0, 0.0);
    prev_m1_ticks = current_m1_ticks;
    prev_m2_ticks = current_m2_ticks;
    g_singleton_->PublishOdometry(delta_time_us / 1'000'000.0, current_vel.linear_x,
                                  current_vel.linear_y, current_vel.angular_z);
  }
}

void TMicroRos::TimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
#if USE_TSD
  TSd::singleton().flush();
#endif
  // SyncTime();

  (void)last_call_time;
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    if (timer != NULL) {
      const size_t MAXSIZE = 512;
      char stats[MAXSIZE];
      TModule::GetStatistics(stats, MAXSIZE);
      snprintf(g_singleton_->string_msg_.data.data, g_singleton_->string_msg_.data.capacity,
               "{\"Stats\": %s}", stats);
      g_singleton_->string_msg_.data.size = strlen(g_singleton_->string_msg_.data.data);
      ignore_result(
          rcl_publish(&g_singleton_->teensy_stats_publisher_, &g_singleton_->string_msg_, nullptr));

      uint32_t error = TRoboClaw::singleton().getError();
      snprintf(g_singleton_->string_msg_.data.data, g_singleton_->string_msg_.data.capacity,
               "{\"LogicVoltage\":%-2.1f,\"MainVoltage\":%-2.1f,\"Encoder_"
               "Left\":%-ld,\"Encoder_Right\":"
               "%-ld,\"LeftMotorCurrent\":%-2.3f,\"RightMotorCurrent\":%-2.3f,"
               "\"LeftMotorSpeed\":%ld,\"RightMotorSpeed\":%ld,"
               "\"Error\":%-lX}",
               TRoboClaw::singleton().GetBatteryLogic(), TRoboClaw::singleton().GetBatteryMain(),
               TRoboClaw::singleton().GetM1Encoder(), TRoboClaw::singleton().GetM2Encoder(),
               TRoboClaw::singleton().GetM1Current(), TRoboClaw::singleton().GetM2Current(),
               TRoboClaw::singleton().GetM1Speed(), TRoboClaw::singleton().GetM2Speed(), error);
      g_singleton_->string_msg_.data.size = strlen(g_singleton_->string_msg_.data.data);
      ignore_result(rcl_publish(&g_singleton_->roboclaw_status_publisher_,
                                &g_singleton_->string_msg_, nullptr));
      // #if USE_TSD
      //       TSd::singleton().log(g_singleton_->string_msg_.data.data);
      // #endif
    }
  }
}

void TMicroRos::TwistCallback(const void *twist_msg) {
  // Nothing needs to be done here. The twise_msg is captured in a global variable
  // declared when subscribing, and that global varialbe is used in the motor timer callback.
}

TMicroRos::TMicroRos()
    : TModule(TModule::kMicroRos),
      accel_quad_pulses_per_second_(1000),
      max_angular_velocity_(0.07),
      max_linear_velocity_(0.3),
      max_seconds_uncommanded_travel_(0.25),
      quad_pulses_per_meter_(3100),
      wheel_radius_(0.05),
      wheel_separation_(0.3579) {
  battery_msg_.header.frame_id = micro_ros_string_utilities_init("main_battery");
  
  // --- Battery State Fields for 10S LiPo ---
  // voltage: Updated dynamically. Typical range: ~30.0V (empty) to 42.0V (full).
  battery_msg_.voltage = NAN; 
  
  // current: Update dynamically if measured.
  battery_msg_.current = NAN; 
  
  // charge, capacity, design_capacity: Set to NaN if unknown (requires BMS/coulomb counter).
  battery_msg_.charge = NAN;
  battery_msg_.capacity = NAN; // If known, set the estimated capacity in Ampere-hours (Ah) when full.
  battery_msg_.design_capacity = NAN; // If known, set the design capacity in Ah.
  
  // percentage: State of Charge (0.0 to 1.0). Calculated in PublishBattery.
  battery_msg_.percentage = NAN; 
  
  // power_supply_status: Updated dynamically.
  battery_msg_.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN; 
  
  // power_supply_health: Update if health information is available.
  battery_msg_.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  
  // power_supply_technology: Set for LiPo.
  battery_msg_.power_supply_technology =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION; // LiPo is a type of Li-ion
      
  // present: Set to true if the battery is connected.
  battery_msg_.present = true;

  // cell_voltage: Requires per-cell monitoring (BMS). 
  // If you have a BMS reporting 10 cell voltages, allocate memory and set size=10.
  // Example (if you had BMS data):
  // battery_msg_.cell_voltage.capacity = 10;
  // battery_msg_.cell_voltage.data = (float*)malloc(battery_msg_.cell_voltage.capacity * sizeof(float));
  // battery_msg_.cell_voltage.size = 10; 
  // Otherwise, leave as default (nullptr, size=0, capacity=0).
  battery_msg_.cell_voltage.data = nullptr;
  battery_msg_.cell_voltage.capacity = 0;
  battery_msg_.cell_voltage.size = 0;

  // cell_temperature: Requires per-cell monitoring (BMS). Leave as default if unavailable.
  battery_msg_.cell_temperature.data = nullptr;
  battery_msg_.cell_temperature.capacity = 0;
  battery_msg_.cell_temperature.size = 0;

  // temperature: Overall battery temperature, if available.
  battery_msg_.temperature = NAN;

  battery_msg_.location = micro_ros_string_utilities_init("Sigyn");
  battery_msg_.serial_number = micro_ros_string_utilities_init("none");

  // --- Other Initializations ---
  // ... (string_msg_, odom_msg_ initializations remain the same) ...
  string_msg_.data.capacity = 512;
  string_msg_.data.data = (char *)malloc(string_msg_.data.capacity * sizeof(char));
  string_msg_.data.size = 0;

  odom_msg_.header.frame_id.capacity = 32;
  odom_msg_.header.frame_id.data =
      (char *)malloc(odom_msg_.header.frame_id.capacity * sizeof(char));
  snprintf(odom_msg_.header.frame_id.data, odom_msg_.header.frame_id.capacity, "odom");
  odom_msg_.header.frame_id.size = strlen(odom_msg_.header.frame_id.data);
  odom_msg_.child_frame_id.capacity = 32;
  odom_msg_.child_frame_id.data = (char *)malloc(odom_msg_.child_frame_id.capacity * sizeof(char));
  snprintf(odom_msg_.child_frame_id.data, odom_msg_.header.frame_id.capacity, "base_link");
  odom_msg_.child_frame_id.size = strlen(odom_msg_.child_frame_id.data);

  odom_msg_.header.frame_id.size = 0;
  for (size_t i = 0; i < (sizeof(odom_msg_.pose.covariance) / sizeof(odom_msg_.pose.covariance[0]));
       i++) {
    odom_msg_.pose.covariance[i] = 0.0;
  }

  odom_msg_.pose.covariance[0] = 0.001;
  odom_msg_.pose.covariance[7] = 0.001;
  odom_msg_.pose.covariance[35] = 0.001;

  for (size_t i = 0;
       i < (sizeof(odom_msg_.twist.covariance) / sizeof(odom_msg_.twist.covariance[0])); i++) {
    odom_msg_.twist.covariance[i] = 0.0;
  }

  odom_msg_.twist.covariance[0] = 0.0001;
  odom_msg_.twist.covariance[7] = 0.0001;
  odom_msg_.twist.covariance[35] = 0.0001;
}

bool TMicroRos::CreateEntities() {
#define DEBUG USE_TSD && true
#if DEBUG
  char diagnostic_message[256];
#endif

  rcl_ret_t rclc_result;
  allocator_ = rcl_get_default_allocator();

  // create init_options
  rclc_result = rclc_support_init(&support_, 0, nullptr, &allocator_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_support_init result: %ld", rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // create node
  rclc_result = rclc_node_init_default(&node_, "teensy_node", "", &support_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_node_init_default result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // create publishers.
  rclc_result = rclc_publisher_init_default(&roboclaw_status_publisher_, &node_,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                            "roboclaw_status");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_publisher_init_default "
           "roboclaw_status result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_publisher_init_default(
      &battery_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "main_battery");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_publisher_init_default "
           "main_battery result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_publisher_init_default(&diagnostics_publisher_, &node_,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                            "teensy_diagnostics");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_publisher_init_default "
           "teensy_diagnostics result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_publisher_init_default(
      &odom_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "wheel_odom");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_publisher_init_default "
           "wheel_odom result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_publisher_init_default(&teensy_stats_publisher_, &node_,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                            "teensy_stats");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_publisher_init_default "
           "teensy_stats result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create subscribers.
  rclc_result = rclc_subscription_init_default(
      &cmd_vel_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_subscription_init_default "
           "cmd_vel result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create timers,
  const unsigned int timer_timeout_ns = 1'000'000'000;
  rclc_result = rclc_timer_init_default2(&timer_, &support_, timer_timeout_ns, TimerCallback, true);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_timer_init_default result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_timer_init_default2(&motor_timer_, &support_, RCL_MS_TO_NS(40),
                                         MotorTimerCallback, true);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_timer_init_default for motor result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create executor
  executor_ = rclc_executor_get_zero_initialized_executor();
  rclc_result = rclc_executor_init(&executor_, &support_.context, 3, &allocator_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_executor_init "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_executor_add_timer(&executor_, &timer_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_executor_add_timer "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_executor_add_timer(&executor_, &motor_timer_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] rclc_executor_add_timer for motor "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  SyncTime();

  rclc_result = rclc_executor_add_subscription(&executor_, &cmd_vel_subscriber_, &twist_msg_,
                                               &TwistCallback, ON_NEW_DATA);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos(teensy)::createEntities] "
           "rclc_executor_add_subscription_with elevator "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  return true;
}

void TMicroRos::DestroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore_result(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  ignore_result(rcl_publisher_fini(&battery_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&diagnostics_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&odom_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&roboclaw_status_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&teensy_stats_publisher_, &node_));
  ignore_result(rcl_subscription_fini(&cmd_vel_subscriber_, &node_));
  ignore_result(rcl_timer_fini(&motor_timer_));
  ignore_result(rcl_timer_fini(&timer_));
  ignore_result(rclc_executor_fini(&executor_));
  ignore_result(rcl_node_fini(&node_));
  ignore_result(rclc_support_fini(&support_));
}

TMicroRos &TMicroRos::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TMicroRos();
  }

  return *g_singleton_;
}

volatile bool TMicroRos::await_time_sync_ = false;
TMicroRos *TMicroRos::g_singleton_ = nullptr;
volatile unsigned long long TMicroRos::time_offset_ = 0;