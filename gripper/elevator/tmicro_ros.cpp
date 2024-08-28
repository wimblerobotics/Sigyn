#include "tmicro_ros.h"

#include <Wire.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <stdint.h>
#include <stdio.h>

#include "tconfiguration.h"
#include "tmodule.h"
#if USE_TSD
#include "tsd.h"
#endif

#define ignore_result(x) \
  if (x) {               \
  }

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      return false;                \
    }                              \
  }

void TMicroRos::SyncTime(const char *caller, uint32_t fixed_time_call_count) {
  static const int timeout_ms = 1000;
  static uint32_t call_count = 0;
  static uint32_t time_at_last_sync = micros();

  call_count++;
  uint32_t start = micros();
  rmw_ret_t sync_result =
      rmw_uros_sync_session(timeout_ms);  // Atttempt synchronization.
  int32_t now = micros();
  float sync_duration_ms = (now - start) / 1000.0;
  float duration_since_last_sync_ms = (now - time_at_last_sync) / 1000.0;
  time_at_last_sync = now;
  if (sync_result == RMW_RET_OK) {
    ros_sync_time_ =
        rmw_uros_epoch_nanos();  // Capture the current ROS time after
                                 // attempted synchronization.
  }

  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "[TMicroRos(gripper)::SyncTime] rmw_uros_sync_session(), caller: %s"
           ", call_count: %ld, "
           ", fixed_time_call_count: %ld"
           ", call result: %ld"
           ", sync_duration_ms: %f"
           ", duration_since_last_sync_ms: %f, new time: %lld.%lld",
           caller, call_count, fixed_time_call_count, sync_result,
           sync_duration_ms, duration_since_last_sync_ms,
           ros_sync_time_ / 1'000'000'000, ros_sync_time_ % 1'000'000'000);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
}

int64_t TMicroRos::FixedTime(const char *caller) {
  static uint32_t fixed_time_call_count = 0;
  int64_t ros_time = rmw_uros_epoch_nanos();
  int64_t skew = ros_time - ros_sync_time_;
  fixed_time_call_count++;
  if (skew < 0) {
    // Time has gone backwards !
    TMicroRos::singleton().SyncTime(caller, fixed_time_call_count);
    ros_time = ros_sync_time_;
  } else if (skew > 30'000'000) {
    // The current time appears to be 30ms or more out of whack from
    // previously synced time
    TMicroRos::singleton().SyncTime(caller, fixed_time_call_count);
    ros_time = ros_sync_time_;
  } else if (skew <= 5'000'000) {
    // If the current time is within 5ms of the last synced time,
    // assume that the current time is probably good.
    // The 5 ms is because the loop rate of this monitor is expected
    // to be more than 20 frames per second.
    ros_sync_time_ = ros_time;
  }

  return ros_time;
}

void TMicroRos::loop() {
  static bool has_homed = false;
  if (!has_homed) {
    while (!AtBottomLimit()) {
      StepPulse(kDown);
    }
    has_homed = true;
  }

  switch (state_) {
    case kWaitingAgent: {
#if USE_TSD
      TSd::singleton().log("INFO [TMicroRos::loop] kWaitingAgent");
#endif
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 500) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? kAgentAvailable
                                                             : kWaitingAgent;
        last_time = uxr_millis();
      }
    } break;

    case kAgentAvailable: {
#if USE_TSD
      TSd::singleton().log("INFO [TMicroRos::loop] kAgentAvailable");
#endif
      if (CreateEntities()) {
#if USE_TSD
        TSd::singleton().log(
            "INFO [TMicroRos::loop] kAgentAvailable successful "
            "CreateEntities");
#endif
        state_ = kAgentConnected;
      } else {
#if USE_TSD
        TSd::singleton().log(
            "ERROR [TMicroRos::loop] kAgentAvailable FAILED CreateEntities");
#endif
        state_ = kWaitingAgent;
        DestroyEntities();
      }
    } break;

    case kAgentConnected: {
#if USE_TSD
      TSd::singleton().log("INFO [TMicroRos::loop] kAgentConnected");
#endif
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 10) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                     ? kAgentConnected
                     : kAgentDisconnected;
        last_time = uxr_millis();
        if (TMicroRos::singleton().state_ == kAgentConnected) {
          rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
        }
      }
    } break;

    case kAgentDisconnected: {
#if USE_TSD
      TSd::singleton().log("INFO [TMicroRos::loop] kAgentDisconnected");
#endif
      DestroyEntities();
      state_ = kWaitingAgent;
    } break;

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
    snprintf(g_singleton_->string_msg_.data.data,
             g_singleton_->string_msg_.data.capacity, "%s", msg);
    g_singleton_->string_msg_.data.size =
        strlen(g_singleton_->string_msg_.data.data);
    ignore_result(rcl_publish(&g_singleton_->diagnostics_publisher_,
                              &g_singleton_->string_msg_, nullptr));
#if USE_TSD
    TSd::singleton().log(msg);
#endif
  }
}

void TMicroRos::setup() {
  static bool is_setup = false;
  if (!is_setup) {
    Wire.begin();
    pinMode(13, OUTPUT);
    pinMode(kPinEcho2, INPUT);     // Bottom limit switch.
    pinMode(kPinTrigger2, INPUT);  // Top limit switch.
    pinMode(kPinEcho3, OUTPUT);
    pinMode(kPinTrigger3, OUTPUT);
    set_microros_transports();
    state_ = kWaitingAgent;
    while (state_ != kAgentConnected) {
      loop();
    }

    is_setup = true;
  }
}

void TMicroRos::TimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    if (timer != NULL) {
      const size_t MAXSIZE = 512;
      char stats[MAXSIZE];
      TModule::GetStatistics(stats, MAXSIZE);
      snprintf(g_singleton_->string_msg_.data.data,
               g_singleton_->string_msg_.data.capacity, "{\"Stats\": %s}",
               stats);
      g_singleton_->string_msg_.data.size =
          strlen(g_singleton_->string_msg_.data.data);
      ignore_result(rcl_publish(&g_singleton_->stats_publisher_,
                                &g_singleton_->string_msg_, nullptr));

      // #if USE_TSD
      //       TSd::singleton().log(g_singleton_->string_msg_.data.data);
      // #endif
    }
  }
}

void TMicroRos::CommandCallback(const void *msg) {
  // 0.1742, 0.178, 1400=>0.182, 2300=>0.1826, 3300=>0.181

  //  4000  1224
  //  2000  847
  //  1000  666
  //  0     488
  // -800   320
  // -1300  250
  // -1800  180
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    const std_msgs__msg__Float32 *command = (const std_msgs__msg__Float32 *)msg;

    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::CommandCallback(] command: %4.3f",
             command->data);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
    float target_position = command->data;
    if (current_position_ != target_position) {
      int32_t pulses_needed =
          abs((target_position - current_position_)) / kMmPerPulse_;
      Direction direction_to_travel =
          target_position > current_position_ ? kUp : kDown;
      StepPulse(direction_to_travel);

      while (pulses_needed-- > 0) {
        if ((direction_to_travel == kUp) && AtTopLimit()) {
          char diagnostic_message[256];
          snprintf(diagnostic_message, sizeof(diagnostic_message),
                   "INFO [TMicroRos::CommandCallback] Already at top limit, "
                   "not moving further up");
          TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
          return;
        }

        if ((direction_to_travel == kDown) && AtBottomLimit()) {
          char diagnostic_message[256];
          snprintf(diagnostic_message, sizeof(diagnostic_message),
                   "INFO [TMicroRos::CommandCallback] Already at bottom limit, "
                   "not moving further down");
          TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
          return;
        }

        char diagnostic_message[256];
        snprintf(diagnostic_message, sizeof(diagnostic_message),
                 "INFO [TMicroRos::CommandCallback] target_position: %7.6f, "
                 "current_position: %7.6f, pulses_needed: %ld",
                 target_position, current_position_, pulses_needed);
        TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
        StepPulse(direction_to_travel);
      }
    }
#if USE_TSD
    TSd::singleton().log(diagnostic_message);
#endif
  }
}

bool TMicroRos::AtBottomLimit() {
  static const float kBottomPosition = 0.2;
  bool atBottom = digitalRead(kPinEcho2);
  if (atBottom) {
    current_position_ = kBottomPosition;
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::AtBottomLimit] At Bottom of elevator, "
             "current_position_: %4.3f",
             current_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return atBottom;
}

bool TMicroRos::AtTopLimit() {
  static const float kTopPosition = 1.375;
  bool atTop = digitalRead(kPinTrigger2);
  if (atTop) {
    current_position_ = kTopPosition;
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::AtTopLimit] At Top of elevator, "
             "current_position_: %4.3f",
             current_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return atTop;
}

void TMicroRos::StepPulse(Direction direction) {
  const int pd = 500;  // Pulse Delay period
  if ((direction == kUp) && AtTopLimit()) {
    return;
  }

  if ((direction == kDown) && AtBottomLimit()) {
    return;
  }

  digitalWrite(kPinTrigger3, direction == kDown);
  digitalWrite(kPinEcho3, HIGH);
  delayMicroseconds(pd);
  digitalWrite(kPinEcho3, LOW);
  delayMicroseconds(pd);
  if (direction == kUp) {
    current_position_ += kMmPerPulse_;
  } else {
    current_position_ -= kMmPerPulse_;
  }
}

TMicroRos::TMicroRos() : TModule(TModule::kMicroRos) {
  string_msg_.data.capacity = 512;
  string_msg_.data.data =
      (char *)malloc(string_msg_.data.capacity * sizeof(char));
  string_msg_.data.size = 0;
}

bool TMicroRos::CreateEntities() {
  allocator_ = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support_, 0, nullptr, &allocator_));

  // create node
  RCCHECK(rclc_node_init_default(&node_, "gripper_node", "", &support_));

  // create publishers.
  RCCHECK(rclc_publisher_init_default(
      &diagnostics_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "gripper_diagnostics"));

  RCCHECK(rclc_publisher_init_default(
      &stats_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "gripper_stats"));

  // Create subscribers.
  RCCHECK(rclc_subscription_init_default(
      &command_subscriber_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "gripper_cmd"));

  // Create timer,
  const unsigned int timer_timeout_ns = 1'000'000'000;
  RCCHECK(rclc_timer_init_default(&timer_, &support_, timer_timeout_ns,
                                  TimerCallback));

  // Create executor
  executor_ = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_, &support_.context, 3, &allocator_));
  RCCHECK(rclc_executor_add_timer(&executor_, &timer_));
  RCCHECK(rclc_executor_add_subscription(&executor_, &command_subscriber_,
                                         &command_msg_, &CommandCallback,
                                         ON_NEW_DATA));
  return true;
}

void TMicroRos::DestroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore_result(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  ignore_result(rcl_publisher_fini(&diagnostics_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&stats_publisher_, &node_));
  ignore_result(rcl_timer_fini(&timer_));
  ignore_result(rcl_subscription_fini(&command_subscriber_, &node_));
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

TMicroRos *TMicroRos::g_singleton_ = nullptr;
const float TMicroRos::kMmPerPulse_ = 0.000181;
float TMicroRos::current_position_ = 0.0;
volatile int64_t TMicroRos::ros_sync_time_ = 0;
