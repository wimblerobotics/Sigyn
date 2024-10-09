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
    // ### Do the same for the extender.
    while (!ElevatorAtBottomLimit()) {
      ElevatorStepPulse(kDown);
    }

    while (!ExtenderAtInLimit()) {
      ExtenderStepPulse(kDown);
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

      doExtenderCommand();
      doElevatorCommand();
      // {
      //   char msg[256];
      //   snprintf(msg, sizeof(msg),
      //            "INFO [TMicroRos::loop] current_elevator_position_: %4.3f"
      //            ", at top: %s"
      //            ", at bottom: %s"
      //            ", extended: %s"
      //            ", retracted: %s",
      //             current_elevator_position_,
      //            ElevatorAtTopLimit() ? "true" : "false",
      //            ElevatorAtBottomLimit() ? "true" : "false",
      //            ExtenderAtOutLimit() ? "true" : "false",
      //            ExtenderAtInLimit() ? "true" : "false");
      //   TMicroRos::singleton().PublishDiagnostic(msg);
      // }
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

    pinMode(kElevatorBottomLimitSwitchPin, INPUT);
    pinMode(kElevatorTopLimitSwitchPin, INPUT);
    pinMode(kElevatorStepPulsePin, OUTPUT);
    pinMode(kElevatorStepDirectionPin, OUTPUT);

    pinMode(kExtenderInLimitSwitchPin, INPUT);
    pinMode(kExtenderOutLimitSwitchPin, INPUT);
    pinMode(kExtenderStepPulsePin, OUTPUT);
    pinMode(kExtenderStepDirectionPin, OUTPUT);

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

void TMicroRos::ElevatorCommandCallback(const void *msg) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    const std_msgs__msg__Float32 *command = (const std_msgs__msg__Float32 *)msg;

    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::ElevatorCommandCallback(] command: %4.3f",
             command->data);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
    float target_position = command->data;
    if (current_elevator_position_ != target_position) {
      elevator_remaining_pulses_ =
          (target_position - current_elevator_position_) / kElevatorMmPerPulse_;
      elevator_has_command_ = true;

      char diagnostic_message[256];
      snprintf(
          diagnostic_message, sizeof(diagnostic_message),
          "INFO [TMicroRos::ElevatorCommandCallback] target_position: %7.6f, "
          "current_position: %7.6f, remaining_pulses: %ld",
          target_position, current_elevator_position_,
          elevator_remaining_pulses_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
    }
  }
#if USE_TSD
  TSd::singleton().log(diagnostic_message);
#endif
}

void TMicroRos::doElevatorCommand() {
  Direction direction_to_travel = elevator_remaining_pulses_ > 0 ? kUp : kDown;
  char diagnostic_message[256];
  if (elevator_has_command_) {
    if ((direction_to_travel == kUp) && ElevatorAtTopLimit()) {
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "INFO [TMicroRos::doElevatorCommand] Already at top limit, "
               "not moving further up, current_position: %7.6f",
               current_elevator_position_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      elevator_has_command_ = false;
      return;
    }

    if ((direction_to_travel == kDown) && ElevatorAtBottomLimit()) {
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "INFO [TMicroRos::doElevatorCommand] Already at bottom limit, "
               "not moving further down. current_position: %7.6f",
               current_extender_position_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      elevator_has_command_ = false;
      return;
    }

    ElevatorStepPulse(direction_to_travel);
    if (elevator_remaining_pulses_ > 0) {
      elevator_remaining_pulses_--;
    } else if (elevator_remaining_pulses_ < 0) {
      elevator_remaining_pulses_++;
    }

    if (elevator_remaining_pulses_ == 0) {
      elevator_has_command_ = false;
    }

    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::doElevatorCommand] remaining_pulses: %ld, "
             "current_position: %7.6f",
             elevator_remaining_pulses_, current_elevator_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }
}

void TMicroRos::doExtenderCommand() {
  Direction direction_to_travel = extender_remaining_pulses_ > 0 ? kUp : kDown;
  char diagnostic_message[256];
  if (extender_has_command_) {
    if ((direction_to_travel == kUp) && ExtenderAtOutLimit()) {
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "INFO [TMicroRos::doExtenderCommand] Already at out limit, "
               "not moving further out, current_position: %7.6f",
               current_extender_position_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      extender_has_command_ = false;
      return;
    }

    if ((direction_to_travel == kDown) && ExtenderAtInLimit()) {
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "INFO [TMicroRos::doExtenderCommand] Already at in limit, "
               "not moving further in. current_position: %7.6f",
               current_extender_position_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      extender_has_command_ = false;
      return;
    }

    ExtenderStepPulse(direction_to_travel);
    if (extender_remaining_pulses_ > 0) {
      extender_remaining_pulses_--;
    } else if (extender_remaining_pulses_ < 0) {
      extender_remaining_pulses_++;
    }

    if (extender_remaining_pulses_ == 0) {
      extender_has_command_ = false;
    }

    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::doExtenderCommand] remaining_pulses: %ld, "
             "current_position: %7.6f",
             extender_remaining_pulses_, current_extender_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }
}

void TMicroRos::ExtenderCommandCallback(const void *msg) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    const std_msgs__msg__Float32 *command = (const std_msgs__msg__Float32 *)msg;

    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::ExtenderCommandCallback(] command: %4.3f",
             command->data);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
    float target_position = command->data;
    if (current_extender_position_ != target_position) {
      extender_remaining_pulses_ =
          (target_position - current_extender_position_) / kExtenderMmPerPulse_;
      extender_has_command_ = true;

      char diagnostic_message[256];
      snprintf(
          diagnostic_message, sizeof(diagnostic_message),
          "INFO [TMicroRos::ExtenderCommandCallback] target_position: %7.6f, "
          "current_position: %7.6f, remaining_pulses: %ld",
          target_position, current_extender_position_,
          extender_remaining_pulses_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
    }
  }
#if USE_TSD
  TSd::singleton().log(diagnostic_message);
#endif
}

bool TMicroRos::ElevatorAtBottomLimit() {
  static const float kBottomPosition = 0.0;
  bool atBottom = digitalRead(kElevatorBottomLimitSwitchPin);
  if (atBottom) {
    current_elevator_position_ = kBottomPosition;
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::ElevatorAtBottomLimit] At bottom of elevator, "
             "current_position_: %4.3f",
             current_elevator_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return atBottom;
}

bool TMicroRos::ElevatorAtTopLimit() {
  static const float kTopPosition = 0.9;
  bool atTop = digitalRead(kElevatorTopLimitSwitchPin);
  if (atTop) {
    current_elevator_position_ = kTopPosition;
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::ElevatorAtTopLimit] At top of elevator, "
             "current_position_: %4.3f",
             current_elevator_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return atTop;
}

bool TMicroRos::ExtenderAtInLimit() {
  static const float kRetractedPosition = 0.0;
  bool atBottom = !digitalRead(kExtenderInLimitSwitchPin);
  if (atBottom) {
    current_extender_position_ = kRetractedPosition;
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::ExtenderAtInLimit] Extender fully retracted, "
             "current_position_: %4.3f",
             current_extender_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return atBottom;
}

bool TMicroRos::ExtenderAtOutLimit() {
  static const float kExtendedPosition = 0.342;
  bool atTop = !digitalRead(kExtenderOutLimitSwitchPin);
  if (atTop) {
    current_extender_position_ = kExtendedPosition;
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMicroRos::ExtenderAtOutLimit] Extender fully extended, "
             "current_position_: %4.3f",
             current_extender_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return atTop;
}

void TMicroRos::ElevatorStepPulse(Direction direction) {
  const int pd = 500;  // Pulse Delay period
  if ((direction == kUp) && ElevatorAtTopLimit()) {
    return;
  }

  if ((direction == kDown) && ElevatorAtBottomLimit()) {
    return;
  }

  digitalWrite(kElevatorStepDirectionPin, direction == kDown);
  digitalWrite(kElevatorStepPulsePin, HIGH);
  delayMicroseconds(pd);
  digitalWrite(kElevatorStepPulsePin, LOW);
  delayMicroseconds(pd);
  if (direction == kUp) {
    current_elevator_position_ += kElevatorMmPerPulse_;
  } else {
    current_elevator_position_ -= kElevatorMmPerPulse_;
  }
}

void TMicroRos::ExtenderStepPulse(Direction direction) {
  const int pd = 500;  // Pulse Delay period
  if ((direction == kUp) && ExtenderAtOutLimit()) {
    return;
  }

  if ((direction == kDown) && ExtenderAtInLimit()) {
    return;
  }

  digitalWrite(kExtenderStepDirectionPin, direction == kUp);
  digitalWrite(kExtenderStepPulsePin, HIGH);
  delayMicroseconds(pd);
  digitalWrite(kExtenderStepPulsePin, LOW);
  delayMicroseconds(pd);
  if (direction == kUp) {
    current_extender_position_ += kExtenderMmPerPulse_;
  } else {
    current_extender_position_ -= kExtenderMmPerPulse_;
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
      &elevator_command_subscriber_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "elevator_cmd"));
  RCCHECK(rclc_subscription_init_default(
      &extender_command_subscriber_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "extender_cmd"));

  // Create timer,
  const unsigned int timer_timeout_ns = 1'000'000'000;
  RCCHECK(rclc_timer_init_default(&timer_, &support_, timer_timeout_ns,
                                  TimerCallback));

  // Create executor
  executor_ = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_, &support_.context, 5, &allocator_));
  RCCHECK(rclc_executor_add_timer(&executor_, &timer_));
  RCCHECK(rclc_executor_add_subscription(
      &executor_, &elevator_command_subscriber_, &elevator_command_msg_,
      &ElevatorCommandCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor_, &extender_command_subscriber_, &extender_command_msg_,
      &ExtenderCommandCallback, ON_NEW_DATA));
  return true;
}

void TMicroRos::DestroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore_result(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  ignore_result(rcl_publisher_fini(&diagnostics_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&stats_publisher_, &node_));
  ignore_result(rcl_timer_fini(&timer_));
  ignore_result(rcl_subscription_fini(&elevator_command_subscriber_, &node_));
  ignore_result(rcl_subscription_fini(&extender_command_subscriber_, &node_));
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
const float TMicroRos::kElevatorMmPerPulse_ = 0.000180369;
const float TMicroRos::kExtenderMmPerPulse_ = 0.000149626;
float TMicroRos::current_elevator_position_ = 0.0;
float TMicroRos::current_extender_position_ = 0.0;
volatile int64_t TMicroRos::ros_sync_time_ = 0;

int32_t TMicroRos::elevator_remaining_pulses_ = 0.0;
int32_t TMicroRos::extender_remaining_pulses_ = 0.0;
bool TMicroRos::elevator_has_command_ = false;
bool TMicroRos::extender_has_command_ = false;
