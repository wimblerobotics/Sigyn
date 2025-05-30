#include "tmicro_ros.h"

#include <Wire.h>
#include <example_interfaces/action/fibonacci.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/service.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <stdint.h>
#include <stdio.h>

#include "sigyn_interfaces/action/move_elevator.h"
#include "sigyn_interfaces/action/move_extender.h"
#include "sigyn_interfaces/srv/gripper_position.h"
#include "tconfiguration.h"
#include "tmodule.h"
#if USE_TSD
#include "tsd.h"
#endif
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <unistd.h>

#include "tmotorClass.h"

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
        rmw_uros_epoch_nanos();  // Capture the current ROS time
                                 // after attempted synchronization.
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
  static bool showedWaitingAgent = false;
  static bool showedAgentAvailable = false;
  static bool showedAgentConnected = false;
  if (!has_homed) {
#if USE_TSD
    TSd::singleton().log("INFO [TMicroRos::loop] homing.");
#endif
    elevator_->Home();
    extender_->Home();
    has_homed = true;
  }

  switch (state_) {
    case kWaitingAgent: {
#if USE_TSD
      if (!showedWaitingAgent) {
        TSd::singleton().log("INFO [TMicroRos::loop] kWaitingAgent");
        showedWaitingAgent = true;
      }
#endif
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 500) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? kAgentAvailable
                                                             : kWaitingAgent;
        last_time = uxr_millis();
      }
      break;
    }

    case kAgentAvailable: {
#if USE_TSD
      if (!showedAgentAvailable) {
        TSd::singleton().log("INFO [TMicroRos::loop] kAgentAvailable");
        showedWaitingAgent = false;
        showedAgentAvailable = true;
      }
#endif
      if (CreateEntities()) {
#if USE_TSD
        TSd::singleton().log(
            "INFO [TMicroRos::loop] kAgentAvailable successful CreateEntities");
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
      break;
    }

    case kAgentConnected: {
      static int32_t spin_some_count = 0;
#if USE_TSD
      if (!showedAgentConnected) {
        TSd::singleton().log("INFO [TMicroRos::loop] kAgentConnected");
        showedWaitingAgent = false;
        showedAgentAvailable = false;
        showedAgentConnected = true;
      }
#endif
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 100) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                     ? kAgentConnected
                     : kAgentDisconnected;
        last_time = uxr_millis();
      }

      elevator_->ContinueOutstandingMovementRequests();
      extender_->ContinueOutstandingMovementRequests();
      if (++spin_some_count > 10) {
        if (TMicroRos::singleton().state_ == kAgentConnected) {
          rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
        }
        spin_some_count = 0;
      }
      break;
    }

    case kAgentDisconnected: {
#if USE_TSD
      TSd::singleton().log("INFO [TMicroRos::loop] kAgentDisconnected");
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
    snprintf(g_singleton_->string_msg_.data.data,
             g_singleton_->string_msg_.data.capacity, "%s", msg);
    g_singleton_->string_msg_.data.size =
        strlen(g_singleton_->string_msg_.data.data);
    ignore_result(rcl_publish(&g_singleton_->diagnostics_publisher_,
                              &g_singleton_->string_msg_, nullptr));
  }
#if USE_TSD
  TSd::singleton().log(msg);
#endif
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

    is_setup = true;
  }
}

void TMicroRos::TimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
#if USE_TSD
  TSd::singleton().flush();
#endif

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

TMicroRos::TMicroRos() : TModule(TModule::kMicroRos) {
  string_msg_.data.capacity = 512;
  string_msg_.data.data =
      (char *)malloc(string_msg_.data.capacity * sizeof(char));
  string_msg_.data.size = 0;
}

bool TMicroRos::CreateEntities() {
// #define DEBUG USE_TSD && true
#define DEBUG true
#if DEBUG
  char diagnostic_message[256];
#endif

  rcl_ret_t rclc_result;
  allocator_ = rcl_get_default_allocator();

  // create init_options
  rclc_result = rclc_support_init(&support_, 0, nullptr, &allocator_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_support_init result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // create node
  rclc_result = rclc_node_init_default(&node_, "gripper_node", "", &support_);
#if DEBUG
  snprintf(
      diagnostic_message, sizeof(diagnostic_message),
      "INFO [TMicroRos::createEntities] rclc_node_init_default result: %ld",
      rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // create publishers.
  rclc_result = rclc_publisher_init_default(
      &diagnostics_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "gripper_diagnostics");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_publisher_init_default "
           "gripper_diagnostics result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_publisher_init_default(
      &stats_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "gripper_stats");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_publisher_init_default "
           "gripper_stats result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create subscribers.
  rclc_result = rclc_subscription_init_default(
      &elevator_command_subscriber_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "elevator_cmd");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_subscription_init_default "
           "elevator_cmd result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_subscription_init_default(
      &extender_command_subscriber_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "extender_cmd");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_subscription_init_default "
           "extender_cmd result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create cmd_vel_gripper subscriber
  rclc_result = rclc_subscription_init_default(
      &cmd_vel_gripper_subscriber_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel_gripper");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_subscription_init_default "
           "cmd_vel_gripper result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create timer,
  const unsigned int timer_timeout_ns = 1'000'000'000;
  rclc_result = rclc_timer_init_default(&timer_, &support_, timer_timeout_ns,
                                        TimerCallback);
#if DEBUG
  snprintf(
      diagnostic_message, sizeof(diagnostic_message),
      "INFO [TMicroRos::createEntities] rclc_timer_init_default result: %ld",
      rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create executor
  executor_ = rclc_executor_get_zero_initialized_executor();
  rclc_result =
      rclc_executor_init(&executor_, &support_.context, 10, &allocator_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_executor_init "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_executor_add_timer(&executor_, &timer_);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_executor_add_timer "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_executor_add_subscription_with_context(
      &executor_, &elevator_command_subscriber_, &elevator_command_msg_,
      &TMicroRos::HandleMoveTopicCallback, elevator_, ON_NEW_DATA);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] "
           "rclc_executor_add_subscription_with_context elevator "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_executor_add_subscription_with_context(
      &executor_, &extender_command_subscriber_, &extender_command_msg_,
      &TMicroRos::HandleMoveTopicCallback, extender_, ON_NEW_DATA);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] "
           "rclc_executor_add_subscription_with_context extender "
           "result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  rclc_result = rclc_executor_add_subscription_with_context(
      &executor_, &cmd_vel_gripper_subscriber_, &cmd_vel_gripper_msg_,
      &TMicroRos::HandleCmdVelGripperCallback, nullptr, ON_NEW_DATA);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] "
           "rclc_executor_add_subscription_with_context cmd_vel_gripper result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create action handler.
  const rosidl_action_type_support_t *elevator_type_support =
      ROSIDL_GET_ACTION_TYPE_SUPPORT(sigyn_interfaces, MoveElevator);
  rclc_result = rclc_action_server_init_default(
      &elevator_action_server_, &node_, &support_, elevator_type_support,
      "move_elevator");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_action_server_init_default "
           "elevator result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

#define NUMBER_OF_SIMULTANEOUS_ELEVATOR_HANDLES 10
  // Goal request storage
  sigyn_interfaces__action__MoveElevator_SendGoal_Request
      elevator_requests[NUMBER_OF_SIMULTANEOUS_ELEVATOR_HANDLES];

  rclc_result = rclc_executor_add_action_server(
      &executor_, &elevator_action_server_,
      NUMBER_OF_SIMULTANEOUS_ELEVATOR_HANDLES, elevator_requests,
      sizeof(sigyn_interfaces__action__MoveElevator_SendGoal_Request),
      TMotorClass::HandleActionRequest,  // Goal request callback
      TMotorClass::HandleActionCancel,   // Goal cancel callback
      (void *)elevator_                  // Context
  );
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_executor_add_action_server "
           "elevator result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create action handler.
  const rosidl_action_type_support_t *extender_type_support =
      ROSIDL_GET_ACTION_TYPE_SUPPORT(sigyn_interfaces, MoveExtender);
  rclc_result = rclc_action_server_init_default(
      &extender_action_server_, &node_, &support_, extender_type_support,
      "move_extender");
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_action_server_init_default "
           "extender result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

#define NUMBER_OF_SIMULTANEOUS_EXTENDER_HANDLES 10
  // Goal request storage
  sigyn_interfaces__action__MoveExtender_SendGoal_Request
      extender_requests[NUMBER_OF_SIMULTANEOUS_EXTENDER_HANDLES];

  rclc_result = rclc_executor_add_action_server(
      &executor_, &extender_action_server_,
      NUMBER_OF_SIMULTANEOUS_EXTENDER_HANDLES, extender_requests,
      sizeof(sigyn_interfaces__action__MoveExtender_SendGoal_Request),
      TMotorClass::HandleActionRequest,  // Goal request callback
      TMotorClass::HandleActionCancel,   // Goal cancel callback
      (void *)extender_                  // Context
  );
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rclc_executor_add_action_server "
           "extender result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif

  // Create elevator gripper service.
  elevator_gripper_service_ = rcl_get_zero_initialized_service();
  rcl_service_options_t service_ops = rcl_service_get_default_options();
  rclc_result = rcl_service_init(
      &elevator_gripper_service_, &node_,
      ROSIDL_GET_SRV_TYPE_SUPPORT(sigyn_interfaces, srv, GripperPosition),
      "elevator_position", &service_ops);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rcl_service_init "
           "elevator_gripper_service_ result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif
  rclc_result = rclc_executor_add_service_with_context(
      &executor_, &elevator_gripper_service_, &elevator_request_msg_,
      &elevator_response_msg_, &TMicroRos::HandleGripperServiceCallback,
      (void *)elevator_);

  // Create extender gripper service.
  extender_gripper_service_ = rcl_get_zero_initialized_service();
  rclc_result = rcl_service_init(
      &extender_gripper_service_, &node_,
      ROSIDL_GET_SRV_TYPE_SUPPORT(sigyn_interfaces, srv, GripperPosition),
      "extender_position", &service_ops);
#if DEBUG
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::createEntities] rcl_service_init "
           "extender_gripper_service_ result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
#endif
  rclc_result = rclc_executor_add_service_with_context(
      &executor_, &extender_gripper_service_, &extender_request_msg_,
      &extender_response_msg_, &TMicroRos::HandleGripperServiceCallback,
      (void *)extender_);

  return true;
}

void TMicroRos::DestroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore_result(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  ignore_result(rcl_service_fini(&extender_gripper_service_, &node_));
  ignore_result(rcl_service_fini(&elevator_gripper_service_, &node_));
  ignore_result(rclc_action_server_fini(&elevator_action_server_, &node_));
  ignore_result(rclc_action_server_fini(&extender_action_server_, &node_));
  ignore_result(rcl_publisher_fini(&diagnostics_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&stats_publisher_, &node_));
  ignore_result(rcl_subscription_fini(&elevator_command_subscriber_, &node_));
  ignore_result(rcl_subscription_fini(&extender_command_subscriber_, &node_));
  ignore_result(rcl_subscription_fini(&cmd_vel_gripper_subscriber_, &node_));
  ignore_result(rcl_timer_fini(&timer_));
  ignore_result(rclc_executor_fini(&executor_));
  ignore_result(rcl_node_fini(&node_));
  ignore_result(rclc_support_fini(&support_));
}

TMicroRos &TMicroRos::singleton() {
  if (!g_singleton_) {
    elevator_ = TMotorClass::CreateMotor(TMotorClass::Elevator);
    extender_ = TMotorClass::CreateMotor(TMotorClass::Extender);
    g_singleton_ = new TMicroRos();
  }

  return *g_singleton_;
}

TMicroRos *TMicroRos::g_singleton_ = nullptr;
volatile int64_t TMicroRos::ros_sync_time_ = 0;

TMotorClass *TMicroRos::elevator_ = nullptr;
TMotorClass *TMicroRos::extender_ = nullptr;

// Add static callback handlers for ROS subscriptions and services
void TMicroRos::HandleMoveTopicCallback(const void *msg, void *context) {
  TMotorClass *motor = (TMotorClass *)context;
  const std_msgs__msg__Float32 *command = (const std_msgs__msg__Float32 *)msg;
  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMicroRos::HandleMoveTopicCallback] command->data: %4.3f",
           command->data);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  motor->SetTargetPosition(command->data);
}

void TMicroRos::HandleCmdVelGripperCallback(const void *msg, void *context) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msg;
  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
    "[TMicroRos::HandleCmdVelGripperCallback] linear.x: %f, angular.z: %f",
    twist->linear.x, twist->angular.z);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  // Move elevator by 1 mm per message if linear.x is nonzero
  TMotorClass* elevator = TMicroRos::get_elevator();
  if (twist->linear.x > 0.0f && elevator) {
    // elevator->MoveByDelta(0.001f);
    elevator->SetTargetPosition(
        elevator->get_current_position() + 0.001f);
  } else if (twist->linear.x < 0.0f && elevator) {
    // elevator->MoveByDelta(-0.001f);
    elevator->SetTargetPosition(
        elevator->get_current_position() - 0.001f);
  }
  // Move extender by 1 mm per message if angular.z is nonzero
  TMotorClass* extender = TMicroRos::get_extender();
  if (twist->angular.z > 0.0f && extender) {
    // extender->MoveByDelta(0.001f);
    extender->SetTargetPosition(
        extender->get_current_position() + 0.001f);
  } else if (twist->angular.z < 0.0f && extender) {
    // extender->MoveByDelta(-0.001f);
    extender->SetTargetPosition(
        extender->get_current_position() - 0.001f);
  }
}

void TMicroRos::HandleGripperServiceCallback(const void *request_msg, void *response_msg, void *context) {
  TMotorClass *motor = (TMotorClass *)context;
  sigyn_interfaces__srv__GripperPosition_Response *res_in =
      (sigyn_interfaces__srv__GripperPosition_Response *)response_msg;
  res_in->position = motor->get_current_position();
}