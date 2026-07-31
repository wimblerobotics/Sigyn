# Sigyn Docking Implementation Plan

**Date:** 2026-07-26  
**Status:** Planning Phase  
**Target:** Autonomous charging station docking with visual servoing

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Overview](#system-overview)
3. [Current Capabilities Analysis](#current-capabilities-analysis)
4. [Nav2 Docking Research](#nav2-docking-research)
5. [Implementation Plan](#implementation-plan)
6. [Parallel Development Paths](#parallel-development-paths)
7. [Critical Open Issues](#critical-open-issues)
8. [Timeline & Milestones](#timeline--milestones)
9. [Testing Strategy](#testing-strategy)
10. [Future Enhancements](#future-enhancements)

---

## Executive Summary

This document outlines the complete implementation plan for autonomous docking to a wall-mounted charging station. The system will use a multi-stage approach combining Nav2 navigation, AprilTag-based visual servoing, and blue cross detection for final positioning.

**Key Objectives:**
- Enable autonomous return-to-charge behavior
- Achieve ±5mm lateral alignment accuracy
- Achieve ±10mm forward positioning accuracy
- Maintain safety system integrity during close-proximity maneuvering
- Support retry logic for failed docking attempts

**Estimated Timeline:** 2.5-3 weeks for MVP (Minimum Viable Product)

---

## System Overview

### Physical Docking Mechanism

**Robot Side:**
- Cone-shaped charging probe on front-right
- Rectangular box with 20mm brass contact strips
- Differential drive robot, 18" (457mm) diameter
- OAK-D Lite camera mounted front-left

**Charging Station Side:**
- Wall-mounted inverted cone with spring-loaded brass contacts
- Large AprilTag (tag36h11, ID 28, size 166mm)
- Blue cross drawn on AprilTag border for final positioning
- Located in computer room, ~8 feet from doorway

**Mechanical Constraints:**
- Final approach must be straight-ahead only (no rotation)
- Rectangular probe requires precise lateral alignment
- Cones provide ~10mm forward tolerance
- Angular tolerance: TBD (to be measured)

### Approach Sequence

```
┌─────────────────────────────────────────────────────────┐
│  Computer Room Entry (~8 feet from charger)             │
│  - Robot enters through narrow doorway                  │
│  - Offset ~2 feet laterally                             │
│  - Must avoid unfolded table near door                  │
└─────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  Coarse Navigation (Nav2)                               │
│  - Move to staging position ~0.5-1.0m from charger      │
│  - Face charging station                                │
└─────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  AprilTag Coarse Servo (1.0m → 0.3m)                    │
│  - Use 3D pose for X, Y, Yaw correction                 │
│  - Forward creep at 0.05 m/s                            │
└─────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  AprilTag Fine Servo (0.3m → 0.2m)                      │
│  - Switch to image-space control                        │
│  - Achieve <5mm lateral error                           │
│  - Forward creep at 0.03 m/s                            │
└─────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  Blue Cross Servo (0.2m → contact)                      │
│  - AprilTag no longer visible                           │
│  - Lateral correction only using cross centroid         │
│  - Forward position from cross area                     │
│  - Very slow forward at 0.02 m/s                        │
└─────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  Verify Charging                                        │
│  - Detect charging current (software detection MVP)     │
│  - Future: INA226 sensor on charging port               │
└─────────────────────────────────────────────────────────┘
```

---

## Current Capabilities Analysis

### AprilTag Detection System

**Topics:**
- `/oakd_apriltag_node/rgb_image` - 10 Hz, sensor_msgs/Image
- `/oakd_apriltag_node/depth_image` - sensor_msgs/Image  
- `/oakd_apriltag_node/points` - sensor_msgs/PointCloud2
- `/oakd_apriltag_node/camera_info` - sensor_msgs/CameraInfo
- `/oakd_apriltag_node/detections` - vision_msgs/Detection3DArray
- `/oakd_apriltag_node/annotated_image` - sensor_msgs/Image (with overlays)

**Detection Data Structure:**
```yaml
header:
  stamp: {sec, nanosec}
  frame_id: oakd_apriltag_optical_frame
detections:
- results:
  - hypothesis:
      class_id: '28'        # AprilTag ID
      score: 38.017         # Detection confidence
    pose:
      pose:
        position: {x, y, z}         # 3D position in camera frame
        orientation: {x, y, z, w}   # Quaternion orientation
```

**Camera Configuration:**
- Resolution: 1080p RGB
- Frame rate: 30 fps (publishing at ~10 Hz)
- Depth resolution: 400p
- Tag size: 0.166m (6.5 inches)
- Family: tag36h11

**Detection Ranges:**
- Far range (1.0m+): Full AprilTag visible and trackable
- Mid range (0.3-1.0m): AprilTag visible, good for servoing
- Near range (0.2-0.3m): AprilTag partially visible, switching to image-space control
- Close range (<0.2m): AprilTag out of frame, blue cross only

**Image Analysis (from provided screenshots):**
- **17-02-14.png** (0.5m distance): Blue cross clearly visible, good contrast
- **15-45-21.png** (docked): Only blue cross visible, positioned left-center in frame
- Note: Non-square aspect ratio in one image indicates potential camera configuration issue

### Safety System Integration

**Current System:**
- Board 1 manages 8 TOF sensors (VL53L0X)
- Update rate: ~25 Hz
- Proximity rings: Ring 3 (warning) and Ring 2 (emergency stop)
- Front sensors: front_left_fwd and front_right_fwd most critical for docking
- Safety aggregated level 0-5 (severity scale)

**Docking Requirements:**
- Disable proximity faults on front sensors during docking
- Maintain side sensor protection (detect misalignment)
- Heartbeat-based safety: 20 Hz heartbeat from ROS, 50ms timeout
- Provide TOF distance feedback for docking state machine

### Behavior Tree System

**Framework:** BT.CPP (ROS 2 Jazzy distribution)  
**Existing Custom Nodes:**
- Navigation wrappers around Nav2
- Custom motion controllers (faster/more precise than Nav2 behaviors)
- Example: Can-fetching behavior tree with object recognition

**Tick Rate:** Configurable, typically 10-20 Hz for normal operation

---

## Nav2 Docking Research

### Opennav_docking Package Analysis

**Available in Jazzy:**
- `DockRobot` action server
- Dock plugin system (ChArUco markers, AprilTags, laser-based)
- Pre-approach staging and path planning
- Designed for omnidirectional or complex approach patterns

**Why Not Using Nav2 Docking:**
1. **Differential Drive Constraint**: System assumes flexible approach paths; we need straight final approach
2. **Custom Vision Pipeline**: Blue cross fallback is unique to our setup
3. **Safety Integration**: Tight coupling with existing Teensy-based safety system
4. **Performance**: Custom motion nodes are faster and more precise
5. **Complexity**: Adding dock plugins would be more work than custom implementation

**Decision:** Implement custom docking action server with behavior tree integration.

---

## Implementation Plan

### Phase 1: Foundation & Sensing (Prerequisite Work)

#### 1.1 Safety System - Docking Mode

**Location:** `wr_teensy_boards/modules/vl53l0x/`, `wr_proto_msgs/`

**Requirements:**
- Add "docking mode" state to Board 1 VL53L0X monitor
- New proto message: `DockingHeartbeat`
  ```protobuf
  message DockingHeartbeat {
    bool docking_active = 1;
    uint32 sequence = 2;
  }
  ```
- Receive heartbeat at 20 Hz from ROS node
- If no heartbeat for >50ms: revert to normal proximity fault behavior
- In docking mode:
  - Front-left-forward: Allow close distances (<100mm), no fault
  - Front-right-forward: Allow close distances (<100mm), no fault  
  - Side sensors: Keep normal fault thresholds
  - Rear sensors: Keep normal fault thresholds
- Send TOF distances to ROS for docking feedback

**Firmware Changes:**
```cpp
// In vl53l0x_monitor.cpp
bool docking_mode_active_ = false;
uint32_t last_docking_heartbeat_ms_ = 0;

void ProcessDockingHeartbeat(const DockingHeartbeat& msg) {
  docking_mode_active_ = msg.docking_active();
  last_docking_heartbeat_ms_ = millis();
}

void CheckDockingTimeout() {
  if (docking_mode_active_ && 
      (millis() - last_docking_heartbeat_ms_ > 50)) {
    docking_mode_active_ = false;
    // Log timeout event
  }
}

// In fault evaluation for front sensors:
if (docking_mode_active_ && 
    (instance == FRONT_LEFT_FWD || instance == FRONT_RIGHT_FWD)) {
  // Don't trigger ring faults for close distances
  return;
}
```

**ROS Node Changes:**
- Create `docking_heartbeat_publisher` in action server
- Publish at 20 Hz during active docking
- Stop publishing when docking complete/aborted

**Estimated Time:** 1-2 days  
**Deliverable:** 
- New proto message definition
- Board 1 firmware update
- ROS integration tested

**Dependencies:** None  
**Testing:**
- Manual approach test with obstacle detection
- Verify side sensors still active
- Verify timeout behavior

---

#### 1.2 Blue Cross Detector Node

**Location:** `sigyn_oakd_detection/sigyn_oakd_detection/blue_cross_detector_node.py`

**Message Definition:**
```python
# sigyn_interfaces/msg/BlueCrossDetection.msg
std_msgs/Header header
bool detected
geometry_msgs/Point2D centroid  # Normalized [0,1] coordinates
float32 area_pixels             # Bounding box area for distance estimation
float32 confidence              # Detection quality 0.0-1.0
```

**Node Implementation:**
```python
class BlueCrossDetectorNode(Node):
    def __init__(self):
        # Subscribe to RGB image
        self.image_sub = self.create_subscription(
            Image, '/oakd_apriltag_node/rgb_image',
            self.image_callback, 10)
        
        # Publish detection
        self.detection_pub = self.create_publisher(
            BlueCrossDetection, '~/blue_cross_detection', 10)
        
        # HSV thresholds for blue (tune experimentally)
        self.blue_lower = np.array([100, 100, 50])
        self.blue_upper = np.array([130, 255, 255])
        
        # Minimum area threshold (pixels)
        self.min_area = 50
        
    def image_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Threshold blue color
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        
        # Morphological operations to reduce noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detection = BlueCrossDetection()
        detection.header = msg.header
        
        if contours:
            # Get largest contour
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area >= self.min_area:
                # Compute centroid
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']
                    
                    # Normalize to [0,1]
                    height, width = cv_image.shape[:2]
                    detection.centroid.x = cx / width
                    detection.centroid.y = cy / height
                    detection.area_pixels = area
                    detection.detected = True
                    
                    # Compute confidence based on shape and size
                    detection.confidence = self.compute_confidence(largest, area)
        
        self.detection_pub.publish(detection)
    
    def compute_confidence(self, contour, area):
        # Cross-like shape should have aspect ratio near 1:1
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        
        # Confidence higher when aspect_ratio near 1.0
        aspect_confidence = 1.0 - abs(1.0 - aspect_ratio)
        
        # Confidence based on area (bigger = more confident)
        area_confidence = min(area / 1000.0, 1.0)
        
        return (aspect_confidence + area_confidence) / 2.0
```

**Parameters:**
- `blue_hsv_lower`: [100, 100, 50] (HSV lower bound)
- `blue_hsv_upper`: [130, 255, 255] (HSV upper bound)
- `min_area_pixels`: 50 (minimum contour area)
- `confidence_threshold`: 0.3 (minimum confidence to publish)

**Estimated Time:** 1 day  
**Deliverable:**
- New detector node
- Message definition in sigyn_interfaces
- Launch file integration

**Dependencies:** None  
**Testing:**
- Test with provided images (0.5m and docked)
- Verify detection at various distances
- Test with different lighting conditions
- Measure false positive rate

**Known Issues:**
- Lighting sensitivity (blue Sharpie may not be robust)
- Background blue objects could cause false detections

---

### Phase 2: Docking Action Server (Core Logic)

#### 2.1 Docking Action Definition

**Location:** `sigyn_interfaces/action/DockToCharger.action`

```python
# Goal
geometry_msgs/PoseStamped initial_approach_pose  # Pre-dock position (~0.5-1.0m)
float32 approach_timeout_sec                     # Max time for entire sequence

---

# Result
bool success
string result_message
int32 result_code
  # 0 = SUCCESS - Charging detected
  # 1 = APRILTAG_LOST - Tag not visible when expected
  # 2 = CROSS_LOST - Blue cross not detected in final phase
  # 3 = TIMEOUT - Exceeded approach_timeout_sec
  # 4 = SAFETY_ABORT - Safety system triggered abort
  # 5 = CHARGE_NOT_DETECTED - Docked but no charging current
  # 6 = NAVIGATION_FAILED - Nav2 couldn't reach initial_approach_pose
  # 7 = ALIGNMENT_FAILED - Unable to align within tolerances

---

# Feedback
string current_phase    # Current state machine phase
float32 lateral_error_m       # Lateral offset from target (meters)
float32 forward_error_m       # Forward distance to target (meters)
float32 yaw_error_rad         # Angular error (radians)
float32 estimated_time_remaining_sec
int32 retry_count             # Number of retry attempts
```

**Estimated Time:** 0.5 days  
**Dependencies:** None

---

#### 2.2 Docking Action Server

**Location:** `sigyn_behavior_trees/src/dock_to_charger_action_server.cpp`

**State Machine:**

```
                    ┌──────────────────────────────┐
                    │   NAVIGATE_TO_APPROACH       │
                    │   - Use Nav2 to reach        │
                    │     pre-dock pose            │
                    └──────────────────────────────┘
                               │
                               │ Nav2 goal reached
                               │ OR AprilTag detected < 1.0m
                               ↓
                    ┌──────────────────────────────┐
                    │   APRILTAG_COARSE_SERVO      │
                    │   Distance: 1.0m → 0.3m      │
                    │   - 3D pose for X, Y, Yaw    │
                    │   - Forward: 0.05 m/s        │
                    │   - Lateral: P-control       │
                    │   - Yaw: P-control           │
                    └──────────────────────────────┘
                               │
                               │ Z < 0.3m
                               │ OR Tag fills frame
                               ↓
                    ┌──────────────────────────────┐
                    │   APRILTAG_FINE_SERVO        │
                    │   Distance: 0.3m → 0.2m      │
                    │   - Image-space control      │
                    │   - Forward: 0.03 m/s        │
                    │   - Target: <5mm lateral err │
                    └──────────────────────────────┘
                               │
                               │ Z < 0.2m
                               │ OR Tag out of frame
                               ↓
                    ┌──────────────────────────────┐
                    │   BLUE_CROSS_SERVO           │
                    │   Distance: 0.2m → contact   │
                    │   - Cross centroid for lat.  │
                    │   - Cross area for forward   │
                    │   - Forward: 0.02 m/s        │
                    └──────────────────────────────┘
                               │
                               │ Large cross area
                               │ OR motor current spike
                               ↓
                    ┌──────────────────────────────┐
                    │   VERIFY_CHARGE              │
                    │   - Stop motors              │
                    │   - Stop docking heartbeat   │
                    │   - Wait for charge detect   │
                    │   - Timeout: 5 seconds       │
                    └──────────────────────────────┘
                               │
                               ↓
                         SUCCESS / FAIL
```

**Control Loop Details:**

```cpp
class DockToChargerActionServer : public rclcpp::Node {
private:
  // Action server
  rclcpp_action::Server<DockToCharger>::SharedPtr action_server_;
  
  // Subscribers
  rclcpp::Subscription<Detection3DArray>::SharedPtr apriltag_sub_;
  rclcpp::Subscription<BlueCrossDetection>::SharedPtr cross_sub_;
  rclcpp::Subscription<SafetyState>::SharedPtr safety_sub_;
  
  // Publishers
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<DockingHeartbeat>::SharedPtr heartbeat_pub_;
  
  // Control parameters
  static constexpr double K_LATERAL = 0.3;   // Lateral P-gain (tune)
  static constexpr double K_YAW = 0.5;       // Yaw P-gain (tune)
  static constexpr double LATERAL_TOLERANCE = 0.005;  // 5mm
  static constexpr double CROSS_CENTER_TOL_PX = 10;   // pixels
  
  // Speed profiles per phase
  static constexpr double SPEED_COARSE = 0.05;   // m/s
  static constexpr double SPEED_FINE = 0.03;     // m/s
  static constexpr double SPEED_CROSS = 0.02;    // m/s
  
  // State
  DockingState current_state_;
  Detection3DArray::SharedPtr last_apriltag_msg_;
  BlueCrossDetection::SharedPtr last_cross_msg_;
  
  // Update loop
  rclcpp::TimerInterface::SharedPtr control_timer_;  // 10 Hz
  rclcpp::TimerInterface::SharedPtr heartbeat_timer_;  // 20 Hz
  
  void control_loop() {
    switch (current_state_) {
      case DockingState::NAVIGATE_TO_APPROACH:
        handle_navigation();
        break;
      case DockingState::APRILTAG_COARSE_SERVO:
        handle_coarse_servo();
        break;
      case DockingState::APRILTAG_FINE_SERVO:
        handle_fine_servo();
        break;
      case DockingState::BLUE_CROSS_SERVO:
        handle_cross_servo();
        break;
      case DockingState::VERIFY_CHARGE:
        handle_verify_charge();
        break;
    }
    
    publish_feedback();
    check_safety_abort();
  }
  
  void handle_coarse_servo() {
    if (!last_apriltag_msg_ || last_apriltag_msg_->detections.empty()) {
      // AprilTag lost - abort or transition to cross servo
      transition_to(DockingState::BLUE_CROSS_SERVO);
      return;
    }
    
    // Extract 3D pose
    auto& detection = last_apriltag_msg_->detections[0];
    auto& pose = detection.results[0].pose.pose;
    
    double z_dist = pose.position.z;  // Forward distance
    double y_error = pose.position.y;  // Lateral error (+ is right)
    
    // Compute yaw error from quaternion
    double yaw_error = compute_yaw_from_quaternion(pose.orientation);
    
    // Proportional control
    Twist cmd;
    cmd.linear.x = SPEED_COARSE;
    cmd.linear.y = -K_LATERAL * y_error;  // Lateral correction
    cmd.angular.z = -K_YAW * yaw_error;
    
    cmd_vel_pub_->publish(cmd);
    
    // Transition check
    if (z_dist < 0.3) {
      transition_to(DockingState::APRILTAG_FINE_SERVO);
    }
  }
  
  void handle_fine_servo() {
    // Similar to coarse but use image-space control
    // Check if tag fills frame or partially out of view
    // If lateral error < 5mm, transition to cross servo
  }
  
  void handle_cross_servo() {
    if (!last_cross_msg_ || !last_cross_msg_->detected) {
      // Cross lost - abort
      abort_with_code(DockResult::CROSS_LOST);
      return;
    }
    
    // Lateral control from centroid X
    double cx_normalized = last_cross_msg_->centroid.x;
    double lateral_error = cx_normalized - 0.5;  // Target center
    
    Twist cmd;
    cmd.linear.x = SPEED_CROSS;
    cmd.linear.y = -K_LATERAL * lateral_error * 0.5;  // Scale factor
    cmd.angular.z = 0.0;  // No rotation in final phase
    
    cmd_vel_pub_->publish(cmd);
    
    // Transition: large cross area indicates very close
    if (last_cross_msg_->area_pixels > 800) {  // Tune threshold
      transition_to(DockingState::VERIFY_CHARGE);
    }
  }
  
  void handle_verify_charge() {
    // Stop motion
    stop_robot();
    stop_docking_heartbeat();
    
    // Wait for charging current detection
    // (Implementation depends on Phase 4.1)
    
    // Timeout after 5 seconds
    // Result: SUCCESS or CHARGE_NOT_DETECTED
  }
  
  void publish_docking_heartbeat() {
    DockingHeartbeat msg;
    msg.docking_active = true;
    msg.sequence = heartbeat_sequence_++;
    heartbeat_pub_->publish(msg);
  }
};
```

**Estimated Time:** 3-4 days  
**Deliverable:** Fully functional action server  
**Dependencies:** 1.1 (Safety system), 1.2 (Blue cross detector), 2.1 (Action definition)  
**Testing:** Manual triggering via `ros2 action send_goal`

---

### Phase 3: Behavior Tree Integration

#### 3.1 Dock To Charger BT Node

**Location:** `sigyn_behavior_trees/include/sigyn_behavior_trees/dock_to_charger.hpp`

```cpp
class DockToCharger : public nav2_behavior_tree::BtActionNode<
  sigyn_interfaces::action::DockToCharger>
{
public:
  DockToCharger(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("dock_pose"),
      BT::InputPort<double>("timeout", 60.0, "Approach timeout seconds"),
      BT::OutputPort<int>("result_code", "Docking result code"),
      BT::OutputPort<std::string>("error_message", "Error description")
    };
  }
  
  void on_tick() override;
  void on_feedback(const std::shared_ptr<const Feedback> feedback) override;
};
```

**XML Usage:**
```xml
<DockToCharger 
  dock_pose="{dock_staging_pose}" 
  timeout="60.0"
  result_code="{docking_result}"
  error_message="{docking_error}"/>
```

**Estimated Time:** 0.5 days  
**Dependencies:** 2.2 (Action server)

---

#### 3.2 Charging State Management BT Nodes

**CheckBatteryNeedsCharging** (Condition Node)
```cpp
class CheckBatteryNeedsCharging : public BT::ConditionNode {
public:
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("battery_threshold", 36.5, "Voltage threshold"),
      BT::InputPort<double>("time_to_charger_sec", 120.0, "Estimated travel time")
    };
  }
  
  BT::NodeStatus tick() override {
    // Subscribe to /sigyn/power/battery
    // Check voltage < threshold
    // Check predicted time to empty > time_to_charger
    return BT::NodeStatus::SUCCESS;  // if charging needed
  }
};
```

**IsChargingComplete** (Condition Node)
```cpp
class IsChargingComplete : public BT::ConditionNode {
public:
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("battery_threshold", 41.0, "Voltage when charged"),
      BT::InputPort<double>("current_threshold", 0.1, "Charging current threshold")
    };
  }
  
  BT::NodeStatus tick() override {
    // Check voltage > threshold AND current < threshold
    return BT::NodeStatus::SUCCESS;  // if fully charged
  }
};
```

**RetryDocking** (Decorator Node)
```cpp
class RetryDocking : public BT::DecoratorNode {
private:
  int max_attempts_;
  int current_attempt_ = 0;
  
public:
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("max_attempts", 3, "Max docking attempts")
    };
  }
  
  BT::NodeStatus tick() override {
    auto child_status = child_node_->executeTick();
    
    if (child_status == BT::NodeStatus::SUCCESS) {
      current_attempt_ = 0;
      return BT::NodeStatus::SUCCESS;
    }
    
    if (child_status == BT::NodeStatus::FAILURE) {
      current_attempt_++;
      if (current_attempt_ < max_attempts_) {
        // Back up 0.5m to re-acquire AprilTag
        backup_robot(0.5);
        return BT::NodeStatus::RUNNING;  // Retry
      }
      return BT::NodeStatus::FAILURE;  // Max attempts exceeded
    }
    
    return child_status;
  }
};
```

**Estimated Time:** 1 day  
**Dependencies:** None (can develop in parallel)

---

#### 3.3 Charging Behavior Subtree

**Location:** `sigyn_behavior_trees/config/charging.xml`

```xml
<?xml version="1.0"?>
<root BTCPP_format="4">
  <BehaviorTree ID="ChargingBehavior">
    <ReactiveSequence name="charging_sequence">
      
      <!-- Check if charging needed -->
      <CheckBatteryNeedsCharging 
        battery_threshold="36.5"
        time_to_charger_sec="180.0"/>
      
      <!-- Retry docking up to 3 times -->
      <RetryDocking max_attempts="3">
        <Sequence name="docking_attempt">
          
          <!-- Navigate to room with charging station -->
          <Sequence name="navigate_to_charging_room">
            <ComputePath 
              goal="{charging_room_waypoint}" 
              path="{path}"/>
            <FollowPath path="{path}"/>
          </Sequence>
          
          <!-- Navigate to pre-dock staging position -->
          <Sequence name="navigate_to_staging">
            <ComputePath 
              goal="{dock_staging_pose}" 
              path="{path}"/>
            <FollowPath path="{path}"/>
          </Sequence>
          
          <!-- Execute docking -->
          <DockToCharger 
            dock_pose="{dock_staging_pose}"
            timeout="60.0"
            result_code="{docking_result}"
            error_message="{docking_error}"/>
          
          <!-- Verify charging started -->
          <IsCharging min_current_a="0.5"/>
          
        </Sequence>
      </RetryDocking>
      
      <!-- Wait for charging to complete -->
      <UntilSuccess>
        <Sequence>
          <Delay delay_msec="30000">  <!-- Check every 30 seconds -->
            <IsChargingComplete 
              battery_threshold="41.0"
              current_threshold="0.1"/>
          </Delay>
        </Sequence>
      </UntilSuccess>
      
      <!-- Back away from charger -->
      <MoveAShortDistanceAhead distance="-0.3" speed="0.05"/>
      
      <!-- Send notification -->
      <SaySomething message="Charging complete. Resuming normal operations."/>
      
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

**Integration with Main Behavior Tree:**
```xml
<BehaviorTree ID="MainPatrolBehavior">
  <ReactiveSequence>
    
    <!-- Charging has highest priority -->
    <SubTree ID="ChargingBehavior"/>
    
    <!-- Normal patrol tasks -->
    <SubTree ID="PatrolBehavior"/>
    
  </ReactiveSequence>
</BehaviorTree>
```

**Estimated Time:** 1 day  
**Dependencies:** 3.1, 3.2

---

### Phase 4: Charging Hardware Detection

#### 4.1 Charging Detection (Software-Only MVP)

**Location:** `sigyn_house_patroller/` or standalone node

**Approach:** Monitor battery voltage slope over time

```python
class ChargingDetectorNode(Node):
    def __init__(self):
        super().__init__('charging_detector')
        
        # Subscribe to battery status
        self.battery_sub = self.create_subscription(
            BatteryStatus, '/sigyn/power/battery',
            self.battery_callback, 10)
        
        # Publish charging state
        self.charging_pub = self.create_publisher(
            BatteryChargingStatus, '~/charging_status', 10)
        
        # Voltage history for slope calculation
        self.voltage_history = deque(maxlen=30)  # 30 seconds at 1 Hz
        
        # Charging detection thresholds
        self.voltage_slope_threshold = 0.1  # V per 30 seconds
        self.min_history_size = 10
        
    def battery_callback(self, msg):
        self.voltage_history.append((time.time(), msg.voltage))
        
        # Calculate voltage slope
        if len(self.voltage_history) >= self.min_history_size:
            times, voltages = zip(*self.voltage_history)
            slope = self.calculate_slope(times, voltages)
            
            # Charging if voltage increasing
            is_charging = slope > self.voltage_slope_threshold
            
            status = BatteryChargingStatus()
            status.is_charging = is_charging
            status.voltage_slope = slope
            status.confidence = self.compute_confidence(slope)
            self.charging_pub.publish(status)
    
    def calculate_slope(self, times, voltages):
        # Linear regression
        times_np = np.array(times) - times[0]
        voltages_np = np.array(voltages)
        return np.polyfit(times_np, voltages_np, 1)[0]
```

**Limitations:**
- Slow detection (requires 10-30 seconds)
- Unreliable when battery near full (voltage plateaus)
- Cannot distinguish between charging and voltage recovery after load removal
- Will miss charging if voltage already stable

**Estimated Time:** 0.5 days  
**Deliverable:** Simple charging detector node  
**Dependencies:** None (can develop in parallel)

---

#### 4.2 INA226 Charger Sensor (Hardware Addition - Future)

**DEFERRED TO POST-MVP**

**Location:** Board 2 hardware + `wr_teensy_boards/modules/battery/`

**Requirements:**
- Install INA226 sensor between charging port and battery
- Monitor charge current directly
- Add to Board 2 battery monitor module
- Publish charge current to ROS via proto messages
- Add safety thresholds:
  - Over-voltage from charger
  - Over-current from charger
  - Unexpected discharge during charging

**Benefits:**
- Instant charging detection (<100ms)
- Reliable at all battery states
- Safety monitoring of charging process
- Detect charging failures immediately

**Estimated Time:** 4-6 hours (hardware installation) + 1 day (firmware)  
**Deliverable:** Real-time charging detection with safety monitoring  
**Dependencies:** Hardware access, additional INA226 sensor

---

### Phase 5: Testing & Refinement

#### 5.1 Component Testing

**Blue Cross Detector:**
- Test detection at 0.2m, 0.5m, 1.0m, 1.5m distances
- Test with various lighting: bright, dim, mixed
- Measure false positive rate with other blue objects in view
- Test with different approach angles (±15°, ±30°)
- Verify confidence scoring

**AprilTag Detection:**
- Measure detection range (max and min distances)
- Test with various angles and orientations
- Measure pose accuracy at different distances
- Identify when tag becomes unreliable (<0.2m)

**Safety System Docking Mode:**
- Verify heartbeat timeout behavior
- Test with obstacle placed in front during docking
- Verify side sensors remain active
- Measure TOF sensor update latency

**Action Server State Transitions:**
- Test each state transition independently
- Test abort scenarios (tag lost, cross lost, safety abort)
- Verify feedback messages
- Test timeout behavior

**Estimated Time:** 2 days

---

#### 5.2 Integration Testing

**Full Docking Sequence:**
- Start from 2m away, facing dock
- Execute complete docking sequence
- Measure success rate (target: >90%)
- Measure final alignment accuracy

**Retry Behavior:**
- Force failures at various stages
- Verify backup and retry logic
- Test max retry limit
- Measure time for retry cycle

**Emergency Scenarios:**
- Walk in front during approach
- Walk in front during final positioning
- Remove power during docking
- Test safety abort and recovery

**Charging Detection:**
- Verify charging state detection
- Test undocking after charging complete
- Test premature undocking detection

**Multi-Run Endurance:**
- 10 consecutive docking attempts
- Measure success rate, alignment accuracy
- Identify failure modes

**Estimated Time:** 2 days

---

#### 5.3 Parameter Tuning

**Control Gains:**
- `K_LATERAL`: Tune for stable lateral correction
- `K_YAW`: Tune for smooth yaw correction
- Verify no oscillations

**Speed Profiles:**
- `SPEED_COARSE`: Balance speed vs. stability
- `SPEED_FINE`: Tune for alignment accuracy
- `SPEED_CROSS`: Very slow for final positioning

**Thresholds:**
- Blue cross detection HSV ranges
- Cross area threshold for "docked" detection
- AprilTag distance transition thresholds (0.3m, 0.2m)
- Lateral alignment tolerance

**Timeouts:**
- Overall docking timeout
- Per-phase timeouts
- Charging verification timeout

**Estimated Time:** 1 day

---

## Parallel Development Paths

```
Path A: Safety & Firmware
┌─────────────────────────────────────┐
│ 1.1 Safety System Docking Mode      │  ← 1-2 days
│   ├─ Proto message definition        │
│   ├─ Board 1 firmware update         │
│   └─ ROS integration                 │
└─────────────────────────────────────┘
              ↓
         [Ready for Phase 2]

Path B: Vision
┌─────────────────────────────────────┐
│ 1.2 Blue Cross Detector Node        │  ← 1 day
│   ├─ Message definition              │
│   ├─ HSV color detection             │
│   └─ Node implementation             │
└─────────────────────────────────────┘
              ↓
         [Ready for Phase 2]

Path C: Actions & BT
┌─────────────────────────────────────┐
│ 2.1 Action Definition                │  ← 0.5 days
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│ 2.2 Docking Action Server            │  ← 3-4 days
│   (Blocks on Paths A & B)            │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│ 3.1 Dock BT Node                     │  ← 0.5 days
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│ 3.2 Charging BT Nodes                │  ← 1 day
│   (Can develop in parallel w/ 3.1)   │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│ 3.3 Charging Subtree XML             │  ← 1 day
└─────────────────────────────────────┘

Path D: Charging Detection (Parallel)
┌─────────────────────────────────────┐
│ 4.1 Software Charging Detection      │  ← 0.5 days
│   (Develops in parallel)             │
└─────────────────────────────────────┘
              ↓
         [Ready when Phase 3 completes]
```

**Critical Path:** A → B → 2.1 → 2.2 → 3.1 → 3.3 → Testing  
**Estimated Duration (Critical Path):** 7.5-9.5 days

---

## Critical Open Issues

### 1. Lighting Robustness
**Problem:** Blue Sharpie cross may not be visible in low or varying light conditions.

**Impact:** Docking failure in final positioning phase.

**Mitigations:**
- **Short-term:** Add confidence threshold, fall back to "blind creep" mode using TOF distance
- **Mid-term:** Use white LED ring light around charging port (future hardware)
- **Long-term:** Replace blue cross with retroreflective tape or dedicated marker

**Status:** Accepted risk for MVP

---

### 2. Image Aspect Ratio Issue
**Problem:** One provided image (17-02-14.png) shows non-square aspect ratio, suggesting camera configuration issue.

**Impact:** Incorrect coordinate calculations in image-space control.

**Action Required:**
- Debug OAK-D camera configuration
- Verify RGB resolution and output format
- Test coordinate normalization with actual hardware
- Update detector nodes if aspect ratio is non-standard

**Priority:** HIGH (affects both AprilTag and blue cross detection)

---

### 3. Yaw Alignment Tolerance
**Problem:** Mechanical tolerance of cone alignment unknown.

**Impact:** May need tighter or looser yaw control than planned.

**Action Required:**
- Measure during Phase 5.1 testing
- Test with intentional yaw misalignment at ±5°, ±10°, ±15°
- Adjust control thresholds and state machine transitions
- Document measured tolerances

**Priority:** MEDIUM (will tune during testing)

---

### 4. Motor Stall Detection
**Problem:** Need to detect when robot pushes too hard against dock (failed mating).

**Impact:** Could damage hardware or get stuck.

**Action Required:**
- Add RoboClaw current monitoring to action server
- Subscribe to motor current messages from Board 1
- Define stall threshold (e.g., >80% max current for >0.5 seconds)
- Abort docking on stall detection
- Trigger retry behavior

**Priority:** HIGH (safety concern)

---

### 5. Charging Dock Position Uncertainty
**Problem:** Need precise TF transform for dock location in map frame.

**Impact:** Nav2 cannot navigate to pre-dock staging position.

**Action Required:**
- Option A: Create static TF publisher with manually measured dock pose
- Option B: Use AprilTag detection to dynamically localize dock
- Option C: Teach pose by driving robot to dock manually and recording position

**Recommended:** Option A for MVP (static TF), upgrade to Option B later

**Priority:** HIGH (required for navigation phase)

---

### 6. Obstacle During Charging
**Problem:** What if someone moves an object in front while robot is charging?

**Impact:** TOF sensors would trigger fault, potentially interrupt charging.

**Strategy:**
- TOF sensors remain active during charging
- Front sensor faults generate alert but don't undock
- Side/rear sensor faults still trigger normal safety response
- Send notification to user if obstacle detected while charging
- Only undock on manual command or charging complete

**Priority:** MEDIUM (can add after MVP)

---

### 7. Cross Detection False Positives
**Problem:** Other blue objects in environment could cause false detections.

**Impact:** Incorrect positioning or failed docking.

**Mitigations:**
- Use region of interest (ROI) based on last AprilTag position
- Only search for cross in center ±30% of image
- Require cross to be near expected size and shape
- Filter detections with low confidence (<0.3)
- Monitor detection stability over multiple frames

**Priority:** MEDIUM (can tune during testing)

---

### 8. Camera Exposure During Approach
**Problem:** Auto-exposure may make AprilTag/cross harder to detect as robot moves.

**Impact:** Lost detections during transitions.

**Mitigations:**
- Test with various lighting conditions
- Consider fixing camera exposure for docking
- Add detection loss recovery behavior (small backup and retry)

**Priority:** LOW (unlikely to be critical)

---

## Timeline & Milestones

### Week 1: Foundation
**Days 1-2:** Phase 1.1 - Safety System Docking Mode
- Proto message definition ✓
- Board 1 firmware changes ✓
- ROS integration ✓
- Testing ✓

**Day 3:** Phase 1.2 - Blue Cross Detector
- Message definition ✓
- Node implementation ✓
- Basic testing ✓

**Milestone:** All prerequisite sensing systems operational

---

### Week 2: Core Docking Logic
**Day 4:** Phase 2.1 - Action Definition ✓

**Days 5-8:** Phase 2.2 - Docking Action Server
- State machine implementation
- Control loops for all phases
- Safety integration
- Manual testing

**Milestone:** Can dock from 1m via action goal

---

### Week 3: Behavior Tree Integration
**Days 9-10:** Phase 3.1 & 3.2
- Dock BT node wrapper ✓
- Charging state BT nodes ✓

**Day 11:** Phase 3.3 - Charging Subtree
- XML tree definition ✓
- Integration with main patrol tree ✓

**Days 11-12:** Phase 4.1 - Software Charging Detection
- Voltage slope monitoring ✓
- Integration testing ✓

**Milestone:** Full autonomous charging behavior functional

---

### Week 4: Testing & Refinement
**Days 13-14:** Phase 5.1 - Component Testing
- Individual subsystem validation
- Edge case identification

**Days 15-16:** Phase 5.2 - Integration Testing
- End-to-end docking tests
- Retry behavior validation
- Emergency scenarios

**Day 17:** Phase 5.3 - Parameter Tuning
- Control gain optimization
- Threshold tuning
- Performance measurement

**Final Milestone:** MVP complete, documented success rate >90%

---

## Testing Strategy

### Unit Tests
- Blue cross detector with synthetic images
- AprilTag pose extraction
- Control law calculations
- State machine transitions

### Integration Tests
- Component interfaces (action server ↔ detectors)
- Safety system heartbeat
- BT node execution

### System Tests
- Full docking sequence from various starting positions
- Multi-attempt retry scenarios
- Charging detection and completion

### Performance Metrics
- **Success Rate:** >90% for MVP, >95% target
- **Alignment Accuracy:** ±5mm lateral, ±10mm forward
- **Time to Dock:** <60 seconds from 1m staging position
- **Retry Recovery:** >80% success on second attempt
- **Safety Response Time:** <100ms abort on fault

---

## Future Enhancements

### Hardware Improvements (Post-MVP)

**Charging Detection:**
- INA226 sensor between charger and battery
- Real-time current monitoring
- Safety checks for abnormal charging

**Visual Guidance:**
- White LED ring around dock for consistent lighting
- Retroreflective tape or dedicated marker instead of blue cross
- Second camera for redundant pose estimation

**Position Confirmation:**
- Microswitch to detect proper forward docking position
- Hall effect sensor for metal contact detection
- Force sensor to detect proper coupling

### Software Enhancements

**Dock Localization:**
- Use AprilTag for dynamic dock localization
- Update map with dock pose automatically
- Support multiple charging stations

**Advanced Control:**
- Model predictive control for smoother approach
- Velocity profiling for optimal speed-accuracy tradeoff
- Adaptive thresholds based on past performance

**Robustness:**
- Visual SLAM during docking for precise localization
- Stereo depth for obstacle detection during approach
- Machine learning for cross detection robustness

**Monitoring & Diagnostics:**
- Log all docking attempts with diagnostics
- Anomaly detection for degraded performance
- Predictive maintenance for dock hardware

---

## Appendix A: Safety System Integration Details

### Docking Heartbeat Message

```protobuf
// wr_proto_msgs/proto/docking_heartbeat.proto
syntax = "proto3";

message DockingHeartbeat {
  // Whether docking is currently active
  bool docking_active = 1;
  
  // Sequence number for dropped message detection
  uint32 sequence = 2;
  
  // Current docking phase (for diagnostics)
  string phase = 3;
  
  // Timestamp (milliseconds since epoch)
  uint64 timestamp_ms = 4;
}
```

### TOF Sensor Configuration During Docking

| Sensor              | Normal Mode       | Docking Mode      |
|---------------------|-------------------|-------------------|
| front_left_fwd      | Ring 3: 130mm     | **Disabled**      |
|                     | Ring 2: 100mm     | **Disabled**      |
| front_right_fwd     | Ring 3: 130mm     | **Disabled**      |
|                     | Ring 2: 100mm     | **Disabled**      |
| front_left          | Ring 3: 130mm     | Ring 3: 130mm     |
|                     | Ring 2: 100mm     | Ring 2: 100mm     |
| front_right         | Ring 3: 130mm     | Ring 3: 130mm     |
|                     | Ring 2: 100mm     | Ring 2: 100mm     |
| left                | Ring 3: 130mm     | Ring 3: 130mm     |
|                     | Ring 2: 100mm     | Ring 2: 100mm     |
| right               | Ring 3: 130mm     | Ring 3: 130mm     |
|                     | Ring 2: 100mm     | Ring 2: 100mm     |
| rear_left           | Ring 3: 130mm     | Ring 3: 130mm     |
|                     | Ring 2: 100mm     | Ring 2: 100mm     |
| rear_right          | Ring 3: 130mm     | Ring 3: 130mm     |
|                     | Ring 2: 100mm     | Ring 2: 100mm     |

---

## Appendix B: Control Tuning Guidelines

### Lateral Control Gain (K_LATERAL)

**Initial Value:** 0.3

**Tuning Process:**
1. Start with low gain (0.1)
2. Increase until lateral oscillations appear
3. Reduce by 50%
4. Test with various lateral offsets (±50mm, ±100mm)

**Target Performance:**
- No oscillations
- Settle time <3 seconds for ±50mm error
- Overshoot <10mm

### Yaw Control Gain (K_YAW)

**Initial Value:** 0.5

**Tuning Process:**
1. Start with low gain (0.2)
2. Test with yaw errors ±10°, ±20°
3. Increase until angular oscillations appear
4. Reduce by 30%

**Target Performance:**
- No oscillations
- Settle time <5 seconds for ±10° error
- Final yaw error <2°

### Speed Profiles

**Coarse Servo:** 0.05 m/s
- Fast enough for reasonable docking time
- Slow enough for AprilTag tracking

**Fine Servo:** 0.03 m/s
- Allows precise lateral alignment
- Gives time for control to settle

**Cross Servo:** 0.02 m/s
- Very slow for final positioning
- Reduces impact force on dock

---

## Appendix C: Troubleshooting Guide

### AprilTag Not Detected

**Symptoms:** Detection array empty or timeout

**Possible Causes:**
- Tag out of camera FOV
- Insufficient lighting
- Tag too far away
- Camera exposure incorrect

**Diagnostics:**
1. Check `/oakd_apriltag_node/rgb_image` - is tag visible?
2. Check detection confidence score - is it too low?
3. Measure distance - is it >2m?
4. Check camera exposure settings

**Solutions:**
- Adjust starting position closer to dock
- Improve lighting in charging area
- Increase tag size
- Tune AprilTag detector parameters

### Blue Cross Not Detected

**Symptoms:** `detected=false` in BlueCrossDetection message

**Possible Causes:**
- Cross out of frame
- HSV thresholds not matching actual color
- Lighting too dim or washed out
- False positive from other blue object

**Diagnostics:**
1. View `/oakd_apriltag_node/rgb_image` - is cross visible?
2. Check HSV values of cross in image
3. Verify confidence score
4. Check area_pixels value

**Solutions:**
- Re-draw cross with thicker marker
- Adjust HSV thresholds in detector
- Add lighting around dock
- Narrow detection ROI

### Lateral Oscillations

**Symptoms:** Robot weaving side-to-side during approach

**Possible Causes:**
- K_LATERAL gain too high
- Delayed feedback from vision
- Noise in pose estimates

**Solutions:**
- Reduce K_LATERAL by 20-30%
- Add low-pass filter to lateral error
- Increase control loop rate
- Check for vision system lag

### Charging Not Detected

**Symptoms:** Docking completes but CHARGE_NOT_DETECTED result

**Possible Causes:**
- Brass contacts not mating
- Charger not powered
- Voltage slope detection too slow
- Software detection threshold incorrect

**Diagnostics:**
1. Manually check contact alignment
2. Measure voltage at charger output
3. Monitor battery voltage during docking
4. Check `/sigyn/power/battery` topic

**Solutions:**
- Adjust final positioning (cross area threshold)
- Verify charger operation
- Tune voltage slope threshold
- Install hardware INA226 sensor

---

## Revision History

| Date       | Version | Author | Changes                                    |
|------------|---------|--------|--------------------------------------------|
| 2026-07-26 | 1.0     | AI     | Initial plan created from requirements     |

---

**END OF DOCUMENT**
