import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import Marker, MarkerArray
from enum import Enum
import numpy as np

class State(Enum):
    FIND_WALL = 0
    FOLLOW_WALL = 1
    NAVIGATE_NARROW_SPACE = 2
    APPROACH_DOORWAY = 3      # New state for doorway approach
    ENTER_DOORWAY = 4         # New state for doorway entry
    RECOVERY_BACKUP = 5
    RECOVERY_TURN = 6

class SpaceType(Enum):
    OPEN = 0
    NARROW_HALLWAY = 1
    DOORWAY = 2
    VERY_NARROW = 3

class PerimeterRoamerV2(Node):
    def __init__(self):
        super().__init__('perimeter_roamer')
        
        self.get_logger().info("Perimeter Roamer V2 Starting Up...")
        
        # --- Core Parameters ---
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.3)
        self.declare_parameter('narrow_space_speed', 0.08)
        self.declare_parameter('wall_distance_setpoint', 0.35)
        self.declare_parameter('wall_following_gain', 0.5)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_angular_speed', 0.8)
        
        # Robot dimensions (from config)
        self.declare_parameter('robot_radius', 0.23)  # 0.46m diameter / 2
        self.declare_parameter('robot_diameter', 0.46)
        
        # Space classification thresholds
        self.declare_parameter('doorway_width_threshold', 0.8)
        self.declare_parameter('narrow_hallway_threshold', 1.2)
        self.declare_parameter('very_narrow_threshold', 0.6)
        
        # Safety parameters - LASER-based instead of costmap
        self.declare_parameter('min_obstacle_distance', 0.15)  # Absolute minimum
        self.declare_parameter('comfortable_distance', 0.25)   # Preferred minimum
        
        # Control parameters
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # Topic names
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        
        # Recovery parameters
        self.declare_parameter('recovery_backup_speed', -0.08)
        self.declare_parameter('recovery_backup_time', 1.5)
        self.declare_parameter('recovery_turn_time', 2.0)
        self.declare_parameter('stuck_timeout', 8.0)
        self.declare_parameter('stuck_distance_threshold', 0.03)
        
        # Additional parameters for doorway approach
        self.declare_parameter('doorway_approach_distance', 0.6)  # Distance to start centering
        self.declare_parameter('doorway_centering_tolerance', 0.05)  # How centered we need to be
        self.declare_parameter('doorway_alignment_angle_tolerance', 0.1)  # Radians (~6°)
        
        # Get parameters
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.narrow_space_speed = self.get_parameter('narrow_space_speed').value
        self.wall_distance_setpoint = self.get_parameter('wall_distance_setpoint').value
        self.wall_following_gain = self.get_parameter('wall_following_gain').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.robot_diameter = self.get_parameter('robot_diameter').value
        
        self.doorway_width_threshold = self.get_parameter('doorway_width_threshold').value
        self.narrow_hallway_threshold = self.get_parameter('narrow_hallway_threshold').value
        self.very_narrow_threshold = self.get_parameter('very_narrow_threshold').value
        
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.comfortable_distance = self.get_parameter('comfortable_distance').value
        
        self.control_frequency = self.get_parameter('control_frequency').value
        self.robot_base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        
        self.recovery_backup_speed = self.get_parameter('recovery_backup_speed').value
        self.recovery_backup_time = self.get_parameter('recovery_backup_time').value
        self.recovery_turn_time = self.get_parameter('recovery_turn_time').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        
        # Get doorway parameters
        self.doorway_approach_distance = self.get_parameter('doorway_approach_distance').value
        self.doorway_centering_tolerance = self.get_parameter('doorway_centering_tolerance').value
        self.doorway_alignment_angle_tolerance = self.get_parameter('doorway_alignment_angle_tolerance').value
        
        self.get_logger().info("--- Parameters Loaded ---")
        self.get_logger().info(f"Robot: diameter={self.robot_diameter}m, radius={self.robot_radius}m")
        self.get_logger().info(f"Speeds: normal={self.forward_speed}, narrow={self.narrow_space_speed}")
        self.get_logger().info(f"Safety: min_dist={self.min_obstacle_distance}, comfort={self.comfortable_distance}")
        self.get_logger().info(f"Space thresholds: doorway<{self.doorway_width_threshold}, narrow<{self.narrow_hallway_threshold}")
        
        # --- State Variables ---
        self.state = State.FIND_WALL
        self.current_space_type = SpaceType.OPEN
        self.last_scan = None
        self.current_odom_pose = None
        self.last_pose_for_stuck_check = None
        self.last_pose_check_time = self.get_clock().now()
        self.recovery_start_time = None
        
        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        
        # Debug publishers
        self.marker_pub = self.create_publisher(MarkerArray, '~/debug_markers', 1)
        
        # Control timer
        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)
        
        self.get_logger().info("Perimeter Roamer V2 Initialized.")
    
    def scan_callback(self, msg):
        self.last_scan = msg
    
    def odom_callback(self, msg):
        self.current_odom_pose = msg.pose.pose
        if self.last_pose_for_stuck_check is None:
            self.last_pose_for_stuck_check = self.current_odom_pose
            self.last_pose_check_time = self.get_clock().now()
    
    def change_state(self, new_state):
        if self.state == new_state:
            return
        self.get_logger().info(f"STATE CHANGE: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.recovery_start_time = None
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped.")
    
    def classify_space(self):
        """
        Classify the current space based on LIDAR data.
        Returns SpaceType enum.
        FIXED: Proper doorway detection - a doorway is a ~1m wide opening, not a large open space.
        """
        if self.last_scan is None:
            return SpaceType.OPEN
        
        # Get distances to left and right
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        front_dist = self.get_front_distance()
        
        # Enhanced debug logging
        self.get_logger().info(f"SPACE_CLASSIFY: L={left_dist:.2f}m, R={right_dist:.2f}m, F={front_dist:.2f}m", throttle_duration_sec=3)
        
        # CORRECTED: Real doorway detection logic
        # A doorway is approximately 0.8-1.2m wide opening in a wall, not a huge open space
        # We detect this by looking for a reasonable-sized opening (not infinite distances)
        
        # First, check if we have walls on both sides (normal corridor)
        if left_dist < 5.0 and right_dist < 5.0:
            total_width = left_dist + right_dist
            
            # Space classification based on total width
            if total_width < self.very_narrow_threshold:
                self.get_logger().info(f"DETECTED: VERY_NARROW corridor (width={total_width:.2f}m)")
                return SpaceType.VERY_NARROW
            elif total_width < self.narrow_hallway_threshold:
                self.get_logger().info(f"DETECTED: NARROW_HALLWAY (width={total_width:.2f}m)")
                return SpaceType.NARROW_HALLWAY
            else:
                self.get_logger().info(f"DETECTED: NORMAL corridor (width={total_width:.2f}m)")
                return SpaceType.OPEN
        
        # Now check for actual doorways - look for reasonable doorway-sized openings
        # A doorway is typically 0.8-1.2m wide, so we look ahead at angles to find the far wall
        
        # Check for doorway on the right (robot's right side opens up)
        if right_dist > 5.0 or right_dist == float('inf'):
            # Look ahead at 30°, 45°, 60° to the right to find the far wall of a doorway
            right_30 = self.get_front_distance(-math.pi/6)   # 30° right
            right_45 = self.get_front_distance(-math.pi/4)   # 45° right
            right_60 = self.get_front_distance(-math.pi/3)   # 60° right
            
            # If we can see into the opening and it looks like a doorway (not just open space)
            if 0.8 <= right_30 <= 2.0 or 0.8 <= right_45 <= 2.0 or 0.8 <= right_60 <= 2.0:
                self.get_logger().info(f"DETECTED: DOORWAY on RIGHT (30°={right_30:.2f}m, 45°={right_45:.2f}m, 60°={right_60:.2f}m)")
                return SpaceType.DOORWAY
            else:
                self.get_logger().info(f"DETECTED: OPEN space on right (not a doorway - no far wall detected)")
                return SpaceType.OPEN
                
        # Check for doorway on the left (robot's left side opens up)
        elif left_dist > 5.0 or left_dist == float('inf'):
            # Look ahead at 30°, 45°, 60° to the left to find the far wall of a doorway
            left_30 = self.get_front_distance(math.pi/6)    # 30° left
            left_45 = self.get_front_distance(math.pi/4)    # 45° left
            left_60 = self.get_front_distance(math.pi/3)    # 60° left
            
            # If we can see into the opening and it looks like a doorway
            if 0.8 <= left_30 <= 2.0 or 0.8 <= left_45 <= 2.0 or 0.8 <= left_60 <= 2.0:
                self.get_logger().info(f"DETECTED: DOORWAY on LEFT (30°={left_30:.2f}m, 45°={left_45:.2f}m, 60°={left_60:.2f}m)")
                return SpaceType.DOORWAY
            else:
                self.get_logger().info(f"DETECTED: OPEN space on left (not a doorway - no far wall detected)")
                return SpaceType.OPEN
        
        # Neither side is particularly open - probably a normal corridor or room
        self.get_logger().info(f"DETECTED: OPEN space (L={left_dist:.2f}m, R={right_dist:.2f}m)")
        return SpaceType.OPEN
    
    def get_side_distance(self, side='right'):
        """
        Get distance to obstacle on specified side using LIDAR.
        More robust than costmap for narrow spaces.
        """
        if self.last_scan is None:
            return float('inf')
        
        ranges = np.array(self.last_scan.ranges)
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment
        
        if side == 'right':
            # -90 degrees (-pi/2)
            target_angle = -math.pi / 2
        else:  # left
            # +90 degrees (+pi/2)
            target_angle = math.pi / 2
        
        # Find the index corresponding to the target angle
        index = int((target_angle - angle_min) / angle_increment)
        
        # Average over a small range for robustness
        window = 5
        start_idx = max(0, index - window // 2)
        end_idx = min(len(ranges), index + window // 2 + 1)
        
        valid_ranges = ranges[start_idx:end_idx]
        valid_ranges = valid_ranges[np.isfinite(valid_ranges)]
        
        if len(valid_ranges) == 0:
            return float('inf')
        
        return np.median(valid_ranges)
    
    def get_front_distance(self, angle_offset=0.0):
        """
        Get distance to obstacle in front, with optional angle offset.
        """
        if self.last_scan is None:
            return float('inf')
        
        ranges = np.array(self.last_scan.ranges)
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment
        
        target_angle = angle_offset  # 0 is straight ahead
        index = int((target_angle - angle_min) / angle_increment)
        
        # Average over a small window
        window = 8
        start_idx = max(0, index - window // 2)
        end_idx = min(len(ranges), index + window // 2 + 1)
        
        valid_ranges = ranges[start_idx:end_idx]
        valid_ranges = valid_ranges[np.isfinite(valid_ranges)]
        
        if len(valid_ranges) == 0:
            return float('inf')
        
        return np.min(valid_ranges)  # Use minimum for safety
    
    def check_immediate_collision_risk(self):
        """
        Check if robot is at immediate risk of collision.
        Uses multiple LIDAR rays around the robot with different thresholds.
        """
        if self.last_scan is None:
            return True
        
        # Check straight ahead with strict threshold
        front_dist = self.get_front_distance(0)
        if front_dist < self.min_obstacle_distance:
            self.get_logger().warn(f"Immediate collision risk straight ahead: {front_dist:.3f}m")
            return True
        
        # Check sides with slightly more lenient threshold (for doorway navigation)
        side_angles = [-math.pi/6, math.pi/6]  # ±30°
        side_threshold = self.min_obstacle_distance * 0.8
        
        for angle in side_angles:
            dist = self.get_front_distance(angle)
            if dist < side_threshold:
                self.get_logger().warn(f"Collision risk at {math.degrees(angle):.1f}°: {dist:.3f}m")
                return True
        
        # Check wider angles with even more lenient threshold
        wide_angles = [-math.pi/3, math.pi/3]  # ±60°
        wide_threshold = self.min_obstacle_distance * 0.6
        
        for angle in wide_angles:
            dist = self.get_front_distance(angle)
            if dist < wide_threshold:
                self.get_logger().warn(f"Wide angle collision risk at {math.degrees(angle):.1f}°: {dist:.3f}m")
                return True
        
        return False
    
    def is_path_clear_ahead(self, check_distance=0.3):
        """
        Check if path ahead is clear for the specified distance.
        """
        front_dist = self.get_front_distance()
        return front_dist > check_distance
    
    def is_wall_on_right(self):
        """
        Check if there's a wall on the right within following distance.
        More reliable wall detection.
        """
        right_dist = self.get_side_distance('right')
        wall_detection_distance = 2.0  # Increased from 1.5 * setpoint
        
        is_wall = right_dist < wall_detection_distance and right_dist != float('inf')
        if is_wall:
            self.get_logger().info(f"WALL DETECTED on right: {right_dist:.2f}m < {wall_detection_distance:.2f}m")
        else:
            self.get_logger().info(f"No wall on right: {right_dist:.2f}m", throttle_duration_sec=3)
        
        return is_wall
    
    def get_wall_following_command(self):
        """
        Generate wall following command with improved straight-line navigation.
        Reduces constant tiny adjustments by using dead zones.
        """
        right_dist = self.get_side_distance('right')
        
        # Adjust setpoint based on space type
        if self.current_space_type == SpaceType.VERY_NARROW:
            setpoint = min(self.wall_distance_setpoint, right_dist * 0.7)
            speed = self.narrow_space_speed * 0.7
        elif self.current_space_type == SpaceType.DOORWAY:
            setpoint = min(self.wall_distance_setpoint, right_dist * 0.8)
            speed = self.narrow_space_speed
        elif self.current_space_type == SpaceType.NARROW_HALLWAY:
            setpoint = self.wall_distance_setpoint * 0.9
            speed = self.narrow_space_speed * 1.2
        else:
            setpoint = self.wall_distance_setpoint
            speed = self.forward_speed
        
        # Wall following PID with dead zone for straight-line navigation
        if right_dist == float('inf'):
            # No wall, turn right to find one
            return speed * 0.7, -self.turn_speed * 0.5
        
        error = setpoint - right_dist
        
        # IMPROVED: Use dead zone to reduce constant adjustments
        dead_zone = 0.05  # 5cm dead zone - don't adjust if within this range
        
        if abs(error) < dead_zone:
            # Close enough to target distance - go straight
            self.get_logger().debug(f"Wall following: Going straight (error={error:.3f}m within dead zone)")
            return speed, 0.0
        else:
            # Need to adjust
            angular_vel = self.kp_angular * error * 0.5  # Reduce gain for gentler adjustments
            angular_vel = max(-self.max_angular_speed * 0.3, min(self.max_angular_speed * 0.3, angular_vel))
            
            self.get_logger().debug(f"Wall following: Adjusting (error={error:.3f}m, angular={angular_vel:.3f})")
            return speed, angular_vel
    
    def navigate_narrow_space(self):
        """
        Navigate through narrow spaces with extreme caution.
        Priority: Safety first, then progress.
        """
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        front_dist = self.get_front_distance()
        
        # SAFETY FIRST: Stop if anything is too close
        min_safe_distance = self.comfortable_distance * 1.5  # Extra safety margin
        
        if front_dist < min_safe_distance:
            self.get_logger().warn(f"STOPPING: Front obstacle at {front_dist:.2f}m < {min_safe_distance:.2f}m")
            return 0.0, 0.0
        
        # Check for doorway scenario (one side open)
        if left_dist == float('inf') or right_dist == float('inf'):
            self.get_logger().info("Doorway detected - using conservative approach")
            
            if right_dist == float('inf'):
                # Doorway on right - check if it's safe to approach
                # Check multiple angles into the doorway
                right_30 = self.get_front_distance(-math.pi/6)   # 30° right
                right_45 = self.get_front_distance(-math.pi/4)   # 45° right
                right_60 = self.get_front_distance(-math.pi/3)   # 60° right
                
                min_doorway_dist = min(right_30, right_45, right_60)
                
                if min_doorway_dist < self.comfortable_distance * 2:
                    # Too close to doorway frame - continue straight slowly
                    self.get_logger().info(f"Doorway frame too close ({min_doorway_dist:.2f}m), continuing straight")
                    return self.narrow_space_speed * 0.3, 0.0
                else:
                    # Safe to approach doorway - turn very gently
                    self.get_logger().info(f"Approaching doorway safely, min_dist={min_doorway_dist:.2f}m")
                    return self.narrow_space_speed * 0.4, -self.turn_speed * 0.15
            
            elif left_dist == float('inf'):
                # Open space on left - just continue cautiously
                return self.narrow_space_speed * 0.5, 0.0
        
        # Both sides detected - corridor navigation
        if left_dist != float('inf') and right_dist != float('inf'):
            total_width = left_dist + right_dist
            
            # Check if corridor is wide enough
            min_required_width = self.robot_diameter + 0.15  # 15cm clearance
            if total_width < min_required_width:
                self.get_logger().warn(f"Corridor too narrow: {total_width:.2f}m < {min_required_width:.2f}m")
                return 0.0, 0.0
            
            # Check if either side is dangerously close
            min_side_clearance = self.robot_radius + 0.05  # 5cm clearance per side
            if left_dist < min_side_clearance or right_dist < min_side_clearance:
                self.get_logger().warn(f"Side clearance insufficient: L={left_dist:.2f}, R={right_dist:.2f}")
                return 0.0, 0.0
            
            # Safe corridor navigation - center following
            center_error = (left_dist - right_dist) / 2.0
            
            # Very conservative angular response
            angular_vel = self.kp_angular * center_error * 0.2  # Much gentler
            angular_vel = max(-self.max_angular_speed * 0.3, min(self.max_angular_speed * 0.3, angular_vel))
            
            # Speed based on corridor width and centering
            if abs(center_error) > 0.1:  # Not well centered
                speed = self.narrow_space_speed * 0.4
            else:
                speed = self.narrow_space_speed * 0.7
            
            self.get_logger().debug(f"Corridor nav: L={left_dist:.2f}, R={right_dist:.2f}, W={total_width:.2f}, err={center_error:.2f}")
            return speed, angular_vel
        
        # Fallback - something unexpected
        self.get_logger().warn("Unexpected navigation scenario, stopping")
        return 0.0, 0.0
    
    def is_stuck(self):
        """
        Check if robot hasn't moved significantly.
        """
        now = self.get_clock().now()
        delta_time = (now - self.last_pose_check_time).nanoseconds / 1e9
        
        if delta_time < self.stuck_timeout:
            return False
        
        if self.current_odom_pose is None or self.last_pose_for_stuck_check is None:
            self.last_pose_check_time = now
            return False
        
        # Calculate movement
        curr = self.current_odom_pose
        last = self.last_pose_for_stuck_check
        
        dx = curr.position.x - last.position.x
        dy = curr.position.y - last.position.y
        dist_moved = math.sqrt(dx*dx + dy*dy)
        
        # Reset for next check
        self.last_pose_for_stuck_check = self.current_odom_pose
        self.last_pose_check_time = now
        
        if dist_moved < self.stuck_distance_threshold:
            self.get_logger().warn(f"Stuck detected! Moved only {dist_moved:.3f}m in {delta_time:.1f}s")
            return True
        
        return False
    
    def control_loop(self):
        """
        Main control loop with simplified state machine.
        """
        if self.current_odom_pose is None or self.last_scan is None:
            self.get_logger().info("Waiting for sensor data...", throttle_duration_sec=3)
            return
        
        # Classify current space
        self.current_space_type = self.classify_space()
        
        # Check for immediate collision risk
        if self.check_immediate_collision_risk():
            if self.state not in [State.RECOVERY_BACKUP, State.RECOVERY_TURN]:
                self.change_state(State.RECOVERY_BACKUP)
                self.stop_robot()
                return
        
        # Check for stuck condition
        if self.state not in [State.RECOVERY_BACKUP, State.RECOVERY_TURN] and self.is_stuck():
            self.change_state(State.RECOVERY_BACKUP)
            return
        
        twist = Twist()
        
        # State machine
        if self.state == State.FIND_WALL:
            # First classify the space to understand our environment
            space_type = self.classify_space()
            self.get_logger().info(f"Space classification: {space_type}", throttle_duration_sec=2)
            
            if self.is_wall_on_right():
                self.get_logger().info("Wall found on right, switching to wall following")
                self.change_state(State.FOLLOW_WALL)
            else:
                # Turn right in place to find a wall - don't move forward
                self.get_logger().info("No wall on right, turning to find wall", throttle_duration_sec=2)
                twist.linear.x = 0.0  # Don't move forward
                twist.angular.z = -self.turn_speed * 0.5  # Turn right at moderate speed
        
        elif self.state == State.FOLLOW_WALL:
            if not self.is_path_clear_ahead(0.5):  # Increased clearance check
                self.get_logger().info("Obstacle ahead, turning left") 
                twist.angular.z = self.turn_speed
            else:
                # More conservative approach to narrow space detection
                front_dist = self.get_front_distance()
                
                # When we detect a doorway, approach it carefully
                if self.current_space_type == SpaceType.DOORWAY and front_dist > 0.8:  # Increased clearance for doorway
                    self.get_logger().info(f"*** DOORWAY DETECTED! Transitioning to APPROACH_DOORWAY. Front clearance: {front_dist:.2f}m ***")
                    self.change_state(State.APPROACH_DOORWAY)
                    linear_vel, angular_vel = self.approach_doorway()
                elif self.current_space_type == SpaceType.VERY_NARROW and front_dist > 0.6:
                    self.get_logger().info(f"*** VERY_NARROW space detected, entering carefully. Front clearance: {front_dist:.2f}m ***")
                    self.change_state(State.NAVIGATE_NARROW_SPACE)
                    linear_vel, angular_vel = self.navigate_narrow_space()
                else:
                    # Stay in wall following mode if not clearly safe
                    linear_vel, angular_vel = self.get_wall_following_command()
                    if self.current_space_type != SpaceType.OPEN:
                        self.get_logger().info(f"Wall following in {self.current_space_type.name} space: front_dist={front_dist:.2f}m", throttle_duration_sec=5)
                
                twist.linear.x = linear_vel
                twist.angular.z = angular_vel
        
        elif self.state == State.NAVIGATE_NARROW_SPACE:
            if not self.is_path_clear_ahead(0.25):
                self.get_logger().info("Obstacle in narrow space, backing out")
                self.change_state(State.RECOVERY_BACKUP)
                self.stop_robot()
                return
            
            # Check if we've exited narrow space
            if self.current_space_type == SpaceType.OPEN:
                self.get_logger().info("Exited narrow space, returning to wall following")
                self.change_state(State.FOLLOW_WALL)
                linear_vel, angular_vel = self.get_wall_following_command()
            else:
                linear_vel, angular_vel = self.navigate_narrow_space()
            
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
        
        elif self.state == State.APPROACH_DOORWAY:
            linear_vel, angular_vel = self.approach_doorway()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
        
        elif self.state == State.ENTER_DOORWAY:
            linear_vel, angular_vel = self.enter_doorway()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
        
        elif self.state == State.RECOVERY_BACKUP:
            now = self.get_clock().now()
            if self.recovery_start_time is None:
                self.recovery_start_time = now
                self.get_logger().info(f"Recovery: backing up for {self.recovery_backup_time}s")
            
            if (now - self.recovery_start_time).nanoseconds / 1e9 > self.recovery_backup_time:
                self.change_state(State.RECOVERY_TURN)
                self.stop_robot()
                return
            else:
                twist.linear.x = self.recovery_backup_speed
        
        elif self.state == State.RECOVERY_TURN:
            now = self.get_clock().now()
            if self.recovery_start_time is None:
                self.recovery_start_time = now
                self.get_logger().info(f"Recovery: turning for {self.recovery_turn_time}s")
            
            if (now - self.recovery_start_time).nanoseconds / 1e9 > self.recovery_turn_time:
                self.change_state(State.FIND_WALL)
                self.stop_robot()
                return
            else:
                twist.angular.z = self.turn_speed
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Debug logging - make it INFO level so we can see it
        space_name = self.current_space_type.name
        self.get_logger().info(f"State: {self.state.name}, Space: {space_name}, Cmd: ({twist.linear.x:.2f}, {twist.angular.z:.2f})", throttle_duration_sec=2)
    
    def approach_doorway(self):
        """
        Approach a detected doorway with comprehensive validation and positioning.
        Robot must:
        1. Clearly identify which side has the doorway opening
        2. Verify the opening is wide enough for the robot to fit through
        3. Center itself perfectly in the hallway
        4. Position itself optimally for a 90-degree turn
        5. Check ahead for obstacles (doors, furniture) before committing
        6. Plan ahead at least one robot diameter
        Returns (linear_vel, angular_vel) tuple.
        """
        # Get comprehensive distance measurements
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        front_dist = self.get_front_distance()
        
        # Get parameters
        approach_distance = self.get_parameter('doorway_approach_distance').value
        centering_tolerance = self.get_parameter('doorway_centering_tolerance').value
        
        # Comprehensive debug logging
        self.get_logger().info(f"APPROACH_DOORWAY: L={left_dist:.2f}m, R={right_dist:.2f}m, F={front_dist:.2f}m")
        
        # STEP 1: Clearly identify which side has the doorway opening
        doorway_on_right = False
        doorway_on_left = False
        
        # CORRECTED: Proper doorway side detection
        # Look for actual doorway structure, not just large distances
        
        # Check right side for doorway - look for opening with far wall
        if right_dist > 5.0 or right_dist == float('inf'):
            # Look into the right opening for doorway structure
            right_30 = self.get_front_distance(-math.pi/6)   # 30° right
            right_45 = self.get_front_distance(-math.pi/4)   # 45° right
            right_60 = self.get_front_distance(-math.pi/3)   # 60° right
            
            # Check if we can see doorway structure (far wall at reasonable distance)
            if 0.8 <= right_30 <= 2.0 or 0.8 <= right_45 <= 2.0 or 0.8 <= right_60 <= 2.0:
                doorway_on_right = True
                self.get_logger().info(f"*** DOORWAY CONFIRMED ON RIGHT: Structure detected (30°={right_30:.2f}m, 45°={right_45:.2f}m, 60°={right_60:.2f}m) ***")
            else:
                self.get_logger().info(f"Right side open but no doorway structure - just open space (30°={right_30:.2f}m, 45°={right_45:.2f}m, 60°={right_60:.2f}m)")
        
        # Check left side for doorway
        if not doorway_on_right and (left_dist > 5.0 or left_dist == float('inf')):
            # Look into the left opening for doorway structure
            left_30 = self.get_front_distance(math.pi/6)    # 30° left
            left_45 = self.get_front_distance(math.pi/4)    # 45° left
            left_60 = self.get_front_distance(math.pi/3)    # 60° left
            
            # Check if we can see doorway structure
            if 0.8 <= left_30 <= 2.0 or 0.8 <= left_45 <= 2.0 or 0.8 <= left_60 <= 2.0:
                doorway_on_left = True
                self.get_logger().info(f"*** DOORWAY CONFIRMED ON LEFT: Structure detected (30°={left_30:.2f}m, 45°={left_45:.2f}m, 60°={left_60:.2f}m) ***")
            else:
                self.get_logger().info(f"Left side open but no doorway structure - just open space (30°={left_30:.2f}m, 45°={left_45:.2f}m, 60°={left_60:.2f}m)")
        
        # If no clear doorway structure detected, don't try to enter
        if not doorway_on_right and not doorway_on_left:
            self.get_logger().warn(f"NO DOORWAY STRUCTURE DETECTED - returning to wall following (L={left_dist:.2f}m, R={right_dist:.2f}m)")
            self.change_state(State.FOLLOW_WALL)
            return self.get_wall_following_command()
        
        if doorway_on_right:
            # STEP 2: Analyze the right doorway opening in detail
            self.get_logger().info("=== ANALYZING RIGHT DOORWAY ===")
            
            # Check multiple angles into the doorway to understand the opening
            doorway_30 = self.get_front_distance(-math.pi/6)   # 30° right
            doorway_45 = self.get_front_distance(-math.pi/4)   # 45° right
            doorway_60 = self.get_front_distance(-math.pi/3)   # 60° right
            doorway_90 = self.get_side_distance('right')       # 90° right (into doorway)
            
            self.get_logger().info(f"Right doorway analysis: 30°={doorway_30:.2f}m, 45°={doorway_45:.2f}m, 60°={doorway_60:.2f}m, 90°={doorway_90:.2f}m")
            
            # STEP 3: Verify opening is wide enough for robot
            # We need to look ahead into the doorway to see the far wall
            # This tells us the actual doorway width
            min_doorway_depth = min(doorway_30, doorway_45, doorway_60, doorway_90)
            
            # Check if there's a door or obstacle blocking the way
            planning_distance = self.robot_diameter * 1.5  # Plan ahead 1.5 robot diameters
            if min_doorway_depth < planning_distance:
                self.get_logger().warn(f"DOORWAY BLOCKED OR TOO NARROW: min_depth={min_doorway_depth:.2f}m < planning_distance={planning_distance:.2f}m")
                self.get_logger().warn("This could be a closed door or furniture - returning to wall following")
                self.change_state(State.FOLLOW_WALL)
                return self.get_wall_following_command()
            
            # Estimate doorway width by looking at angles
            # A typical doorway should show depth at multiple angles
            if doorway_45 > 1.0 and doorway_60 > 0.8:
                estimated_doorway_width = min(doorway_45 * 0.7, doorway_60 * 1.0)  # Conservative estimate
                self.get_logger().info(f"Estimated doorway width: {estimated_doorway_width:.2f}m")
                
                if estimated_doorway_width < self.robot_diameter + 0.1:  # Need 10cm clearance
                    self.get_logger().warn(f"DOORWAY TOO NARROW: estimated_width={estimated_doorway_width:.2f}m < required={self.robot_diameter + 0.1:.2f}m")
                    self.change_state(State.FOLLOW_WALL)
                    return self.get_wall_following_command()
            else:
                self.get_logger().warn(f"Cannot estimate doorway width reliably: 45°={doorway_45:.2f}m, 60°={doorway_60:.2f}m")
                # Continue with caution but reduce confidence
            
            # STEP 4: Check if we're close enough to start positioning
            if front_dist < approach_distance:
                self.get_logger().info(f"Close enough to doorway ({front_dist:.2f}m < {approach_distance:.2f}m) - starting positioning sequence")
                
                # STEP 5: Ensure we're centered in the hallway first
                if left_dist != float('inf'):  # We have a left wall to reference
                    # Calculate how far we are from the center of the hallway
                    # Ideal position: equal distance from both walls, accounting for robot radius
                    ideal_distance_from_left_wall = self.robot_radius + 0.1  # 10cm safety margin
                    hallway_center_error = left_dist - ideal_distance_from_left_wall
                    
                    self.get_logger().info(f"Hallway centering check: left_dist={left_dist:.2f}m, ideal={ideal_distance_from_left_wall:.2f}m, error={hallway_center_error:.2f}m")
                    
                    if abs(hallway_center_error) > centering_tolerance:
                        # Need to center in hallway first
                        self.get_logger().info(f"CENTERING IN HALLWAY: error={hallway_center_error:.3f}m (tolerance={centering_tolerance:.3f}m)")
                        linear_vel = 0.015  # Very slow forward motion while centering
                        if hallway_center_error > 0:
                            angular_vel = -0.08  # Move right (away from left wall)
                            self.get_logger().info("Adjusting RIGHT (away from left wall)")
                        else:
                            angular_vel = 0.08   # Move left (toward left wall)
                            self.get_logger().info("Adjusting LEFT (toward left wall)")
                        return linear_vel, angular_vel
                
                # STEP 6: Position optimally for the 90-degree turn
                # We want to be positioned so we can make a clean turn without hitting the doorway frame
                optimal_turn_distance = self.robot_radius + 0.2  # 20cm clearance from doorway edge
                
                self.get_logger().info(f"Turn positioning check: doorway_30={doorway_30:.2f}m, optimal={optimal_turn_distance:.2f}m")
                
                if doorway_30 > optimal_turn_distance and doorway_45 > optimal_turn_distance * 0.8:
                    self.get_logger().info("*** PERFECTLY POSITIONED FOR 90° RIGHT TURN! ***")
                    self.get_logger().info("All safety checks passed - starting doorway entry sequence")
                    # We're positioned correctly - start the 90-degree turn
                    self.change_state(State.ENTER_DOORWAY)
                    return 0.0, -0.2  # Start pure rotation to the right
                else:
                    # Need to adjust position slightly
                    if doorway_30 < optimal_turn_distance:
                        # Too close to doorway edge - back up a tiny bit
                        self.get_logger().info(f"Too close to doorway edge - backing up slightly (30°={doorway_30:.2f}m)")
                        return -0.01, 0.0  # Very slow backup
                    else:
                        # Move forward a bit more to get better turning position
                        self.get_logger().info(f"Adjusting forward position for optimal turn")
                        return 0.02, 0.0
            else:
                # Still approaching the doorway
                self.get_logger().info(f"APPROACHING DOORWAY: {front_dist:.2f}m > {approach_distance:.2f}m")
                return 0.04, 0.0  # Slow, steady approach
                
        elif doorway_on_left:
            # STEP 2: Analyze the left doorway opening in detail
            self.get_logger().info("=== ANALYZING LEFT DOORWAY ===")
            
            # Check multiple angles into the doorway
            doorway_30 = self.get_front_distance(math.pi/6)    # 30° left
            doorway_45 = self.get_front_distance(math.pi/4)    # 45° left
            doorway_60 = self.get_front_distance(math.pi/3)    # 60° left  
            doorway_90 = self.get_side_distance('left')        # 90° left
            
            self.get_logger().info(f"Left doorway analysis: 30°={doorway_30:.2f}m, 45°={doorway_45:.2f}m, 60°={doorway_60:.2f}m, 90°={doorway_90:.2f}m")
            
            # STEP 3: Verify opening is wide enough and not blocked
            min_doorway_depth = min(doorway_30, doorway_45, doorway_60, doorway_90)
            planning_distance = self.robot_diameter * 1.5
            
            if min_doorway_depth < planning_distance:
                self.get_logger().warn(f"LEFT DOORWAY BLOCKED OR TOO NARROW: min_depth={min_doorway_depth:.2f}m < planning_distance={planning_distance:.2f}m")
                self.change_state(State.FOLLOW_WALL)
                return self.get_wall_following_command()
            
            # Estimate doorway width
            if doorway_45 > 1.0 and doorway_60 > 0.8:
                estimated_doorway_width = min(doorway_45 * 0.7, doorway_60 * 1.0)
                self.get_logger().info(f"Estimated left doorway width: {estimated_doorway_width:.2f}m")
                
                if estimated_doorway_width < self.robot_diameter + 0.1:
                    self.get_logger().warn(f"LEFT DOORWAY TOO NARROW: estimated_width={estimated_doorway_width:.2f}m")
                    self.change_state(State.FOLLOW_WALL)
                    return self.get_wall_following_command()
            
            # STEP 4: Position for left doorway
            if front_dist < approach_distance:
                self.get_logger().info(f"Close enough to left doorway - starting positioning")
                
                # STEP 5: Center in hallway using right wall as reference
                if right_dist != float('inf'):
                    ideal_distance_from_right_wall = self.robot_radius + 0.1
                    hallway_center_error = right_dist - ideal_distance_from_right_wall
                    
                    self.get_logger().info(f"Hallway centering (left doorway): right_dist={right_dist:.2f}m, ideal={ideal_distance_from_right_wall:.2f}m, error={hallway_center_error:.2f}m")
                    
                    if abs(hallway_center_error) > centering_tolerance:
                        self.get_logger().info(f"CENTERING IN HALLWAY for LEFT turn: error={hallway_center_error:.3f}m")
                        linear_vel = 0.015
                        if hallway_center_error > 0:
                            angular_vel = 0.08   # Move left (away from right wall)
                            self.get_logger().info("Adjusting LEFT (away from right wall)")
                        else:
                            angular_vel = -0.08  # Move right (toward right wall)
                            self.get_logger().info("Adjusting RIGHT (toward right wall)")
                        return linear_vel, angular_vel
                
                # STEP 6: Position for left turn
                optimal_turn_distance = self.robot_radius + 0.2
                
                if doorway_30 > optimal_turn_distance and doorway_45 > optimal_turn_distance * 0.8:
                    self.get_logger().info("*** PERFECTLY POSITIONED FOR 90° LEFT TURN! ***")
                    self.change_state(State.ENTER_DOORWAY)
                    return 0.0, 0.2  # Pure rotation to the left
                else:
                    if doorway_30 < optimal_turn_distance:
                        self.get_logger().info(f"Too close to left doorway edge - backing up slightly")
                        return -0.01, 0.0
                    else:
                        self.get_logger().info(f"Adjusting forward position for left turn")
                        return 0.02, 0.0
            else:
                self.get_logger().info(f"APPROACHING LEFT DOORWAY: {front_dist:.2f}m > {approach_distance:.2f}m")
                return 0.04, 0.0
        else:
            # This should never happen given our improved detection logic
            self.get_logger().error("CRITICAL: No doorway detected in APPROACH_DOORWAY state!")
            self.change_state(State.FOLLOW_WALL)
            return self.get_wall_following_command()
    
    def enter_doorway(self):
        """
        Execute the doorway entry sequence with comprehensive safety checks.
        This handles the 90-degree turn and forward motion through the doorway,
        continuously checking for obstacles like doors or furniture.
        Returns (linear_vel, angular_vel) tuple.
        """
        front_dist = self.get_front_distance()
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        
        self.get_logger().info(f"ENTER_DOORWAY: L={left_dist:.2f}m, R={right_dist:.2f}m, F={front_dist:.2f}m")
        
        # Determine doorway direction based on current readings
        doorway_on_right = (right_dist == float('inf') or right_dist > left_dist * 2.5)
        doorway_on_left = (left_dist == float('inf') or left_dist > right_dist * 2.5)
        
        # CRITICAL SAFETY CHECK: Ensure we have clearance ahead
        planning_ahead_distance = self.robot_diameter * 1.2  # Plan ahead 1.2 robot diameters
        if not self.is_path_clear_ahead(planning_ahead_distance):
            self.get_logger().warn(f"DOORWAY ENTRY ABORTED: Insufficient clearance ahead ({front_dist:.2f}m < {planning_ahead_distance:.2f}m)")
            self.get_logger().warn("This could indicate a closed door or obstacle in the doorway")
            self.change_state(State.RECOVERY_BACKUP)
            return 0.0, 0.0
        
        if doorway_on_right:
            # Entering doorway on the right
            self.get_logger().info("=== ENTERING RIGHT DOORWAY ===")
            
            # Comprehensive angle analysis for right turn
            right_15 = self.get_front_distance(-math.pi/12)  # 15° right
            right_30 = self.get_front_distance(-math.pi/6)   # 30° right  
            right_45 = self.get_front_distance(-math.pi/4)   # 45° right
            right_60 = self.get_front_distance(-math.pi/3)   # 60° right
            right_90 = self.get_side_distance('right')       # 90° right (into doorway)
            
            self.get_logger().info(f"Right entry analysis: 15°={right_15:.2f}m, 30°={right_30:.2f}m, 45°={right_45:.2f}m, 60°={right_60:.2f}m, 90°={right_90:.2f}m")
            
            # Check for obstacles at each angle - could be door frame, partially open door, furniture
            min_safe_distance = self.robot_radius + 0.1  # 10cm clearance
            
            if right_15 < min_safe_distance:
                self.get_logger().warn(f"OBSTACLE at 15° right: {right_15:.2f}m - possible door frame or partially open door")
                self.change_state(State.RECOVERY_BACKUP)
                return 0.0, 0.0
            
            # Determine our turning progress
            if right_90 > 0.8 and right_60 > 0.6:
                # We're well aligned with the doorway - can move forward
                self.get_logger().info("RIGHT TURN COMPLETE - Moving forward through doorway")
                self.get_logger().info(f"Clearances: 60°={right_60:.2f}m, 90°={right_90:.2f}m")
                
                # Double-check for doors or obstacles ahead in the new direction
                forward_clearance = min(right_60, right_90)
                if forward_clearance < self.robot_diameter:
                    self.get_logger().warn(f"BLOCKED PASSAGE: forward_clearance={forward_clearance:.2f}m < robot_diameter={self.robot_diameter:.2f}m")
                    self.change_state(State.RECOVERY_BACKUP)
                    return 0.0, 0.0
                
                return 0.025, 0.0   # Slow, careful forward motion
                
            elif right_30 > 0.4 and right_45 > 0.3:
                # Still turning - continue the 90° turn
                self.get_logger().info("Continuing 90° RIGHT turn - good clearance ahead")
                return 0.0, -0.15  # Moderate right turn
            else:
                # Not enough clearance to continue turning
                self.get_logger().warn(f"INSUFFICIENT CLEARANCE for right turn: 30°={right_30:.2f}m, 45°={right_45:.2f}m")
                self.change_state(State.RECOVERY_BACKUP)
                return 0.0, 0.0
                
        elif doorway_on_left:
            # Entering doorway on the left
            self.get_logger().info("=== ENTERING LEFT DOORWAY ===")
            
            # Comprehensive angle analysis for left turn
            left_15 = self.get_front_distance(math.pi/12)   # 15° left
            left_30 = self.get_front_distance(math.pi/6)    # 30° left
            left_45 = self.get_front_distance(math.pi/4)    # 45° left
            left_60 = self.get_front_distance(math.pi/3)    # 60° left
            left_90 = self.get_side_distance('left')        # 90° left (into doorway)
            
            self.get_logger().info(f"Left entry analysis: 15°={left_15:.2f}m, 30°={left_30:.2f}m, 45°={left_45:.2f}m, 60°={left_60:.2f}m, 90°={left_90:.2f}m")
            
            # Check for obstacles
            min_safe_distance = self.robot_radius + 0.1
            
            if left_15 < min_safe_distance:
                self.get_logger().warn(f"OBSTACLE at 15° left: {left_15:.2f}m - possible door frame or partially open door")
                self.change_state(State.RECOVERY_BACKUP)
                return 0.0, 0.0
            
            # Determine turning progress
            if left_90 > 0.8 and left_60 > 0.6:
                # Well aligned - move forward
                self.get_logger().info("LEFT TURN COMPLETE - Moving forward through doorway")
                
                forward_clearance = min(left_60, left_90)
                if forward_clearance < self.robot_diameter:
                    self.get_logger().warn(f"BLOCKED PASSAGE: forward_clearance={forward_clearance:.2f}m")
                    self.change_state(State.RECOVERY_BACKUP)
                    return 0.0, 0.0
                
                return 0.025, 0.0
                
            elif left_30 > 0.4 and left_45 > 0.3:
                # Continue turning left
                self.get_logger().info("Continuing 90° LEFT turn - good clearance ahead")
                return 0.0, 0.15   # Moderate left turn
            else:
                self.get_logger().warn(f"INSUFFICIENT CLEARANCE for left turn: 30°={left_30:.2f}m, 45°={left_45:.2f}m")
                self.change_state(State.RECOVERY_BACKUP)
                return 0.0, 0.0
        else:
            # No clear doorway - this shouldn't happen
            self.get_logger().error("CRITICAL: No doorway detected during ENTER_DOORWAY state!")
            self.change_state(State.FOLLOW_WALL)
            return self.get_wall_following_command()
        
        # Check if we've successfully entered and space has opened up
        if self.current_space_type == SpaceType.OPEN:
            self.get_logger().info("*** DOORWAY ENTRY SUCCESSFUL - Space opened up! Returning to wall following ***")
            self.change_state(State.FOLLOW_WALL)
            return self.get_wall_following_command()


def main(args=None):
    rclpy.init(args=args)
    node = PerimeterRoamerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
