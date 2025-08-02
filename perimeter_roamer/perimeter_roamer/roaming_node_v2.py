import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
import math
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from geometry_msgs.msg import Twist, Point, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import Marker, MarkerArray
from enum import Enum
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor

class State(Enum):
    FIND_WALL = 0
    FOLLOW_WALL = 1
    NAVIGATE_NARROW_SPACE = 2
    APPROACH_DOORWAY = 3
    ENTER_DOORWAY = 4
    POST_DOORWAY_FORWARD = 5
    RECOVERY_BACKUP = 6
    RECOVERY_TURN = 7

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
        self.declare_parameter('doorway_detection_threshold', 1.5,
                                 ParameterDescriptor(description='Distance in meters a side reading must exceed to be considered a doorway.'))
        self.declare_parameter('wall_disappearance_threshold', 2.0,
                                 ParameterDescriptor(description='Distance in meters the opposite wall must exceed to confirm alignment with a doorway.'))
        self.declare_parameter('post_doorway_forward_time', 3.0,
                                 ParameterDescriptor(description='Time in seconds to move forward after entering a doorway to clear it.'))
        self.declare_parameter('turn_duration', 2.5,
                                 ParameterDescriptor(description='Time in seconds to execute a 90-degree turn into a doorway.'))

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
        self.doorway_detection_threshold = self.get_parameter('doorway_detection_threshold').value
        self.wall_disappearance_threshold = self.get_parameter('wall_disappearance_threshold').value
        self.post_doorway_forward_time = self.get_parameter('post_doorway_forward_time').value
        self.turn_duration = self.get_parameter('turn_duration').value

        # State variables for doorway navigation
        self.turn_direction = 0  # 1 for port (+Y), -1 for starboard (-Y)
        self.turn_start_time = None
        self.post_doorway_start_time = None

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
        
        # TF2 for coordinate frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
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
        
        # Wait for transforms to become available
        self.wait_for_transforms()
        
        self.get_logger().info("Perimeter Roamer V2 Initialized.")
    
    def wait_for_transforms(self):
        """Wait for tf2 transforms to become available before starting control loop."""
        self.get_logger().info("Waiting for tf2 transforms to become available...")
        
        # Wait for laser scan data first
        while self.last_scan is None and rclpy.ok():
            self.get_logger().info("Waiting for first laser scan...")
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not rclpy.ok():
            return
            
        # Now wait for transform to be available
        max_wait_time = 10.0  # Maximum time to wait in seconds
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            try:
                # IMPORTANT: Use the actual frame_id from the scan message, not hardcoded names
                scan_frame_id = self.last_scan.header.frame_id
                
                # Try to get the transform from base_link to the actual LIDAR frame
                transform = self.tf_buffer.lookup_transform(
                    scan_frame_id,  # target frame (actual laser frame from scan)
                    self.robot_base_frame,  # source frame (base_link)
                    rclpy.time.Time(),  # latest available
                    timeout=Duration(seconds=0.1)
                )
                self.get_logger().info(f"Transform from {self.robot_base_frame} to {scan_frame_id} is now available!")
                break
                
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                current_time = self.get_clock().now()
                elapsed = (current_time - start_time).nanoseconds / 1e9
                
                if elapsed > max_wait_time:
                    self.get_logger().warn(f"Timeout waiting for transform after {max_wait_time}s. Proceeding anyway...")
                    break
                    
                if elapsed > 2.0:  # Only log every 2 seconds to avoid spam
                    scan_frame_name = self.last_scan.header.frame_id if self.last_scan else "unknown"
                    self.get_logger().info(f"Still waiting for transform ({elapsed:.1f}s): {self.robot_base_frame} -> {scan_frame_name}: {e}")
                
                rclpy.spin_once(self, timeout_sec=0.1)
    
    def scan_callback(self, msg):
        self.last_scan = msg
    
    def odom_callback(self, msg):
        self.current_odom_pose = msg.pose.pose
        if self.last_pose_for_stuck_check is None:
            self.last_pose_for_stuck_check = self.current_odom_pose
            self.last_pose_check_time = self.get_clock().now()
    
    def change_state(self, new_state):
        """Changes the robot's state and resets doorway variables if exiting doorway sequence."""
        if self.state != new_state:
            self.get_logger().info(self.format_position_debug(f"STATE CHANGE: {self.state.name} -> {new_state.name}"))
            
            # If we are exiting the doorway navigation sequence, reset all related variables
            if self.state in [State.APPROACH_DOORWAY, State.ENTER_DOORWAY, State.POST_DOORWAY_FORWARD] and \
               new_state not in [State.APPROACH_DOORWAY, State.ENTER_DOORWAY, State.POST_DOORWAY_FORWARD]:
                self.reset_doorway_state()

            self.state = new_state
            self.state_start_time = self.get_clock().now()
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def get_robot_position_in_map(self):
        """
        Get the robot's current position in the map frame.
        Returns (x, y) tuple in map coordinates, or (None, None) if transform fails.
        """
        if self.current_odom_pose is None:
            return None, None
            
        try:
            # Create a PointStamped with robot position in odom frame
            point_odom = PointStamped()
            point_odom.header.frame_id = self.odom_frame
            point_odom.header.stamp = self.get_clock().now().to_msg()
            point_odom.point.x = self.current_odom_pose.position.x
            point_odom.point.y = self.current_odom_pose.position.y
            point_odom.point.z = 0.0
            
            # Transform to map frame
            point_map = self.tf_buffer.transform(point_odom, 'map', timeout=Duration(seconds=0.1))
            
            return point_map.point.x, point_map.point.y
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Log the transform failure and try base_link to map directly  
            self.get_logger().debug(f"Odom->Map transform failed: {e}, trying base_link->map")
            try:
                # Alternative: try base_link to map transform directly
                transform = self.tf_buffer.lookup_transform('map', self.robot_base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
                return transform.transform.translation.x, transform.transform.translation.y
            except (LookupException, ConnectivityException, ExtrapolationException) as e2:
                # Only return odom coordinates as last resort and warn user
                self.get_logger().warn(f"All map transforms failed ({e}, {e2}), returning odom coordinates as fallback")
                return self.current_odom_pose.position.x, self.current_odom_pose.position.y
    
    def format_position_debug(self, message):
        """
        Add robot position information to debug messages.
        Returns formatted string with position info in MAP frame.
        """
        map_x, map_y = self.get_robot_position_in_map()
        if map_x is not None and map_y is not None:
            return f"[Map: ({map_x:.2f}, {map_y:.2f})] {message}"
        else:
            return f"[Map: (unknown)] {message}"
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped.")
    
    def classify_space(self):
        """
        Classify the current space based on LIDAR data.
        Returns SpaceType enum.
        
        Using tf2 to transform all measurements to base_link frame:
        - Forward = +X direction 
        - Left = +Y direction
        - Right = -Y direction (using negative Y for right)
        
        FIXED: Proper doorway detection - a doorway is a ~1m wide opening, not a large open space.
        """
        if self.last_scan is None:
            return SpaceType.OPEN
        
        # Get distances using tf2-transformed measurements to base_link frame
        right_dist = self.get_side_distance('right')    # -Y direction in base_link
        left_dist = self.get_side_distance('left')      # +Y direction in base_link
        forward_dist = self.get_front_distance()        # +X direction in base_link
        
        # Enhanced debug logging with base_link coordinate frame references  
        # Show the ratio analysis for doorway detection
        right_left_ratio = right_dist / left_dist if left_dist > 0.1 else float('inf')
        left_right_ratio = left_dist / right_dist if right_dist > 0.1 else float('inf')
        
        self.get_logger().info(self.format_position_debug(f"SPACE_CLASSIFY: Right(-Y)={right_dist:.2f}m, Left(+Y)={left_dist:.2f}m, Forward(+X)={forward_dist:.2f}m, R/L={right_left_ratio:.2f}, L/R={left_right_ratio:.2f}"), throttle_duration_sec=3)
        
        # CORRECTED: Real doorway detection logic
        # A doorway is approximately 0.8-1.2m wide opening in a wall, not a huge open space
        # We detect this by looking for a reasonable-sized opening (not infinite distances)
        
        # First, check if we have walls on both right and left sides (normal corridor)
        if right_dist < 5.0 and left_dist < 5.0:
            # Calculate corridor width (distance from right wall to left wall)
            corridor_width = right_dist + left_dist
            
            # Space classification based on total corridor width
            if corridor_width < self.very_narrow_threshold:
                self.get_logger().info(f"DETECTED: VERY_NARROW corridor (width={corridor_width:.2f}m)")
                return SpaceType.VERY_NARROW
            elif corridor_width < self.narrow_hallway_threshold:
                self.get_logger().info(f"DETECTED: NARROW_HALLWAY (width={corridor_width:.2f}m)")
                return SpaceType.NARROW_HALLWAY
            else:
                self.get_logger().info(self.format_position_debug(f"DETECTED: NORMAL corridor (width={corridor_width:.2f}m, left={left_dist:.2f}m, right={right_dist:.2f}m) - no wall breaks detected"))
                return SpaceType.OPEN
        
        # Now check for actual doorways - look for BREAKS IN WALLS
        # A doorway is simply a gap where there used to be a wall - not about depth beyond the wall
        
        # SIMPLIFIED: Check for doorway on the right (-Y) side 
        # Look for significant increase in right side distance indicating a wall break
        if right_dist > left_dist * 2.0 and right_dist > 1.5:  # Right side much more open - wall break detected
            # This is a break in the right wall - likely a doorway
            forward_clearance = self.get_front_distance()
            min_approach_clearance = self.robot_diameter * 3.0  # Need substantial clearance to approach safely
            
            if forward_clearance > min_approach_clearance:
                self.get_logger().info(self.format_position_debug(f"DETECTED: DOORWAY on RIGHT(-Y) - Wall break detected (right={right_dist:.2f}m >> left={left_dist:.2f}m, ratio={right_dist/left_dist:.2f})"))
                return SpaceType.DOORWAY
            else:
                self.get_logger().info(self.format_position_debug(f"RIGHT wall break detected but insufficient forward clearance ({forward_clearance:.2f}m < {min_approach_clearance:.2f}m) - staying in corridor mode"))
                return SpaceType.NARROW_HALLWAY
                
        # SIMPLIFIED: Check for doorway on the left (+Y) side
        elif left_dist > right_dist * 2.0 and left_dist > 1.5:  # Left side much more open - wall break detected
            # This is a break in the left wall - likely a doorway
            forward_clearance = self.get_front_distance()
            min_approach_clearance = self.robot_diameter * 3.0  # Need substantial clearance to approach safely
            
            if forward_clearance > min_approach_clearance:
                self.get_logger().info(self.format_position_debug(f"DETECTED: DOORWAY on LEFT(+Y) - Wall break detected (left={left_dist:.2f}m >> right={right_dist:.2f}m, ratio={left_dist/right_dist:.2f})"))
                return SpaceType.DOORWAY
            else:
                self.get_logger().info(self.format_position_debug(f"LEFT wall break detected but insufficient forward clearance ({forward_clearance:.2f}m < {min_approach_clearance:.2f}m) - staying in corridor mode"))
                return SpaceType.NARROW_HALLWAY
        
        # Neither side is particularly open - probably a normal corridor or room
        self.get_logger().info(f"DETECTED: OPEN space (Right(-Y)={right_dist:.2f}m, Left(+Y)={left_dist:.2f}m)")
        return SpaceType.OPEN
    
    def get_distance_in_direction(self, direction_x, direction_y, max_range=10.0):
        """
        Get distance to obstacle in specified direction using tf2 to transform LIDAR data to base_link frame.
        
        Args:
            direction_x: X component of direction vector in base_link frame (+X = forward)
            direction_y: Y component of direction vector in base_link frame (+Y = left)
            max_range: Maximum range to check
            
        Returns:
            Distance to obstacle in meters, or float('inf') if no obstacle
        """
        if self.last_scan is None:
            return float('inf')
            
        # Ensure we have a valid frame_id in the scan message
        if not hasattr(self.last_scan, 'header') or not hasattr(self.last_scan.header, 'frame_id'):
            self.get_logger().warn("Invalid scan message: missing header or frame_id")
            return float('inf')
            
        if not self.last_scan.header.frame_id:
            self.get_logger().warn("Invalid scan message: empty frame_id")
            return float('inf')
            
        try:
            # IMPORTANT: Use the actual frame_id from the scan message, not hardcoded names
            scan_frame_id = self.last_scan.header.frame_id
            
            # Get transform from base_link to laser frame (to know which laser ray to use)
            # Use latest available transform instead of exact timestamp to avoid timing issues
            transform = self.tf_buffer.lookup_transform(
                scan_frame_id,  # target frame (laser frame from scan message)
                self.robot_base_frame,  # source frame (base_link)
                rclpy.time.Time(),  # Use latest available transform
                timeout=Duration(seconds=0.5)  # Shorter timeout to fail faster
            )
            
            # Create a point in the target direction in base_link frame
            target_point_stamped = PointStamped()
            target_point_stamped.header.frame_id = self.robot_base_frame
            target_point_stamped.header.stamp = rclpy.time.Time().to_msg()  # Use current time
            target_point_stamped.point.x = direction_x
            target_point_stamped.point.y = direction_y
            target_point_stamped.point.z = 0.0
            
            # Transform to laser frame to find which laser ray to use
            target_point_laser_stamped = tf2_geometry_msgs.do_transform_point(target_point_stamped, transform)
            
            # Calculate angle in laser frame
            target_angle_laser = math.atan2(target_point_laser_stamped.point.y, target_point_laser_stamped.point.x)
            
            # Find corresponding LIDAR ray
            ranges = np.array(self.last_scan.ranges)
            angle_min = self.last_scan.angle_min
            angle_increment = self.last_scan.angle_increment
            
            index = int((target_angle_laser - angle_min) / angle_increment)
            
            # Ensure index is within bounds
            if index < 0 or index >= len(ranges):
                return float('inf')
            
            # Average over a small window for robustness
            window = 5
            start_idx = max(0, index - window // 2)
            end_idx = min(len(ranges), index + window // 2 + 1)
            
            valid_ranges = ranges[start_idx:end_idx]
            valid_ranges = valid_ranges[np.isfinite(valid_ranges)]
            valid_ranges = valid_ranges[valid_ranges <= max_range]
            
            if len(valid_ranges) == 0:
                return float('inf')
            
            return np.median(valid_ranges)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # More informative error logging
            scan_frame_name = self.last_scan.header.frame_id if self.last_scan else "unknown"
            self.get_logger().warn(f"TF2 transform failed ({self.robot_base_frame} -> {scan_frame_name}): {e}")
            return float('inf')
        except Exception as e:
            self.get_logger().error(f"Unexpected error in get_distance_in_direction: {e}")
            return float('inf')
    
    def get_side_distance(self, side='right'):
        """
        Get distance to obstacle on specified side using tf2 transformations to base_link frame.
        
        Args:
            side: 'right' or 'left' relative to base_link frame
        """
        if side == 'right':
            # Right side in base_link frame is -Y direction
            return self.get_distance_in_direction(0.0, -1.0)
        else:  # left
            # Left side in base_link frame is +Y direction
            return self.get_distance_in_direction(0.0, 1.0)
    
    def get_front_distance(self, angle_offset=0.0):
        """
        Get distance to obstacle in front using tf2 transformations to base_link frame.
        
        Args:
            angle_offset: Angle offset from forward direction (radians)
        """
        # Forward in base_link frame is +X direction
        direction_x = math.cos(angle_offset)
        direction_y = math.sin(angle_offset)
        return self.get_distance_in_direction(direction_x, direction_y)
    
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
        Navigate through narrow spaces by centering the robot.
        Priority: Safety first, then progress.
        """
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        front_dist = self.get_front_distance()
        
        # SAFETY FIRST: Stop if anything is too close
        min_safe_distance = self.comfortable_distance
        
        if front_dist < min_safe_distance:
            self.get_logger().warn(f"STOPPING: Front obstacle at {front_dist:.2f}m < {min_safe_distance:.2f}m")
            self.change_state(State.RECOVERY_BACKUP)
            return 0.0, 0.0
        
        # If one side is open, it might be a doorway.
        if self.current_space_type == SpaceType.DOORWAY:
             self.get_logger().info(self.format_position_debug("Doorway detected while in narrow space, transitioning to approach."))
             self.change_state(State.APPROACH_DOORWAY)
             return 0.0, 0.0

        # Both sides detected - corridor navigation
        if left_dist != float('inf') and right_dist != float('inf'):
            # Safe corridor navigation - center following
            # Positive error means we are too close to the right wall, so we should turn left (positive angular.z)
            center_error = left_dist - right_dist
            
            # Proportional controller for centering
            angular_vel = self.kp_angular * center_error
            angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
            
            # Slow down if turning sharply or not centered
            speed = self.narrow_space_speed
            if abs(center_error) > 0.1 or abs(angular_vel) > 0.2:
                speed *= 0.7
            
            total_width = left_dist + right_dist
            self.get_logger().debug(f"Corridor nav: L={left_dist:.2f}, R={right_dist:.2f}, W={total_width:.2f}, err={center_error:.2f}, cmd_vel=({speed:.2f}, {angular_vel:.2f})")
            return speed, angular_vel
        
        # Fallback - something unexpected, maybe one wall disappeared
        self.get_logger().warn("Lost one wall in narrow space, switching to FIND_WALL")
        self.change_state(State.FIND_WALL)
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
    
    def reset_doorway_state(self):
        """Resets all state variables related to doorway navigation."""
        self.get_logger().info("Resetting doorway navigation state.")
        self.turn_direction = 0
        self.turn_start_time = None
        self.post_doorway_start_time = None

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
            self.get_logger().info(self.format_position_debug(f"Space classification: {space_type}"), throttle_duration_sec=2)
            
            if self.is_wall_on_right():
                self.get_logger().info(self.format_position_debug("Wall found on right, switching to wall following"))
                self.change_state(State.FOLLOW_WALL)
            else:
                # Turn right in place to find a wall - don't move forward
                self.get_logger().info(self.format_position_debug("No wall on right, turning to find wall"), throttle_duration_sec=2)
                twist.linear.x = 0.0  # Don't move forward
                twist.angular.z = -self.turn_speed * 0.5  # Turn right at moderate speed
        
        elif self.state == State.FOLLOW_WALL:
            # In narrow spaces, we should center ourselves, not follow one wall.
            if self.current_space_type in [SpaceType.NARROW_HALLWAY, SpaceType.VERY_NARROW]:
                self.get_logger().info(self.format_position_debug(f"In {self.current_space_type.name}, switching to centering behavior."), throttle_duration_sec=5)
                self.change_state(State.NAVIGATE_NARROW_SPACE)
                return # State changed, next loop will execute NAVIGATE_NARROW_SPACE logic

            if not self.is_path_clear_ahead(0.5):  # Increased clearance check
                self.get_logger().info(self.format_position_debug("Obstacle ahead, turning left")) 
                twist.angular.z = self.turn_speed
            else:
                # More conservative approach to narrow space detection
                front_dist = self.get_front_distance()
                
                # FIXED: Only transition to doorway approach when we have sufficient forward clearance
                # AND we're not too close to the doorway opening itself
                if self.current_space_type == SpaceType.DOORWAY and front_dist > 1.2:  # Increased minimum clearance
                    # Additional safety check: ensure we're not already at the doorway entrance
                    right_dist = self.get_side_distance('right')
                    left_dist = self.get_side_distance('left')
                    
                    # Only approach doorway if we have sufficient maneuvering room
                    min_maneuvering_distance = self.robot_diameter * 2.0  # Need space to position properly
                    
                    if front_dist > min_maneuvering_distance:
                        self.get_logger().info(self.format_position_debug(f"*** DOORWAY DETECTED! Transitioning to APPROACH_DOORWAY. Front clearance: {front_dist:.2f}m, Min required: {min_maneuvering_distance:.2f}m ***"))
                        self.change_state(State.APPROACH_DOORWAY)
                        # Let the next loop handle the approach logic
                        return
                    else:
                        # Too close to doorway - continue wall following until we have proper clearance
                        self.get_logger().info(self.format_position_debug(f"DOORWAY detected but too close ({front_dist:.2f}m < {min_maneuvering_distance:.2f}m) - continuing wall following"))
                        linear_vel, angular_vel = self.get_wall_following_command()
                else:
                    # Stay in wall following mode if not clearly safe
                    linear_vel, angular_vel = self.get_wall_following_command()
                    if self.current_space_type != SpaceType.OPEN:
                        self.get_logger().info(self.format_position_debug(f"Wall following in {self.current_space_type.name} space: front_dist={front_dist:.2f}m"), throttle_duration_sec=5)
                
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
        
        elif self.state == State.POST_DOORWAY_FORWARD:
            now = self.get_clock().now()
            if self.post_doorway_start_time is None:
                self.post_doorway_start_time = now
                self.get_logger().info(f"Post-doorway forward: moving forward for {self.post_doorway_forward_time}s")
            
            if (now - self.post_doorway_start_time).nanoseconds / 1e9 > self.post_doorway_forward_time:
                self.get_logger().info("Post-doorway forward complete, returning to wall following")
                self.change_state(State.FOLLOW_WALL)
                linear_vel, angular_vel = self.get_wall_following_command()
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0  # Move straight ahead
        
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
        self.get_logger().info(self.format_position_debug(f"State: {self.state.name}, Space: {space_name}, Cmd: ({twist.linear.x:.2f}, {twist.angular.z:.2f})"), throttle_duration_sec=2)
    
    def approach_doorway(self):
        """
        Approach a detected doorway with comprehensive validation and positioning.
        
        Coordinate frame: base_link (robot-centric)
        - port = +Y direction (left side from rider perspective)
        - starboard = -Y direction (right side from rider perspective)
        - forward = +X direction
        
        Robot must:
        1. Clearly identify which side has the doorway opening
        2. Verify the opening is wide enough for the robot to fit through
        3. Move forward until properly positioned for doorway entry
        4. Center itself perfectly in the hallway
        5. Position itself optimally for a 90-degree turn
        6. Check ahead for obstacles (doors, furniture) before committing
        7. Plan ahead at least one robot diameter
        Returns (linear_vel, angular_vel) tuple.
        """
        # Get comprehensive distance measurements
        port_dist = self.get_side_distance('left')    # +Y direction
        starboard_dist = self.get_side_distance('right')  # -Y direction
        forward_dist = self.get_front_distance()      # +X direction
        
        # Get parameters
        approach_distance = self.get_parameter('doorway_approach_distance').value
        centering_tolerance = self.get_parameter('doorway_centering_tolerance').value
        
        # Comprehensive debug logging with clear coordinate frame references
        self.get_logger().info(self.format_position_debug(f"APPROACH_DOORWAY: Port(+Y)={port_dist:.2f}m, Starboard(-Y)={starboard_dist:.2f}m, Forward(+X)={forward_dist:.2f}m"))
        
        # STEP 1: Clearly identify which side has the doorway opening
        doorway_on_starboard = False  # -Y direction (right from rider perspective)
        doorway_on_port = False       # +Y direction (left from rider perspective)
        
        # CORRECTED: Proper doorway side detection using coordinate frame references
        # Look for actual doorway structure, not just large distances
        
        # Check starboard (-Y) side for doorway - look for opening with far wall
        if starboard_dist > 5.0 or starboard_dist == float('inf'):
            # Look into the starboard (-Y) opening for doorway structure
            starboard_30 = self.get_front_distance(-math.pi/6)   # 30° toward -Y (starboard)
            starboard_45 = self.get_front_distance(-math.pi/4)   # 45° toward -Y (starboard)
            starboard_60 = self.get_front_distance(-math.pi/3)   # 60° toward -Y (starboard)
            
            # Check if we can see doorway structure (far wall at reasonable distance)
            if 0.8 <= starboard_30 <= 2.0 or 0.8 <= starboard_45 <= 2.0 or 0.8 <= starboard_60 <= 2.0:
                doorway_on_starboard = True
                self.get_logger().info(self.format_position_debug(f"*** DOORWAY CONFIRMED ON STARBOARD(-Y): Structure detected (30°={starboard_30:.2f}m, 45°={starboard_45:.2f}m, 60°={starboard_60:.2f}m) ***"))
            else:
                self.get_logger().info(self.format_position_debug(f"Starboard(-Y) side open but no doorway structure - just open space (30°={starboard_30:.2f}m, 45°={starboard_45:.2f}m, 60°={starboard_60:.2f}m)"))
        
        # Check port (+Y) side for doorway
        if not doorway_on_starboard and (port_dist > 5.0 or port_dist == float('inf')):
            # Look into the port (+Y) opening for doorway structure
            port_30 = self.get_front_distance(math.pi/6)    # 30° toward +Y (port)
            port_45 = self.get_front_distance(math.pi/4)    # 45° toward +Y (port)
            port_60 = self.get_front_distance(math.pi/3)    # 60° toward +Y (port)
            
            # Check if we can see doorway structure
            if 0.8 <= port_30 <= 2.0 or 0.8 <= port_45 <= 2.0 or 0.8 <= port_60 <= 2.0:
                doorway_on_port = True
                self.get_logger().info(self.format_position_debug(f"*** DOORWAY CONFIRMED ON PORT(+Y): Structure detected (30°={port_30:.2f}m, 45°={port_45:.2f}m, 60°={port_60:.2f}m) ***"))
            else:
                self.get_logger().info(self.format_position_debug(f"Port(+Y) side open but no doorway structure - just open space (30°={port_30:.2f}m, 45°={port_45:.2f}m, 60°={port_60:.2f}m)"))
        
        # If no clear doorway structure detected, don't try to enter
        if not doorway_on_starboard and not doorway_on_port:
            self.get_logger().warn(self.format_position_debug(f"NO DOORWAY STRUCTURE DETECTED - returning to wall following (Port(+Y)={port_dist:.2f}m, Starboard(-Y)={starboard_dist:.2f}m)"))
            self.change_state(State.FOLLOW_WALL)
            return self.get_wall_following_command()
        
        # STEP 2: CRITICAL FIX - Ensure we're far enough from the doorway to safely approach
        # Don't immediately turn into the doorway - approach it properly first
        min_approach_distance = self.robot_diameter * 2.5  # Need room to maneuver
        
        if forward_dist < min_approach_distance:
            self.get_logger().info(self.format_position_debug(f"TOO CLOSE to doorway opening ({forward_dist:.2f}m < {min_approach_distance:.2f}m) - moving forward first"))
            # Move forward slowly to get proper positioning
            return 0.05, 0.0  # Slow forward motion only
        
        if doorway_on_starboard:
            # STEP 3: Analyze the starboard (-Y) doorway opening in detail
            self.get_logger().info("=== ANALYZING STARBOARD(-Y) DOORWAY ===")
            
            # Check multiple angles into the doorway to understand the opening
            doorway_30 = self.get_front_distance(-math.pi/6)   # 30° toward -Y (starboard)
            doorway_45 = self.get_front_distance(-math.pi/4)   # 45° toward -Y (starboard)
            doorway_60 = self.get_front_distance(-math.pi/3)   # 60° toward -Y (starboard)
            doorway_90 = self.get_side_distance('right')       # 90° toward -Y (into doorway)
            
            self.get_logger().info(f"Starboard(-Y) doorway analysis: 30°={doorway_30:.2f}m, 45°={doorway_45:.2f}m, 60°={doorway_60:.2f}m, 90°={doorway_90:.2f}m")
            
            # STEP 4: Verify opening is wide enough for robot
            # We need to look ahead into the doorway to see the far wall
            # This tells us the actual doorway width
            min_doorway_depth = min(doorway_30, doorway_45, doorway_60, doorway_90)
            
            # Check if there's a door or obstacle blocking the way
            planning_distance = self.robot_diameter * 1.5  # Plan ahead 1.5 robot diameters
            if min_doorway_depth < planning_distance:
                self.get_logger().warn(f"STARBOARD(-Y) DOORWAY BLOCKED OR TOO NARROW: min_depth={min_doorway_depth:.2f}m < planning_distance={planning_distance:.2f}m")
                self.get_logger().warn("This could be a closed door or furniture - returning to wall following")
                self.change_state(State.FOLLOW_WALL)
                return self.get_wall_following_command()
            
            # Estimate doorway width by looking at angles
            # A typical doorway should show depth at multiple angles
            if doorway_45 > 1.0 and doorway_60 > 0.8:
                estimated_doorway_width = min(doorway_45 * 0.7, doorway_60 * 1.0)  # Conservative estimate
                self.get_logger().info(f"Estimated doorway width: {estimated_doorway_width:.2f}m")
                
                if estimated_doorway_width < self.robot_diameter + 0.1:  # Need 10cm clearance
                    self.get_logger().warn(f"STARBOARD(-Y) DOORWAY TOO NARROW: estimated_width={estimated_doorway_width:.2f}m < required={self.robot_diameter + 0.1:.2f}m")
                    self.change_state(State.FOLLOW_WALL)
                    return self.get_wall_following_command()
            else:
                self.get_logger().warn(f"Cannot estimate doorway width reliably: 45°={doorway_45:.2f}m, 60°={doorway_60:.2f}m")
                # Continue with caution but reduce confidence
            
            # STEP 5: Check if we're close enough to start positioning
            if forward_dist < approach_distance:
                self.get_logger().info(f"Close enough to doorway ({forward_dist:.2f}m < {approach_distance:.2f}m) - starting positioning sequence")
                
                # STEP 6: Ensure we're centered in the hallway first
                if port_dist != float('inf'):  # We have a port (+Y) wall to reference
                    # Calculate how far we are from the center of the hallway
                    # Ideal position: equal distance from both walls, accounting for robot radius
                    ideal_distance_from_port_wall = self.robot_radius + 0.1  # 10cm safety margin
                    hallway_center_error = port_dist - ideal_distance_from_port_wall
                    
                    self.get_logger().info(f"Hallway centering check: port_dist(+Y)={port_dist:.2f}m, ideal={ideal_distance_from_port_wall:.2f}m, error={hallway_center_error:.2f}m")
                    
                    if abs(hallway_center_error) > centering_tolerance:
                        # Need to center in hallway first
                        self.get_logger().info(f"CENTERING IN HALLWAY: error={hallway_center_error:.3f}m (tolerance={centering_tolerance:.3f}m)")
                        linear_vel = 0.015  # Very slow forward motion while centering
                        if hallway_center_error > 0:
                            angular_vel = -0.08  # Turn toward -Y (starboard, away from port wall)
                            self.get_logger().info("Adjusting toward STARBOARD(-Y) (away from port wall)")
                        else:
                            angular_vel = 0.08   # Turn toward +Y (port, toward port wall)
                            self.get_logger().info("Adjusting toward PORT(+Y) (toward port wall)")
                        return linear_vel, angular_vel
                
                # STEP 7: Position optimally for the 90-degree turn toward -Y (starboard)
                # We want to be positioned so we can make a clean turn without hitting the doorway frame
                optimal_turn_distance = self.robot_radius + 0.2  # 20cm clearance from doorway edge
                
                self.get_logger().info(f"Turn positioning check: doorway_30={doorway_30:.2f}m, optimal={optimal_turn_distance:.2f}m")
                
                if doorway_30 > optimal_turn_distance and doorway_45 > optimal_turn_distance * 0.8:
                    self.get_logger().info("*** PERFECTLY POSITIONED FOR 90° STARBOARD(-Y) TURN! ***")
                    self.get_logger().info("All safety checks passed - starting doorway entry sequence")
                    # We're positioned correctly - start the 90-degree turn toward -Y (starboard)
                    self.change_state(State.ENTER_DOORWAY)
                    return 0.0, -0.2  # Start pure rotation toward -Y (starboard)
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
                self.get_logger().info(f"APPROACHING STARBOARD(-Y) DOORWAY: {forward_dist:.2f}m > {approach_distance:.2f}m")
                return 0.04, 0.0  # Slow, steady approach
                
        elif doorway_on_port:
            # STEP 3: Analyze the port (+Y) doorway opening in detail
            self.get_logger().info("=== ANALYZING PORT(+Y) DOORWAY ===")
            
            # Check multiple angles into the doorway to understand the opening
            doorway_30 = self.get_front_distance(math.pi/6)    # 30° toward +Y (port)
            doorway_45 = self.get_front_distance(math.pi/4)    # 45° toward +Y (port)
            doorway_60 = self.get_front_distance(math.pi/3)    # 60° toward +Y (port)
            doorway_90 = self.get_side_distance('left')        # 90° toward +Y (into doorway)
            
            self.get_logger().info(f"Port(+Y) doorway analysis: 30°={doorway_30:.2f}m, 45°={doorway_45:.2f}m, 60°={doorway_60:.2f}m, 90°={doorway_90:.2f}m")
            
            # STEP 4: Verify opening is wide enough and not blocked
            min_doorway_depth = min(doorway_30, doorway_45, doorway_60, doorway_90)
            planning_distance = self.robot_diameter * 1.5
            
            if min_doorway_depth < planning_distance:
                self.get_logger().warn(f"PORT(+Y) DOORWAY BLOCKED OR TOO NARROW: min_depth={min_doorway_depth:.2f}m < planning_distance={planning_distance:.2f}m")
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
            
            # STEP 5: Position for port (+Y) doorway
            if forward_dist < approach_distance:
                self.get_logger().info(f"Close enough to port(+Y) doorway - starting positioning")
                
                # STEP 6: Center in hallway using starboard (-Y) wall as reference
                if starboard_dist != float('inf'):
                    ideal_distance_from_starboard_wall = self.robot_radius + 0.1
                    hallway_center_error = starboard_dist - ideal_distance_from_starboard_wall
                    
                    self.get_logger().info(f"Hallway centering (port doorway): starboard_dist(-Y)={starboard_dist:.2f}m, ideal={ideal_distance_from_starboard_wall:.2f}m, error={hallway_center_error:.2f}m")
                    
                    if abs(hallway_center_error) > centering_tolerance:
                        self.get_logger().info(f"CENTERING IN HALLWAY for PORT(+Y) turn: error={hallway_center_error:.3f}m")
                        linear_vel = 0.015
                        if hallway_center_error > 0:
                            angular_vel = 0.08   # Turn toward +Y (port, away from starboard wall)
                            self.get_logger().info("Adjusting toward PORT(+Y) (away from starboard wall)")
                        else:
                            angular_vel = -0.08  # Turn toward -Y (starboard, toward starboard wall)
                            self.get_logger().info("Adjusting toward STARBOARD(-Y) (toward starboard wall)")
                        return linear_vel, angular_vel
                
                # STEP 7: Position for port (+Y) turn
                optimal_turn_distance = self.robot_radius + 0.2
                
                if doorway_30 > optimal_turn_distance and doorway_45 > optimal_turn_distance * 0.8:
                    self.get_logger().info("*** PERFECTLY POSITIONED FOR 90° PORT(+Y) TURN! ***")
                    self.change_state(State.ENTER_DOORWAY)
                    return 0.0, 0.2  # Pure rotation toward +Y (port)
                else:
                    if doorway_30 < optimal_turn_distance:
                        self.get_logger().info(f"Too close to port(+Y) doorway edge - backing up slightly")
                        return -0.01, 0.0
                    else:
                        self.get_logger().info(f"Adjusting forward position for port(+Y) turn")
                        return 0.02, 0.0
            else:
                self.get_logger().info(f"APPROACHING PORT(+Y) DOORWAY: {forward_dist:.2f}m > {approach_distance:.2f}m")
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
