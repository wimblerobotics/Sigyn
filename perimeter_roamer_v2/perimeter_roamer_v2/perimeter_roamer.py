#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
import math
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from geometry_msgs.msg import Twist, Point, PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
from enum import Enum
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import time

class State(Enum):
    """Robot states for navigation"""
    FIND_WALL = 0
    FOLLOW_WALL = 1
    ROOM_PATROL = 2
    HALLWAY_NAVIGATION = 3
    DOORWAY_APPROACH = 4
    DOORWAY_CENTERING = 5
    DOORWAY_ENTRY = 6
    POST_DOORWAY_FORWARD = 7
    RECOVERY_BACKUP = 8
    RECOVERY_TURN = 9
    IDLE = 10

class SpaceType(Enum):
    """Types of spaces the robot can be in"""
    UNKNOWN = 0
    ROOM = 1
    HALLWAY = 2
    DOORWAY = 3
    VERY_NARROW = 4

class PerimeterRoamer(Node):
    """Advanced perimeter roaming robot controller for house patrolling"""
    
    def __init__(self):
        super().__init__('perimeter_roamer_v2')
        
        self.get_logger().info("Perimeter Roamer V2 Starting Up...")
        
        # --- Core Parameters ---
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.3)
        self.declare_parameter('narrow_space_speed', 0.08)
        self.declare_parameter('wall_distance_setpoint', 1.0)  # 1 meter from wall for room patrol
        self.declare_parameter('hallway_distance_setpoint', 0.5)  # 0.5 meter from wall for hallway
        self.declare_parameter('wall_following_gain', 0.5)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_angular_speed', 0.8)
        
        # Robot dimensions (from your config)
        self.declare_parameter('robot_radius', 0.44)  # 0.44m radius as specified
        self.declare_parameter('robot_diameter', 0.88)
        
        # Space classification thresholds
        self.declare_parameter('doorway_width_threshold', 0.8)
        self.declare_parameter('narrow_hallway_threshold', 1.2)
        self.declare_parameter('very_narrow_threshold', 0.6)
        self.declare_parameter('room_detection_threshold', 2.0)  # Minimum space to be considered a room
        
        # Safety parameters
        self.declare_parameter('min_obstacle_distance', 0.15)
        self.declare_parameter('comfortable_distance', 0.25)
        
        # Control parameters
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        
        # Frame handling parameters
        self.declare_parameter('transform_timeout', 0.1)
        
        # Topic names
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        
        # Recovery parameters
        self.declare_parameter('recovery_backup_speed', -0.08)
        self.declare_parameter('recovery_backup_time', 1.5)
        self.declare_parameter('recovery_turn_time', 2.0)
        self.declare_parameter('stuck_timeout', 8.0)
        self.declare_parameter('stuck_distance_threshold', 0.03)
        
        # Doorway navigation parameters
        self.declare_parameter('doorway_approach_distance', 0.6)
        self.declare_parameter('doorway_centering_tolerance', 0.05)
        self.declare_parameter('doorway_alignment_angle_tolerance', 0.1)
        self.declare_parameter('doorway_detection_threshold', 1.5)
        self.declare_parameter('wall_disappearance_threshold', 2.0)
        self.declare_parameter('post_doorway_forward_time', 3.0)
        self.declare_parameter('turn_duration', 2.5)
        
        # Room patrol parameters
        self.declare_parameter('room_patrol_speed', 0.12)
        self.declare_parameter('room_patrol_distance', 1.0)
        self.declare_parameter('room_patrol_turn_angle', 0.3)  # radians
        
        # Hallway navigation parameters
        self.declare_parameter('hallway_speed', 0.10)
        self.declare_parameter('hallway_centering_gain', 0.8)
        
        # Get parameters
        self._get_parameters()
        
        # Initialize state variables
        self.current_state = State.IDLE
        self.current_space_type = SpaceType.UNKNOWN
        self.last_position = None
        self.last_time = None
        self.stuck_start_time = None
        self.doorway_state = {}
        self.room_patrol_state = {}
        
        # Initialize sensor data variables
        self.scan_data = None
        self.odom_data = None
        
        # Initialize subscribers and publishers
        self._setup_communication()
        
        # Initialize TF
        self._setup_tf()
        
        # Initialize system ready flag
        self.system_ready = False
        
        # Start initialization timer
        self.init_timer = self.create_timer(0.5, self.check_system_ready)
        
        # Start control loop (but it won't run until system is ready)
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        # Debug timer will be started when system is ready
        self.debug_timer = None
        
        self.get_logger().info("Perimeter Roamer V2 initialized successfully")
        self.get_logger().info(f"Control frequency: {self.control_frequency} Hz")
        # Note: Topic names may be remapped by launch file
        self.get_logger().info(f"Publishing to cmd_vel topic: cmd_vel (may be remapped)")
        self.get_logger().info(f"Subscribing to scan topic: {self.scan_topic}")
        self.get_logger().info(f"Subscribing to odom topic: {self.odom_topic}")
        self.get_logger().info("Waiting for system to be ready (transforms, sensors)...")
    
    def check_system_ready(self):
        """Check if all required system components are ready"""
        if self.system_ready:
            return
        
        # Check if we have sensor data
        sensors_ready = (self.scan_data is not None and self.odom_data is not None)
        
        # Check if transforms are available
        transforms_ready = False
        try:
            # Check odom to base_link transform
            self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),  # Use zero time to get latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Check lidar frame to base_link transform (if different)
            if self.scan_data and self.scan_data.header.frame_id != self.base_frame:
                # Use the actual scan frame and check transform from base_link to scan frame
                scan_frame_id = self.scan_data.header.frame_id
                self.tf_buffer.lookup_transform(
                    scan_frame_id,  # target frame (actual laser frame from scan)
                    self.base_frame,  # source frame (base_link)
                    rclpy.time.Time(),  # Use zero time to get latest available transform
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            
            transforms_ready = True
            self.get_logger().info(f"All required transforms are available")
        except Exception as e:
            transforms_ready = False
            self.get_logger().debug(f"Transform check failed: {e}")
        
        # Log status
        if not sensors_ready:
            self.get_logger().info("Waiting for sensor data...")
        elif not transforms_ready:
            self.get_logger().info("Waiting for transforms...")
        else:
            self.system_ready = True
            self.get_logger().info("System ready! Starting navigation...")
            # Stop the init timer since we're ready
            self.init_timer.cancel()
            # Start debug timer now that we're ready
            self.debug_timer = self.create_timer(2.0, self.debug_status)
    
    def _get_parameters(self):
        """Get all parameters from the parameter server"""
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.narrow_space_speed = self.get_parameter('narrow_space_speed').value
        self.wall_distance_setpoint = self.get_parameter('wall_distance_setpoint').value
        self.hallway_distance_setpoint = self.get_parameter('hallway_distance_setpoint').value
        self.wall_following_gain = self.get_parameter('wall_following_gain').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.robot_diameter = self.get_parameter('robot_diameter').value
        
        self.doorway_width_threshold = self.get_parameter('doorway_width_threshold').value
        self.narrow_hallway_threshold = self.get_parameter('narrow_hallway_threshold').value
        self.very_narrow_threshold = self.get_parameter('very_narrow_threshold').value
        self.room_detection_threshold = self.get_parameter('room_detection_threshold').value
        
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.comfortable_distance = self.get_parameter('comfortable_distance').value
        
        self.control_frequency = self.get_parameter('control_frequency').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Get use_sim_time parameter if available, otherwise default to False
        try:
            self.use_sim_time = self.get_parameter('use_sim_time').value
        except:
            self.use_sim_time = False
            self.get_logger().info("use_sim_time parameter not found, defaulting to False")
        
        self.transform_timeout = self.get_parameter('transform_timeout').value
        
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        
        self.recovery_backup_speed = self.get_parameter('recovery_backup_speed').value
        self.recovery_backup_time = self.get_parameter('recovery_backup_time').value
        self.recovery_turn_time = self.get_parameter('recovery_turn_time').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        
        self.doorway_approach_distance = self.get_parameter('doorway_approach_distance').value
        self.doorway_centering_tolerance = self.get_parameter('doorway_centering_tolerance').value
        self.doorway_alignment_angle_tolerance = self.get_parameter('doorway_alignment_angle_tolerance').value
        self.doorway_detection_threshold = self.get_parameter('doorway_detection_threshold').value
        self.wall_disappearance_threshold = self.get_parameter('wall_disappearance_threshold').value
        self.post_doorway_forward_time = self.get_parameter('post_doorway_forward_time').value
        self.turn_duration = self.get_parameter('turn_duration').value
        
        self.room_patrol_speed = self.get_parameter('room_patrol_speed').value
        self.room_patrol_distance = self.get_parameter('room_patrol_distance').value
        self.room_patrol_turn_angle = self.get_parameter('room_patrol_turn_angle').value
        
        self.hallway_speed = self.get_parameter('hallway_speed').value
        self.hallway_centering_gain = self.get_parameter('hallway_centering_gain').value
    
    def _setup_communication(self):
        """Setup subscribers and publishers"""
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # Publishers
        # Use remapped topic name directly (not the parameter value)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',  # This will be remapped by launch file
            10
        )
        
        self.space_type_pub = self.create_publisher(
            String,
            'space_type',
            10
        )
        
        self.state_pub = self.create_publisher(
            String,
            'roamer_state',
            10
        )
        
        # Visualization publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'roamer_markers',
            10
        )
    
    def _setup_tf(self):
        """Setup TF buffer and listener with proper clock handling"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Get the clock type to determine if we're in simulation or real time
        self.clock_type = self.get_clock().clock_type
        self.get_logger().info(f"Clock type: {self.clock_type}")
        
        # Initialize transform cache
        self.transform_cache = {}
        self.last_transform_time = None
    
    def wait_for_transforms(self):
        """Wait for transforms to be available with proper clock handling"""
        try:
            # Check for odom -> base_link transform
            odom_transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),  # Use latest available transform
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            
            # Check for map -> base_link transform for position logging
            map_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),  # Use latest available transform
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            
            # Cache the transforms
            self.transform_cache['odom_to_base'] = odom_transform
            self.transform_cache['map_to_base'] = map_transform
            self.last_transform_time = self.get_clock().now()
            
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(f"Transform not available yet: {e}")
            return False
    
    def get_transform_to_base_link(self, from_frame, to_frame='base_link', timeout_seconds=None):
        """Get transform to base_link frame with proper clock handling"""
        if timeout_seconds is None:
            timeout_seconds = self.transform_timeout
            
        try:
            # Use zero time to get the latest available transform
            # This is more reliable in simulation than using current time
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),  # Use latest available transform
                timeout=rclpy.duration.Duration(seconds=timeout_seconds)
            )
            
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get transform from {from_frame} to {to_frame}: {e}")
            return None
    
    def transform_point_to_base_link(self, point, from_frame):
        """Transform a point to base_link frame"""
        try:
            transform = self.get_transform_to_base_link(from_frame)
            if transform is None:
                return None
            
            # Create a PointStamped message with zero time (latest available)
            point_stamped = PointStamped()
            point_stamped.header.frame_id = from_frame
            point_stamped.header.stamp = rclpy.time.Time().to_msg()  # Use zero time
            point_stamped.point = point
            
            # Transform the point
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return transformed_point.point
            
        except Exception as e:
            self.get_logger().warn(f"Failed to transform point: {e}")
            return None
    
    def get_position_in_map_frame(self):
        """Get robot position in map frame for logging purposes"""
        try:
            if self.odom_data is None:
                return None
                
            # Get the transform from odom to map
            transform = self.get_transform_to_base_link('odom', 'map')
            if transform is None:
                return None
                
            # Create a PointStamped with the robot's position in odom frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'odom'
            point_stamped.header.stamp = rclpy.time.Time().to_msg()
            point_stamped.point.x = self.odom_data.pose.pose.position.x
            point_stamped.point.y = self.odom_data.pose.pose.position.y
            point_stamped.point.z = self.odom_data.pose.pose.position.z
            
            # Transform to map frame
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            return (transformed_point.point.x, transformed_point.point.y)
            
        except Exception as e:
            self.get_logger().debug(f"Failed to get position in map frame: {e}")
            return None
    
    def get_position_debug_string(self):
        """Get a formatted string with both odom and map coordinates for debug logging"""
        if self.odom_data is None:
            return "odom=None, map=None"
        
        # Odom coordinates
        odom_x = self.odom_data.pose.pose.position.x
        odom_y = self.odom_data.pose.pose.position.y
        odom_str = f"odom=({odom_x:.2f}, {odom_y:.2f})"
        
        # Map coordinates
        map_position = self.get_position_in_map_frame()
        if map_position is not None:
            map_str = f"map=({map_position[0]:.2f}, {map_position[1]:.2f})"
        else:
            map_str = "map=None"
        
        return f"{odom_str}, {map_str}"
    
    def scan_callback(self, msg):
        """Process incoming laser scan data with proper frame handling"""
        # Store the original scan data
        self.scan_data = msg
        
        # Log frame information for debugging
        if hasattr(self, 'last_scan_frame') and self.last_scan_frame != msg.header.frame_id:
            self.get_logger().info(f"LaserScan frame changed from {self.last_scan_frame} to {msg.header.frame_id}")
        
        self.last_scan_frame = msg.header.frame_id
        
        # Debug: Log first scan received
        if not hasattr(self, 'scan_received'):
            self.get_logger().info(f"First LIDAR scan received: {len(msg.ranges)} points, frame: {msg.header.frame_id}")
            self.scan_received = True
    
    def odom_callback(self, msg):
        """Process incoming odometry data with proper frame handling"""
        # Store the original odometry data
        self.odom_data = msg
        
        # Log frame information for debugging
        if hasattr(self, 'last_odom_frame') and self.last_odom_frame != msg.header.frame_id:
            self.get_logger().info(f"Odometry frame changed from {self.last_odom_frame} to {msg.header.frame_id}")
        
        self.last_odom_frame = msg.header.frame_id
        
        # Debug: Log first odometry received
        if not hasattr(self, 'odom_received'):
            self.get_logger().info(f"First odometry received: frame: {msg.header.frame_id}")
            self.odom_received = True
        
        # Update position with proper frame handling
        self._update_position()
    
    def _update_position(self):
        """Update robot position and check for stuck condition with proper frame handling"""
        if self.odom_data is None:
            return
        
        # Get the current position from odometry
        # Note: We assume odometry is already in the correct frame (usually odom)
        # If we need to transform it, we would do so here
        current_position = (
            self.odom_data.pose.pose.position.x,
            self.odom_data.pose.pose.position.y
        )
        
        # Get current time with proper clock handling
        current_time = self.get_clock().now()
        
        # Log position changes for debugging (at reduced frequency)
        if hasattr(self, 'last_log_time'):
            time_since_last_log = (current_time - self.last_log_time).nanoseconds / 1e9
            if time_since_last_log > 10.0:  # Log every 10 seconds
                self.get_logger().debug(f"Robot position: ({current_position[0]:.2f}, {current_position[1]:.2f})")
                self.last_log_time = current_time
        else:
            self.last_log_time = current_time
        
        # Check for stuck condition
        if self.last_position is not None and self.last_time is not None:
            distance = math.sqrt(
                (current_position[0] - self.last_position[0])**2 +
                (current_position[1] - self.last_position[1])**2
            )
            
            time_diff = (current_time - self.last_time).nanoseconds / 1e9
            
            if distance < self.stuck_distance_threshold and time_diff > 1.0:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time
                elif (current_time - self.stuck_start_time).nanoseconds / 1e9 > self.stuck_timeout:
                    self.get_logger().warn("Robot appears to be stuck!")
                    self.change_state(State.RECOVERY_BACKUP)
            else:
                self.stuck_start_time = None
        
        self.last_position = current_position
        self.last_time = current_time
    
    def change_state(self, new_state):
        """Change robot state and log the transition"""
        if new_state != self.current_state:
            self.get_logger().info(f"State change: {self.current_state.name} -> {new_state.name}")
            self.current_state = new_state
            
            # Publish state for monitoring
            state_msg = String()
            state_msg.data = new_state.name
            self.state_pub.publish(state_msg)
    
    def classify_space(self):
        """Classify the current space type based on LIDAR data"""
        if self.scan_data is None:
            return SpaceType.UNKNOWN
        
        # Get distances in different directions
        front_dist = self.get_front_distance()
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        
        # Calculate space width (minimum of left and right distances)
        space_width = min(left_dist, right_dist) * 2  # Multiply by 2 for full width
        
        # Doorway detection parameters
        doorway_min_opening = 0.6   # Minimum opening width for a doorway (0.65m doors exist)
        doorway_max_opening = 1.0   # Maximum opening width for a doorway (0.73m is typical)
        doorway_depth_threshold = 1.5  # How deep the opening should be
        
        # Check for doorway opening on each side
        # A doorway is detected when one side shows a narrow opening within doorway range
        # The key insight: we're looking for a side distance that matches doorway dimensions
        
        is_doorway_left = (left_dist >= doorway_min_opening and 
                          left_dist <= doorway_depth_threshold)
        is_doorway_right = (right_dist >= doorway_min_opening and 
                           right_dist <= doorway_depth_threshold)
        
        # Log doorway detection details for debugging
        position_str = self.get_position_debug_string()
        self.get_logger().info(f"Space classification at {position_str}: "
                              f"left={left_dist:.2f}m, right={right_dist:.2f}m, front={front_dist:.2f}m, "
                              f"space_width={space_width:.2f}m, doorway_left={is_doorway_left}, doorway_right={is_doorway_right}")
        
        # Classify space type
        if is_doorway_left or is_doorway_right:
            self.get_logger().info(f"DOORWAY detected! Opening on {'left' if is_doorway_left else 'right'} side")
            return SpaceType.DOORWAY
        elif space_width < self.very_narrow_threshold:
            return SpaceType.VERY_NARROW
        elif space_width < self.doorway_width_threshold:
            return SpaceType.DOORWAY
        elif space_width < self.narrow_hallway_threshold:
            return SpaceType.HALLWAY
        elif front_dist > self.room_detection_threshold:
            return SpaceType.ROOM
        else:
            return SpaceType.HALLWAY  # Default to hallway if unclear
    
    def get_distance_in_direction(self, direction_x, direction_y, max_range=10.0):
        """Get distance in a specific direction with proper frame handling"""
        if self.scan_data is None:
            return max_range
        
        # LaserScan data is typically in the laser frame, not base_link
        # This is normal and expected behavior
        
        # Convert direction to angle
        angle = math.atan2(direction_y, direction_x)
        
        # Convert to scan index
        scan_angle = self.scan_data.angle_min + angle
        scan_angle = self.normalize_angle(scan_angle)
        
        # Find closest scan index
        angle_diff = abs(scan_angle - self.scan_data.angle_min)
        index = int(angle_diff / self.scan_data.angle_increment)
        
        if 0 <= index < len(self.scan_data.ranges):
            distance = self.scan_data.ranges[index]
            if distance < max_range and distance > self.scan_data.range_min:
                return distance
        
        return max_range
    
    def get_side_distance(self, side='right'):
        """Get distance to the side (left or right) with proper frame handling"""
        if self.scan_data is None:
            return 10.0
        
        # LaserScan data is typically in the laser frame, not base_link
        # This is normal and expected behavior
        
        # Find the scan index for 90 degrees (right) or -90 degrees (left)
        if side == 'right':
            target_angle = math.pi / 2
        else:
            target_angle = -math.pi / 2
        
        # Convert to scan index
        scan_angle = self.scan_data.angle_min + target_angle
        scan_angle = self.normalize_angle(scan_angle)
        
        angle_diff = abs(scan_angle - self.scan_data.angle_min)
        index = int(angle_diff / self.scan_data.angle_increment)
        
        if 0 <= index < len(self.scan_data.ranges):
            distance = self.scan_data.ranges[index]
            if distance < 10.0 and distance > self.scan_data.range_min:
                return distance
        
        return 10.0
    
    def get_front_distance(self, angle_offset=0.0):
        """Get distance in front of the robot"""
        return self.get_distance_in_direction(1.0, angle_offset)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def check_immediate_collision_risk(self):
        """Check if robot is at immediate risk of collision"""
        if self.scan_data is None:
            return False
        
        # Check front, left, and right for immediate obstacles
        front_dist = self.get_front_distance()
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        
        return (front_dist < self.min_obstacle_distance or
                left_dist < self.min_obstacle_distance or
                right_dist < self.min_obstacle_distance)
    
    def is_path_clear_ahead(self, check_distance=0.3):
        """Check if path ahead is clear"""
        front_dist = self.get_front_distance()
        return front_dist > check_distance
    
    def get_wall_following_command(self, target_distance):
        """Generate wall following command"""
        if self.scan_data is None:
            return Twist()
        
        # Get distance to wall on the right
        wall_distance = self.get_side_distance('right')
        
        # Calculate error
        error = wall_distance - target_distance
        
        # Generate control command
        cmd = Twist()
        cmd.linear.x = self.forward_speed
        
        # Angular control based on wall distance error
        angular_velocity = -self.wall_following_gain * error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        return cmd
    
    def get_hallway_centering_command(self):
        """Generate hallway centering command"""
        if self.scan_data is None:
            return Twist()
        
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        
        # Calculate center error (positive = closer to right wall, need to turn left)
        center_error = right_dist - left_dist
        
        cmd = Twist()
        cmd.linear.x = self.hallway_speed
        
        # Angular control to center in hallway  
        angular_velocity = self.hallway_centering_gain * center_error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        # Log hallway centering debug info occasionally
        if hasattr(self, 'last_hallway_debug_time'):
            if (self.get_clock().now() - self.last_hallway_debug_time).nanoseconds > 2e9:  # Every 2 seconds
                self.get_logger().info(f"Hallway centering: left={left_dist:.2f}m, right={right_dist:.2f}m, error={center_error:.2f}, angular={angular_velocity:.2f}")
                self.last_hallway_debug_time = self.get_clock().now()
        else:
            self.last_hallway_debug_time = self.get_clock().now()
        
        return cmd
    
    def get_room_patrol_command(self):
        """Generate room patrol command"""
        if self.scan_data is None:
            return Twist()
        
        # Get distance to nearest wall
        min_distance = min(
            self.get_front_distance(),
            self.get_side_distance('left'),
            self.get_side_distance('right')
        )
        
        # If too close to wall, turn away
        if min_distance < self.room_patrol_distance:
            cmd = Twist()
            cmd.angular.z = self.room_patrol_turn_angle
            return cmd
        
        # Otherwise, move forward while maintaining distance from walls
        return self.get_wall_following_command(self.room_patrol_distance)
    
    def approach_doorway(self):
        """Approach and center in doorway"""
        if self.scan_data is None:
            return Twist()
        
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        
        # Calculate center error
        center_error = left_dist - right_dist
        
        cmd = Twist()
        cmd.linear.x = self.narrow_space_speed
        
        # Strong centering control for doorway
        angular_velocity = 2.0 * center_error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        # Check if centered enough to proceed
        if abs(center_error) < self.doorway_centering_tolerance:
            self.change_state(State.DOORWAY_ENTRY)
        
        return cmd
    
    def enter_doorway(self):
        """Execute doorway entry"""
        if self.scan_data is None:
            return Twist()
        
        # Move forward slowly through doorway
        cmd = Twist()
        cmd.linear.x = self.narrow_space_speed
        
        # Minimal angular correction to stay centered
        left_dist = self.get_side_distance('left')
        right_dist = self.get_side_distance('right')
        center_error = left_dist - right_dist
        
        angular_velocity = 0.5 * center_error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        # Check if we've cleared the doorway
        front_dist = self.get_front_distance()
        if front_dist > self.doorway_approach_distance:
            self.change_state(State.POST_DOORWAY_FORWARD)
        
        return cmd
    
    def post_doorway_forward(self):
        """Move forward after clearing doorway"""
        cmd = Twist()
        cmd.linear.x = self.forward_speed
        
        # Start timer for this state
        if 'post_doorway_start' not in self.doorway_state:
            self.doorway_state['post_doorway_start'] = self.get_clock().now()
        
        elapsed_time = (self.get_clock().now() - self.doorway_state['post_doorway_start']).nanoseconds / 1e9
        
        if elapsed_time > self.post_doorway_forward_time:
            self.doorway_state.clear()
            self.change_state(State.FIND_WALL)
        
        return cmd
    
    def recovery_backup(self):
        """Recovery behavior - backup"""
        cmd = Twist()
        cmd.linear.x = self.recovery_backup_speed
        
        # Start timer for this state
        if 'recovery_start' not in self.doorway_state:
            self.doorway_state['recovery_start'] = self.get_clock().now()
        
        elapsed_time = (self.get_clock().now() - self.doorway_state['recovery_start']).nanoseconds / 1e9
        
        if elapsed_time > self.recovery_backup_time:
            self.doorway_state.clear()
            self.change_state(State.RECOVERY_TURN)
        
        return cmd
    
    def recovery_turn(self):
        """Recovery behavior - turn"""
        cmd = Twist()
        cmd.angular.z = self.turn_speed
        
        # Start timer for this state
        if 'recovery_start' not in self.doorway_state:
            self.doorway_state['recovery_start'] = self.get_clock().now()
        
        elapsed_time = (self.get_clock().now() - self.doorway_state['recovery_start']).nanoseconds / 1e9
        
        if elapsed_time > self.recovery_turn_time:
            self.doorway_state.clear()
            self.change_state(State.FIND_WALL)
        
        return cmd
    
    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def validate_sensor_data(self):
        """Validate that sensor data is available and in correct frames"""
        if self.scan_data is None:
            self.get_logger().warn("No LIDAR scan data available")
            return False
        
        if self.odom_data is None:
            self.get_logger().warn("No odometry data available")
            return False
        
        # LaserScan data is typically in the laser frame, not base_link
        # This is normal and expected behavior - no need to warn
        
        # Check odometry data frame
        if self.odom_data.header.frame_id != self.odom_frame:
            self.get_logger().warn(f"Odometry frame mismatch: expected {self.odom_frame}, got {self.odom_data.header.frame_id}")
            # In a real implementation, you might want to transform the odometry data here
            # For now, we'll continue but log the warning
        
        return True
    
    def control_loop(self):
        """Main control loop with proper frame handling"""
        # Only run if system is ready
        if not self.system_ready:
            return
        
        # Wait for transforms with proper clock handling
        if not self.wait_for_transforms():
            self.get_logger().debug("Waiting for transforms...")
            return
        
        # Validate sensor data
        if not self.validate_sensor_data():
            self.get_logger().debug("Sensor data validation failed")
            return
        
        # Log that we're in the main control loop (only once)
        if not hasattr(self, 'control_loop_started'):
            self.get_logger().info("Control loop running - processing sensor data")
            self.control_loop_started = True
        
        # Classify current space
        new_space_type = self.classify_space()
        if new_space_type != self.current_space_type:
            self.get_logger().info(f"Space type changed: {self.current_space_type.name} -> {new_space_type.name}")
            self.current_space_type = new_space_type
            
            # Publish space type for monitoring
            space_msg = String()
            space_msg.data = new_space_type.name
            self.space_type_pub.publish(space_msg)
        
        # Check for immediate collision risk
        if self.check_immediate_collision_risk():
            self.get_logger().warn("Collision risk detected!")
            self.change_state(State.RECOVERY_BACKUP)
        
        # Generate control command based on current state
        cmd = Twist()
        
        if self.current_state == State.IDLE:
            self.get_logger().info("Starting robot navigation - transitioning from IDLE to FIND_WALL")
            self.change_state(State.FIND_WALL)
        
        elif self.current_state == State.FIND_WALL:
            # Find a wall to follow
            if self.current_space_type == SpaceType.ROOM:
                self.change_state(State.ROOM_PATROL)
            elif self.current_space_type == SpaceType.HALLWAY:
                self.change_state(State.HALLWAY_NAVIGATION)
            elif self.current_space_type == SpaceType.DOORWAY:
                self.change_state(State.DOORWAY_APPROACH)
            else:
                # Default wall following behavior
                cmd = self.get_wall_following_command(self.wall_distance_setpoint)
        
        elif self.current_state == State.ROOM_PATROL:
            if self.current_space_type == SpaceType.DOORWAY:
                self.change_state(State.DOORWAY_APPROACH)
            else:
                cmd = self.get_room_patrol_command()
        
        elif self.current_state == State.HALLWAY_NAVIGATION:
            if self.current_space_type == SpaceType.DOORWAY:
                self.change_state(State.DOORWAY_APPROACH)
            else:
                cmd = self.get_hallway_centering_command()
        
        elif self.current_state == State.DOORWAY_APPROACH:
            cmd = self.approach_doorway()
        
        elif self.current_state == State.DOORWAY_ENTRY:
            cmd = self.enter_doorway()
        
        elif self.current_state == State.POST_DOORWAY_FORWARD:
            cmd = self.post_doorway_forward()
        
        elif self.current_state == State.RECOVERY_BACKUP:
            cmd = self.recovery_backup()
        
        elif self.current_state == State.RECOVERY_TURN:
            cmd = self.recovery_turn()
        
        # Store last command for debugging
        self.last_cmd_vel = cmd
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log command if it's non-zero
        if abs(cmd.linear.x) > 0.001 or abs(cmd.angular.z) > 0.001:
            self.get_logger().debug(f"Publishing command: linear={cmd.linear.x:.3f}, angular={cmd.angular.z:.3f}")
        
        # Publish visualization markers
        self._publish_markers()
    
    def _publish_markers(self):
        """Publish visualization markers with proper clock handling"""
        marker_array = MarkerArray()
        
        # Create marker for current space type
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "space_type"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.text = self.current_space_type.name
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker_array.markers.append(marker)
        
        # Create marker for clock type (for debugging)
        clock_marker = Marker()
        clock_marker.header.frame_id = self.base_frame
        clock_marker.header.stamp = self.get_clock().now().to_msg()
        clock_marker.ns = "clock_type"
        clock_marker.id = 1
        clock_marker.type = Marker.TEXT_VIEW_FACING
        clock_marker.action = Marker.ADD
        clock_marker.pose.position.x = 0.0
        clock_marker.pose.position.y = 0.0
        clock_marker.pose.position.z = 1.5
        clock_marker.text = f"Clock: {self.clock_type.name}"
        clock_marker.scale.z = 0.15
        clock_marker.color.r = 0.0
        clock_marker.color.g = 1.0
        clock_marker.color.b = 0.0
        clock_marker.color.a = 1.0
        
        marker_array.markers.append(clock_marker)
        
        self.marker_pub.publish(marker_array)
    
    def debug_status(self):
        """Print debug status information"""
        # Only show debug status if system is ready or we're still initializing
        if not self.system_ready:
            return  # Don't show debug status during initialization
        
        status_msg = f"DEBUG STATUS:"
        status_msg += f"\n  State: {self.current_state.name}"
        status_msg += f"\n  Space Type: {self.current_space_type.name}"
        
        # Sensor data status
        if self.scan_data is not None:
            status_msg += f"\n  LIDAR: Available (frame: {self.scan_data.header.frame_id})"
            if len(self.scan_data.ranges) > 0:
                min_range = min(self.scan_data.ranges)
                max_range = max(self.scan_data.ranges)
                status_msg += f" - ranges: {min_range:.2f} to {max_range:.2f}"
        else:
            status_msg += f"\n  LIDAR: NOT AVAILABLE"
        
        if self.odom_data is not None:
            status_msg += f"\n  ODOM: Available (frame: {self.odom_data.header.frame_id})"
            if self.last_position is not None:
                # Show position in odom frame
                status_msg += f" - odom position: ({self.last_position[0]:.2f}, {self.last_position[1]:.2f})"
                
                # Also show position in map frame if available
                map_position = self.get_position_in_map_frame()
                if map_position is not None:
                    status_msg += f" - map position: ({map_position[0]:.2f}, {map_position[1]:.2f})"
        else:
            status_msg += f"\n  ODOM: NOT AVAILABLE"
        
        # Transform status
        odom_tf_available = False
        map_tf_available = False
        
        try:
            self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),  # Use latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            odom_tf_available = True
        except:
            pass
            
        try:
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),  # Use latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            map_tf_available = True
        except:
            pass
        
        if odom_tf_available and map_tf_available:
            status_msg += f"\n  TF: Available ({self.odom_frame} -> {self.base_frame}, {self.map_frame} -> {self.base_frame})"
        elif odom_tf_available:
            status_msg += f"\n  TF: Partial ({self.odom_frame} -> {self.base_frame} only)"
        else:
            status_msg += f"\n  TF: NOT AVAILABLE"
        
        # Command status
        if hasattr(self, 'last_cmd_vel'):
            status_msg += f"\n  Last CMD: linear={self.last_cmd_vel.linear.x:.3f}, angular={self.last_cmd_vel.angular.z:.3f}"
        else:
            status_msg += f"\n  Last CMD: None"
        
        self.get_logger().info(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerimeterRoamer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 