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
        self.declare_parameter('transform_timeout', 10.0)
        
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
        
        # Initialize sensor data variables (all in base_link frame after transformation)
        self.scan_data_raw = None  # Raw scan data in original frame
        self.scan_data = None      # Transformed scan data in base_link frame
        self.odom_data_raw = None  # Raw odom data in original frame  
        self.odom_data = None      # Transformed odom data in base_link frame
        self.lidar_to_base_yaw_offset = 0.0
        
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
        self.get_logger().info(f"Publishing to cmd_vel_nav topic for navigation commands")
        self.get_logger().info(f"Subscribing to scan topic: {self.scan_topic}")
        self.get_logger().info(f"Subscribing to odom topic: {self.odom_topic}")
        self.get_logger().info("Waiting for system to be ready (transforms, sensors)...")
    
    def check_system_ready(self):
        """Check if all required system components are ready"""
        if self.system_ready:
            return
        
        # Check if we have sensor data (both raw and transformed)
        sensors_ready = (self.scan_data_raw is not None and self.scan_data is not None and 
                         self.odom_data_raw is not None and self.odom_data is not None)
        
        # If scan data is in a different frame, check if we have transform capability
        if self.scan_data_raw and self.scan_data_raw.header.frame_id != self.base_frame:
            # Check if we can transform laser frame to base_link
            try:
                # Always use zero time for transform lookups - this gets the latest available transform
                lookup_time = rclpy.time.Time()
                self.get_logger().info(f"LIDAR transform lookup - Using zero time (latest available): {lookup_time.nanoseconds}")
                
                self.tf_buffer.lookup_transform(
                    self.base_frame,  # target frame
                    self.scan_data_raw.header.frame_id,  # source frame (actual laser frame)
                    lookup_time,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.get_logger().info(f"Transform available: {self.scan_data_raw.header.frame_id} -> {self.base_frame}")
            except Exception as e:
                sensors_ready = False
                self.get_logger().info(f"LIDAR transform not ready: {e}")
        
        # If odom data is in a different frame, check if we have transform capability
        if self.odom_data_raw and self.odom_data_raw.header.frame_id != self.base_frame:
            try:
                # Always use zero time for transform lookups - this gets the latest available transform
                lookup_time = rclpy.time.Time()
                self.get_logger().info(f"Odom transform lookup - Using zero time (latest available): {lookup_time.nanoseconds}")
                
                self.tf_buffer.lookup_transform(
                    self.base_frame,  # target frame
                    self.odom_data_raw.header.frame_id,  # source frame (actual odom frame)
                    lookup_time,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.get_logger().info(f"Transform available: {self.odom_data_raw.header.frame_id} -> {self.base_frame}")
            except Exception as e:
                sensors_ready = False
                self.get_logger().info(f"Odom transform not ready: {e}")
        
        # Check if transforms are available (more robust checking)
        transforms_ready = True  # Assume ready unless we find missing transforms
        
        try:
            # Check odom to base_link transform (only if we have odom data)
            if self.odom_data_raw:
                self.tf_buffer.lookup_transform(
                    self.base_frame,  # target frame
                    self.odom_data_raw.header.frame_id,  # source frame (actual odom frame)
                    rclpy.time.Time(),  # Use zero time to get latest available transform
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            
            # Check lidar frame to base_link transform (only if we have scan data)
            if self.scan_data_raw:
                self.tf_buffer.lookup_transform(
                    self.base_frame,  # target frame
                    self.scan_data_raw.header.frame_id,  # source frame (actual laser frame)
                    rclpy.time.Time(),  # Use zero time to get latest available transform
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            
            self.get_logger().info(f"All required transforms are available")
        except Exception as e:
            transforms_ready = False
            self.get_logger().info(f"Transform check failed: {e}")
        
        # Log status
        if not sensors_ready:
            self.get_logger().info("Waiting for sensor data...")
        elif not transforms_ready:
            self.get_logger().info("Waiting for transforms...")
        else:
            self.system_ready = True
            self.system_ready_time = self.get_clock().now()  # Track when system became ready
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
        # Use the parameter value which can be remapped by launch file or command line
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel_nav',  # Publish directly to cmd_vel_nav
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
        # This function should only be called if we have sensor data to know actual frame IDs
        if self.odom_data_raw is None:
            self.get_logger().debug("No odom data available yet for transform checking")
            return False
            
        try:
            # Always use zero time for transform lookups - this gets the latest available transform
            lookup_time_odom = rclpy.time.Time()
            lookup_time_map = rclpy.time.Time()
            self.get_logger().info(f"Using zero time (latest available) for transforms: {lookup_time_odom.nanoseconds}")
            
            # Check for actual odom frame -> base_link transform (use actual frame from message)
            actual_odom_frame = self.odom_data_raw.header.frame_id
            odom_transform = self.tf_buffer.lookup_transform(
                actual_odom_frame,  # Use actual frame from odometry message
                self.base_frame,
                lookup_time_odom,
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            
            # Check for map -> base_link transform for position logging
            map_transform = self.tf_buffer.lookup_transform(
                self.map_frame,  # Map frame is typically fixed as 'map'
                self.base_frame,
                lookup_time_map,
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            
            # Cache the transforms
            self.transform_cache['odom_to_base'] = odom_transform
            self.transform_cache['map_to_base'] = map_transform
            self.last_transform_time = self.get_clock().now()
            
            self.get_logger().info(f"Successfully cached transforms: {actual_odom_frame}->base_link and map->base_link")
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Be more specific about which transform failed
            if 'map' in str(e).lower():
                self.get_logger().info(f"Map transform not available yet: {e}")
            elif actual_odom_frame in str(e):
                self.get_logger().info(f"Odom transform ({actual_odom_frame}) not available yet: {e}")
            else:
                self.get_logger().info(f"Transform not available yet: {e}")
            return False
    
    def get_transform_to_base_link(self, from_frame, to_frame='base_link', timeout_seconds=None):
        """Get transform to base_link frame with proper clock handling"""
        if timeout_seconds is None:
            timeout_seconds = self.transform_timeout
            
        try:
            # Always use zero time for transform lookups - this gets the latest available transform
            # and works reliably in both simulation and real robot scenarios
            lookup_time = rclpy.time.Time()
            # self.get_logger().info(f"get_transform_to_base_link - Using zero time (latest available): {lookup_time.nanoseconds}")
            
            # Use zero time for transform lookup
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                lookup_time,
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
            if self.odom_data is None or self.odom_data_raw is None:
                return None
                
            # Get the actual frame from the odometry message
            actual_odom_frame = self.odom_data_raw.header.frame_id
                
            # Get the transform from actual odom frame to map
            transform = self.get_transform_to_base_link(actual_odom_frame, self.map_frame)
            if transform is None:
                return None
                
            # Create a PointStamped with the robot's position in actual odom frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = actual_odom_frame  # Use actual frame from message
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
    
    def get_distance_at_angle(self, angle_deg):
        if self.scan_data is None: return float('inf')
        
        # We need to transform the desired angle from the robot's frame to the LIDAR's frame
        angle_rad_base = math.radians(angle_deg)
        
        try:
            # Create a point 1 meter away in the desired direction in the base_link frame
            point_base = PointStamped()
            point_base.header.frame_id = self.base_frame  # Use parameter instead of hardcoded
            point_base.point.x = 1.0 * math.cos(angle_rad_base)
            point_base.point.y = 1.0 * math.sin(angle_rad_base)

            # Find the transform from base_link to the LIDAR frame
            transform = self.get_transform_to_base_link(
                self.scan_data.header.frame_id)

            # Transform the point to the LIDAR frame
            point_lidar = tf2_geometry_msgs.do_transform_point(point_base, transform)

            # Calculate the angle in the LIDAR frame
            angle_rad_lidar = math.atan2(point_lidar.point.y, point_lidar.point.x)

            # Handle wraparound case where scan crosses 0° (angle_max < angle_min)
            if self.scan_data.angle_max < self.scan_data.angle_min:
                # Normalize the target angle to the scan's coordinate system
                if angle_rad_lidar < 0:
                    angle_rad_lidar += 2 * math.pi
                
                # Calculate index based on position relative to angle_min
                if angle_rad_lidar >= self.scan_data.angle_min:
                    # First part of scan (e.g., +90° towards +180°/0°)
                    angle_from_start = angle_rad_lidar - self.scan_data.angle_min
                else:
                    # Second part of scan (e.g., 0° towards -90°)
                    # We're past the wraparound point
                    angle_from_start = (2 * math.pi - self.scan_data.angle_min) + angle_rad_lidar
                
                index = int(angle_from_start / self.scan_data.angle_increment)
            else:
                # Normal case where angle_min < angle_max
                index = int((angle_rad_lidar - self.scan_data.angle_min) / self.scan_data.angle_increment)
            
            # DEBUG: Log the transformation details
            if angle_deg in [-90, 0, 90]:  # Only log for the main directions
                wraparound = self.scan_data.angle_max < self.scan_data.angle_min
                total_rays = len(self.scan_data.ranges)
                self.get_logger().info(f"DEBUG angle={angle_deg}°: base_point=({point_base.point.x:.2f},{point_base.point.y:.2f}) -> lidar_point=({point_lidar.point.x:.2f},{point_lidar.point.y:.2f}) -> lidar_angle={math.degrees(angle_rad_lidar):.1f}° -> index={index}/{total_rays}, wraparound={wraparound}")
                # self.get_logger().info(f"LIDAR range: {math.degrees(self.scan_data.angle_min):.1f}° to {math.degrees(self.scan_data.angle_max):.1f}°, increment={math.degrees(self.scan_data.angle_increment):.2f}°")
            
            if 0 <= index < len(self.scan_data.ranges):
                # Average over a small window for robustness
                start = max(0, index - 2)
                end = min(len(self.scan_data.ranges), index + 3)
                valid_ranges = [r for r in self.scan_data.ranges[start:end] if np.isfinite(r)]
                distance = np.mean(valid_ranges) if valid_ranges else float('inf')
                
                # DEBUG: Log range details for main directions
                # if angle_deg in [-90, 0, 90]:
                #     self.get_logger().info(f"DEBUG angle={angle_deg}°: scan_range[{index}]={self.scan_data.ranges[index]:.2f}, window_avg={distance:.2f}")
                
                return distance

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform error in get_distance_at_angle: {e}", throttle_duration_sec=5)
            return float('inf')

        return float('inf')

    def debug_coordinate_systems(self):
        """Debug method to understand coordinate system orientations"""
        if not self.scan_data:
            return
            
        self.get_logger().info("=== COORDINATE SYSTEM DEBUG ===")
        
        # Get some sample distances at key angles
        angles = [0, 45, 90, 135, 180, -135, -90, -45]  # degrees
        for angle_deg in angles:
            dist = self.get_distance_at_angle(angle_deg)
            self.get_logger().info(f"Distance at {angle_deg:4d}°: {dist:.2f}m")
        
        # Show LIDAR frame info
        # self.get_logger().info(f"LIDAR frame: '{self.scan_data.header.frame_id}'")
        # self.get_logger().info(f"LIDAR angle range: {math.degrees(self.scan_data.angle_min):.1f}° to {math.degrees(self.scan_data.angle_max):.1f}°")
        # self.get_logger().info(f"LIDAR increment: {math.degrees(self.scan_data.angle_increment):.2f}°")
        # self.get_logger().info(f"LIDAR range count: {len(self.scan_data.ranges)}")
        
        # Show transform chain
        # try:
        #     now = self.get_clock().now()
        #     transform = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=Duration(seconds=10.0))
        #     pos = transform.transform.translation
        #     quat = transform.transform.rotation
            
        #     # Convert quaternion to yaw
        #     import math
        #     yaw = math.atan2(2 * (quat.w * quat.z + quat.x * quat.y), 
        #                     1 - 2 * (quat.y**2 + quat.z**2))
        #     yaw_deg = math.degrees(yaw)
            
        #     self.get_logger().info(f"Robot pose in map: x={pos.x:.2f}, y={pos.y:.2f}, yaw={yaw_deg:.1f}°")
            
        # except Exception as e:
        #     self.get_logger().warn(f"Could not get robot pose: {e}")
        
        # self.get_logger().info("=== END DEBUG ===")

    def scan_callback(self, msg):
        """Process incoming laser scan data and transform to base_link frame"""
        # Store the raw scan data
        self.scan_data_raw = msg
        
        # Transform scan data to base_link frame
        self.scan_data = self.transform_scan_to_base_link(msg)
        
        # self.debug_coordinate_systems()

        # Log frame information for debugging
        if hasattr(self, 'last_scan_frame') and self.last_scan_frame != msg.header.frame_id:
            self.get_logger().info(f"LaserScan frame changed from {self.last_scan_frame} to {msg.header.frame_id}")
        
        self.last_scan_frame = msg.header.frame_id
        
        # Debug: Log first scan received with detailed frame info
        if not hasattr(self, 'scan_received'):
            self.get_logger().info(f"First LIDAR scan received: {len(msg.ranges)} points, frame: {msg.header.frame_id}")
            self.get_logger().info(f"LIDAR angle_min={msg.angle_min:.3f}, angle_max={msg.angle_max:.3f}, increment={msg.angle_increment:.4f}")
            self.get_logger().info(f"Transforming from {msg.header.frame_id} to {self.base_frame} for navigation")
            self.scan_received = True
    
    def odom_callback(self, msg):
        """Process incoming odometry data and transform to base_link frame"""
        # Store the raw odometry data
        self.odom_data_raw = msg
        
        # Transform odometry data to base_link frame
        self.odom_data = self.transform_odom_to_base_link(msg)
        
        # Log frame information for debugging
        if hasattr(self, 'last_odom_frame') and self.last_odom_frame != msg.header.frame_id:
            self.get_logger().info(f"Odometry frame changed from {self.last_odom_frame} to {msg.header.frame_id}")
        
        self.last_odom_frame = msg.header.frame_id
        
        # Debug: Log first odometry received with actual frame info
        if not hasattr(self, 'odom_received'):
            self.get_logger().info(f"First odometry received: frame: {msg.header.frame_id}, child_frame: {msg.child_frame_id}")
            self.get_logger().info(f"Parameter odom_frame='{self.odom_frame}', actual msg frame='{msg.header.frame_id}'")
            if msg.header.frame_id != self.odom_frame:
                self.get_logger().warn(f"FRAME MISMATCH: odom_frame parameter ('{self.odom_frame}') != actual message frame ('{msg.header.frame_id}')")
            self.get_logger().info(f"Transforming from {msg.header.frame_id} to {self.base_frame} for navigation")
            self.odom_received = True
        
        # Update position with properly transformed data
        self._update_position()
    
    def _update_position(self):
        """Update robot position and check for stuck condition using transformed odometry data"""
        if self.odom_data is None:
            return
        
        # Get the current position from transformed odometry (now in base_link frame)
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
                self.get_logger().debug(f"Robot position (base_link frame): ({current_position[0]:.2f}, {current_position[1]:.2f})")
                self.last_log_time = current_time
        else:
            self.last_log_time = current_time
        
        # Check for stuck condition - only when robot should be moving
        # Don't check for stuck during IDLE, RECOVERY states, or first few seconds after system ready
        should_check_stuck = (
            self.current_state not in [State.IDLE, State.RECOVERY_BACKUP, State.RECOVERY_TURN] and
            hasattr(self, 'system_ready_time') and 
            (current_time - self.system_ready_time).nanoseconds / 1e9 > 5.0  # Wait 5 seconds after system ready
        )
        
        if should_check_stuck and self.last_position is not None and self.last_time is not None:
            distance = math.sqrt(
                (current_position[0] - self.last_position[0])**2 +
                (current_position[1] - self.last_position[1])**2
            )
            
            time_diff = (current_time - self.last_time).nanoseconds / 1e9
            
            if distance < self.stuck_distance_threshold and time_diff > 1.0:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time
                    self.get_logger().info(f"Possible stuck condition detected: moved {distance:.3f}m in {time_diff:.1f}s")
                elif (current_time - self.stuck_start_time).nanoseconds / 1e9 > self.stuck_timeout:
                    self.get_logger().warn("Robot appears to be stuck!")
                    self.change_state(State.RECOVERY_BACKUP)
            else:
                self.stuck_start_time = None
        
        self.last_position = current_position
        self.last_time = current_time
    
    def transform_scan_to_base_link(self, scan_msg):
        """Transform laser scan from its frame to base_link frame"""
        if scan_msg.header.frame_id == self.base_frame:
            # Already in base_link frame, just return a copy with correct frame
            transformed_scan = LaserScan()
            transformed_scan.header.frame_id = self.base_frame
            transformed_scan.header.stamp = scan_msg.header.stamp
            transformed_scan.angle_min = scan_msg.angle_min
            transformed_scan.angle_max = scan_msg.angle_max
            transformed_scan.angle_increment = scan_msg.angle_increment
            transformed_scan.time_increment = scan_msg.time_increment
            transformed_scan.scan_time = scan_msg.scan_time
            transformed_scan.range_min = scan_msg.range_min
            transformed_scan.range_max = scan_msg.range_max
            transformed_scan.ranges = list(scan_msg.ranges)
            transformed_scan.intensities = list(scan_msg.intensities) if scan_msg.intensities else []
            return transformed_scan
        
        try:
            # Get transform from laser frame to base_link
            transform = self.get_transform_to_base_link(scan_msg.header.frame_id, self.base_frame)
            if transform is None:
                self.get_logger().warn(f"Could not get transform from {scan_msg.header.frame_id} to {self.base_frame}, using raw scan")
                return scan_msg
            
            # Create a new LaserScan message in base_link frame
            transformed_scan = LaserScan()
            transformed_scan.header.frame_id = self.base_frame
            transformed_scan.header.stamp = scan_msg.header.stamp
            
            # Get the rotation from laser frame to base_link
            # Extract rotation from transform quaternion
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert quaternion to yaw angle (rotation around z-axis)
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw_offset = math.atan2(siny_cosp, cosy_cosp)
            
            
            # Apply rotation to transform angles to base_link frame
            # After transformation, 0° will be forward in base_link frame
            transformed_scan.angle_min = self.normalize_angle(scan_msg.angle_min + yaw_offset)
            transformed_scan.angle_max = self.normalize_angle(scan_msg.angle_max + yaw_offset)
            transformed_scan.angle_increment = scan_msg.angle_increment
            transformed_scan.time_increment = scan_msg.time_increment
            transformed_scan.scan_time = scan_msg.scan_time
            transformed_scan.range_min = scan_msg.range_min
            transformed_scan.range_max = scan_msg.range_max
            
            # Copy range data (distances don't change, only the angle interpretation)
            transformed_scan.ranges = list(scan_msg.ranges)
            transformed_scan.intensities = list(scan_msg.intensities) if scan_msg.intensities else []
            
            # DEBUG: Check the transformed ranges
            if not hasattr(self, 'transform_debug_logged'):
                valid_ranges = [r for r in transformed_scan.ranges if math.isfinite(r) and r > 0.1]
                if valid_ranges:
                    min_valid = min(valid_ranges)
                    max_valid = max(valid_ranges)
                    self.get_logger().info(f"Transformed scan ranges: {len(valid_ranges)}/{len(transformed_scan.ranges)} valid, min={min_valid:.2f}, max={max_valid:.2f}")
                else:
                    self.get_logger().warn(f"NO VALID RANGES in transformed scan! All {len(transformed_scan.ranges)} ranges are invalid")
                # Show first 10 ranges as example
                sample_ranges = [f"{i}:{transformed_scan.ranges[i]:.2f}" for i in range(min(10, len(transformed_scan.ranges)))]
                self.get_logger().info(f"First 10 transformed ranges: {sample_ranges}")
                self.transform_debug_logged = True
            
            # Store the yaw offset for debugging
            self.lidar_to_base_yaw_offset = yaw_offset
            
            return transformed_scan
            
        except Exception as e:
            self.get_logger().warn(f"Failed to transform scan to base_link: {e}, using raw scan")
            return scan_msg
    
    def transform_odom_to_base_link(self, odom_msg):
        """Transform odometry data from its frame to base_link frame"""
        if odom_msg.header.frame_id == self.base_frame:
            # Already in base_link frame, just return a copy
            self.get_logger().debug(f"Odometry already in {self.base_frame} frame")
            return odom_msg
        
        # Check if transform is needed and available
        try:
            # Get transform from odom frame to base_link
            transform = self.get_transform_to_base_link(odom_msg.header.frame_id, self.base_frame)
            if transform is None:
                self.get_logger().debug(f"Transform not available from {odom_msg.header.frame_id} to {self.base_frame}, using raw odom")
                # Return raw odom but mark it properly
                odom_copy = Odometry()
                odom_copy.header = odom_msg.header
                odom_copy.child_frame_id = odom_msg.child_frame_id
                odom_copy.pose = odom_msg.pose
                odom_copy.twist = odom_msg.twist
                return odom_copy
            
            # Create a transformed odometry message
            transformed_odom = Odometry()
            transformed_odom.header.frame_id = self.base_frame  # Now expressing everything in base_link terms
            transformed_odom.header.stamp = odom_msg.header.stamp
            transformed_odom.child_frame_id = self.base_frame
            
            # Transform the pose using the transform
            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose  # Extract the pose from PoseWithCovariance
            
            transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            transformed_odom.pose.pose = transformed_pose_stamped.pose  # Put it back in PoseWithCovariance
            transformed_odom.pose.covariance = odom_msg.pose.covariance
            
            # Twist (velocity) should typically already be in base_link frame, so copy it
            transformed_odom.twist = odom_msg.twist
            
            self.get_logger().debug(f"Successfully transformed odom from {odom_msg.header.frame_id} to {self.base_frame}")
            return transformed_odom
            
        except Exception as e:
            self.get_logger().debug(f"Failed to transform odom to base_link: {e}, using raw odom")
            # Return raw odom as fallback
            odom_copy = Odometry()
            odom_copy.header = odom_msg.header
            odom_copy.child_frame_id = odom_msg.child_frame_id
            odom_copy.pose = odom_msg.pose
            odom_copy.twist = odom_msg.twist
            return odom_copy
    
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
        front_dist = self.get_distance_at_angle(0.0)
        left_dist = self.get_distance_at_angle(90.0)
        right_dist = self.get_distance_at_angle(270.0)
        
        # DEBUG: Log the distance readings
        self.get_logger().info(f"Distance readings: front={front_dist:.2f}m, left={left_dist:.2f}m, right={right_dist:.2f}m")
        
        # Calculate space width (minimum of left and right distances)
        space_width = min(left_dist, right_dist) * 2  # Multiply by 2 for full width
        
        # Doorway detection parameters
        doorway_min_opening = 0.6   # Minimum opening width for a doorway (0.65m doors exist)
        doorway_max_opening = 1.2   # Maximum opening width for a doorway (increased)
        doorway_depth_threshold = 1.2  # How deep the opening should be (reduced from 1.5)
        
        # Check for doorway opening on each side
        # A doorway is detected when one side shows a significant wall disappearance
        # AND the space is narrow enough to be a doorway
        
        is_doorway_left = (left_dist >= doorway_min_opening and 
                          left_dist <= doorway_depth_threshold and
                          right_dist < doorway_max_opening)  # Other side should be close (wall)
        is_doorway_right = (right_dist >= doorway_min_opening and 
                           right_dist <= doorway_depth_threshold and
                           left_dist < doorway_max_opening)  # Other side should be close (wall)
        
        # Log doorway detection details for debugging
        position_str = self.get_position_debug_string()
        
        # Add LIDAR debugging info - now using transformed data
        if self.scan_data:
            lidar_debug = f"LIDAR (base_link): angle_min={self.scan_data.angle_min:.2f}, angle_max={self.scan_data.angle_max:.2f}, " \
                         f"increment={self.scan_data.angle_increment:.3f}, rays={len(self.scan_data.ranges)}"
        else:
            lidar_debug = "LIDAR: Not available"
        
        self.get_logger().info(f"Space classification at {position_str}: "
                              f"left={left_dist:.2f}m, right={right_dist:.2f}m, front={front_dist:.2f}m, "
                              f"space_width={space_width:.2f}m, doorway_left={is_doorway_left}, doorway_right={is_doorway_right}")
        
        # Log LIDAR info occasionally for debugging
        if hasattr(self, 'last_lidar_debug_time'):
            if (self.get_clock().now() - self.last_lidar_debug_time).nanoseconds > 5e9:  # Every 5 seconds
                self.get_logger().info(lidar_debug)
                self.last_lidar_debug_time = self.get_clock().now()
        else:
            self.last_lidar_debug_time = self.get_clock().now()
        
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
    
    # def get_side_distance(self, side='right'):
    #     """Get distance to the side (left or right) using transformed scan data in base_link frame"""
    #     if self.scan_data is None:
    #         return 10.0
        
    #     # In base_link frame after transformation: +X is forward, +Y is left, -Y is right
    #     # Now that we've transformed the data, 0° is indeed forward
    #     if side == 'right':
    #         # Look directly to the right (-Y direction)
    #         return self.get_distance_in_direction(0.0, -1.0)
    #     else:
    #         # Look directly to the left (+Y direction)
    #         return self.get_distance_in_direction(0.0, 1.0)
    
    # def get_front_distance(self, angle_offset=0.0):
    #     """Get distance in front of the robot using multiple rays for better detection"""
    #     # Use multiple rays in front for better obstacle detection
    #     # In base_link frame: +X is forward, so we look in +X direction with small Y offsets
    #     angles = [angle_offset - 0.2, angle_offset, angle_offset + 0.2]  # ±0.2 radians (±11.5°)
    #     distances = []
        
    #     for angle in angles:
    #         # Convert angle offset to direction vector
    #         # Forward direction (1.0, 0.0) with angular offset
    #         direction_x = math.cos(angle)
    #         direction_y = math.sin(angle)
    #         dist = self.get_distance_in_direction(direction_x, direction_y)
    #         if dist < 10.0:  # Valid reading
    #             distances.append(dist)
        
    #     # Return the minimum distance for safety
    #     if distances:
    #         return min(distances)
    #     else:
    #         return 10.0
    
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
        front_dist = self.get_distance_at_angle(0.0)
        left_dist = self.get_distance_at_angle(90.0)
        right_dist = self.get_distance_at_angle(270.0)
        
        return (front_dist < self.min_obstacle_distance or
                left_dist < self.min_obstacle_distance or
                right_dist < self.min_obstacle_distance)
    
    def is_path_clear_ahead(self, check_distance=0.3):
        """Check if path ahead is clear"""
        front_dist = self.get_distance_at_angle(0.0)
        return front_dist > check_distance
    
    def get_wall_following_command(self, target_distance):
        """Generate wall following command - follow right wall safely"""
        if self.scan_data is None:
            return Twist()
        
        # Get distance to wall on the right
        wall_distance = self.get_distance_at_angle(270.0)
        left_distance = self.get_distance_at_angle(90.0)
        front_distance = self.get_distance_at_angle(0.0)
        
        # Calculate error (positive = too close to wall, negative = too far from wall)
        error = target_distance - wall_distance
        
        # Generate control command
        cmd = Twist()
        cmd.linear.x = self.forward_speed
        
        # Safety check - if either side is very close, be very careful
        if wall_distance < 0.3 or left_distance < 0.3:
            cmd.linear.x = self.narrow_space_speed  # Slow down
            self.get_logger().warn(f"Close to walls: left={left_distance:.2f}m, right={wall_distance:.2f}m, slowing down")
        
        # Angular control based on wall distance error
        # If error > 0: too close to right wall, turn LEFT (+angular in ROS2)
        # If error < 0: too far from right wall, turn RIGHT (-angular in ROS2)
        angular_velocity = self.wall_following_gain * error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        # Debug logging for wall following
        self.get_logger().info(f"Wall following: right={wall_distance:.2f}m, target={target_distance:.2f}m, error={error:.2f}, angular={angular_velocity:.2f}")
        
        return cmd
    
    def get_hallway_centering_command(self):
        """Generate hallway centering command"""
        if self.scan_data is None:
            return Twist()
        
        left_dist = self.get_distance_at_angle(90.0)
        right_dist = self.get_distance_at_angle(270.0)
        front_dist = self.get_distance_at_angle(0.0)
        
        # Calculate center error for ROS2 coordinate system
        # In ROS2: +angular.z = turn LEFT (counter-clockwise), -angular.z = turn RIGHT (clockwise)
        # If left_dist < right_dist, robot is closer to left wall, need to turn RIGHT (-angular)
        # If right_dist < left_dist, robot is closer to right wall, need to turn LEFT (+angular)
        center_error = left_dist - right_dist  # CORRECTED: This gives the correct sign for ROS2
        
        cmd = Twist()
        cmd.linear.x = self.hallway_speed
        
        # Safety check - slow down if front obstacle detected
        if front_dist < self.comfortable_distance:
            cmd.linear.x = self.hallway_speed * 0.5  # Slow down
            self.get_logger().warn(f"Front obstacle detected at {front_dist:.2f}m, slowing down")
        
        # Angular control to center in hallway  
        angular_velocity = self.hallway_centering_gain * center_error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        # Log hallway centering debug info occasionally
        if hasattr(self, 'last_hallway_debug_time'):
            if (self.get_clock().now() - self.last_hallway_debug_time).nanoseconds > 2e9:  # Every 2 seconds
                self.get_logger().info(f"Hallway centering: left={left_dist:.2f}m, right={right_dist:.2f}m, front={front_dist:.2f}m, error={center_error:.2f}, angular={angular_velocity:.2f} (+ = turn LEFT, - = turn RIGHT)")
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
            self.get_distance_at_angle(0.0),
            self.get_distance_at_angle(90.0),
            self.get_distance_at_angle(270.0)
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
        
        left_dist = self.get_distance_at_angle(90.0)
        right_dist = self.get_distance_at_angle(270.0)
        front_dist = self.get_distance_at_angle(0.0)
        
        # Calculate center error for ROS2 coordinate system
        # In ROS2: +angular.z = turn LEFT (counter-clockwise), -angular.z = turn RIGHT (clockwise)
        # If left_dist < right_dist, robot is closer to left wall, need to turn RIGHT (-angular)
        # If right_dist < left_dist, robot is closer to right wall, need to turn LEFT (+angular)
        center_error = left_dist - right_dist  # CORRECTED: This gives the correct sign for ROS2
        
        cmd = Twist()
        
        # Slow down when approaching doorway for better control
        cmd.linear.x = self.narrow_space_speed * 0.7  # Even slower approach
        
        # Strong centering control for doorway approach (reduced gain to prevent oscillation)
        angular_velocity = 0.8 * center_error  # Further reduced gain
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        # Debug logging for doorway approach
        self.get_logger().info(f"Doorway approach: left={left_dist:.2f}m, right={right_dist:.2f}m, center_error={center_error:.2f}, angular={angular_velocity:.2f} (+ = turn LEFT, - = turn RIGHT)")
        
        # Check if centered enough to proceed
        if abs(center_error) < self.doorway_centering_tolerance:
            self.get_logger().info(f"Doorway centered! Error={center_error:.3f}, transitioning to DOORWAY_ENTRY")
            self.change_state(State.DOORWAY_ENTRY)
        
        # Safety check - if too close to front obstacle, stop and re-center
        if front_dist < 0.4:
            cmd.linear.x = 0.0  # Stop forward motion
            self.get_logger().warn(f"Too close to front obstacle ({front_dist:.2f}m), stopping to center")
        
        return cmd
    
    def enter_doorway(self):
        """Execute doorway entry"""
        if self.scan_data is None:
            return Twist()
        
        # Move forward slowly through doorway
        cmd = Twist()
        cmd.linear.x = self.narrow_space_speed
        
        # Minimal angular correction to stay centered
        left_dist = self.get_distance_at_angle(90.0)
        right_dist = self.get_distance_at_angle(270.0)
        center_error = left_dist - right_dist  # CORRECTED: This gives the correct sign for ROS2
        
        angular_velocity = 0.5 * center_error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        cmd.angular.z = angular_velocity
        
        # Check if we've cleared the doorway
        front_dist = self.get_distance_at_angle(0.0)
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
        """Recovery behavior - backup only if it's safe"""
        cmd = Twist()
        
        # Check if it's safe to backup (no obstacles behind)
        # Look behind: -X direction in base_link frame
        back_dist = self.get_distance_at_angle(180.0)  # Check behind
        if back_dist > self.comfortable_distance:
            cmd.linear.x = self.recovery_backup_speed
        else:
            cmd.linear.x = 0.0  # Don't move if obstacle behind
            self.get_logger().warn("Cannot backup - obstacle behind!")
        
        # Start timer for this state
        if 'recovery_start' not in self.doorway_state:
            self.doorway_state['recovery_start'] = self.get_clock().now()
        
        elapsed_time = (self.get_clock().now() - self.doorway_state['recovery_start']).nanoseconds / 1e9
        
        if elapsed_time > self.recovery_backup_time:
            self.doorway_state.clear()
            self.change_state(State.RECOVERY_TURN)
        
        return cmd
    
    def recovery_turn(self):
        """Recovery behavior - turn only if safe"""
        cmd = Twist()
        
        # Only turn if not at immediate collision risk
        if not self.check_immediate_collision_risk():
            cmd.angular.z = self.turn_speed
        else:
            cmd.angular.z = 0.0  # Don't turn if still at collision risk
            self.get_logger().warn("Cannot turn - still at collision risk!")
        
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
        """Validate that sensor data is available and properly transformed to base_link"""
        if self.scan_data is None:
            self.get_logger().warn("No transformed LIDAR scan data available")
            return False
        
        if self.odom_data is None:
            self.get_logger().warn("No transformed odometry data available")
            return False
        
        # Verify that transformed LIDAR data is in base_link frame
        if self.scan_data.header.frame_id != self.base_frame:
            self.get_logger().warn(f"LIDAR data frame mismatch: expected {self.base_frame}, got {self.scan_data.header.frame_id}")
            return False
        
        # For odometry, be more flexible - if transform wasn't available, we can still use it
        # but log what frame we're actually using
        if self.odom_data.header.frame_id != self.base_frame:
            self.get_logger().info(f"Using odometry data in {self.odom_data.header.frame_id} frame (transform to {self.base_frame} not available)")
        
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
        
        # Check for immediate collision risk - CRITICAL SAFETY CHECK
        if self.check_immediate_collision_risk():
            self.get_logger().warn("EMERGENCY STOP - Collision risk detected!")
            # EMERGENCY STOP - override all other commands
            cmd = Twist()  # Zero velocity command
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.change_state(State.RECOVERY_BACKUP)
            return  # Exit immediately without other commands
        
        # Generate control command based on current state
        cmd = Twist()
        
        if self.current_state == State.IDLE:
            self.get_logger().info("Starting robot navigation - transitioning from IDLE to FIND_WALL")
            self.change_state(State.FIND_WALL)
            # Continue to execute FIND_WALL logic immediately rather than waiting for next cycle
            
        if self.current_state == State.FIND_WALL:
            # Find a wall to follow - be VERY careful not to crash into walls
            if self.current_space_type == SpaceType.ROOM:
                self.change_state(State.ROOM_PATROL)
            elif self.current_space_type == SpaceType.HALLWAY:
                self.change_state(State.HALLWAY_NAVIGATION)
            elif self.current_space_type == SpaceType.DOORWAY:
                self.change_state(State.DOORWAY_APPROACH)
            else:
                # SAFE wall finding behavior - move forward slowly and check space
                left_dist = self.get_distance_at_angle(90.0)
                right_dist = self.get_distance_at_angle(270.0)
                front_dist = self.get_distance_at_angle(0.0)
                
                cmd = Twist()
                
                # If we're in a very tight space (both sides close), just move forward slowly
                if left_dist < 0.6 and right_dist < 0.6:
                    self.get_logger().info(f"Tight space detected (L={left_dist:.2f}, R={right_dist:.2f}), moving forward slowly")
                    cmd.linear.x = 0.05  # Very slow forward motion
                    cmd.angular.z = 0.0  # No turning in tight spaces
                # If there's much more space on one side, gradually turn toward it
                elif left_dist > right_dist + 0.3:  # Much more space on left
                    self.get_logger().info(f"More space on left (L={left_dist:.2f}, R={right_dist:.2f}), turning gently left")
                    cmd.linear.x = 0.08
                    cmd.angular.z = 0.1  # Gentle left turn
                elif right_dist > left_dist + 0.3:  # Much more space on right
                    self.get_logger().info(f"More space on right (L={left_dist:.2f}, R={right_dist:.2f}), turning gently right")
                    cmd.linear.x = 0.08
                    cmd.angular.z = -0.1  # Gentle right turn
                else:
                    # Fairly balanced space, just move forward
                    self.get_logger().info(f"Balanced space (L={left_dist:.2f}, R={right_dist:.2f}), moving forward")
                    cmd.linear.x = 0.08
                    cmd.angular.z = 0.0
        
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
        
        # Sensor data status - show both raw and transformed data
        if self.scan_data is not None:
            status_msg += f"\n  LIDAR: Available (transformed to {self.base_frame})"
            if len(self.scan_data.ranges) > 0:
                min_range = min(self.scan_data.ranges)
                max_range = max(self.scan_data.ranges)
                status_msg += f" - ranges: {min_range:.2f} to {max_range:.2f}"
        else:
            status_msg += f"\n  LIDAR: NOT AVAILABLE"
        
        if self.odom_data is not None:
            status_msg += f"\n  ODOM: Available (transformed to {self.base_frame})"
            if self.last_position is not None:
                # Show position in base_link frame (transformed)
                status_msg += f" - base_link position: ({self.last_position[0]:.2f}, {self.last_position[1]:.2f})"
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