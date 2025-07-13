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
import sys # For main function

class State(Enum):
    FIND_WALL = 0
    FOLLOW_WALL = 1
    AVOID_OBSTACLE = 2
    RECOVERY_BACKUP = 3
    RECOVERY_TURN = 4
    # ROAMING was used inconsistently, FOLLOW_WALL is clearer
    # Let's remove ROAMING to avoid confusion
    # ROAMING = 5

class PerimeterRoamer(Node):
    def __init__(self):
        super().__init__('perimeter_roamer')

        self.get_logger().info("Perimeter Roamer Node Starting Up...")

        # --- ROS2 Parameters ---
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.4)
        self.declare_parameter('wall_distance_setpoint', 0.5)
        self.declare_parameter('wall_following_gain', 0.3)
        self.declare_parameter('obstacle_distance_threshold', 0.5)
        self.declare_parameter('corner_lookahead_distance', 0.3)
        self.declare_parameter('corner_lookahead_angle', math.pi / 4)
        self.declare_parameter('costmap_check_distance', 0.5)
        self.declare_parameter('costmap_lethal_threshold', 254)
        self.declare_parameter('costmap_inscribed_threshold', 99)
        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('safety_bubble_distance', 0.3)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('max_angular_speed', 1.0)
        # --- Additional configuration parameters that were missing ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('robot_width', 0.3)
        self.declare_parameter('costmap_check_points_front', 5)
        self.declare_parameter('stuck_timeout', 5.0)
        self.declare_parameter('stuck_distance_threshold', 0.02)
        self.declare_parameter('stuck_angle_threshold', 0.1) # In radians
        self.declare_parameter('corner_check_angle', 0.785) # 45 degrees
        self.declare_parameter('wall_presence_threshold', 0.2)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('recovery_backup_time', 2.0)
        self.declare_parameter('recovery_backup_speed', -0.1)
        self.declare_parameter('recovery_turn_time', 2.5)
        self.declare_parameter('recovery_turn_angle', 1.57) # 90 degrees

        # Get parameters
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.wall_distance_setpoint = self.get_parameter('wall_distance_setpoint').get_parameter_value().double_value
        self.wall_following_gain = self.get_parameter('wall_following_gain').get_parameter_value().double_value
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').get_parameter_value().double_value
        self.corner_lookahead_distance = self.get_parameter('corner_lookahead_distance').get_parameter_value().double_value
        self.corner_lookahead_angle = self.get_parameter('corner_lookahead_angle').get_parameter_value().double_value
        self.costmap_check_distance = self.get_parameter('costmap_check_distance').get_parameter_value().double_value
        self.costmap_lethal_threshold = self.get_parameter('costmap_lethal_threshold').get_parameter_value().integer_value
        self.costmap_inscribed_threshold = self.get_parameter('costmap_inscribed_threshold').get_parameter_value().integer_value
        # Robot Dimensions
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.safety_bubble_radius = self.get_parameter('safety_bubble_distance').get_parameter_value().double_value
        self.robot_base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # --- Get additional configuration parameters ---
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').get_parameter_value().string_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.robot_width = self.get_parameter('robot_width').get_parameter_value().double_value
        self.costmap_check_points_front = self.get_parameter('costmap_check_points_front').get_parameter_value().integer_value
        stuck_timeout_sec = self.get_parameter('stuck_timeout').get_parameter_value().double_value
        self.stuck_timeout_dur = Duration(seconds=stuck_timeout_sec)
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').get_parameter_value().double_value
        self.stuck_angle_threshold = self.get_parameter('stuck_angle_threshold').get_parameter_value().double_value
        self.corner_check_angle = self.get_parameter('corner_check_angle').get_parameter_value().double_value
        self.wall_presence_threshold = self.get_parameter('wall_presence_threshold').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.recovery_backup_time = self.get_parameter('recovery_backup_time').get_parameter_value().double_value
        self.recovery_backup_speed = self.get_parameter('recovery_backup_speed').get_parameter_value().double_value
        self.recovery_turn_time = self.get_parameter('recovery_turn_time').get_parameter_value().double_value
        self.recovery_turn_angle = self.get_parameter('recovery_turn_angle').get_parameter_value().double_value


        self.get_logger().info("--- Loaded Parameters ---")
        self.get_logger().info(f"Speeds: Forward={self.forward_speed}, Turn={self.turn_speed}, MaxAngular={self.max_angular_speed}")
        self.get_logger().info(f"Wall Following: Distance={self.wall_distance_setpoint}, Gain={self.wall_following_gain}, Kp={self.kp_angular}")
        self.get_logger().info(f"Obstacle Avoidance: FrontDist(Legacy)={self.obstacle_distance_threshold}")
        self.get_logger().info(f"Corner Avoidance: Lookahead={self.corner_lookahead_distance}m, Angle={math.degrees(self.corner_lookahead_angle)}deg")
        self.get_logger().info(f"Costmap: CheckDist={self.costmap_check_distance:.2f}, LethalThreshold={self.costmap_lethal_threshold}, InscribedThreshold={self.costmap_inscribed_threshold}")
        self.get_logger().info(f"Robot: BaseFrame='{self.robot_base_frame}', Radius={self.robot_radius}m, SafetyBubble={self.safety_bubble_radius}m, Width={self.robot_width}m")
        self.get_logger().info(f"Topics: cmd_vel='{self.cmd_vel_topic}', costmap='{self.local_costmap_topic}'")
        self.get_logger().info(f"Stuck Detection: Timeout={stuck_timeout_sec}s, Dist={self.stuck_distance_threshold}m, Angle={math.degrees(self.stuck_angle_threshold)}deg")
        self.get_logger().info(f"Recovery: BackupTime={self.recovery_backup_time}s, BackupSpeed={self.recovery_backup_speed}m/s, TurnTime={self.recovery_turn_time}s")
        self.get_logger().info("-------------------------")

        # --- State Variables ---
        self.state = State.FIND_WALL
        self.last_scan = None # Still useful for potential future logic or debugging
        self.current_odom_pose = None
        self.local_costmap = None
        self.costmap_metadata = None
        self.costmap_frame = "" # Will be populated from costmap message header

        # Stuck detection variables
        self.last_pose_for_stuck_check = None
        self.last_pose_check_time = self.get_clock().now()
        self.last_state_change_time = self.get_clock().now() # For state timeouts

        # Recovery variables
        self.recovery_start_time = None

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Define QoS profile for costmap (volatile, last is best)
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # BEST_EFFORT can be okay too if network is good
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Matches Nav2 costmap publisher settings
        )
        scan_qos = QoSProfile(
             reliability=ReliabilityPolicy.BEST_EFFORT, # Sensor data often best effort
             history=HistoryPolicy.KEEP_LAST,
             depth=1
        )


        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, self.local_costmap_topic, self.costmap_callback, costmap_qos)
        # Add a subscription to the costmap updates
        self.costmap_update_sub = self.create_subscription(
            OccupancyGridUpdate,
            self.local_costmap_topic + '_updates', # Topic is often costmap_topic + "_updates"
            self.costmap_update_callback,
            costmap_qos)


        # Optional: Marker publisher for debugging costmap check points
        self.marker_pub = self.create_publisher(MarkerArray, '~/costmap_check_points', 1)


        # === Main Control Timer ===
        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info("Perimeter Roamer Node Initialized.")


    # --- Callbacks ---
    def scan_callback(self, msg):
        self.last_scan = msg

    def odom_callback(self, msg):
        self.current_odom_pose = msg.pose.pose
        # Initialize stuck check pose if needed
        if self.last_pose_for_stuck_check is None:
             self.last_pose_for_stuck_check = self.current_odom_pose
             self.last_pose_check_time = self.get_clock().now()


    def costmap_callback(self, msg):
        """Callback for the full costmap topic."""
        # This is the unified variable for costmap data
        self.local_costmap = msg.data
        self.costmap_metadata = msg.info
        self.costmap_frame = msg.header.frame_id
        # self.get_logger().debug(f"Full costmap received. Frame: {self.costmap_frame}")

    def costmap_update_callback(self, msg):
        """Callback to handle OccupancyGridUpdate messages."""
        if self.local_costmap is None or self.costmap_metadata is None:
            self.get_logger().warn("Received costmap update before full costmap. Ignoring.")
            return

        # The data in OccupancyGridUpdate is a 1D array for a rectangular region
        # of the costmap.
        map_width = self.costmap_metadata.width
        
        # Update the relevant part of the local_costmap
        for y in range(msg.height):
            for x in range(msg.width):
                # Calculate index in the update data
                update_index = y * msg.width + x
                # Calculate index in the full costmap
                map_x = msg.x + x
                map_y = msg.y + y
                map_index = map_y * map_width + map_x
                
                if 0 <= map_index < len(self.local_costmap):
                    self.local_costmap[map_index] = msg.data[update_index]
                else:
                    self.get_logger().warn(f"Costmap update index [{map_index}] out of bounds for map size [{len(self.local_costmap)}].")

    # --- Utility Functions ---

    def euler_from_quaternion(self, quaternion):
        """Converts a quaternion message to euler angles."""
        # Correction: euler_from_quaternion expects a list/tuple, not individual args
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return tf_transformations.euler_from_quaternion(q)

    def check_costmap_for_collision(self):
        """
        Checks the local costmap ahead of the robot for obstacles using TF for precision.
        This is the primary forward-checking function.
        """
        if self.local_costmap is None or self.costmap_metadata is None or not self.costmap_frame:
            self.get_logger().warn("Costmap data not available for collision check.", throttle_duration_sec=5)
            return True # Assume collision if no data

        try:
            # Get the robot's current pose in the costmap frame
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame,       # Target frame
                self.robot_base_frame,    # Source frame
                rclpy.time.Time(),        # Get latest available
                timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed from '{self.robot_base_frame}' to '{self.costmap_frame}': {e}")
            return True # Assume collision if transform fails

        # Robot's position and orientation in costmap frame
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        # Use the corrected helper function call
        _, _, robot_theta = self.euler_from_quaternion(transform.transform.rotation)

        # Define check points relative to the robot's base_link frame
        check_points_base = []
        num_checks = self.costmap_check_points_front
        half_width = self.robot_width / 2.0

        # Create a line of check points in front of the robot
        if num_checks <= 1:
            check_points_base.append(Point(x=self.costmap_check_distance, y=0.0, z=0.0))
        else:
            for i in range(num_checks):
                y_offset = half_width - (i * self.robot_width / (num_checks - 1))
                check_points_base.append(Point(x=self.costmap_check_distance, y=y_offset, z=0.0))

        # Transform check points to costmap frame and check cost
        resolution = self.costmap_metadata.resolution
        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        width = self.costmap_metadata.width
        height = self.costmap_metadata.height

        for pt_base in check_points_base:
            # Manual 2D transformation from robot frame to costmap frame
            pt_map_x = robot_x + pt_base.x * math.cos(robot_theta) - pt_base.y * math.sin(robot_theta)
            pt_map_y = robot_y + pt_base.x * math.sin(robot_theta) + pt_base.y * math.cos(robot_theta)

            # Convert world coordinates (costmap frame) to map cell coordinates
            mx = int((pt_map_x - origin_x) / resolution)
            my = int((pt_map_y - origin_y) / resolution)

            # Check if cell is within map bounds
            if 0 <= mx < width and 0 <= my < height:
                index = my * width + mx
                cost = self.local_costmap[index]
                if cost >= self.costmap_inscribed_threshold:
                    self.get_logger().warning(f"Collision check: Obstacle detected at costmap cell ({mx},{my}) with cost {cost}")
                    return True # Collision detected
            else:
                # Point is outside the local costmap, which is risky.
                self.get_logger().warning("Collision check: Check point is outside local costmap bounds.")
                return True # Treat as collision

        return False # Path is clear

    def check_safety_bubble(self):
        """
        Checks if any obstacle has breached the robot's safety bubble using the costmap.
        """
        if self.local_costmap is None or self.costmap_metadata is None or not self.costmap_frame:
            self.get_logger().warn("No costmap data for safety bubble check.", throttle_duration_sec=5)
            return True # Assume breach if no data

        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, self.robot_base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup for safety bubble failed: {e}")
            return True

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        resolution = self.costmap_metadata.resolution
        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        width = self.costmap_metadata.width
        height = self.costmap_metadata.height

        # Check a circular region around the robot.
        check_radius_in_cells = int(self.safety_bubble_radius / resolution)
        robot_mx = int((robot_x - origin_x) / resolution)
        robot_my = int((robot_y - origin_y) / resolution)

        for my_offset in range(-check_radius_in_cells, check_radius_in_cells + 1):
            for mx_offset in range(-check_radius_in_cells, check_radius_in_cells + 1):
                # Check if the point is within the circular bubble
                if mx_offset**2 + my_offset**2 > check_radius_in_cells**2:
                    continue

                mx = robot_mx + mx_offset
                my = robot_my + my_offset

                if 0 <= mx < width and 0 <= my < height:
                    index = my * width + mx
                    cost = self.local_costmap[index]
                    if cost >= self.costmap_inscribed_threshold: # Use inscribed threshold for safety
                        self.get_logger().fatal(
                            f"SAFETY BUBBLE BREACHED! Obstacle with cost ({cost}) detected in cell ({mx}, {my})."
                        )
                        return True # Breach detected
        return False # No breach

    def is_stuck(self):
        """ Checks if the robot has moved significantly since the last check. """
        now = self.get_clock().now()
        delta_time = now - self.last_pose_check_time

        if delta_time < self.stuck_timeout_dur:
            return False # Not enough time passed

        if self.current_odom_pose is None or self.last_pose_for_stuck_check is None:
             # Reset timer if we don't have pose data
            self.last_pose_check_time = now
            return False

        # Calculate distance and angle change in the odom frame
        curr = self.current_odom_pose
        last = self.last_pose_for_stuck_check

        dx = curr.position.x - last.position.x
        dy = curr.position.y - last.position.y
        dist_moved = math.sqrt(dx*dx + dy*dy)

        # Use the helper function for quaternion to euler conversion
        curr_yaw = self.euler_from_quaternion(curr.orientation)[2]
        last_yaw = self.euler_from_quaternion(last.orientation)[2]
        angle_moved = abs(self.normalize_angle(curr_yaw - last_yaw))

        # Reset for next check
        self.last_pose_for_stuck_check = self.current_odom_pose
        self.last_pose_check_time = now

        # Check if movement is below thresholds
        if dist_moved < self.stuck_distance_threshold and angle_moved < self.stuck_angle_threshold:
            self.get_logger().warn(f"Stuck detected! Moved {dist_moved:.3f}m and {math.degrees(angle_moved):.1f}deg in {delta_time.nanoseconds / 1e9:.1f}s.")
            return True
        else:
            #self.get_logger().debug(f"Movement check: Moved {dist_moved:.3f}m and {math.degrees(angle_moved):.1f}deg.")
            return False

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        """ Publishes zero velocity """
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped.")

    def check_for_corner_collision(self):
        """
        Checks for obstacles in the 'inner corner' sector during a right turn using the costmap.
        """
        if self.local_costmap is None or self.costmap_metadata is None:
            return True # Assume collision

        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, self.robot_base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup for corner check failed: {e}")
            return True

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        _, _, robot_theta = self.euler_from_quaternion(transform.transform.rotation)

        # Define a check point angled into the corner
        check_x_base = self.corner_lookahead_distance * math.cos(-self.corner_check_angle / 2)
        check_y_base = self.corner_lookahead_distance * math.sin(-self.corner_check_angle / 2)

        # Transform check point to costmap frame
        pt_map_x = robot_x + check_x_base * math.cos(robot_theta) - check_y_base * math.sin(robot_theta)
        pt_map_y = robot_y + check_x_base * math.sin(robot_theta) + check_y_base * math.cos(robot_theta)

        # Convert to map cell and check cost
        resolution = self.costmap_metadata.resolution
        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        width = self.costmap_metadata.width
        height = self.costmap_metadata.height

        mx = int((pt_map_x - origin_x) / resolution)
        my = int((pt_map_y - origin_y) / resolution)

        if 0 <= mx < width and 0 <= my < height:
            index = my * width + mx
            cost = self.local_costmap[index]
            if cost >= self.costmap_inscribed_threshold: # Use inscribed threshold
                self.get_logger().warn(f"Potential corner collision DETECTED via costmap at cell ({mx}, {my}), cost={cost}")
                return True
        else:
            self.get_logger().warn("Corner check point is outside the local costmap.")
            # This could be risky, so maybe treat as a collision
            return True

        return False

    def check_obstacle_in_front(self):
        """
        Primary obstacle check using the costmap. This replaces the old LIDAR-based check.
        """
        return self.check_costmap_for_collision()

    def get_side_wall_distance(self):
        """
        Calculates the distance to the wall on the right side using the costmap.
        This simulates a "virtual scan" into the costmap data.
        Returns the distance, or float('inf') if no wall is detected.
        """
        if self.local_costmap is None or self.costmap_metadata is None:
            return float('inf')

        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, self.robot_base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup for wall distance check failed: {e}")
            return float('inf')

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        # Use the corrected helper function call
        _, _, robot_theta = self.euler_from_quaternion(transform.transform.rotation)

        # Get costmap properties from metadata
        resolution = self.costmap_metadata.resolution
        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        width = self.costmap_metadata.width
        height = self.costmap_metadata.height

        # We will check a ray perpendicular to the robot's right side (-90 degrees)
        check_angle = self.normalize_angle(robot_theta - math.pi / 2)
        
        # Search for an obstacle along this ray from the robot's edge outwards
        max_search_dist = self.wall_distance_setpoint + 0.5 # Search a bit beyond the follow distance
        
        if resolution <= 0:
            self.get_logger().error("Costmap resolution is zero or negative, cannot perform wall distance check.")
            return float('inf')

        for step in range(1, int(max_search_dist / resolution)):
            dist = step * resolution
            # Point in costmap frame to check
            check_x = robot_x + dist * math.cos(check_angle)
            check_y = robot_y + dist * math.sin(check_angle)

            # Convert to map cell
            mx = int((check_x - origin_x) / resolution)
            my = int((check_y - origin_y) / resolution)

            if 0 <= mx < width and 0 <= my < height:
                index = my * width + mx
                cost = self.local_costmap[index]
                if cost >= self.costmap_inscribed_threshold: # Use inscribed threshold
                    # Found a wall, return the distance from the robot's center
                    return dist
            else:
                # Ray is outside the costmap, so no wall found in this direction
                return float('inf')
        
        # No wall found within the search distance
        return float('inf')

    def is_wall_on_right(self):
        """
        Checks if there is a wall on the right within a certain threshold using the costmap.
        """
        dist = self.get_side_wall_distance()
        # Check if the distance is less than the follow distance plus a threshold
        return dist < (self.wall_distance_setpoint + self.wall_presence_threshold)


    # --- Main Control Loop ---
    def control_loop(self):
        """ Main state machine and control logic """
        if self.current_odom_pose is None or self.local_costmap is None:
            self.get_logger().info("Waiting for odom and costmap data...", throttle_duration_sec=5)
            return

        # --- High-priority safety checks that can override the current state ---
        # 1. Safety Bubble (immediate stop)
        if self.check_safety_bubble():
            self.stop_robot()
            self.get_logger().error("EMERGENCY STOP: Safety bubble breached. Halting until manually cleared.")
            # This is a serious condition, so we stop and do not proceed.
            # A more advanced system might enter a special error state.
            return

        # 2. Stuck Detection
        if self.state not in [State.RECOVERY_BACKUP, State.RECOVERY_TURN] and self.is_stuck():
            self.get_logger().warn("Stuck detected, initiating recovery.")
            self.state = State.RECOVERY_BACKUP
            self.recovery_start_time = self.get_clock().now()
            # The recovery state will handle motion, so we can return here.
            # This prevents the old state's logic from running for one cycle.
            self.control_loop() # Re-run to immediately execute recovery logic
            return

        twist = Twist()

        # --- State-specific logic ---

        if self.state == State.FIND_WALL:
            # Use costmap to check for front obstacle
            if self.check_obstacle_in_front():
                self.get_logger().info("Obstacle detected while finding wall, turning left.")
                twist.angular.z = self.turn_speed
            else:
                # Move forward until a wall is on the right
                twist.linear.x = self.forward_speed
                # Use costmap to check for wall on the right
                if self.is_wall_on_right():
                    self.get_logger().info("Wall found on the right. Switching to FOLLOW_WALL.")
                    self.state = State.FOLLOW_WALL
                    self.stop_robot() # Stop briefly to ensure a clean transition
                else:
                    # Gently curve right to find a wall if none is in front
                    twist.angular.z = -self.turn_speed * 0.5

        elif self.state == State.FOLLOW_WALL:
            # Primary check: front obstacle from costmap
            if self.check_obstacle_in_front():
                self.get_logger().info("FOLLOW_WALL: Front obstacle detected. Switching to AVOID_OBSTACLE.")
                self.state = State.AVOID_OBSTACLE
                self.stop_robot() # Stop before turning
                return # Re-run control loop immediately in new state
            # Secondary check: inner corner collision during right turns
            elif self.check_for_corner_collision():
                 self.get_logger().info("FOLLOW_WALL: Inner corner collision detected. Overriding to turn left.")
                 twist.linear.x = 0.05 # Slow down
                 twist.angular.z = self.turn_speed # Turn left away from corner
            # Wall following logic
            else:
                wall_dist = self.get_side_wall_distance()
                if wall_dist == float('inf'):
                    # Wall lost, gently turn right to find it again
                    self.get_logger().info("Wall lost, curving right to find it.")
                    twist.linear.x = self.forward_speed * 0.75
                    twist.angular.z = -self.turn_speed * 0.5
                else:
                    # Wall is present, apply PID control to maintain distance
                    error = self.wall_distance_setpoint - wall_dist
                    twist.linear.x = self.forward_speed
                    # Use a proportional controller for angular velocity
                    twist.angular.z = self.kp_angular * error
                    # Clamp the angular velocity to the maximum
                    twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, twist.angular.z))


        elif self.state == State.AVOID_OBSTACLE:
            # Turn left until the way is clear according to the costmap
            twist.angular.z = self.turn_speed
            if not self.check_obstacle_in_front():
                self.get_logger().info("Path is clear. Switching back to FOLLOW_WALL.")
                self.state = State.FOLLOW_WALL
                self.stop_robot() # Stop to ensure clean transition
                return # Re-run control loop

        elif self.state == State.RECOVERY_BACKUP:
            # Move backward for a set duration
            duration_in_state = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
            if duration_in_state < self.recovery_backup_time:
                twist.linear.x = self.recovery_backup_speed
                twist.angular.z = 0.0
            else:
                self.get_logger().info("Backup complete. Switching to RECOVERY_TURN.")
                self.state = State.RECOVERY_TURN
                self.recovery_start_time = self.get_clock().now() # Reset timer for the turn
                self.stop_robot()
                return # Re-run control loop

        elif self.state == State.RECOVERY_TURN:
            # Turn for a set duration
            duration_in_state = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
            if duration_in_state < self.recovery_turn_time:
                twist.angular.z = self.turn_speed # Turn left
            else:
                self.get_logger().info("Recovery turn complete. Switching to FIND_WALL.")
                self.state = State.FIND_WALL
                self.stop_robot()
                return # Re-run control loop

        # Publish the final command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PerimeterRoamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.get_logger().info("Executing final shutdown...")
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
