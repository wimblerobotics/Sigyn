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
    NAVIGATE_DOORWAY = 5

class DoorwayStatus(Enum):
    """ Represents the result of a doorway check. """
    NONE = 0
    FULL_DOORWAY = 1  # Obstacles on both sides, forming a clear passage
    LEFT_WALL_ONLY = 2 # Obstacle on the left, open on the right
    RIGHT_WALL_ONLY = 3 # Obstacle on the right, open on the left


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
        # --- Parameters for Doorway Navigation ---
        self.declare_parameter('doorway_detection_width', 0.8) # Max width to be considered a doorway (m)
        self.declare_parameter('doorway_detection_skew', 0.2) # Skew for doorway detection
        self.declare_parameter('doorway_forward_speed', 0.15) # Slower speed for safety
        self.declare_parameter('doorway_kp_angular', 2.0) # Proportional gain for centering
        self.declare_parameter('doorway_scan_angle_start', 1.047) # 60 degrees
        self.declare_parameter('doorway_scan_angle_end', 2.094) # 120 degrees
        self.declare_parameter('doorway_aim_distance', 0.5) # How far ahead to project the target point
        self.declare_parameter('doorway_safety_bubble_distance', 0.15) # Smaller bubble for tight spaces
        self.declare_parameter('doorway_costmap_check_distance', 0.25) # Shorter lookahead in doorways
        self.declare_parameter('doorway_cost_threshold', 180) # Higher cost threshold for traversing inflated areas in doorways
        self.declare_parameter('doorway_max_angle_for_full_speed', math.radians(45)) # Max angle for full speed in doorway (radians)
        self.declare_parameter('doorway_commit_time', 2.0) # How long to commit to a doorway before re-checking

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

        # --- Initialize state variables that are used in logging ---
        self.costmap_frame = "" # Will be populated from costmap message header

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
        # --- Get Doorway Navigation Parameters ---
        self.doorway_detection_width = self.get_parameter('doorway_detection_width').get_parameter_value().double_value
        self.doorway_detection_skew = self.get_parameter('doorway_detection_skew').get_parameter_value().double_value
        self.doorway_forward_speed = self.get_parameter('doorway_forward_speed').get_parameter_value().double_value
        self.doorway_kp_angular = self.get_parameter('doorway_kp_angular').get_parameter_value().double_value
        self.doorway_scan_angle_start = self.get_parameter('doorway_scan_angle_start').get_parameter_value().double_value
        self.doorway_scan_angle_end = self.get_parameter('doorway_scan_angle_end').get_parameter_value().double_value
        self.doorway_aim_distance = self.get_parameter('doorway_aim_distance').get_parameter_value().double_value
        self.doorway_safety_bubble_distance = self.get_parameter('doorway_safety_bubble_distance').get_parameter_value().double_value
        self.doorway_costmap_check_distance = self.get_parameter('doorway_costmap_check_distance').get_parameter_value().double_value
        self.doorway_cost_threshold = self.get_parameter('doorway_cost_threshold').get_parameter_value().integer_value
        self.doorway_max_angle_for_full_speed = self.get_parameter('doorway_max_angle_for_full_speed').get_parameter_value().double_value
        self.doorway_commit_time = self.get_parameter('doorway_commit_time').get_parameter_value().double_value # How long to commit to a doorway before re-checking

        self.get_logger().info('--- Loaded Parameters ---')
        self.get_logger().info(f"Robot Base Frame: {self.robot_base_frame}, Odom Frame: {self.odom_frame}")
        self.get_logger().info(f"Speeds: Forward={self.forward_speed}, Turn={self.turn_speed}, MaxAngular={self.max_angular_speed}")
        self.get_logger().info(f"Wall Following: Distance={self.wall_distance_setpoint}, Gain={self.wall_following_gain}, Kp={self.kp_angular}")
        self.get_logger().info(f"Obstacle Avoidance: FrontDist(Legacy)={self.obstacle_distance_threshold}")
        self.get_logger().info(f"Corner Avoidance: Lookahead={self.corner_lookahead_distance}m, Angle={math.degrees(self.corner_lookahead_angle)}deg")
        self.get_logger().info(f"Costmap: CheckDist={self.costmap_check_distance:.2f}, LethalThreshold={self.costmap_lethal_threshold}, InscribedThreshold={self.costmap_inscribed_threshold}")
        self.get_logger().info(f"Robot: BaseFrame='{self.robot_base_frame}', Radius={self.robot_radius}m, SafetyBubble={self.safety_bubble_radius}m, Width={self.robot_width}m")
        self.get_logger().info(f"Topics: cmd_vel='{self.cmd_vel_topic}', costmap='{self.local_costmap_topic}'")
        self.get_logger().info(f"Stuck Detection: Timeout={stuck_timeout_sec}s, Dist={self.stuck_distance_threshold}m, Angle={math.degrees(self.stuck_angle_threshold)}deg")
        self.get_logger().info(f"Recovery: BackupTime={self.recovery_backup_time}s, BackupSpeed={self.recovery_backup_speed}m/s, TurnTime={self.recovery_turn_time}s")
        self.get_logger().info(f"Doorway Nav: DetectWidth={self.doorway_detection_width}m, Speed={self.doorway_forward_speed}m/s, Kp={self.doorway_kp_angular}")
        self.get_logger().info(f"Doorway Scan: Start={math.degrees(self.doorway_scan_angle_start)}deg, End={math.degrees(self.doorway_scan_angle_end)}deg, AimDist={self.doorway_aim_distance}m")
        self.get_logger().info(f"Doorway Safety: BubbleDist={self.doorway_safety_bubble_distance}m, CheckDist={self.doorway_costmap_check_distance}m, CostThreshold={self.doorway_cost_threshold}")
        self.get_logger().info(f"Doorway Turn: MaxAngleForFullSpeed={math.degrees(self.doorway_max_angle_for_full_speed)}deg")
        self.get_logger().info(f"Doorway Commit Time: {self.doorway_commit_time}s")
        self.get_logger().info('-------------------------')

        # --- State Variables ---
        self.state = State.FIND_WALL
        self.last_scan = None # Still useful for potential future logic or debugging
        self.current_odom_pose = None
        self.local_costmap = None
        self.costmap_metadata = None
        # self.costmap_frame has been moved up to be initialized before logging statements

        # Stuck detection variables
        self.last_pose_for_stuck_check = None
        self.last_pose_check_time = self.get_clock().now()
        self.last_state_change_time = self.get_clock().now() # For state timeouts

        # Recovery variables
        self.recovery_start_time = None
        # Doorway navigation variables
        self.doorway_target_point = None # The point to aim for in the doorway
        self.doorway_status = DoorwayStatus.NONE
        self.doorway_target_point = None # Will be a Point msg
        self.doorway_entry_time = None # Timestamp when we enter the doorway state

        # For stuck detection
        self.last_pose_for_stuck_check = None
        self.last_pose_check_time = self.get_clock().now()

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

    def change_state(self, new_state):
        """ Handles state transitions and associated logic. """
        if self.state == new_state:
            return # No change

        self.get_logger().info(f"STATE CHANGE: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.recovery_start_time = None # Reset recovery timer on any state change

        # Special handling for entering NAVIGATE_DOORWAY
        if new_state == State.NAVIGATE_DOORWAY:
            self.doorway_entry_time = self.get_clock().now()
            self.get_logger().info(f"Committing to doorway navigation for {self.doorway_commit_time}s.")


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

        # --- State-dependent check distance ---
        check_dist = self.doorway_costmap_check_distance if self.state == State.NAVIGATE_DOORWAY else self.costmap_check_distance
        # --- State-dependent cost threshold ---
        collision_threshold = self.doorway_cost_threshold if self.state == State.NAVIGATE_DOORWAY else self.costmap_inscribed_threshold


        # Define check points relative to the robot's base_link frame
        check_points_base = []
        num_checks = self.costmap_check_points_front
        half_width = self.robot_width / 2.0

        # Create a line of check points in front of the robot
        if num_checks <= 1:
            check_points_base.append(Point(x=check_dist, y=0.0, z=0.0))
        else:
            for i in range(num_checks):
                y_offset = half_width - (i * self.robot_width / (num_checks - 1))
                check_points_base.append(Point(x=check_dist, y=y_offset, z=0.0))

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
                if cost >= collision_threshold:
                    self.get_logger().warning(f"Collision check: Obstacle detected at costmap cell ({mx},{my}) with cost {cost} (threshold: {collision_threshold})")
                    return True # Collision detected
            else:
                # Point is outside the local costmap, which is risky.
                self.get_logger().warning("Collision check: Check point is outside local costmap bounds.")
                return True # Treat as collision

        return False # Path is clear

    def is_path_clear(self, linear_vel, angular_vel, state_name_for_log=""):
        """
        Predicts the robot's path for a short duration and checks for collisions along it.
        Returns True if the path is clear, False otherwise.
        """
        if self.local_costmap is None or self.costmap_metadata is None or not self.costmap_frame:
            self.get_logger().warn(f"({state_name_for_log}) Path check skipped: No costmap.", throttle_duration_sec=5)
            return False # Assume not clear if no data

        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, self.robot_base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"({state_name_for_log}) TF lookup for path check failed: {e}")
            return False

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        _, _, robot_theta = self.euler_from_quaternion(transform.transform.rotation)

        resolution = self.costmap_metadata.resolution
        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        width = self.costmap_metadata.width
        height = self.costmap_metadata.height

        # State-dependent thresholds
        collision_threshold = self.doorway_cost_threshold if self.state == State.NAVIGATE_DOORWAY else self.costmap_inscribed_threshold
        prediction_time = 0.5 # How far in the future to check (seconds)
        num_steps = 5 # Number of points to check along the path

        for i in range(1, num_steps + 1):
            dt = (prediction_time / num_steps) * i
            
            # Simple kinematic model for path prediction (arc)
            if abs(angular_vel) > 1e-3:
                radius = linear_vel / angular_vel
                d_theta = angular_vel * dt
                dx_robot = radius * math.sin(d_theta)
                dy_robot = radius * (1 - math.cos(d_theta))
            else: # Straight line
                d_theta = 0
                dx_robot = linear_vel * dt
                dy_robot = 0

            # Transform predicted point from robot frame to costmap frame
            pt_map_x = robot_x + (dx_robot * math.cos(robot_theta) - dy_robot * math.sin(robot_theta))
            pt_map_y = robot_y + (dx_robot * math.sin(robot_theta) + dy_robot * math.cos(robot_theta))

            # Convert to map cell and check cost
            mx = int((pt_map_x - origin_x) / resolution)
            my = int((pt_map_y - origin_y) / resolution)

            if 0 <= mx < width and 0 <= my < height:
                index = my * width + mx
                cost = self.local_costmap[index]
                if cost >= collision_threshold:
                    self.get_logger().warning(f"({state_name_for_log}) Predicted collision at cell ({mx},{my}), cost={cost}, threshold={collision_threshold}")
                    return False
            else:
                self.get_logger().warning(f"({state_name_for_log}) Predicted path goes off costmap.")
                return False # Path goes into unknown territory

        return True # Path is clear

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

        # --- State-dependent safety bubble radius ---
        bubble_radius = self.doorway_safety_bubble_distance if self.state == State.NAVIGATE_DOORWAY else self.safety_bubble_radius
        # --- State-dependent cost threshold ---
        collision_threshold = self.doorway_cost_threshold if self.state == State.NAVIGATE_DOORWAY else self.costmap_inscribed_threshold

        if self.state == State.NAVIGATE_DOORWAY:
            self.get_logger().debug(f"Using smaller doorway safety bubble: {bubble_radius}m and threshold: {collision_threshold}")


        # Check a circular region around the robot.
        check_radius_in_cells = int(bubble_radius / resolution)
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
                    if cost >= collision_threshold: # Use state-dependent threshold
                        self.get_logger().fatal(
                            f"SAFETY BUBBLE BREACHED! Obstacle with cost ({cost}) at cell ({mx}, {my}) vs threshold ({collision_threshold})."
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
        # --- State-dependent cost threshold ---
        collision_threshold = self.doorway_cost_threshold if self.state == State.NAVIGATE_DOORWAY else self.costmap_inscribed_threshold


        mx = int((pt_map_x - origin_x) / resolution)
        my = int((pt_map_y - origin_y) / resolution)

        if 0 <= mx < width and 0 <= my < height:
            index = my * width + mx
            cost = self.local_costmap[index]
            if cost >= collision_threshold: # Use state-dependent threshold
                self.get_logger().warn(f"Potential corner collision DETECTED via costmap at cell ({mx}, {my}), cost={cost}, threshold={collision_threshold}")
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

    def get_obstacle_distance_at_angle(self, robot_pose_stamped, angle_rad, collision_threshold_override=None):
        """
        Calculates distance to the first obstacle along a ray at a specific angle
        relative to the robot's forward direction. Reuses a given transform.
        Returns a tuple of (distance, point_in_costmap_frame) or (inf, None).
        """
        if self.local_costmap is None or self.costmap_metadata is None:
            return float('inf'), None

        robot_x = robot_pose_stamped.transform.translation.x
        robot_y = robot_pose_stamped.transform.translation.y
        _, _, robot_theta = self.euler_from_quaternion(robot_pose_stamped.transform.rotation)

        resolution = self.costmap_metadata.resolution
        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        width = self.costmap_metadata.width
        height = self.costmap_metadata.height
        
        # --- State-dependent cost threshold ---
        if collision_threshold_override is not None:
            collision_threshold = collision_threshold_override
        else:
            collision_threshold = self.doorway_cost_threshold if self.state == State.NAVIGATE_DOORWAY else self.costmap_inscribed_threshold


        # The absolute angle in the costmap frame
        check_angle = self.normalize_angle(robot_theta + angle_rad)
        max_search_dist = self.doorway_detection_width * 1.5 # Search further than the expected width

        if resolution <= 0: return float('inf'), None

        for step in range(1, int(max_search_dist / resolution)):
            dist = step * resolution
            check_x = robot_x + dist * math.cos(check_angle)
            check_y = robot_y + dist * math.sin(check_angle)

            mx = int((check_x - origin_x) / resolution)
            my = int((check_y - origin_y) / resolution)

            if 0 <= mx < width and 0 <= my < height:
                index = my * width + mx
                if self.local_costmap[index] >= collision_threshold:
                    # Found an obstacle at this distance
                    point = Point(x=check_x, y=check_y, z=0.0)
                    return dist, point
            else:
                # Ray has gone off the map, stop searching along this ray
                return float('inf'), None

        return float('inf'), None

    def scan_arc_for_obstacle(self, robot_pose_stamped, start_angle, end_angle, num_rays=5, collision_threshold_override=None):
        """
        Scans an arc for the nearest obstacle point.
        Returns the point in the costmap frame or None.
        """
        min_dist = float('inf')
        nearest_point = None
        
        if num_rays < 2: num_rays = 2 # Need at least 2 rays for an arc

        for i in range(num_rays):
            angle = start_angle + (end_angle - start_angle) * i / (num_rays -1)
            dist, point = self.get_obstacle_distance_at_angle(robot_pose_stamped, angle, collision_threshold_override)
            if dist < min_dist:
                min_dist = dist
                nearest_point = point
        
        return nearest_point

    def get_side_wall_distance(self, side='right'):
        """
        Calculates the distance to a wall on the specified side using the costmap.
        'side' can be 'right' or 'left'.
        Returns the distance, or float('inf') if no wall is detected.
        """
        if self.local_costmap is None or self.costmap_metadata is None:
            return float('inf')

        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, self.robot_base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup for side wall check failed: {e}")
            return float('inf')

        if side == 'right':
            angle = -math.pi / 2 # -90 degrees
        else: # left
            angle = math.pi / 2 # +90 degrees
        
        dist, _ = self.get_obstacle_distance_at_angle(transform, angle)
        return dist


    def is_wall_on_right(self):
        """
        Checks if there is a wall on the right within a certain threshold using the costmap.
        """
        dist = self.get_side_wall_distance(side='right')
        # Check if the distance is less than the follow distance plus a threshold
        return dist < (self.wall_distance_setpoint + self.wall_presence_threshold)

    def check_for_doorway(self):
        """
        Detects if the robot is in a narrow corridor or doorway by scanning arcs.
        If a doorway is found, it calculates and stores the target point to aim for.
        This logic can handle detecting both sides of a passage, or just one side ("skimming").
        Returns a tuple (DoorwayStatus, target_point), where target_point is a Point msg.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, self.robot_base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup for doorway check failed: {e}")
            return DoorwayStatus.NONE, None

        # When checking for a doorway, use the higher threshold to see past inflation
        doorway_check_threshold = self.doorway_cost_threshold

        # Scan for an obstacle on the left side (e.g., from 45 to 135 degrees)
        left_obstacle_point = self.scan_arc_for_obstacle(
            transform,
            self.doorway_scan_angle_start,      # e.g., 45 deg
            self.doorway_scan_angle_end,        # e.g., 135 deg
            num_rays=5, # More rays for a wider arc
            collision_threshold_override=doorway_check_threshold
        )

        # Scan for an obstacle on the right side (e.g., from -135 to -45 degrees)
        right_obstacle_point = self.scan_arc_for_obstacle(
            transform,
            -self.doorway_scan_angle_end,       # e.g., -135 deg
            -self.doorway_scan_angle_start,     # e.g., -45 deg
            num_rays=5, # More rays for a wider arc
            collision_threshold_override=doorway_check_threshold
        )

        # Get robot's current orientation for target point calculation
        _, _, robot_theta = self.euler_from_quaternion(transform.transform.rotation)
        
        # Case 1: Both sides detected (classic doorway)
        if left_obstacle_point and right_obstacle_point:
            dist = math.sqrt(
                (left_obstacle_point.x - right_obstacle_point.x)**2 +
                (left_obstacle_point.y - right_obstacle_point.y)**2
            )
            self.get_logger().debug(f"Doorway check (Both Sides): LeftPt=({left_obstacle_point.x:.2f}, {left_obstacle_point.y:.2f}), RightPt=({right_obstacle_point.x:.2f}, {right_obstacle_point.y:.2f}), MeasuredWidth={dist:.2f}m")

            if dist < self.doorway_detection_width:
                # Midpoint between the two detected obstacles
                mid_x = (left_obstacle_point.x + right_obstacle_point.x) / 2.0
                mid_y = (left_obstacle_point.y + right_obstacle_point.y) / 2.0
                target_point = Point(x=mid_x, y=mid_y, z=0.0)
                self.get_logger().info(f"Doorway detected (Centering). Width: {dist:.2f}m. Target: ({mid_x:.2f}, {mid_y:.2f})")
                return DoorwayStatus.FULL_DOORWAY, target_point
        
        # Case 2: Only one side is detected (skimming behavior)
        elif right_obstacle_point:
            # This logic allows the robot to navigate around a corner into a doorway
            # by "skimming" the one wall it can see.
            # Wall is on the right, so we want to aim to the left of it.
            # The offset is perpendicular to the robot's heading (robot_theta + pi/2)
            offset_angle = robot_theta + math.pi / 2.0
            target_x = right_obstacle_point.x + self.doorway_detection_skew * math.cos(offset_angle)
            target_y = right_obstacle_point.y + self.doorway_detection_skew * math.sin(offset_angle)
            target_point = Point(x=target_x, y=target_y, z=0.0)
            self.get_logger().info(f"Doorway detected (Skimming Right Wall). Target: ({target_x:.2f}, {target_y:.2f})")
            return DoorwayStatus.RIGHT_WALL_ONLY, target_point

        elif left_obstacle_point:
            # Wall is on the left, so we want to aim to the right of it.
            # The offset is perpendicular to the robot's heading (robot_theta - pi/2)
            offset_angle = robot_theta - math.pi / 2.0
            target_x = left_obstacle_point.x + self.doorway_detection_skew * math.cos(offset_angle)
            target_y = left_obstacle_point.y + self.doorway_detection_skew * math.sin(offset_angle)
            target_point = Point(x=target_x, y=target_y, z=0.0)
            self.get_logger().info(f"Doorway detected (Skimming Left Wall). Target: ({target_x:.2f}, {target_y:.2f})")
            return DoorwayStatus.LEFT_WALL_ONLY, target_point

        # No passage detected
        return DoorwayStatus.NONE, None


    # --- Main Control Loop ---
    def control_loop(self):
        """ Main state machine and control logic """
        if self.current_odom_pose is None or self.local_costmap is None:
            self.get_logger().info("Waiting for odom and costmap data...", throttle_duration_sec=5)
            return

        # --- High-priority safety checks that can override the current state ---
        # 1. Safety Bubble (immediate stop)
        if self.check_safety_bubble():
            self.change_state(State.RECOVERY_BACKUP)
            self.stop_robot() # Stop immediately
            return

        # 2. Stuck Detection
        if self.state not in [State.RECOVERY_BACKUP, State.RECOVERY_TURN] and self.is_stuck():
            self.change_state(State.RECOVERY_BACKUP)
            return

        # --- State Machine Logic ---
        twist = Twist()

        # --- State: FIND_WALL ---
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
                    self.change_state(State.FOLLOW_WALL)
                    self.stop_robot() # Stop briefly to ensure a clean transition
                else:
                    # Gently curve right to find a wall if none is in front
                    twist.angular.z = -self.turn_speed * 0.5

        # --- State: FOLLOW_WALL ---
        elif self.state == State.FOLLOW_WALL:
            # Highest priority in this state: check for a doorway to navigate through
            doorway_status, doorway_target = self.check_for_doorway()
            if doorway_status != DoorwayStatus.NONE:
                self.get_logger().info("FOLLOW_WALL: Doorway detected ahead. Switching to NAVIGATE_DOORWAY.")
                self.doorway_status = doorway_status
                self.doorway_target_point = doorway_target
                self.change_state(State.NAVIGATE_DOORWAY)
                self.stop_robot()
                return # Re-run loop in new state

            # Primary check: front obstacle from costmap
            if self.check_obstacle_in_front():
                self.get_logger().info("FOLLOW_WALL: Front obstacle detected. Switching to AVOID_OBSTACLE.")
                self.change_state(State.AVOID_OBSTACLE)
                self.stop_robot() # Stop before turning
                return # Re-run control loop immediately in new state
            # Secondary check: inner corner collision during right turns
            elif self.check_for_corner_collision():
                 self.get_logger().info("FOLLOW_WALL: Inner corner collision detected. Overriding to turn left.")
                 twist.linear.x = 0.05 # Slow down
                 twist.angular.z = self.turn_speed # Turn left away from corner
            # Wall following logic
            else:
                wall_dist = self.get_side_wall_distance(side='right')
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


        # --- State: AVOID_OBSTACLE ---
        elif self.state == State.AVOID_OBSTACLE:
            # Turn left until the way is clear according to the costmap
            twist.angular.z = self.turn_speed
            if not self.check_obstacle_in_front():
                self.get_logger().info("Path is clear. Switching back to FOLLOW_WALL.")
                self.change_state(State.FOLLOW_WALL)
                self.stop_robot() # Stop to ensure clean transition
                return # Re-run control loop

        # --- State: NAVIGATE_DOORWAY ---
        elif self.state == State.NAVIGATE_DOORWAY:
            # This state is now entered if check_for_doorway finds any kind of opening.
            # The target point and status were already set when transitioning to this state.

            # Check if we should re-evaluate the doorway presence.
            # We commit for a certain time to avoid flickering.
            time_in_state = (self.get_clock().now() - self.doorway_entry_time).nanoseconds / 1e9
            
            # During the initial commit period, we don't re-evaluate. We trust the initial target.
            # This prevents flickering if the robot's orientation causes the detection to change slightly.
            if time_in_state > self.doorway_commit_time:
                self.get_logger().debug("Doorway commit time elapsed. Now dynamically updating target.")
                new_status, new_target = self.check_for_doorway()

                if new_status == DoorwayStatus.NONE:
                    self.get_logger().info("Lost sight of doorway/opening after commit time. Returning to FOLLOW_WALL.")
                    self.change_state(State.FOLLOW_WALL)
                    return
                else:
                    # After the commit time, we continuously update the target for a smoother path.
                    if new_status != self.doorway_status:
                         self.get_logger().info(f"Doorway status updated while navigating: {self.doorway_status.name} -> {new_status.name}")
                    self.doorway_status = new_status
                    self.doorway_target_point = new_target

            # If an immediate forward obstacle appears, abort and recover.
            if self.check_obstacle_in_front():
                 self.get_logger().warn("Obstacle detected while navigating doorway. Aborting to recovery.")
                 self.change_state(State.RECOVERY_BACKUP)
                 return

            # Aim for the calculated target point (midpoint, or skewed point)
            if self.doorway_target_point is None:
                self.get_logger().error("In NAVIGATE_DOORWAY but target point is None. Aborting to recovery.")
                self.change_state(State.RECOVERY_BACKUP)
                return

            try:
                # We need the robot's pose in the costmap frame to calculate the angle to the target
                transform = self.tf_buffer.lookup_transform(self.costmap_frame, self.robot_base_frame, rclpy.time.Time())
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y
                _, _, robot_theta = self.euler_from_quaternion(transform.transform.rotation)

                # Angle to the target point
                angle_to_target = math.atan2(self.doorway_target_point.y - robot_y, self.doorway_target_point.x - robot_x)
                angle_error = self.normalize_angle(angle_to_target - robot_theta)

                # Proportional control for angular velocity
                angular_vel = self.doorway_kp_angular * angle_error

                # Set a constant forward speed for the doorway, but slow down for sharp turns
                if abs(angle_error) > self.doorway_max_angle_for_full_speed:
                    # Scale speed down if the required turn is sharp
                    scale_factor = self.doorway_max_angle_for_full_speed / abs(angle_error)
                    linear_vel = self.doorway_forward_speed * scale_factor
                    self.get_logger().debug(f"Doorway nav: Sharp turn detected (error: {math.degrees(angle_error):.1f}deg). Scaling speed to {linear_vel:.2f}m/s.")
                else:
                    linear_vel = self.doorway_forward_speed


                # Safety check before publishing
                if self.is_path_clear(linear_vel, angular_vel, "NAVIGATE_DOORWAY"):
                    twist.linear.x = linear_vel
                    twist.angular.z = angular_vel
                else:
                    self.get_logger().warn("Path is not clear to the doorway target. Aborting to recovery.")
                    self.stop_robot() # Explicitly stop before changing state
                    self.change_state(State.RECOVERY_BACKUP)
                    twist = Twist() # Ensure we stop if path is not clear

                status_str = self.doorway_status.name
                self.get_logger().info(f"NAV_DOOR [{status_str}]: AngleErr={math.degrees(angle_error):.1f}, LinVel={twist.linear.x:.2f}, AngVel={twist.angular.z:.2f}")

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f"TF lookup failed in NAVIGATE_DOORWAY: {e}")
                self.stop_robot()

        # --- State: RECOVERY_BACKUP ---
        elif self.state == State.RECOVERY_BACKUP:
            now = self.get_clock().now()
            # If this is the first tick in this state, start the timer
            if self.recovery_start_time is None:
                self.get_logger().info(f"Starting recovery: Backing up for {self.recovery_backup_time} seconds.")
                self.recovery_start_time = now

            # Check if backup time has elapsed
            if (now - self.recovery_start_time).nanoseconds / 1e9 > self.recovery_backup_time:
                self.get_logger().info("Backup complete. Switching to RECOVERY_TURN.")
                self.change_state(State.RECOVERY_TURN)
                self.stop_robot()
                return
            else:
                # Command the robot to move backward
                twist.linear.x = self.recovery_backup_speed

        # --- State: RECOVERY_TURN ---
        elif self.state == State.RECOVERY_TURN:
            now = self.get_clock().now()
            # If this is the first tick in this state, start the timer
            if self.recovery_start_time is None:
                self.get_logger().info(f"Starting recovery: Turning for {self.recovery_turn_time} seconds.")
                self.recovery_start_time = now

            # Check if turn time has elapsed
            if (now - self.recovery_start_time).nanoseconds / 1e9 > self.recovery_turn_time:
                self.get_logger().info("Recovery turn complete. Switching to FIND_WALL to restart logic.")
                self.change_state(State.FIND_WALL)
                self.stop_robot()
                return
            else:
                # Command the robot to turn (e.g., left)
                twist.angular.z = self.turn_speed


        # --- Publish the final command ---
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
