import debugpy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations # sudo apt install python3-transforms3d
import math
from enum import Enum

from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray # For debugging

class State(Enum):
    FIND_WALL = 0
    TURN_LEFT = 1
    FOLLOW_WALL = 2
    NAVIGATE_DOORWAY = 3
    RECOVERY_BACKUP = 4
    RECOVERY_TURN = 5
    STOPPED = 6

class PerimeterRoamer(Node):
    def __init__(self):
        super().__init__('perimeter_roamer')

        self.get_logger().info("Perimeter Roamer Node Starting Up...")

        # === Parameters ===
        self.declare_parameters(
            namespace='',
            parameters=[
                ('forward_speed', 0.15),
                ('turn_speed', 0.4),
                ('max_angular_speed', 0.4),
                ('wall_follow_distance', 0.5),
                ('wall_dist_error_gain', 0.3),
                ('side_scan_angle_min_deg', -80.0),
                ('side_scan_angle_max_deg', -40.0),
                ('wall_presence_threshold', 0.3),
                ('front_obstacle_distance', 0.3),
                ('front_angle_width_deg', 20.0),
                ('costmap_check_distance', 0.2),
                ('costmap_check_points_front', 3),
                ('robot_width', 0.3),
                ('costmap_lethal_threshold', 253),
                ('control_frequency', 10.0),
                ('goal_tolerance_dist', 0.1),
                ('stuck_timeout', 5.0),
                ('stuck_distance_threshold', 0.02),
                ('stuck_angle_threshold_deg', 5.0),
                ('recovery_backup_dist', -0.1),
                ('recovery_backup_time', 1.5),
                ('recovery_turn_angle', 1.57),
                ('recovery_turn_time', 2.0),
                ('robot_base_frame', 'base_link'),
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),
                ('scan_topic', '/scan'),
                ('odom_topic', '/odom'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('local_costmap_topic', '/local_costmap/costmap'),
                # Doorway navigation parameters
                ('doorway_width_threshold', 0.8),
                ('doorway_approach_distance', 1.0),
                ('doorway_passage_speed', 0.1),
                ('narrow_passage_threshold', 0.6),
            ])

        # Read parameters
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value
        self.kp_angular = self.get_parameter('wall_dist_error_gain').value
        self.side_scan_angle_min = math.radians(self.get_parameter('side_scan_angle_min_deg').value)
        self.side_scan_angle_max = math.radians(self.get_parameter('side_scan_angle_max_deg').value)
        self.wall_presence_threshold = self.get_parameter('wall_presence_threshold').value
        self.front_obstacle_distance = self.get_parameter('front_obstacle_distance').value
        self.front_angle_width = math.radians(self.get_parameter('front_angle_width_deg').value)
        self.costmap_check_distance = self.get_parameter('costmap_check_distance').value
        self.costmap_check_points_front = self.get_parameter('costmap_check_points_front').value
        self.robot_width = self.get_parameter('robot_width').value
        self.costmap_lethal_threshold = self.get_parameter('costmap_lethal_threshold').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.goal_tolerance_dist = self.get_parameter('goal_tolerance_dist').value
        self.stuck_timeout_dur = Duration(seconds=self.get_parameter('stuck_timeout').value)
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        self.stuck_angle_threshold = math.radians(self.get_parameter('stuck_angle_threshold_deg').value)
        self.recovery_backup_dist = self.get_parameter('recovery_backup_dist').value
        self.recovery_backup_time = self.get_parameter('recovery_backup_time').value
        self.recovery_turn_angle = self.get_parameter('recovery_turn_angle').value
        self.recovery_turn_time = self.get_parameter('recovery_turn_time').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        
        # Doorway navigation parameters
        self.doorway_width_threshold = self.get_parameter('doorway_width_threshold').value
        self.doorway_approach_distance = self.get_parameter('doorway_approach_distance').value
        self.doorway_passage_speed = self.get_parameter('doorway_passage_speed').value
        self.narrow_passage_threshold = self.get_parameter('narrow_passage_threshold').value

        self.get_logger().info(f"Params: FollowDist={self.wall_follow_distance:.2f}, FwdSpeed={self.forward_speed:.2f}, TurnSpeed={self.turn_speed:.2f}")

        # === State & Data ===
        self.state = State.FIND_WALL
        self.last_scan = None
        self.current_odom_pose = None
        self.local_costmap = None
        self.costmap_metadata = None
        self.costmap_frame = ""

        # Stuck detection
        self.last_pose_check_time = self.get_clock().now()
        self.last_pose_for_stuck_check = None

        # Recovery state tracking
        self.recovery_start_time = None
        self.recovery_start_pose = None
        
        # Doorway navigation state
        self.doorway_target_angle = 0.0

        # === TF ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # === Publishers & Subscribers ===
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
        self.local_costmap = msg.data
        self.costmap_metadata = msg.info
        self.costmap_frame = msg.header.frame_id # Store the frame_id
        # self.get_logger().debug(f"Costmap updated: Res={msg.info.resolution:.3f}, W={msg.info.width}, H={msg.info.height}, Frame='{self.costmap_frame}'")


    # --- Utility Functions ---
    def get_scan_indices(self, angle_min_rad, angle_max_rad):
        """Calculates laser scan indices corresponding to angle ranges."""
        if not self.last_scan or self.last_scan.angle_increment == 0.0:
             return None, None

        angle_min_scan = self.last_scan.angle_min
        angle_max_scan = self.last_scan.angle_max
        angle_inc = self.last_scan.angle_increment

        # Ensure requested angles are within scan limits, handle angle wrap if necessary
        eff_angle_min = max(angle_min_rad, angle_min_scan)
        eff_angle_max = min(angle_max_rad, angle_max_scan)

        if eff_angle_max < eff_angle_min:
            # This could happen if the range crosses the 0/2pi boundary, handle if necessary
            # For typical side/front checks, this isn't usually an issue.
            return None, None

        start_index = int(math.floor((eff_angle_min - angle_min_scan) / angle_inc))
        end_index = int(math.ceil((eff_angle_max - angle_min_scan) / angle_inc))

        # Clamp indices to valid range
        start_index = max(0, start_index)
        end_index = min(len(self.last_scan.ranges) - 1, end_index)

        if start_index > end_index: return None, None

        return start_index, end_index

    def check_obstacle_in_scan_sector(self, angle_min_rad, angle_max_rad, max_distance):
        """ Checks for obstacles within a given scan sector and distance. Returns (obstacle_found, min_distance) """
        if not self.last_scan: return False, float('inf')

        start_idx, end_idx = self.get_scan_indices(angle_min_rad, angle_max_rad)
        if start_idx is None: return False, float('inf') # No points in angular range

        min_dist_found = float('inf')
        obstacle_detected = False

        # Ensure indices are within list bounds after calculation
        num_ranges = len(self.last_scan.ranges)
        start_idx = min(start_idx, num_ranges - 1)
        end_idx = min(end_idx, num_ranges - 1)

        relevant_ranges = self.last_scan.ranges[start_idx : end_idx + 1]
        for r in relevant_ranges:
            # Check for valid range reading
            if self.last_scan.range_min < r < self.last_scan.range_max:
                if r < max_distance:
                    obstacle_detected = True
                    min_dist_found = min(min_dist_found, r)

        return obstacle_detected, min_dist_found

    def get_distance_in_scan_sector(self, angle_min_rad, angle_max_rad):
        """ Calculates the average distance of valid points in a scan sector. Returns (valid_measurement, average_distance) """
        if not self.last_scan: return False, float('inf')

        start_idx, end_idx = self.get_scan_indices(angle_min_rad, angle_max_rad)
        if start_idx is None: return False, float('inf')

        distances = []
        num_ranges = len(self.last_scan.ranges)
        start_idx = min(start_idx, num_ranges - 1)
        end_idx = min(end_idx, num_ranges - 1)


        relevant_ranges = self.last_scan.ranges[start_idx : end_idx + 1]
        for r in relevant_ranges:
             # Use a slightly larger range than wall_follow_distance to ensure we detect the wall even if we are a bit far
            max_relevant_dist = self.wall_follow_distance + self.wall_presence_threshold + 0.2 # Be generous
            if self.last_scan.range_min < r < max_relevant_dist:
                distances.append(r)

        if not distances:
            return False, float('inf')

        # Optional: Could use median instead of average for robustness to outliers
        avg_dist = sum(distances) / len(distances)
        return True, avg_dist

    def check_costmap_for_collision(self):
        """ Checks the local costmap ahead of the robot for lethal obstacles. """
        if self.local_costmap is None or self.costmap_metadata is None or not self.costmap_frame:
            self.get_logger().warn("Costmap data not available for collision check.")
            return True # Assume collision if no data? Safer option.

        # Be less aggressive about costmap collisions in narrow spaces
        is_doorway, opening_width, _ = self.detect_doorway_ahead()
        if is_doorway or opening_width < self.narrow_passage_threshold:
            # In narrow spaces, only worry about truly lethal obstacles
            lethal_threshold = 254  # More permissive
            check_distance = 0.15   # Shorter look-ahead
        else:
            lethal_threshold = self.costmap_lethal_threshold
            check_distance = self.costmap_check_distance

        try:
            # Get the robot's current pose in the costmap frame
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, # Target frame
                self.robot_base_frame, # Source frame
                rclpy.time.Time(), # Get latest available
                timeout=Duration(seconds=0.1)) # Small timeout
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed from {self.robot_base_frame} to {self.costmap_frame}: {e}")
            return True # Assume collision if transform fails

        # Robot's position and orientation in costmap frame
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        q = transform.transform.rotation
        _, _, robot_theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Define check points relative to the robot's base_link frame
        check_points_base = []
        num_checks = self.costmap_check_points_front
        half_width = self.robot_width / 2.0

        if num_checks <= 1:
            check_points_base.append(Point(x=check_distance, y=0.0, z=0.0))
        else:
            for i in range(num_checks):
                y_offset = half_width - (i * self.robot_width / (num_checks - 1))
                check_points_base.append(Point(x=check_distance, y=y_offset, z=0.0))

        # Transform check points to costmap frame and check cost
        resolution = self.costmap_metadata.resolution
        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        width = self.costmap_metadata.width
        height = self.costmap_metadata.height

        markers = MarkerArray() # For visualization
        collision_detected = False

        for i, pt_base in enumerate(check_points_base):
            # Manual 2D transformation
            pt_map_x = robot_x + pt_base.x * math.cos(robot_theta) - pt_base.y * math.sin(robot_theta)
            pt_map_y = robot_y + pt_base.x * math.sin(robot_theta) + pt_base.y * math.cos(robot_theta)

            # Convert world coordinates (costmap frame) to map cell coordinates
            mx = int((pt_map_x - origin_x) / resolution)
            my = int((pt_map_y - origin_y) / resolution)

            marker = Marker()
            marker.header.frame_id = self.costmap_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "costmap_checks"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pt_map_x
            marker.pose.position.y = pt_map_y
            marker.pose.position.z = 0.1 # Slightly above ground
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.8

            # Check if cell is within map bounds
            if 0 <= mx < width and 0 <= my < height:
                index = my * width + mx
                try:
                    cost = self.local_costmap[index]
                    # self.get_logger().debug(f"Checking point ({pt_map_x:.2f}, {pt_map_y:.2f}) -> cell ({mx}, {my}), cost={cost}")

                    if cost >= lethal_threshold:
                        self.get_logger().warn(f"Costmap collision detected at cell ({mx}, {my}), cost={cost}")
                        collision_detected = True
                        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0 # Red
                    else:
                         marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 # Green

                except IndexError:
                    self.get_logger().warn(f"Costmap index out of bounds: ({mx}, {my}), index={index}, size={len(self.local_costmap)}")
                    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0 # Yellow (Error)
                    collision_detected = True # Treat index error as potential collision
            else:
                # Point is outside the local costmap - could be dangerous depending on setup
                self.get_logger().warn(f"Check point ({pt_map_x:.2f}, {pt_map_y:.2f}) outside local costmap bounds.")
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0 # Blue (Outside)
                # Decide if this constitutes a collision - maybe if robot is moving fast?
                # For now, let's NOT treat being outside the *local* costmap as collision,
                # as the robot might just be turning near the edge.
                # collision_detected = True

            markers.markers.append(marker)

        # Publish markers for debugging
        if markers.markers:
            self.marker_pub.publish(markers)

        return collision_detected

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

        curr_q = curr.orientation
        last_q = last.orientation
        # Ensure quaternions are valid
        if all(hasattr(q, attr) for q in [curr_q, last_q] for attr in ['x','y','z','w']):
            curr_yaw = tf_transformations.euler_from_quaternion([curr_q.x, curr_q.y, curr_q.z, curr_q.w])[2]
            last_yaw = tf_transformations.euler_from_quaternion([last_q.x, last_q.y, last_q.z, last_q.w])[2]
            angle_moved = abs(self.normalize_angle(curr_yaw - last_yaw))
        else:
            angle_moved = 0 # Cannot calculate if orientations are invalid

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

    def detect_doorway_ahead(self):
        """Detects if there's a doorway/opening ahead of the robot."""
        if not self.last_scan:
            return False, 0.0, 0.0
        
        # Look ahead at different angles to find an opening
        search_angles = []
        search_distance = self.doorway_approach_distance
        
        # Create array of angles to check (from left to right)
        num_checks = 20
        for i in range(num_checks):
            angle = math.radians(-45 + (i * 90.0 / (num_checks - 1)))  # -45° to +45°
            search_angles.append(angle)
        
        # Find the widest clear opening
        best_opening_width = 0.0
        best_opening_center = 0.0
        current_opening_start = None
        
        for angle in search_angles:
            # Check if this angle is clear
            obstacle_found, dist = self.check_obstacle_in_scan_sector(
                angle - math.radians(2), angle + math.radians(2), search_distance)
            
            if not obstacle_found:  # Clear space
                if current_opening_start is None:
                    current_opening_start = angle
            else:  # Obstacle found
                if current_opening_start is not None:
                    # End of opening - calculate width
                    opening_width = abs(angle - current_opening_start) * search_distance
                    if opening_width > best_opening_width:
                        best_opening_width = opening_width
                        best_opening_center = (current_opening_start + angle) / 2.0
                    current_opening_start = None
        
        # Check if we have a doorway
        is_doorway = best_opening_width > self.doorway_width_threshold
        return is_doorway, best_opening_width, best_opening_center

    # --- Main Control Loop ---
    def control_loop(self):
        if self.last_scan is None or self.current_odom_pose is None:
            self.get_logger().info("Waiting for sensor data (scan, odom)...", throttle_duration_sec=5)
            return

        # --- Pre-checks (Higher Priority) ---
        # 1. Costmap Collision Check (most reliable immediate obstacle check)
        if self.state not in [State.RECOVERY_BACKUP, State.RECOVERY_TURN, State.STOPPED]: # Don't override recovery
            if self.check_costmap_for_collision():
                self.get_logger().warn("Costmap reports imminent collision! Switching to TURN_LEFT.")
                self.state = State.TURN_LEFT
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed # Turn left away from obstacle
                self.cmd_vel_pub.publish(twist)
                return # Skip rest of loop

        # 2. Stuck Check
        if self.state not in [State.RECOVERY_BACKUP, State.RECOVERY_TURN, State.STOPPED]:
             if self.is_stuck():
                  self.get_logger().warn("Stuck detected. Initiating recovery.")
                  self.state = State.RECOVERY_BACKUP
                  self.recovery_start_time = self.get_clock().now()
                  self.recovery_start_pose = self.current_odom_pose # Store pose before backing up
                  # Stop robot before starting recovery sequence
                  self.stop_robot()
                  return


        # --- State Machine Logic ---
        twist = Twist()
        now = self.get_clock().now()

        if self.state == State.FIND_WALL:
            # Move forward until obstacle detected by SCAN (quicker check) or costmap (handled above)
            front_obstacle, dist = self.check_obstacle_in_scan_sector(
                -self.front_angle_width / 2.0, self.front_angle_width / 2.0, self.front_obstacle_distance)

            if front_obstacle:
                self.get_logger().info(f"Scan detected obstacle at {dist:.2f}m. Transitioning to TURN_LEFT.")
                self.state = State.TURN_LEFT
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed # Start turning left
            else:
                # Go forward if clear
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

        elif self.state == State.TURN_LEFT:
            # Turn left until front is clear and wall is detected on the right
            front_obstacle, _ = self.check_obstacle_in_scan_sector(
                 -self.front_angle_width / 2.0, self.front_angle_width / 2.0, self.front_obstacle_distance * 1.2) # Slightly larger distance to ensure clearance
            wall_on_right, right_dist = self.get_distance_in_scan_sector(self.side_scan_angle_min, self.side_scan_angle_max)

            # Check if costmap is also clear in front before proceeding
            costmap_clear = not self.check_costmap_for_collision()

            # Condition to switch to FOLLOW_WALL:
            # - Wall must be detected on the right within a reasonable range
            # - Front must be clear according to both scan and costmap
            if wall_on_right and (self.wall_follow_distance - self.goal_tolerance_dist < right_dist < self.wall_follow_distance + self.wall_presence_threshold) \
               and not front_obstacle and costmap_clear:
                self.get_logger().info(f"Aligned with wall on right (dist {right_dist:.2f}m). Transitioning to FOLLOW_WALL.")
                self.state = State.FOLLOW_WALL
                # Start moving forward slowly, wall following logic will adjust speed/angle
                twist.linear.x = self.forward_speed * 0.5
                twist.angular.z = 0.0
            else:
                # Continue turning left
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
                # Optional: If front becomes blocked while turning, maybe stop turning or reverse slightly?

        elif self.state == State.FOLLOW_WALL:
            # Check for doorway ahead
            is_doorway, opening_width, opening_center = self.detect_doorway_ahead()
            
            # Primary check: Front obstacle via SCAN
            front_obstacle, front_dist = self.check_obstacle_in_scan_sector(
                 -self.front_angle_width / 2.0, self.front_angle_width / 2.0, self.front_obstacle_distance)

            if front_obstacle and not is_doorway:
                # Regular obstacle (not a doorway)
                self.get_logger().info(f"Obstacle detected ahead ({front_dist:.2f}m) while following wall. Transitioning to TURN_LEFT.")
                self.state = State.TURN_LEFT
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
            elif is_doorway and opening_width > self.robot_width * 1.5:
                # Doorway detected and wide enough - navigate through it
                self.get_logger().info(f"Doorway detected! Width: {opening_width:.2f}m, Center: {math.degrees(opening_center):.1f}°")
                self.state = State.NAVIGATE_DOORWAY
                self.doorway_target_angle = opening_center
                # Slow down and aim for center of opening
                twist.linear.x = self.doorway_passage_speed
                twist.angular.z = opening_center * 0.5  # Gentle turn toward center
            else:
                # Normal wall following
                wall_on_right, right_dist = self.get_distance_in_scan_sector(self.side_scan_angle_min, self.side_scan_angle_max)

                if not wall_on_right or right_dist > self.wall_follow_distance + self.wall_presence_threshold:
                    # Lost the wall - turn right to find it
                    self.get_logger().warn("Lost wall on right. Turning right.")
                    twist.linear.x = self.forward_speed * 0.3
                    twist.angular.z = -self.turn_speed * 0.8
                else:
                    # Wall following with gentler control for tight spaces
                    error = self.wall_follow_distance - right_dist
                    
                    # Reduce gain in narrow spaces
                    if opening_width > 0 and opening_width < self.narrow_passage_threshold:
                        gain_factor = 0.3  # Much gentler control in narrow spaces
                        speed_factor = 0.5  # Slower in narrow spaces
                    else:
                        gain_factor = 1.0
                        speed_factor = 1.0
                    
                    twist.angular.z = self.kp_angular * error * gain_factor
                    twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, twist.angular.z))
                    
                    # Adjust speed based on turning
                    speed_reduction_factor = 1.0 - min(abs(twist.angular.z) / self.max_angular_speed, 0.7)
                    twist.linear.x = self.forward_speed * speed_reduction_factor * speed_factor

        elif self.state == State.NAVIGATE_DOORWAY:
            # Navigate through the doorway
            front_obstacle, front_dist = self.check_obstacle_in_scan_sector(
                -self.front_angle_width / 4.0, self.front_angle_width / 4.0, self.front_obstacle_distance)
            
            if front_obstacle and front_dist < 0.3:
                # Too close to obstacle while in doorway
                self.get_logger().warn("Obstacle too close while navigating doorway. Starting recovery.")
                self.state = State.RECOVERY_BACKUP
                self.recovery_start_time = self.get_clock().now()
                self.recovery_start_pose = self.current_odom_pose
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Continue through doorway
                is_doorway, opening_width, opening_center = self.detect_doorway_ahead()
                
                if is_doorway and opening_width > self.robot_width * 1.2:
                    # Still in doorway - aim for center
                    twist.linear.x = self.doorway_passage_speed
                    twist.angular.z = opening_center * 0.3  # Gentle correction
                else:
                    # Exited doorway - return to wall following
                    self.get_logger().info("Exited doorway. Returning to wall following.")
                    self.state = State.FOLLOW_WALL
                    twist.linear.x = self.forward_speed * 0.5
                    twist.angular.z = 0.0

        elif self.state == State.RECOVERY_BACKUP:
             if self.recovery_start_time is None: # Should have been set when entering state
                 self.get_logger().error("Recovery state entered without start time!")
                 self.state = State.FIND_WALL # Try to recover by finding wall again
                 return

             elapsed_time = (now - self.recovery_start_time).nanoseconds / 1e9

             if elapsed_time < self.recovery_backup_time:
                  # Command backup speed
                  twist.linear.x = self.recovery_backup_dist / self.recovery_backup_time # Average speed to cover distance
                  twist.angular.z = 0.0
                  self.get_logger().info(f"Recovery: Backing up ({elapsed_time:.1f}s / {self.recovery_backup_time:.1f}s)")
             else:
                  # Finished backing up, transition to recovery turn
                  self.get_logger().info("Recovery: Finished backup. Transitioning to RECOVERY_TURN.")
                  self.state = State.RECOVERY_TURN
                  self.recovery_start_time = now # Reset timer for turn phase
                  # Get current yaw to calculate target yaw
                  q = self.current_odom_pose.orientation
                  _, _, self.recovery_start_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                  self.recovery_target_yaw = self.normalize_angle(self.recovery_start_yaw + self.recovery_turn_angle) # Turn left
                  twist.linear.x = 0.0
                  twist.angular.z = self.turn_speed # Start turning


        elif self.state == State.RECOVERY_TURN:
            if self.recovery_start_time is None or self.recovery_start_yaw is None:
                 self.get_logger().error("Recovery turn state invalid.")
                 self.state = State.FIND_WALL
                 return

            elapsed_time = (now - self.recovery_start_time).nanoseconds / 1e9
            q = self.current_odom_pose.orientation
            _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            angle_diff = self.normalize_angle(self.recovery_target_yaw - current_yaw)

            # Check if turn completed or timed out
            if abs(angle_diff) < math.radians(5.0): # 5 degree tolerance
                 self.get_logger().info("Recovery: Turn completed.")
                 self.state = State.FIND_WALL # Go back to finding a wall
                 self.stop_robot() # Stop briefly before proceeding
                 # Reset stuck detection completely after recovery
                 self.last_pose_for_stuck_check = None
                 self.last_pose_check_time = self.get_clock().now()
                 return
            elif elapsed_time > self.recovery_turn_time:
                 self.get_logger().warn("Recovery: Turn timed out.")
                 self.state = State.FIND_WALL # Try finding wall anyway
                 self.stop_robot()
                 self.last_pose_for_stuck_check = None
                 self.last_pose_check_time = self.get_clock().now()
                 return
            else:
                 # Continue turning
                 twist.linear.x = 0.0
                 # Turn direction based on remaining angle (should be positive for left turn target)
                 twist.angular.z = self.turn_speed * (1.0 if angle_diff > 0 else -1.0)
                 self.get_logger().info(f"Recovery: Turning ({elapsed_time:.1f}s / {self.recovery_turn_time:.1f}s). Target: {math.degrees(self.recovery_target_yaw):.1f}, Current: {math.degrees(current_yaw):.1f}")


        elif self.state == State.STOPPED:
            # Do nothing, publish zero velocity
            pass # twist is already zero


        # --- Publish Command ---
        # Safety check: If any speed is NaN or Inf, stop the robot
        if not all(math.isfinite(v) for v in [twist.linear.x, twist.angular.z]):
             self.get_logger().error(f"Non-finite velocity command detected! State={self.state}. Stopping robot.")
             self.stop_robot()
             self.state = State.STOPPED # Go to stopped state on error
        else:
             self.cmd_vel_pub.publish(twist)


# --- Main Execution ---
def main(args=None):
    # debugpy.listen(("0.0.0.0", 5678))  # Listen on all interfaces
    # debugpy.wait_for_client()  # Optional: wait for debugger to attach
    rclpy.init(args=args)
    node = PerimeterRoamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping robot...")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure robot stops before shutting down
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    