import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor

import math
import numpy as np
from enum import Enum

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformException
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Twist, Point, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class State(Enum):
    """Defines the robot's operational states."""
    IDLE = 0
    FIND_WALL = 1
    PATROL_PERIMETER = 2
    NAVIGATE_HALLWAY = 3
    APPROACH_DOORWAY = 4
    EXECUTE_TURN = 5
    TRAVERSE_DOORWAY = 6
    RECOVERY = 7


class RoamingNodeV3(Node):
    """
    A robust autonomous navigation node for patrolling a home environment,
    with specialized logic for hallway and doorway traversal.
    """

    def __init__(self):
        super().__init__('roaming_node_v3')
        self.get_logger().info("Sigyn House Patroller v3 is starting up...")

        self._declare_parameters()
        self._init_variables()
        self._init_ros_comms()
        self._wait_for_transforms()

        self.get_logger().info("RoamingNodeV3 initialized successfully.")

    def _declare_parameters(self):
        """Declares and gets all ROS parameters for easy tuning."""
        self.declare_parameter('control_frequency', 10.0, ParameterDescriptor(description='Main control loop frequency in Hz.'))
        
        # Speeds
        self.declare_parameter('patrol_speed', 0.15, ParameterDescriptor(description='Speed for open space patrol.'))
        self.declare_parameter('hallway_speed', 0.1, ParameterDescriptor(description='Speed for navigating narrow hallways.'))
        self.declare_parameter('doorway_speed', 0.08, ParameterDescriptor(description='Speed for traversing doorways.'))
        self.declare_parameter('turn_speed', 0.35, ParameterDescriptor(description='Angular speed for turns.'))
        self.declare_parameter('recovery_speed', -0.1, ParameterDescriptor(description='Speed for backing up during recovery.'))

        # Distances & Thresholds
        self.declare_parameter('patrol_wall_distance', 1.0, ParameterDescriptor(description='Desired distance from wall in open spaces.'))
        self.declare_parameter('min_obstacle_distance', 0.35, ParameterDescriptor(description='Minimum distance to an obstacle to trigger avoidance.'))
        self.declare_parameter('hallway_width_threshold', 1.5, ParameterDescriptor(description='Max width to be considered a hallway.'))
        self.declare_parameter('hallway_wall_distance', 0.5, ParameterDescriptor(description='Desired distance from wall in hallways.'))
        self.declare_parameter('doorway_opening_threshold', 1.8, ParameterDescriptor(description='Min side distance to be considered a doorway opening.'))
        self.declare_parameter('doorway_traverse_distance', 1.2, ParameterDescriptor(description='Distance to move forward to clear a doorway.'))

        # PID Controllers
        self.declare_parameter('wall_follow_kp', 1.0, ParameterDescriptor(description='Proportional gain for wall following.'))
        self.declare_parameter('centering_kp', 1.8, ParameterDescriptor(description='Proportional gain for hallway centering.'))

        # Timers & Tolerances
        self.declare_parameter('stuck_timeout', 8.0, ParameterDescriptor(description='Time in seconds of no movement to be considered stuck.'))
        self.declare_parameter('stuck_distance_threshold', 0.05, ParameterDescriptor(description='Distance moved threshold for stuck detection.'))
        self.declare_parameter('recovery_backup_duration', 2.0, ParameterDescriptor(description='Duration for backing up in recovery.'))
        self.declare_parameter('recovery_turn_duration', 2.5, ParameterDescriptor(description='Duration for turning in recovery.'))

        # Frames
        self.declare_parameter('robot_base_frame', 'base_link', ParameterDescriptor(description='The robot\'s base frame.'))
        self.declare_parameter('odom_frame', 'odom', ParameterDescriptor(description='The odometry frame.'))

    def _init_variables(self):
        """Initializes node variables and gets parameters."""
        # State Machine
        self.state = State.IDLE
        self.state_start_time = self.get_clock().now()
        self.target_yaw = 0.0
        self.start_pose = None
        self.turn_direction = 0 # 1 for left, -1 for right

        # Get Parameters
        for param_name in self._get_parameter_names():
            setattr(self, param_name, self.get_parameter(param_name).value)

        self.hallway_wall_distance = self.get_parameter('hallway_wall_distance').value

        # Sensor data
        self.last_scan = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.last_pose_stuck_check = None
        self.last_pose_stuck_time = self.get_clock().now()

        self.get_logger().info("Parameters loaded and variables initialized.")

    def _get_parameter_names(self):
        # Helper to avoid repeating parameter names
        return [
            'control_frequency', 'patrol_speed', 'hallway_speed', 'doorway_speed',
            'turn_speed', 'recovery_speed', 'patrol_wall_distance', 'min_obstacle_distance',
            'hallway_width_threshold', 'doorway_opening_threshold', 'doorway_traverse_distance',
            'wall_follow_kp', 'centering_kp', 'stuck_timeout', 'stuck_distance_threshold',
            'recovery_backup_duration', 'recovery_turn_duration', 'robot_base_frame', 'odom_frame'
        ]

    def _init_ros_comms(self):
        """Initializes ROS publishers, subscribers, and timers."""
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Don't start the timer yet - wait for transforms first
        self.get_logger().info("ROS communications established.")

    def _wait_for_transforms(self):
        """Waits for essential transforms to be available before starting the main loop."""
        self.get_logger().info("Waiting for initial transforms to become available...")
        
        # First, wait for basic sensor data
        self.get_logger().info("Waiting for initial sensor data...")
        while rclpy.ok() and (self.last_scan is None or self.current_pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Initial sensor data received.")
        
        # Wait for the transform from the robot's base frame to the odom frame.
        self.get_logger().info(f"Waiting for transform {self.odom_frame} -> {self.robot_base_frame}...")
        transform_ready = False
        while rclpy.ok() and not transform_ready:
            try:
                self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.robot_base_frame,
                    rclpy.time.Time(),  # Use zero time to get latest available transform
                    timeout=Duration(seconds=1.0)
                )
                self.get_logger().info(f"Transform {self.odom_frame} -> {self.robot_base_frame} is available.")
                transform_ready = True
            except TransformException as ex:
                self.get_logger().info(f"Still waiting for transform {self.odom_frame} -> {self.robot_base_frame}: {ex}")
                rclpy.spin_once(self, timeout_sec=0.5)

        # CRITICAL: Wait for the transform from map to base_link (this is what we need for coordinate logging)
        self.get_logger().info("Waiting for transform map -> base_link...")
        map_base_transform_ready = False
        attempt_count = 0
        max_attempts = 10  # Reduce to 5 seconds at 0.5s intervals
        
        while rclpy.ok() and not map_base_transform_ready and attempt_count < max_attempts:
            try:
                self.tf_buffer.can_transform(
                    'map',
                    self.robot_base_frame,
                    rclpy.time.Time(),  # Use zero time to get latest available transform
                    timeout=Duration(seconds=0.1)  # Short timeout for can_transform
                )
                # If can_transform succeeds, try the actual lookup
                self.tf_buffer.lookup_transform(
                    'map',
                    self.robot_base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1)
                )
                self.get_logger().info("Transform map -> base_link is available.")
                map_base_transform_ready = True
            except TransformException as ex:
                attempt_count += 1
                self.get_logger().info(f"Waiting for map->base_link transform (attempt {attempt_count}/{max_attempts}): {ex}")
                rclpy.spin_once(self, timeout_sec=0.5)
        
        if not map_base_transform_ready:
            self.get_logger().error("CRITICAL: Map->base_link transform not available!")
            self.get_logger().error("This should not happen if the simulation is running properly.")
            self.get_logger().error("Please check:")
            self.get_logger().error("1. Navigation stack is running (nav2)")
            self.get_logger().error("2. Robot localization is active (AMCL or equivalent)")
            self.get_logger().error("3. Map server is publishing the map")
            self.get_logger().error("Robot CANNOT operate without this transform!")
            # Exit gracefully without calling rclpy.shutdown() here
            return
        
        # Test the coordinate logging right now
        test_log = self.format_log("Testing coordinate logging after transform setup")
        self.get_logger().info(test_log)
        
        # Now start the control timer
        self.get_logger().info("All transforms ready. Starting control loop...")
        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)

    def change_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(self.format_log(f"State change: {self.state.name} -> {new_state.name}"))
            self.state = new_state
            self.state_start_time = self.get_clock().now()
            self.start_pose = self.current_pose

    def format_log(self, msg):
        """Prepends the robot's map coordinates to a log message."""
        if self.current_pose is None:
            return f"[Map: (no_pose)] {msg}"
        
        try:
            # Get current position in base_link frame (robot's center)
            base_point = PointStamped()
            base_point.header.frame_id = self.robot_base_frame
            base_point.header.stamp = rclpy.time.Time().to_msg()  # Use zero time for latest transform
            base_point.point.x = 0.0  # Robot's center in its own frame
            base_point.point.y = 0.0
            base_point.point.z = 0.0

            # Transform to map frame - this MUST work since we verified the transform exists
            map_point = self.tf_buffer.transform(base_point, 'map', timeout=Duration(seconds=0.1))
            return f"[Map: ({map_point.point.x:.2f}, {map_point.point.y:.2f})] {msg}"
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException) as e:
            # This should not happen since we verified the transform exists during startup
            self.get_logger().error(f"CRITICAL: Lost map->base_link transform during operation: {e}")
            return f"[Map: (TRANSFORM_ERROR)] {msg}"


    def debug_coordinate_systems(self):
        """Debug method to understand coordinate system orientations"""
        if not self.last_scan:
            return
            
        self.get_logger().info(self.format_log("=== COORDINATE SYSTEM DEBUG ==="))
        
        # Get some sample distances at key angles
        angles = [0, 45, 90, 135, 180, -135, -90, -45]  # degrees
        for angle_deg in angles:
            dist = self.get_distance_at_angle(angle_deg)
            self.get_logger().info(self.format_log(f"Distance at {angle_deg:4d}°: {dist:.2f}m"))
        
        # Show LIDAR frame info
        self.get_logger().info(self.format_log(f"LIDAR frame: '{self.last_scan.header.frame_id}'"))
        self.get_logger().info(self.format_log(f"LIDAR angle range: {math.degrees(self.last_scan.angle_min):.1f}° to {math.degrees(self.last_scan.angle_max):.1f}°"))
        self.get_logger().info(self.format_log(f"LIDAR increment: {math.degrees(self.last_scan.angle_increment):.2f}°"))
        self.get_logger().info(self.format_log(f"LIDAR range count: {len(self.last_scan.ranges)}"))
        
        # Show transform chain
        try:
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=Duration(seconds=1.0))
            pos = transform.transform.translation
            quat = transform.transform.rotation
            
            # Convert quaternion to yaw
            import math
            yaw = math.atan2(2 * (quat.w * quat.z + quat.x * quat.y), 
                            1 - 2 * (quat.y**2 + quat.z**2))
            yaw_deg = math.degrees(yaw)
            
            self.get_logger().info(self.format_log(f"Robot pose in map: x={pos.x:.2f}, y={pos.y:.2f}, yaw={yaw_deg:.1f}°"))
            
        except Exception as e:
            self.get_logger().warn(self.format_log(f"Could not get robot pose: {e}"))
        
        self.get_logger().info(self.format_log("=== END DEBUG ==="))

    def scan_callback(self, msg):
        self.last_scan = msg
        
        # Run coordinate debug once every 50 scans (about every 5 seconds at 10Hz)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 1
            
        if self._debug_counter == 50:
            self._debug_counter = 0
            self.debug_coordinate_systems()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.last_pose_stuck_check is None:
            self.last_pose_stuck_check = self.current_pose
        
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def control_loop(self):
        if self.last_scan is None or self.current_pose is None:
            self.get_logger().info("Waiting for initial sensor data...", throttle_duration_sec=5)
            return
        
        # Don't check for stuck during the first 10 seconds to allow initialization
        time_since_start = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if self.state != State.RECOVERY and time_since_start > 10.0 and self.is_stuck():
            self.change_state(State.RECOVERY)

        twist_cmd = Twist()
        if self.state == State.IDLE:
            self.change_state(State.FIND_WALL)
        elif self.state == State.FIND_WALL:
            twist_cmd = self.execute_find_wall()
        elif self.state == State.PATROL_PERIMETER:
            twist_cmd = self.execute_patrol_perimeter()
        elif self.state == State.NAVIGATE_HALLWAY:
            twist_cmd = self.execute_navigate_hallway()
        elif self.state == State.APPROACH_DOORWAY:
            twist_cmd = self.execute_approach_doorway()
        elif self.state == State.EXECUTE_TURN:
            twist_cmd = self.execute_turn()
        elif self.state == State.TRAVERSE_DOORWAY:
            twist_cmd = self.execute_traverse_doorway()
        elif self.state == State.RECOVERY:
            twist_cmd = self.execute_recovery()

        self.cmd_vel_pub.publish(twist_cmd)

    def execute_find_wall(self):
        twist = Twist()
        right_dist = self.get_distance_at_angle(-90)
        front_dist = self.get_distance_at_angle(0)
        left_dist = self.get_distance_at_angle(90)
        
        # DEBUG: Add more detailed logging
        self.get_logger().info(self.format_log(f"FIND_WALL: front={front_dist:.2f}, right={right_dist:.2f}, left={left_dist:.2f}, patrol_distance={self.patrol_wall_distance}"))
        self.get_logger().info(self.format_log(f"FIND_WALL DEBUG: LIDAR frame='{self.last_scan.header.frame_id}', scan_count={len(self.last_scan.ranges)}, angle_range=[{math.degrees(self.last_scan.angle_min):.1f}°, {math.degrees(self.last_scan.angle_max):.1f}°]"))
        
        # If there's open space ahead and no wall on right, just start patrolling
        if front_dist > self.min_obstacle_distance * 2.0:
            self.get_logger().info(self.format_log(f"Open space ahead ({front_dist:.2f}m), starting patrol mode"))
            self.change_state(State.PATROL_PERIMETER)
            return twist
        
        # If there's a wall on the right at good distance, start wall following
        if right_dist < self.patrol_wall_distance * 1.5:
            self.get_logger().info(self.format_log(f"Found wall on right at {right_dist:.2f}m, switching to patrol"))
            self.change_state(State.PATROL_PERIMETER)
            return twist
        
        # Turn right to find a wall (positive angular.z = left turn, negative = right turn)
        self.get_logger().info(self.format_log(f"No wall found on right ({right_dist:.2f}m > {self.patrol_wall_distance * 1.5:.2f}m), turning right to find wall"))
        twist.angular.z = -self.turn_speed * 0.7  # NEGATIVE = turn RIGHT
        return twist

    def execute_patrol_perimeter(self):
        twist = Twist()
        front_dist = self.get_distance_at_angle(0)
        right_dist = self.get_distance_at_angle(-90)
        left_dist = self.get_distance_at_angle(90)

        self.get_logger().info(self.format_log(f"PATROL: front={front_dist:.2f}, right={right_dist:.2f}, left={left_dist:.2f}, min_obstacle={self.min_obstacle_distance}"), throttle_duration_sec=1)

        if front_dist < self.min_obstacle_distance:
            self.get_logger().info(self.format_log(f"Front obstacle at {front_dist:.2f}m < {self.min_obstacle_distance}m, going to recovery"))
            self.change_state(State.RECOVERY)
            return twist

        if left_dist + right_dist < self.hallway_width_threshold:
            self.get_logger().info(self.format_log(f"Narrow space detected (L+R={left_dist + right_dist:.2f} < {self.hallway_width_threshold}), switching to hallway mode"))
            self.change_state(State.NAVIGATE_HALLWAY)
            return twist

        # Debug the wall following logic
        error = self.patrol_wall_distance - right_dist
        angular_cmd = np.clip(self.wall_follow_kp * error, -self.turn_speed, self.turn_speed)
        
        self.get_logger().info(self.format_log(f"PATROL: error={error:.2f}, angular_cmd={angular_cmd:.2f}, moving forward at {self.patrol_speed}"), throttle_duration_sec=2)
        
        twist.angular.z = angular_cmd
        twist.linear.x = self.patrol_speed
        return twist

    def execute_navigate_hallway(self):
        twist = Twist()
        front_dist = self.get_distance_at_angle(0)
        right_dist = self.get_distance_at_angle(-90)
        left_dist = self.get_distance_at_angle(90)

        if front_dist < self.min_obstacle_distance:
            self.change_state(State.RECOVERY)
            return twist

        if left_dist + right_dist > self.hallway_width_threshold * 1.2:
            self.change_state(State.PATROL_PERIMETER)
            return twist
        
        if right_dist > self.doorway_opening_threshold:
            self.turn_direction = -1 # Right turn
            self.change_state(State.APPROACH_DOORWAY)
            return twist
        
        if left_dist > self.doorway_opening_threshold:
            self.turn_direction = 1 # Left turn
            self.change_state(State.APPROACH_DOORWAY)
            return twist

        error = right_dist - left_dist
        twist.angular.z = np.clip(self.centering_kp * error, -self.turn_speed, self.turn_speed)
        twist.linear.x = self.hallway_speed
        return twist

    def execute_approach_doorway(self):
        twist = Twist()
        # Follow the wall opposite to the opening until it disappears
        if self.turn_direction == -1: # Doorway on right, follow left wall
            wall_dist = self.get_distance_at_angle(90)
            error = self.hallway_wall_distance - wall_dist
            twist.angular.z = np.clip(self.wall_follow_kp * -error, -self.turn_speed, self.turn_speed)
        else: # Doorway on left, follow right wall
            wall_dist = self.get_distance_at_angle(-90)
            error = self.hallway_wall_distance - wall_dist
            twist.angular.z = np.clip(self.wall_follow_kp * error, -self.turn_speed, self.turn_speed)
        
        twist.linear.x = self.hallway_speed

        if wall_dist > self.doorway_opening_threshold:
            self.get_logger().info("Aligned with doorway, preparing to turn.")
            self.target_yaw = self.normalize_angle(self.current_yaw + (math.pi / 2 * self.turn_direction))
            self.change_state(State.EXECUTE_TURN)
        
        return twist

    def execute_turn(self):
        twist = Twist()
        yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        
        if abs(yaw_error) < 0.1: # 0.1 rad is about 5.7 degrees
            self.get_logger().info("Turn complete.")
            self.change_state(State.TRAVERSE_DOORWAY)
            return twist

        twist.angular.z = np.sign(yaw_error) * self.turn_speed
        return twist

    def execute_traverse_doorway(self):
        twist = Twist()
        dist_moved = self.get_distance_moved()

        if dist_moved >= self.doorway_traverse_distance:
            self.get_logger().info("Doorway traversed.")
            self.change_state(State.FIND_WALL)
            return twist
        
        twist.linear.x = self.doorway_speed
        return twist

    def execute_recovery(self):
        twist = Twist()
        time_in_state = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

        if time_in_state < self.recovery_backup_duration:
            twist.linear.x = self.recovery_speed
        elif time_in_state < self.recovery_backup_duration + self.recovery_turn_duration:
            twist.angular.z = self.turn_speed
        else:
            self.change_state(State.FIND_WALL)
        
        return twist

    def get_distance_at_angle(self, angle_deg):
        if self.last_scan is None: return float('inf')
        
        # We need to transform the desired angle from the robot's frame to the LIDAR's frame
        angle_rad_base = math.radians(angle_deg)
        
        try:
            # Create a point 1 meter away in the desired direction in the base_link frame
            point_base = PointStamped()
            point_base.header.frame_id = self.robot_base_frame
            point_base.point.x = 1.0 * math.cos(angle_rad_base)
            point_base.point.y = 1.0 * math.sin(angle_rad_base)

            # Find the transform from base_link to the LIDAR frame
            transform = self.tf_buffer.lookup_transform(
                self.last_scan.header.frame_id, self.robot_base_frame, 
                rclpy.time.Time(),  # Use zero time for latest available transform
                timeout=Duration(seconds=0.1))

            # Transform the point to the LIDAR frame
            point_lidar = tf2_geometry_msgs.do_transform_point(point_base, transform)

            # Calculate the angle in the LIDAR frame
            angle_rad_lidar = math.atan2(point_lidar.point.y, point_lidar.point.x)

            # Find the corresponding index in the laser scan
            index = int((angle_rad_lidar - self.last_scan.angle_min) / self.last_scan.angle_increment)
            
            # DEBUG: Log the transformation details
            if angle_deg in [-90, 0, 90]:  # Only log for the main directions
                self.get_logger().info(f"DEBUG angle={angle_deg}°: base_point=({point_base.point.x:.2f},{point_base.point.y:.2f}) -> lidar_point=({point_lidar.point.x:.2f},{point_lidar.point.y:.2f}) -> lidar_angle={math.degrees(angle_rad_lidar):.1f}° -> index={index}")
            
            if 0 <= index < len(self.last_scan.ranges):
                # Average over a small window for robustness
                start = max(0, index - 2)
                end = min(len(self.last_scan.ranges), index + 3)
                valid_ranges = [r for r in self.last_scan.ranges[start:end] if np.isfinite(r)]
                distance = np.mean(valid_ranges) if valid_ranges else float('inf')
                
                # DEBUG: Log range details for main directions
                if angle_deg in [-90, 0, 90]:
                    self.get_logger().info(f"DEBUG angle={angle_deg}°: scan_range[{index}]={self.last_scan.ranges[index]:.2f}, window_avg={distance:.2f}")
                
                return distance

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform error in get_distance_at_angle: {e}", throttle_duration_sec=5)
            return float('inf')

        return float('inf')

    def is_stuck(self):
        if self.last_pose_stuck_check is None: 
            return False
        
        time_since_check = (self.get_clock().now() - self.last_pose_stuck_time).nanoseconds / 1e9
        if time_since_check > self.stuck_timeout:
            dist_moved = math.sqrt(
                (self.current_pose.position.x - self.last_pose_stuck_check.position.x)**2 +
                (self.current_pose.position.y - self.last_pose_stuck_check.position.y)**2
            )
            if dist_moved < self.stuck_distance_threshold:
                self.get_logger().warn(self.format_log(f"Robot is stuck! Moved only {dist_moved:.3f}m in {time_since_check:.1f}s"))
                return True
            
            self.last_pose_stuck_check = self.current_pose
            self.last_pose_stuck_time = self.get_clock().now()
        return False

    def get_distance_moved(self):
        if not self.start_pose: return 0.0
        return math.sqrt(
            (self.current_pose.position.x - self.start_pose.position.x)**2 +
            (self.current_pose.position.y - self.start_pose.position.y)**2
        )

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = RoamingNodeV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RoamingNodeV3.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
