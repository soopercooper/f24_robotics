import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import ReliabilityPolicy, QoSProfile
from transforms3d.euler import quat2euler
import math
import logging

# Constants for robot movement and distances
LINEAR_VEL = 0.22               # Default linear velocity
STOP_DISTANCE = 0.5             # Distance at which the robot should stop to avoid collision
LIDAR_ERROR = 0.05              # Error tolerance for LIDAR readings
LIDAR_AVOID_DISTANCE = 0.7      # Distance threshold for obstacle avoidance
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR  # Calculated safe stop distance considering error

# Indices in the LaserScan array for different directions relative to robot orientation
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90

# Rotation and speed parameters
MAX_ROTATION_SPEED = 0.5        # Maximum rotational speed
MAX_CHECK_SPEED = 1.0           # Speed for scanning rotation
NO_ROTATION_SPEED = 0.0         # Zero rotational speed (no rotation)
MAX_LINEAR_SPEED = 0.2          # Maximum linear speed
NO_LINEAR_SPEED = 0.0           # Zero linear speed (no forward movement)

def euler_from_quaternion(quat):
    return quat2euler([quat[3], quat[0], quat[1], quat[2]])

class RandomWalk(Node):
    """
    A ROS2 node that controls a Turtlebot to move randomly and avoid obstacles,
    using LaserScan data for obstacle avoidance and AprilTag detection for logging detections.
    """

    def __init__(self):
        super().__init__('random_walk_node')

        # Initialize variables
        self.turn = True  # Flag to initiate scanning rotation
        self.rotating = False  # Flag to indicate if rotation is in progress
        self.starting_yaw = None  # Yaw angle at the start of rotation
        self.current_yaw = 0.0  # Current yaw angle
        self.position_x = 0.0  # Current x position
        self.position_y = 0.0  # Current y position

        self.scan_cleaned = []         # Processed LIDAR scan data

        # Set to track IDs of detected AprilTags to avoid logging duplicates
        self.detected_tags = set()

        # Set up logging to a file
        logging.basicConfig(filename='apriltag_detections.log', level=logging.INFO)
        self.logger = logging.getLogger('AprilTagLogger')

        # Initialize variable to store the last log message
        self.last_log_message = ''

        # Create publisher for movement commands (Twist messages on 'cmd_vel' topic)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscription to LaserScan data topic '/scan'
        self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Create subscription to AprilTag detections topic '/detections'
        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Create subscription to Odometry data topic '/odom'
        self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Initialize Twist message for controlling linear and angular velocity
        self.cmd = Twist()

        # Create a timer to call `timer_callback` at a fixed rate (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def listener_callback1(self, msg):
        """
        Callback function for LaserScan data.
        Cleans and processes the incoming LIDAR scan data for obstacle avoidance.

        :param msg: The LaserScan message containing ranges of obstacles around the robot
        """
        scan = msg.ranges
        # Replace infinite values with a max range (3.5m) for processing
        self.scan_cleaned = [min(r, 3.5) if r != float('Inf') else 3.5 for r in scan]

    def odometry_callback(self, msg):
        """
        Callback function for Odometry data.
        Updates the robot's current yaw angle and position.

        :param msg: The Odometry message containing pose and orientation data
        """
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw % (2 * math.pi)  # Normalize yaw to [0, 2π)
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

    def apriltag_callback(self, msg):
        """
        Callback function for AprilTag detections.
        Logs information about newly detected AprilTags to avoid repeated logging of the same tag.

        :param msg: The AprilTagDetectionArray message containing detected AprilTags data
        """
        timestamp = self.get_clock().now().to_msg()
        for detection in msg.detections:
            tag_id = detection.id[0]
            if tag_id not in self.detected_tags:
                self.detected_tags.add(tag_id)
                # Extract the relative position to the tag
                pose = detection.pose.pose.pose
                relative_position = (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                )
                # Log the detection with detailed information
                log_message = (
                    f"Time: {timestamp.sec}.{timestamp.nanosec}, "
                    f"Robot Position: ({self.position_x:.2f}, {self.position_y:.2f}), "
                    f"Orientation: {math.degrees(self.current_yaw):.2f}°, "
                    f"Tag ID: {tag_id}, Relative Position: {relative_position}"
                )
                self.logger.info(log_message)
                # Also print to console if the message is different
                console_message = f"Tag detected: ID {tag_id}"
                if console_message != self.last_log_message:
                    self.get_logger().info(console_message)
                    self.last_log_message = console_message

    def timer_callback(self):
        """
        Timer callback function that executes at a fixed interval.
        It uses processed LIDAR data to control the robot's movement for obstacle avoidance and navigation.
        """
        # If we are supposed to turn (scan for AprilTags)
        if self.turn:
            if not self.rotating:
                # Start the rotation
                self.starting_yaw = self.current_yaw
                self.rotating = True
                new_message = "Starting 360-degree rotation for scanning."
                if new_message != self.last_log_message:
                    self.get_logger().info(new_message)
                    self.last_log_message = new_message

            # Calculate the yaw difference
            yaw_difference = (self.current_yaw - self.starting_yaw) % (2 * math.pi)
            if yaw_difference >= (2 * math.pi - 0.1):  # Allow a small margin
                # Completed full rotation
                self.rotating = False
                self.turn = False
                self.cmd.angular.z = NO_ROTATION_SPEED
                self.cmd.linear.x = NO_LINEAR_SPEED
                new_message = "Completed rotation. Resuming normal movement."
                if new_message != self.last_log_message:
                    self.get_logger().info(new_message)
                    self.last_log_message = new_message
            else:
                # Continue rotating
                self.cmd.angular.z = MAX_CHECK_SPEED
                self.cmd.linear.x = NO_LINEAR_SPEED
                self.publisher_.publish(self.cmd)
                return  # Wait until rotation is complete

        # Obstacle avoidance and movement logic
        if len(self.scan_cleaned) == 0:
            # No LIDAR data available
            return

        # Determine minimum distances in critical directions from the LaserScan data
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])   # Left side
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]) # Right side
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]) # Front

        # If there's an obstacle too close in front of the robot
        if front_lidar_min < SAFE_STOP_DISTANCE:
            # Stop linear movement and rotate to avoid obstacle
            self.cmd.linear.x = NO_LINEAR_SPEED
            self.cmd.angular.z = MAX_ROTATION_SPEED
            new_message = "Obstacle detected ahead. Rotating to avoid."
            if new_message != self.last_log_message:
                self.get_logger().info(new_message)
                self.last_log_message = new_message
        # If there's an obstacle ahead but not too close, start avoiding it
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            # Move forward slowly and turn away from the obstacle
            self.cmd.linear.x = MAX_LINEAR_SPEED / 2
            if right_lidar_min > left_lidar_min:
                self.cmd.angular.z = -MAX_ROTATION_SPEED  # Turn right
                new_message = "Avoiding obstacle: turning right."
            else:
                self.cmd.angular.z = MAX_ROTATION_SPEED  # Turn left
                new_message = "Avoiding obstacle: turning left."
            if new_message != self.last_log_message:
                self.get_logger().info(new_message)
                self.last_log_message = new_message
        else:
            # Path is clear ahead, move forward
            self.cmd.linear.x = MAX_LINEAR_SPEED
            self.cmd.angular.z = NO_ROTATION_SPEED
            # Adjust orientation to maintain a certain distance from walls
            if right_lidar_min > 1.0:
                # If the right side is clear, turn slightly right
                self.cmd.angular.z = -MAX_ROTATION_SPEED / 2
                new_message = "Adjusting course: slight turn right."
            elif right_lidar_min < 0.5:
                # If the right side is too close, turn left
                self.cmd.angular.z = MAX_ROTATION_SPEED / 2
                new_message = "Adjusting course: slight turn left."
            else:
                self.cmd.angular.z = NO_ROTATION_SPEED
                new_message = "Moving forward."
            if new_message != self.last_log_message:
                self.get_logger().info(new_message)
                self.last_log_message = new_message

        # Publish the computed velocity command to the robot
        self.publisher_.publish(self.cmd)

        # Periodically initiate scanning rotation
        if not self.turn and not self.rotating:
            # After a certain time or condition, start scanning again
            # For example, after moving a certain distance
            self.turn = True  # Set to True to start scanning in the next cycle

def main(args=None):
    """
    Entry point for the ROS2 node.
    Initializes the ROS2 communication, creates the RandomWalk node,
    and spins it to process callbacks until shutdown.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the RandomWalk node
    random_walk_node = RandomWalk()

    # Spin the node so its callbacks can be processed
    rclpy.spin(random_walk_node)

    # Once the node is killed or exits, destroy it and shut down ROS2
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
