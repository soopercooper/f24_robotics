import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from apriltag_msgs.msg import AprilTagDetectionArray  # Import AprilTag message type
import math
from time import time, sleep

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
MAX_ROTATION_SPEED = 0.5   # Maximum rotational speed
MAX_CHECK_SPEED = 1.5708
NO_ROTATION_SPEED = 0.0    # Zero rotational speed (no rotation)
MAX_LINEAR_SPEED = 0.2     # Maximum linear speed
NO_LINEAR_SPEED = 0.0      # Zero linear speed (no forward movement)

class RandomWalk(Node):
    """
    A ROS2 node that controls a Turtlebot to move randomly and avoid obstacles,
    using LaserScan data for obstacle avoidance and AprilTag detection for logging detections.
    """
    def __init__(self):
        super().__init__('random_walk_node')
        
        # Initialize variables
        self.turn = True
        self.start_time = time()
        self.expected_runtime = math.radians(360) / MAX_CHECK_SPEED

        self.scan_cleaned = []         # Processed LIDAR scan data
        self.turtlebot_moving = False  # Flag to indicate whether Turtlebot is moving
        
        # Create publisher for movement commands (Twist messages on 'cmd_vel' topic)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscription to LaserScan data topic '/scan'
        # This subscription uses a QoS profile with a best effort reliability since LaserScan data is frequent
        self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Create subscription to AprilTag detections topic '/detections'
        # Also uses a QoS profile with a best effort reliability
        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Initialize Twist message for controlling linear and angular velocity
        self.cmd = Twist()
        
        # Create a timer to call `timer_callback` at a fixed rate (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Set to track IDs of detected AprilTags to avoid logging duplicates
        self.detected_tags = set()

    def listener_callback1(self, msg):
        """
        Callback function for LaserScan data.
        Cleans and processes the incoming LIDAR scan data for obstacle avoidance.
        
        :param msg: The LaserScan message containing ranges of obstacles around the robot
        """
        scan = msg.ranges
        # Replace infinite values with a max range (3.5m) for processing
        # If a range is not infinite, limit it to a maximum of 3.5 if it's more than that (for safety)
        self.scan_cleaned = [min(r, 3.5) if r != float('Inf') else 3.5 for r in scan]

    def apriltag_callback(self, msg):
        """
        Callback function for AprilTag detections.
        Logs information about newly detected AprilTags to avoid repeated logging of the same tag.
        
        :param msg: The AprilTagDetectionArray message containing detected AprilTags data
        """
        for detection in msg.detections:
            tag_id = detection.id[0]  # Extract the first ID from the detected tag (assuming single ID)
            # Check if we have already detected this tag ID
            if tag_id not in self.detected_tags:
                # Add this tag ID to the set of detected tags to avoid duplicates
                self.detected_tags.add(tag_id)
                # Log information about the detected tag, including its ID and full detection info
                self.get_logger().info(f"Tag detected: ID {tag_id}, Info: {detection}")

    def timer_callback(self):
        """
        Timer callback function that executes at a fixed interval (0.5 seconds).
        It uses processed LIDAR data to control the robot's movement for obstacle avoidance and navigation.
        """
        # If we have no processed scan data, stop the robot and return
        self.current_time = time()
        if(self.current_time - self.start_time > self.expected_runtime):
            if(self.turn == True):
                print("turn is now false")
                self.turn = False
                self.start_time = time()
                self.expected_runtime = 10
            elif(self.turn == False):
                print("turn is now true")
                self.turn = True
                self.start_time = time()
                self.expected_runtime = math.radians(360) / MAX_CHECK_SPEED

        if(self.turn == True):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = MAX_CHECK_SPEED
        else:
            if len(self.scan_cleaned) == 0:
                self.turtlebot_moving = False
                return
            # Determine minimum distances in critical directions from the LaserScan data
            left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])   # Minimum distance on left side
            right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]) # Minimum distance on right side
            front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]) # Minimum distance in front

            # If there's an obstacle too close in front of the robot
            if front_lidar_min < SAFE_STOP_DISTANCE:
                # Stop linear movement and rotate to avoid obstacle
                self.cmd.linear.x = NO_LINEAR_SPEED
                self.cmd.angular.z = MAX_ROTATION_SPEED
            # If there's an obstacle ahead but not too close, start avoiding it
            elif front_lidar_min < LIDAR_AVOID_DISTANCE:
                # Move forward slowly
                self.cmd.linear.x = MAX_LINEAR_SPEED / 4
                # Rotate away from the closer side to avoid the obstacle
                self.cmd.angular.z = -MAX_ROTATION_SPEED if right_lidar_min > left_lidar_min else MAX_ROTATION_SPEED
            else:
                # Clear path ahead, move forward
                self.cmd.linear.x = MAX_LINEAR_SPEED
                self.cmd.angular.z = NO_ROTATION_SPEED
                # Additional logic for adjusting orientation based on right side distance
                if right_lidar_min > 1.0:
                    # If the right side is clear, turn slightly right
                    self.cmd.angular.z = -MAX_ROTATION_SPEED
                    self.cmd.linear.x = MAX_LINEAR_SPEED / 2
                elif right_lidar_min < 0.2:
                    # If the right side is too close, turn left to keep distance
                    self.cmd.angular.z = MAX_ROTATION_SPEED
                    self.cmd.linear.x = MAX_LINEAR_SPEED / 2

            # Publish the computed velocity command to the robot
        self.publisher_.publish(self.cmd)


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
