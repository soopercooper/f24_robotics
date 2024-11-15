import rclpy
# import the ROS2 Python libraries to use its features
from rclpy.node import Node
# import the Twist module from geometry_msgs interface to control robot movement
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface to receive LIDAR data
from sensor_msgs.msg import LaserScan
# import the Odometry module from nav_msgs interface to track robot position
from nav_msgs.msg import Odometry
# import Quality of Service (QoS) to set the correct profile and reliability for reading sensor data
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

# Constants defining robot behavior and safety thresholds
LINEAR_VEL = 0.22  # Default linear velocity of the robot when moving forward
STOP_DISTANCE = 0.5  # Minimum distance to an obstacle before stopping, default 0.2 meters
LIDAR_ERROR = 0.05  # Error margin for LIDAR readings
LIDAR_AVOID_DISTANCE = .7  # Distance at which the robot should start avoiding an obstacle, default 0.5 meters
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR  # Safe distance to avoid a collision

# Indices of the LIDAR scan array corresponding to different directions relative to the robot
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90
MAX_ROTATION_SPEED = 0.5  # Maximum angular velocity of the robot when turning
NO_ROTATION_SPEED = 0.0  # Angular velocity of the robot when not turning
MAX_LINEAR_SPEED = 0.2  # Maximum linear velocity of the robot when moving forward
NO_LINEAR_SPEED = 0.0  # Linear velocity of the robot when not moving forward

class RandomWalk(Node):

    def __init__(self):
        # Initialize the node with the name 'random_walk_node'
        super().__init__('random_walk_node')
        
        # Initialize variables for storing sensor data and robot state
        self.scan_cleaned = []  # Cleaned LIDAR data (filtered and processed)
        self.stall = False  # Flag to detect if the robot is stalled
        self.turtlebot_moving = False  # Flag to indicate if the robot is currently moving
        self.first_pos_store = False  # Flag to store the first position of the robot
        
        # Create a publisher for controlling the robot's velocity
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to the LIDAR scan topic to receive distance measurements
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Subscribe to the odometry topic to receive position and orientation data
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Initialize additional variables for controlling the robot
        self.laser_forward = 0  # Variable to store forward LIDAR data
        self.odom_data = 0  # Variable to store odometry data
        self.pose_saved = ''  # Variable to save the robot's position
        self.cmd = Twist()  # Twist message used to control the robot's velocity
        
        # Create a timer to call the timer_callback function periodically (every 0.5 seconds)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback1(self, msg1):
        """Callback function to process incoming LIDAR scan data."""
        # Store the received scan data in a temporary variable
        scan = msg1.ranges
        
        # Clear the cleaned scan data list before processing new data
        self.scan_cleaned = []
        
        # Process the raw scan data to handle infinite distances and NaN values
        for reading in scan:
            if reading == float('Inf'):
                # Replace 'Inf' readings with a maximum distance value
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                # Replace 'NaN' readings with zero
                self.scan_cleaned.append(0.0)
            else:
                # Keep valid readings as they are
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        """Callback function to process incoming odometry data."""
        # Extract position and orientation data from the odometry message
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        
        self.pose_saved = position

    def timer_callback(self):
        """Callback function to control robot movement based on sensor data."""
        # If no LIDAR data is available, stop the robot
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return
        
        # Extract minimum distances in left, right, and front directions from LIDAR data
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # Check if there's an obstacle within the safe stopping distance
        if front_lidar_min < SAFE_STOP_DISTANCE:
            # if self.stall:
            self.cmd.linear.x = NO_LINEAR_SPEED
            self.cmd.angular.z = MAX_ROTATION_SPEED
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Rotating')
            return
        
        # Check if the robot should start avoiding an obstacle
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            # Slow down and turn in the direction with more space
            self.cmd.linear.x = MAX_LINEAR_SPEED / 4
            text_direction = None
            if right_lidar_min > left_lidar_min:
                self.cmd.angular.z = -MAX_ROTATION_SPEED  # Turn right
                text_direction = 'right'
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning %s' % text_direction)
            self.turtlebot_moving = True
            return
        
        # If there's no immediate obstacle, move forward at the normal speed
        else:
            self.cmd.linear.x = MAX_LINEAR_SPEED
            self.cmd.angular.z = NO_ROTATION_SPEED
            # Check if the right wall is too far away
            if right_lidar_min > 1.0:
                self.cmd.angular.z = -MAX_ROTATION_SPEED
                # Move closer to follow the wall
                self.cmd.linear.x = MAX_LINEAR_SPEED / 2
                self.get_logger().info('Moving closer to the wall')
            # Check if the right wall is too close
            elif right_lidar_min < 0.2:
                self.cmd.angular.z = MAX_ROTATION_SPEED
                # Move away from the wall
                self.cmd.linear.x = MAX_LINEAR_SPEED / 2
                self.get_logger().info('Moving away from the wall')
            # If the right wall is within the acceptable range, adjust angular speed smoothly
            elif right_lidar_min > 0.5:
                # Apply a cubic function to smoothly reduce angular speed
                # Scale the rotation speed based on the distance to the wall (between 0.3 and 1.0)
                proportional_speed = (right_lidar_min - 0.5) / (1.0 - 0.5)  # Normalize between 0 and 1
                self.cmd.angular.z = -MAX_ROTATION_SPEED * (proportional_speed ** .5)
                self.cmd.linear.x = MAX_LINEAR_SPEED * 0.75
                self.get_logger().info(f'Adjusting rotation, angular.z = {self.cmd.angular.z}')

            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
        
        # Log a message if the robot is stalled (if stall detection is implemented)
        if self.stall:
            # self.get_logger().info('Stall reported')
            pass

def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)
    
    # Create an instance of the RandomWalk node
    random_walk_node = RandomWalk()
    
    # Keep the node running, waiting for data and processing callbacks
    rclpy.spin(random_walk_node)
    
    # Destroy the node explicitly when shutting down
    random_walk_node.destroy_node()
    
    # Shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
