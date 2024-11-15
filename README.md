# AprilTag Search and Wall Following

## Algorithm
1. **Wall Following**:
   - The robot moves along walls using LIDAR data.
   - Adjusts speed and direction to avoid obstacles while maintaining proximity to walls.

2. **AprilTag Detection**:
   - Subscribes to `/detections` for AprilTag data.
   - Logs tag ID and details upon detection.
   - Prevents re-logging of the same tags.

## Requirements
- ROS2 Humble
- Install dependencies:
  - `ros-humble-apriltag-ros`
  - `ros-humble-v4l2-camera`
- Enable the camera using `raspi-config`.

## Launch
Run the launch file to start the AprilTag node and the robot controller.
