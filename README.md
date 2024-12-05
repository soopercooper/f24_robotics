# AprilTag Search and Wall Following

## Algorithm
1. **Wall Following**:
   - The robot moves along walls using LIDAR data.
   - Adjusts speed and direction to avoid obstacles while maintaining proximity to walls.
   - Turns around in a circle every 10 seconds to try to find AprilTags.

2. **AprilTag Detection**:
   - Subscribes to `/detections` for AprilTag data.
   - Logs tag ID and details upon detection.
   - Prevents re-logging of the same tags.

## Requirements
- ROS2 Humble
- Install dependencies:
  - `ros-humble-apriltag-ros`
  - `ros-humble-v4l2-camera`
- Enable the camera using `raspi-config`

## Running The Simulation

Terminal 1:
```bash
bash build_and_run_simulation.sh
```

Terminal 2:
```bash
bash run_python.sh
```

## Running On The Robot

Terminal 1:
```bash
bash run_bringup.sh
```

Terminal 2:
```bash
bash run_rosbag.sh
```

Terminal 3:
```bash
bash run_local.sh
```

## See the rosbag
```bash
bash run_rosbag_playback.sh <bagfile>
```

## Launch
Run the launch file to start the AprilTag node and the robot controller.


# Install packages ->

```
raspi-config
ros-humble-image-common
ros-humble-image-transport
ros-humble-v4l2-camera
ros-humble-rqt-image-view
ros-humble-apriltag-ros
ros-humble-apriltag-msgs
```

## Calibration

Calibrate the camera using the following command:
```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 image:=/image_raw camera:=/
```

Then update the camera info in the launch file.
```
parameters=[{
    'camera_info_url': 'file:///path/to/your/calibration.yaml', # Replace with the actual path
    'tag_family': 'tag36h11',
    'tag_size': 0.165,  # Replace with the actual size of your tags
    'camera_frame': 'camera_link'
}]
```