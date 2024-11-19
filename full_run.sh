#!/bin/bash

# Build the ROS 2 workspace
colcon build

# Source the setup file
source install/setup.bash

# Set the TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Run the v4l2_camera node in a new terminal ros2 run v4l2_camera v4l2_camera_node
gnome-terminal -- bash -c "source install/setup.bash && ros2 run v4l2_camera v4l2_camera_node; exec bash"

# Run the apriltag_ros node in a new terminal -> ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file $(ros2 pkg prefix apriltag_ros)/share/apriltag_ros/cfg/tags_36h11.yaml
gnome-terminal -- bash -c "source install/setup.bash && ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file \$(ros2 pkg prefix apriltag_ros)/share/apriltag_ros/cfg/tags_36h11.yaml; exec bash"

# Run the rqt_image_view -> ros2 run rqt_image_view rqt_image_view
gnome-terminal -- bash -c "source install/setup.bash && ros2 run rqt_image_view rqt_image_view; exec bash"

# Launch the TurtleBot3 bringup in a new terminal -> ros2 launch turtlebot3_bringup robot.launch.py
gnome-terminal -- bash -c "source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py; exec bash"

# Run the bag recording -> ros2 bag record /detections

# Run the Webots ROS2 Homework1 Python node in a new terminal -> ros2 run webots_ros2_homework1_python webots_ros2_homework1_python
gnome-terminal -- bash -c "source install/setup.bash && ros2 run webots_ros2_homework1_python webots_ros2_homework1_python; exec bash"

echo "All processes have been launched in separate terminals."
