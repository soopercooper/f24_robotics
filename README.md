# Cooper Olson Final Project : Autonomous Indoor Search 

The trials were run on 'Greek Assembly Hall Final.wbt' and 'maze.wbt'. To launch either world, make sure the file name in the launch file is the correct .wbt file.

## Algorithm
1. **Wall Following**:
   - The robot moves along walls using LIDAR data.
   - Adjusts speed and direction to avoid obstacles while maintaining proximity to walls.
   - ~~Turns around in a circle every 10 seconds to try to find AprilTags.~~ Took this out to improve coverage rate and reduce mapping error

## Requirements
- ROS2 Humble
- Install dependencies:
   https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup (install humble cartographer mapping dependencies)
   pip install transform3d

## Running The Simulation

Terminal 1 (Launchs the specified world file):
```bash
bash build_and_run_simulation.sh
```

Terminal 2 (Runs the controller):
```bash
bash run_python.sh
```

Terminal 3 (Runs cartographer mapping):
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

Terminal 4 (Saves map after turtlebot has explored):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
