# Cooper Olson Final Project : Autonomous Indoor Search 

The trials were run on 'Greek Assembly Hall Final.wbt' and 'maze.wbt'. To launch either world, make sure the file name in the launch file is the correct .wbt file. Trials were run in randomized starting locations.

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

Maze Trial Run 1 Map
![map1mazepic](https://github.com/user-attachments/assets/36d8893f-1802-4e38-b59d-ea8812c96953)
Maze Trial Run 2 Map
![map2mazepic](https://github.com/user-attachments/assets/7129c60e-7d57-40ef-9afb-8f3ba98cf7cc)
Maze Trial Run 3 Map
![map3mazepic](https://github.com/user-attachments/assets/13f394dc-7bd7-4352-abd9-17c8c4dbec19)
Greek Hall Trial Run 1 Map
![map1greekpic](https://github.com/user-attachments/assets/fe86ae62-b3a5-4964-9a27-e81f5fcb711f)
Greek Hall Trial Run 2 Map
![map2greekpic](https://github.com/user-attachments/assets/0aec6e76-3a84-4855-a484-456b013e0376)
Greek Hall Trial Run 3 Map
![map3greekpic](https://github.com/user-attachments/assets/9ef5b64f-f6b3-411f-ba10-acdafec5b919)
