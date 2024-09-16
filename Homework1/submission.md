# Homework 1 CS460

Avery Fernandez

## Paths and Total Distances

|  Zone  | Trial | Path Length | Max Distance |
|--------|-------|-------------|--------------|
| Zone 1 |   1   |    83.8     |     9.1      |
| Zone 1 |   2   |    80.1     |     8.6      |
| Zone 1 |   3   |    86.1     |     8.9      |
| Zone 1 |   4   |    85.5     |     8.4      |
| Zone 1 |   5   |    93.0     |     8.7      |
|--------|-------|-------------|--------------|
| Zone 2 |   1   |    78.1     |     8.2      |
| Zone 2 |   2   |    80.5     |     8.3      |
| Zone 2 |   3   |    82.5     |     7.4      |
| Zone 2 |   4   |    94.1     |     7.9      |
| Zone 2 |   5   |    76.4     |     7.9      |
|--------|-------|-------------|--------------|
| Zone 3 |   1   |    74.0     |     11.0     |
| Zone 3 |   2   |    69.0     |     10.3     |
| Zone 3 |   3   |    66.1     |     9.9      |
| Zone 3 |   4   |    81.0     |     10.4     |
| Zone 3 |   5   |    75.9     |     10.9     |
|--------|-------|-------------|--------------|
| Zone 4 |   1   |    74.9     |     10.3     |
| Zone 4 |   2   |    76.7     |     10.9     |
| Zone 4 |   3   |    73.7     |     10.4     |
| Zone 4 |   4   |    83.7     |     10.4     |
| Zone 4 |   5   |    85.3     |     10.8     |
|--------|-------|-------------|--------------|

## Robot Paths

### Zone 1

![Zone 1](./Zone%201_combined_map.png)

### Zone 2

![Zone 2](./Zone%202_combined_map.png)

### Zone 3

![Zone 3](./Zone%203_combined_map.png)

### Zone 4

![Zone 4](./Zone%204_combined_map.png)


## Approach

In this project, I implemented a wall follow technique to explore a simulated apartment environment.

### Odometry and Position Tracking

In `listener_callback2`, the odometry data is used to track the robot’s position and log it during each trial. The first position is stored when the robot starts, and subsequent positions are logged at regular intervals during the robot’s exploration. This ensures that the robot's movement throughout the apartment zones is recorded. Additionally, the starting position for each trial is read from the world file using the extract_turtlebot_start() function. This function reads the translation and rotation parameters from the Webots world file to ensure that the correct starting position is used for each trial.

```python
self.pose_saved = position
self.file_name = f'Homework1/data/robot_position_{turtlebot_start["translation"]}_{counter}.txt'
with open(self.file_name, 'a') as f:
    f.write(f'x: {self.pose_saved.x}, y: {self.pose_saved.y}\n')
```

### Obstacle Avoidance

The core functionality of the robot's movement lies in the `timer_callback` function. Here, the robot determines its action based on the LIDAR readings. If an obstacle is detected within the `SAFE_STOP_DISTANCE`, the robot stops and rotates to avoid the object. The robot also takes a preemptive approach to avoid obstacles within the `LIDAR_AVOID_DISTANCE` by slowing down and adjusting its direction based off if there are walls close to it. This dynamic control mechanism ensures that the robot can explore the apartment safely without requiring prior knowledge of obstacles.

```python
if front_lidar_min < SAFE_STOP_DISTANCE:
    self.cmd.linear.x = NO_LINEAR_SPEED
    self.cmd.angular.z = MAX_ROTATION_SPEED
    self.publisher_.publish(self.cmd)
elif front_lidar_min < LIDAR_AVOID_DISTANCE:
    self.cmd.linear.x = MAX_LINEAR_SPEED / 4
    if right_lidar_min < left_lidar_min:
        self.cmd.angular.z = -MAX_ROTATION_SPEED
    self.publisher_.publish(self.cmd)
```

### Wall Following

To enhance the robot's exploration, the controller incorporates wall-following behavior. When no obstacles are detected directly in front of the robot, the controller checks the right side of the robot to ensure it maintains a safe distance from the wall. If the robot drifts too far from the wall, it adjusts its angular velocity to move closer. This method ensures the robot covers the apartment zones without colliding with walls.

```python
if right_lidar_min > 1.0:
    self.cmd.angular.z = -MAX_ROTATION_SPEED
    self.cmd.linear.x = MAX_LINEAR_SPEED / 2
elif right_lidar_min < 0.2:
    self.cmd.angular.z = MAX_ROTATION_SPEED
    self.cmd.linear.x = MAX_LINEAR_SPEED / 2
elif right_lidar_min > 0.5:
    proportional_speed = (right_lidar_min - 0.5) / (1.0 - 0.5)
    self.cmd.angular.z = -MAX_ROTATION_SPEED * (proportional_speed ** .5)
    self.cmd.linear.x = MAX_LINEAR_SPEED * 0.75
```

## Discussion of Implementation

### Advantages

- **Dynamic Obstacle Avoidance**: The robot can detect obstacles within the predefined `SAFE_STOP_DISTANCE` and `LIDAR_AVOID_DISTANCE`, enabling it to either stop or turn away before a collision occurs. This makes the approach highly adaptable to unknown environments where obstacles may appear unpredictably, ensuring the robot can explore safely. The preemptive nature of this system is a key strength. By slowing down and turning as soon as obstacles are detected, the robot avoids unnecessary delays and erratic movements that would occur with reactive, last-minute stopping. This results in smoother, more continuous navigation, which improves exploration efficiency and minimizes the risk of getting stuck.
- **Wall Following for Efficient Coverage**: The wall-following behavior is another advantage of the implementation, as it ensures that the robot effectively covers the apartment environment without colliding with walls. By keeping a safe distance from the wall and adjusting its angular velocity based on the proximity of the wall, the robot can navigate the space efficiently. This method is especially beneficial in environments with complex structures or tight spaces, where staying close to the walls is a strategic way to ensure the robot covers more ground while avoiding obstacles in the center of the room. The use of a proportional control mechanism to adjust the robot’s speed relative to the wall adds flexibility to the approach. By smoothly varying the angular velocity based on the distance to the wall, the robot can follow walls without making sharp, jerky turns.

### Disadvantages

- **Small Spaces Wall Following**: While wall following is an effective strategy for exploring environments with clear boundaries, it can lead to local minima in certain situations. If the robot becomes trapped in a corner or narrow passage, it may struggle to navigate out of the confined space. Since the robot primarily reacts to LIDAR readings from its immediate surroundings, it can sometimes oscillate between closely spaced obstacles or follow the same wall repeatedly without exploring the rest of the area. 
- **Limited Global Awareness**: The current implementation does not incorporate any global awareness or mapping capabilities, meaning the robot does not know which parts of the environment it has already explored. This will lead to redundant coverage of the same areas, especially if the robot encounters stalls or obstacles that force it to backtrack.
- **Stall Recovery Challenges**: The robot tries to avoid stalls by preemptively slowing down and adjusting its direction when obstacles are detected. However, in some cases, the robot may still get stuck due to the nature of the environment or the robot's movement. The current implementation does not include a systematic stall recovery mechanism, which could limit the robot's ability to explore the entire environment effectively.

### Rationale for Approach

The chosen approach leverages a combination of dynamic obstacle avoidance and wall-following behavior to enable the robot to explore the apartment zones autonomously. By using LIDAR data to detect obstacles and adjust its movement accordingly, the robot can navigate the environment safely and efficiently. The wall-following behavior complements the obstacle avoidance mechanism by ensuring that the robot covers the space systematically without colliding with walls. This approach strikes a balance between reactive and proactive navigation strategies, allowing the robot to adapt to its surroundings while maintaining a consistent exploration pattern.

## Github

https://github.com/AveryUALibrary/f24_robotics/tree/main