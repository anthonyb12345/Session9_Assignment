# Session9_Assignment

## Overview
This ROS 2 project involves multiple nodes and services to control and interact with a TurtleBot3 robot. The key components are:

1. **Wall Finder Service Server (`wall_finder_server`)**: This node finds the closest wall and navigates the robot towards it.
2. **Robot Driver Node (`robot_driver`)**: This node interacts with the Wall Finder Service to find a wall and then follows it.
3. **Lap Time Action Server (`lap_time_server`)**: This node measures the time taken for the robot to complete a lap around an arena.
4. **Lap Time Action Client (`lap_time_client.py`)**: This node interacts with the Lap Time Action Server to start and monitor the lap timing process.


## Launch File

### Functionality
- Launches all the nodes (`wall_finder_server`, `robot_driver`, `lap_time_server`, and `lap_time_client`) together.

## Instructions
### Install

### Build the ROS 2 Package

1. Navigate to your ROS 2 workspace:
    ```sh
    cd ~/ros2_ws
    ```
2. Build the workspace:
    ```sh
   colcon build
    ```
3. Source the setup script:
    ```sh
   source install/setup.bash
    ```
4. Set the TurtleBot3 model:
    ```sh
   export TURTLEBOT3_MODEL=burger
    ```
   
### Run the Launch File

   ```sh
   ros2 launch wall_follower_with_gazebo.py

   ```
