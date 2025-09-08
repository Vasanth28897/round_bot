# Round Bot
This package contains creating 2d costmap and 3d costmap using STVL(Spatio Temporal Vortex Layer) for the navigation implemented on the round_bot in the simulation.

## Requirements
* Ubuntu - 22.04
* ROS2 - Humble

### Workflow for the 2d costmap and navigation
* To drive and play with the round_bot, launch this command

    ```bash
    ros2 launch round_bot bringup_launch.py
    
    ```

* Drive the round_bot using this command in the terminal

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    ```

* To generate a slam, use this command(use the teleop_twist_keyboard to drive and map the environment)

    ```bash
    ros2 launch round_bot slam_launch.py
    
    ```

* Use this command to save the map file after map is generated

    ```bash
    ros2 run nav2_map_server map_saver_cli -f map_folder/map_file_name
    
    ```


* To make the round_bot go autonomously, use this below command (Note : Don't forget to add the map_filename.yaml file in the navigation_launch.py file)

    ```bash
    ros2 launch round_bot navigation_launch.py
    
    ```

    the round bot localize automatically, you can send the goal using  `2d_goal` in the rviz, or you can send the `send_goal` command like this below in another terminal

    ```bash
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
    
    ```


### Workflow for the 3d costmap generation 
* To drive and play with the round_bot, launch this command

    ```bash
    ros2 launch round_bot bringup_with_depth_camera_launch.
    
    ```

* Drive the round_bot using this command in the terminal

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    ```

* To generate a slam, use this command(use the teleop_twist_keyboard to drive and map the environment)

    ```bash
    ros2 launch round_bot slam_with_depth_camera_launch.py
    
    ```

* Use this command to save the map file after map is generated

    ```bash
    ros2 run nav2_map_server map_saver_cli -f map_folder/map_file_name
    
    ```


* To make the round_bot go autonomously, use this below command (Note : Don't forget to add the map_filename.yaml file in the navigation_launch.py file)

    ```bash
    ros2 launch round_bot navigation_with_stvl_launch.py
    
    ```

    the round bot localize automatically, you can send the goal using  `2d_goal` in the rviz, or you can send the `send_goal` command like this below in another terminal

    ```bash
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
    
    ```

 
