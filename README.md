# Round Bot
This package is created to simulate the dynamic obstacle avoidance using teb_local_planner in the simulation. This round bot uses the diff_drive system plugin. 

## Requirements
* Ubuntu - 22.04
* ROS2 - Humble
* Igntion gazebo - Gazebo fortress(Gazebo sim version 6.17.0)
* Timed Elastic Band(TEB) Local Planner

### Steps to build
* Create a workspace 
* Clone this round_bot repo under the src directory
* Clone the teb_local_planner(humble-devel branch) and costmap_converter(humble branch) package under the src directory.
    ```
    git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
    ```
    ```
    git clone -b humble https://github.com/rst-tu-dortmund/costmap_converter.git
    ```
* Build the workspace

* To drive and play with the round_bot, launch this command

    ```bash
    ros2 launch round_bot bringup_launch.py
    ```

* Drive the round_bot using this command in the terminal

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

## Model added in thes sdf file
* There is a model named(person standing) is added in the `edifice.sdf` file. The model is available [here](https://app.gazebosim.org/OpenRobotics/fuel/models/Standing%20person). Download this model, and place it under this directory `/home/user/.ignition/fuel/fuel.gazebosim.org/openrobotics/models`. 

* To generate a map, use this command(use the teleop_twist_keyboard to drive the robot and map the world). while generating the map, remember to comment out the include tag for the model in the edifice.sdf file.

    ```bash
    ros2 launch round_bot slam_launch.py
    ```

* Use this command to save the map file after mapping is generated

    ```bash
    ros2 run nav2_map_server map_saver_cli -f map_folder/map_file_name
    ```

## Localization
* The gz.msgs.Pose_V message type from the gazebo has no frame id in the header. `pose_frame_id_adder.py` is written to add the frame id as `map`, so the navigation part works fine.

* AMCL (Adaptive (or KLD-sampling) Monte Carlo localization) is used here to localize the robot in the robot. It localizes automatically, because the `set_initial_pose` parameter is set as `true` in the `localizaton.yaml` file. 

## Navigation
### Making the model as dynamic obstacle in the world
* The model(person standing) is added in the world, to make the model move and make it dynamic, a custom plugin `move_model.cpp` is written and the plugin is included in the `edifice.sdf` file. Which makes the model moves in loop along the given waypoints.

### Removing Lidar Data 
* Here 2D lidar is used, Obviously the lidar data will fall on the model(person standing) and marks it as lethal obstacle. A custom node is written `scan_filter_node.cpp` to consider as a clear space, where the lidar data which is falling on the model poses. So it won't mark as a lethal obstacle during navigation.

with Scan filter 
![scan_filter_on](https://github.com/Vasanth28897/round_bot/blob/new_gazebo_dynamic_obstacle/docs/scan_filter_on.gif)

without Scan filter
![scan_filter_on](https://github.com/Vasanth28897/round_bot/blob/new_gazebo_dynamic_obstacle/docs/scan_filter_off.gif)


### Obstacle Avoidance
* TEB local planner subscribes the `/obstacles` topic which has the ObsatcleArrayMsg data for the obstacles position and orientation datas of the dynamic obstacles. `obstacle_processor.py` subscribes the topic `/person_pose_info_with_frame` and publishes the topic 
`/obstacles`. Now the teb always knows where the dynamic obstacle is in the world, avoid if its in the way.

* To run the navigation, use this below command (Note : Don't forget to add the map_filename.yaml file in the navigation_launch.py file). `pose_frame_id_adder`, `scan_filter_node` and `obstacle_processor` node are included in the `navigation_launch.py` file.

    ```bash
    ros2 launch round_bot navigation_launch.py
    ```

![dynamic_obstacle_avoiding](https://github.com/Vasanth28897/round_bot/blob/new_gazebo_dynamic_obstacle/docs/avoid_dynamic_obstacle.gif)
