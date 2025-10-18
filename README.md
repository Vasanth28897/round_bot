# Round Bot in OUTDOOR 
This package is to implement the GPS localization and gps based waypoint follower in the outdoor simulated environment using ros2 navigation stack with MPPI controller and Timed Elastic Band(TEB) local planner. 

## Requirements
* Ubuntu - 22.04
* ROS2 - Humble
* Igntion gazebo - Gazebo fortress(Gazebo sim version 6.17.0)
* Timed Elastic Band(TEB) Local Planner
* Model Predictive Path Integral Controller(MPPI)

### Steps to build
* Create a workspace 
* Clone this round_bot repo under the src directory
    ```
    git repo -b round_bot_in_outdoor https://github.com/Vasanth28897/round_bot.git
    ```
* Clone the teb_local_planner(humble-devel branch) and costmap_converter(humble branch) package under the src directory.
    ```
    git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
    ```
    ```
    git clone -b humble https://github.com/rst-tu-dortmund/costmap_converter.git
    ```

* Build the workspace

* To spwan the robot in the world, launch this command

    ```bash
    ros2 launch round_bot bringup.launch.py
    ```

* Drive the round_bot using this command in the terminal

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

## Model added in the sdf file
* There is a model named(person standing) is added in the `sonoma.sdf` file. The model is available [here](https://app.gazebosim.org/OpenRobotics/fuel/models/Standing%20person). Download this model, and place it under this directory `/home/user/.ignition/fuel/fuel.gazebosim.org/openrobotics/models`. 

## Localization using GPS + IMU + Odometry
* Because of localizing the robot using GPS we don't need to use AMCL and map_server to localize, this is where the `EKF` and `Navsat` comes in and fuses the odometry, GPS, snd IMU and helps to localize the robot accurately in the outdoor environment. The `dual_ekf_navsat_params.yaml` under the `config` directory and `bringup.launch.py` is included in this launch file. Uncomment the rviz node if you want to open it.

    ```
    ros2 launch round_bot dual_ekf_navast.launch.py
    ```

## Mapviz
* To make sure the localization is done or not, using mapviz we can make sure the robot is localized. To visualize the satellite map for the sonoma world we can use the `Stadiamaps` on mapviz. `gps_wpf_demo.mvc` file is configured for it. Launch this file along with dual_ekf_navsat.launch.py file.
    ```
    ros2 launch round_bot mapviz.launch.py
    ```
![gps localization & Mapviz](https://github.com/Vasanth28897/round_bot/blob/round_bot_in_outdoor/docs/gps_localization_and_mapviz.png)

## Navigation
* There are no pre-generated maps used, `staic_layer`s are removed in both `local_costmap` and `global_costmaps`. `TEB` and `MPPI` local planner can be used in this repository, because why not? If you want to select a specific local planner for navigation, change the `nav2_params_file` in the `navigaiton.launch.py` file. The `nav2_params_teb.yaml` for TEB and `nav2_params_mppi.yaml` for MPPI file are stored under the `config` directory in the round_bot package.  
    ```
    ros2 launch round_bot navigation.launch.py
    ```
NOTE: When you want to test the navigation with TEB including the dynamic obstacle, you have to uncomment the below nodes in the `navigation.launch.py` file
* pose_frame_id_adder - added a frame id for the dynamic obstacle
* obstacle_processor - Publishes the obstacle topic
* scan_filter_node - removing the laserscan data which are falling on the moving model so it won't mark as lethal obstacle.

![navigation](https://github.com/Vasanth28897/round_bot/blob/round_bot_in_outdoor/docs/navigation.png)

## Waypoint Follower
### Logged Waypoint Follower
* `demo_waypoints.yaml` file have the gps coordinates for the waypoints, which is goals the robot has to reach one after another. Run this node along with `navigation.launch.py` file. If you want to monitor in mapviz, run this `mapviz.launch.py` also.
    ```
    ros2 run round_bot logged_waypoint_follower.py
    ```
![logged_waypoint_follower](https://github.com/Vasanth28897/round_bot/blob/round_bot_in_outdoor/docs/logged_waypoint_follower.mp4)

### Interactive Waypoint Follower
* To run this node you must have to run the `mapviz.launch.py` file along with the `navigation.launch.py` file. Click very near the robot somewhere in the mpaviz window. You can see The robot starts to move towards the goal.(some issues with this one, working on it)
    ```
    ros2 run round_bot interactive_waypoint_follower.py
    ```
![interactive_waypoint_follower](https://github.com/Vasanth28897/round_bot/blob/round_bot_in_outdoor/docs/interactive_waypoint_follower.mp4)

## TODO
There are some flaws in interactive_waypoint_follower node and the dynamic obstacle avoidance in both TEB local planner and MPPI controller. 