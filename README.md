# Round Bot in OUTDOOR 
This package is to implement the GPS localization and gps based waypoint follower in the outdoor simulated environment using ros2 navigation stack and 
dynamic obstacle avoidance using MPPI controller. 

## Requirements
* Ubuntu - 22.04
* ROS2 - Humble
* Igntion gazebo - Gazebo fortress(Gazebo sim version 6.17.0)

### Steps to build
* Create a workspace 
* Clone this round_bot repo under the src directory
    ```
    git repo -b round_bot_in_outdoor https://github.com/Vasanth28897/round_bot.git
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
* Because of localizing the robot using GPS we don't need to use AMCL and map_server to localize, this is where the `EKF` and `Navsat` comes in and fuses the three and helps to localize the robot accurately in the outdoor environment. The `dual_ekf_navsat_params.yaml` under the `config` directory and `bringup.launch.py` is included in this launch file. Uncomment the rviz node if you want to open it.

    ```
    ros2 launch round_bot dual_ekf_navast.launch.py
    ```

## Mapviz
* To make sure the localization is done or not, using mapviz we can make sure the robot is localized. To visualize the satellite map for the world we can use the `Stadiamaps` on mapviz. `gps_wpf_demo.mvc` file is configured for it. 
    ```
    ros2 launch round_bot mapviz.launch.py
    ```
![gps localization & Mapviz](https://github.com/Vasanth28897/round_bot/blob/new_gazebo_dynamic_obstacle/docs/gps_localization_and_mapviz.png)

## Navigation
* There are no pre-generated maps used, `staic_layer`s are removed in both `local_costmap` and `global_costmaps`. Right now `MPPI controller` is used in the `controller_server`. There are some issues come along when TEB is used. I am working on it to solve that. 
    ```
    ros2 launch round_bot navigation_no_map.launch.py
    ```

![navigation](https://github.com/Vasanth28897/round_bot/blob/new_gazebo_dynamic_obstacle/docs/navigation.png)


## Task completed so far
* GPS localization is working
* Static obstacle avoidance is done and for dynamic obstacle avoidance still need to tune the parameters. NOTE: My machine is not much capable of running high computation background work. Normally the real-time-factor in gazebo is like 70% in my system. when i include the dynamic model in the world, it does high computation. The real-time-factor goes down to 25%, which is not good for smooth performance. Therefore, i barely noticed the dynamic obstalce avoidance is working fine or not. But i tuned the parameter for that. If you have machine which is capable of running high computation with graphics card, please let me know, whether the dynamic obstacle is working good or still need to tune parameter which makes it work.
* MPPI controller is used in the local planner(no modifications done in the code, but tuning the parameters mostly make the planner works perfect). 

## Task not completed yet
* GPS based Waypoint follower(working on it).
* I am trying to implement the dynamic obstacle avoidance using TEB local planner, but it is so senstitive to timestamps when looking up transforms. I am wokring on it. If that issue is solved, i can use the TEB.