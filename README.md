# round_bot
Practice on Deploying a custom bot using nav2_stack(simulation) in ROS2-Humble

#### Workflow
* To drive and play with the round_bot, launch this command

    ```ros2 launch round_bot bringup_launch.py```

* Drive the round_bot using this command in the terminal

    ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```

* To generate a slam, use this command(use the teleop_twist_keyboard to drive and map the environment)

    ```ros2 launch round_bot slam_launch.py```

* Use this command to save the map file after map is generated

    ```ros2 run nav2_map_server map_saver_cli -f map_folder/map_file_name```


* To make the round_bot go autonomously, use this below command (Note : Don't forget to add the map_filename.yaml file in the navigation_launch.py file)

    ```ros2 launch round_bot navigation_launch.py```

    you have to localize the bot in the rviz using `2d_pose_estimate` and then give the `2d_goal`, or you can send the `send_goal` command like this below in another terminal

    ```ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"```

### TODO
* Implement a arm robot on top of the round_bot
 
