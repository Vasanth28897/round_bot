# round_bot
Practice on Deploying a custom bot using ROS2-Humble

#### Workflow
* To drive and play with the round_bot, launch this command

    ```ros2 launch round_bot bringup_launch.py```

* Drive the round_bot using this command in the terminal

    ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```

* To generate a slam, use this command(use the teleop_twist_keyboard to drive and map the environment)

    ```ros2 launch round_bot slam_launch.py```

* To localise and navigate the round_bot, use this command

    ```ros2 launch round_bot navigation_launch.py```

    you have to localize the bot in the rviz using 2d_pose_estimate and then give the 2d_goal.

### ISSUE
* The gobal_costmap and local_costmap are not showing the color variation in the rviz.

### TODO
* Implement a arm robot on top of the round_bot
 
