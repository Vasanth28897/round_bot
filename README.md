# round_bot
Practice on Deploying a custom bot using ROS2-Humble

#### Workflow
* Launch the file using this command
    ros2 launch round_bot visualize.launch.py

* Drive the robot using this node
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

### Issue
Have to resolve this below one
* Message Filter dropping message: frame 'lidar_link' at time 69.890 for reason 'discarding message because the queue is full'.
* Robot is not moving in gazebo and rviz but wheels are rotating.
 
