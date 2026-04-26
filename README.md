# Round Bot
This package contains docker to execute the gazebo harmonic and the robot model(round_bot) and execute the slam and navigation in the local system.

## Requirements
* ROS2 - Humble
* Docker
* ROS2 - Navigation

## Objective
* I wanted to run the gazebo Harmonic and spawn the robot model in the gazebo inside the docker container alone.
* The slam and Navigation exection happening in the local system(host). 
* To establish the communication between docker container and the local sytem.
* If you want to check the repo in the github -> 
   [round_bot](https://github.com/Vasanth28897/round_bot)


## Cloning the repo
* Create a workspace to build the package
* under the src directory clone the repo from github

   ``` bash
   git clone -b humble-gazebo-latest https://github.com/Vasanth28897/round_bot.git
   ``` 

## Execution
* This below command build the docker container and image, the `Dockerfile` is placed under the docker directory

    ```bash
    docker build -t <image_name> .
    ```

* Make sure the docker image is created by using this command `docker images`. You must see this output like this

    ```bash
    pc@pc:~/round_bot_ws$ docker images
    REPOSITORY                    TAG                     IMAGE ID       CREATED        SIZE
    ros2-humble-gazebo-harmonic   latest                  84740332c855   8 hours ago    5.71GB
    ros                           humble-ros-core-jammy   97ffc2601d2f   3 months ago   424MB
    ```

* To run the docker container and access the docker volume 

    ``` bash
    ./src/round_bot/docker/run_image.sh
    ```
    After this command executed, the terminal looks like this
    ``` bash
    pc@pc:~/$ ./src/round_bot/docker/run_image.sh 
    Attaching to container round_bot_dev...
    root@pc:~/round_bot_ws# 
    ```

* Build the package 
    ```bash
    colcon build --symlink-install
    ```

* Make sure the RMW_IMPLEMENTATION set in both the local and the docker `.bashrc` file. Run this command in the terminal to see which DDS is installed 
`echo $RMW_IMPLEMENTATION` it has to give output `rmw_cyclonedds_cpp`. Otherwise set this using this command and source it. 
    
    ```bash
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
    ``` 
    ```bash
    source ~/.bashrc
    ``` 

* set the local host in the local system
    ```bash
    xhost +local:root
    ```

* If the build is succesfull, source the workspace and run the launch file to launch the gazebo and spawn the robot

    ```bash
    ros2 launch round_bot bringup_launch.py
    ```

![Simulation Image](https://github.com/Vasanth28897/round_bot/blob/humble_gazebo_latest/docs/robot_spawn.png)

* Check the listed topics in the local system 
    
    ```
    pc@pc:~/round_bot__ws$ ros2 topic list
    /camera/image_raw
    /clicked_point
    /clock
    /cmd_vel
    /goal_pose
    /initialpose
    /joint_states
    /odom
    /parameter_events
    /robot_description
    /rosout
    /scan
    /tf
    /tf_static
    ```
    
* Check the listed topics in the docker 
    
    ```bash
    root@pc:~/round_bot_harmonic_ws# gz topic -l
    /camera/camera_info
    /camera/image_raw
    /clock
    /cmd_vel
    /gazebo/resource_paths
    /gui/camera/pose
    /gui/record_video/stats
    /joint_states
    /keyboard/keypress
    /marker
    /model/Mecanum_lift/cmd_vel
    /model/Mecanum_lift/odometry
    /odom
    /scan
    /scan/points
    /sensors/marker
    /stats
    /subt_performer_detector
    /tf
    /world/Edifice/clock
    /world/Edifice/dynamic_pose/info
    /world/Edifice/pose/info
    /world/Edifice/scene/deletion
    /world/Edifice/scene/info
    /world/Edifice/state
    /world/Edifice/stats
    ```
* you can drive the robot using this command inside the container and from the host by the topic `/cmd_vel`

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

## SLAM_TOOLBOX (run from the host)
* To generate a map, use this command(use the teleop_twist_keyboard to drive and map the environment). 

    ```bash
    ros2 launch round_bot slam_launch.py
    ```

* Use this command to save the map file after map is generated

    ```bash
    ros2 run nav2_map_server map_saver_cli -f workspace/src/round_bot/maps/map_file_name
    ```

## NAVIGATION (run from the host)
* To check the navigation, use this below command (Note : Don't forget to add the map_filename.yaml file in the navigation_launch.py file)

    ```bash
    ros2 launch round_bot navigation_launch.py
    ```

* Give the goal by pressing `2d_goal` and click anywhere in the map in rviz environment, or you can send the `send_goal` command like this below in another terminal

    ```bash
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
    ```

### ROUTE SERVER NAVIGATION with .geojson file
## Convert pgm to GeoTIFF 
* Instead of using QGIS georeferencer (which is for real-world GPS coordinates, cannot give gps for indoor), use GDAL to convert your PGM directly to a GeoTIFF using your map's origin and resolution

* Install this gdal 

    ```bash
    sudo apt install gdal-bin python3-gdal
    ```
* convert the pgm to tif with correct transform, this is the conversion calculation with the pgm file we have

    ```
    origin: [-15.1, -7.92, 0]   # this is lower-left corner
    resolution: 0.05
    map size: 602 x 307 pixels

    upper_left_x = -15.1
    upper_left_y = -7.92 + (307 * 0.05) = -7.92 + 15.35 = 7.43

    lower_right_x = -15.1 + (602 * 0.05) = -15.1 + 30.1 = 15.0
    lower_right_y = -7.92
    ```

* Conversion command

    ```bash
    gdal_translate \
        -of GTiff \
        -a_ullr -15.1 7.43 15.0 -7.92 \
        -a_srs EPSG:3857 \
        ~/ros2_ws/src/round_bot/maps/edifice.pgm \
        ~/ros2_ws/src/round_bot/maps/edifice.tif
    ```

* Open the tif file with QGIS, once the tif file is loaded, you can verify the co-ordinates when you hove over the map in gqis, the coordinates shown at the bottom matches the map coordinates(-15 to 15 in X, -8 to 7 in Y)

    ```bash
    qgis ~/ros2_ws/src/round_bot/maps/edifice.tif
    ```

* Then to add the nodes, edges and create a geojson file, follow [this](https://docs.nav2.org/tutorials/docs/route_server_tools/route_graph_generation.html) documentation from ros2. 

* Create a directory named `graphs` under the package and add the directory name in the `CMakelists.txt` file, we need to use the `export_shapefiles.py` from the nav2_route package, to export the .geojson file using `nodes.shp` and `edges.shp` which are created from the qgis. Run this command, after installing the geopandas

    ```bash
    python3 /opt/ros/humble/share/nav2_route/graphs/scripts/export_shapefiles.py \
        graphs/edifice_graph \
        edges.shp \
        nodes.shp
    ```

* By default the geojson file creates with date and time. you can keep it or not, upto you.

* Add the .geojson file path in the route_server like this

    ```
    graph_filepath: "/home/vasanth/ros2_ws/src/round_bot/graphs/edifice_graph.geojson"
    ```

* In Humble the `navigate_via_route` plugin in BT_navigator is not supported. So i created a python file to execute all the routes from the .geojson file and use navigate_to_pose makes the robot follows all of it. Run this python script along with `navigation.launch.py` file. Or you can make it as a node and run using ros2 run

    ```bash
    python3 ~/ros2_ws/src/round_bot/scripts/route_graph_patrol.py
    ```

## NOTE
* It doesn't matter if you add the route_server pluign in the params file or in the launch file, as long as you have the python file to make the robot follow the route_grah from .geojson file.
