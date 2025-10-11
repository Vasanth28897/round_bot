import os
from launch import LaunchDescription
from launch.substitutions import Command,  LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'round_bot'
  pkg_share = get_package_share_directory(package_name)

  xacro_file_path = os.path.join(pkg_share, 'description', 'round_bot.urdf.xacro')
  bridge_params = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
  world_file = os.path.join(pkg_share, "worlds", "sonoma.sdf")
  robot_description = Command(['xacro ', xacro_file_path])
  
  rviz_launch = LaunchConfiguration('rviz_launch', default='true')

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
  )
  
  # Include the Gazebo launch file, provided by the ros_gz_sim package
  gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true', 'use_sim_time': 'true'}.items()
  )

  # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
  spawn_entity = Node(package='ros_gz_sim', executable='create',
                      parameters=[{'use_sim_time': True}],
                      arguments=['-topic', 'robot_description',
                                  '-name', 'round_bot',
                                  # '-x', '-5.6',
                                  # '-y', '0.0',
                                  # '-z', '0.1'],
                                  '-x', '281',
                                  '-y', '-139',
                                  '-z', '2.72',
                                  '-Y', '-0.65'],
                      output='screen'
  )
  
  ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'use_sim_time': True}],
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}',
    ],
    output='screen'
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', os.path.join(pkg_share, 'rviz', 'visualize.rviz')],
    output='screen',
    parameters=[{'use_sim_time': True}],
    condition=IfCondition(rviz_launch),
  )

  return LaunchDescription([
    robot_state_publisher,
    gazebo,
    spawn_entity,
    ros_gz_bridge,
    rviz_node
  ])


