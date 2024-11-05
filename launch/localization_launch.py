import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'round_bot'
  pkg_share = get_package_share_directory(package_name)
  nav2_pkg_share = get_package_share_directory('nav2_bringup')

  lifecycle_nodes = ['map_server', 'amcl']
  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

  localize_rviz_launch = LaunchConfiguration('rviz_launch', default='true') 

  localization_params_file = os.path.join(pkg_share,'config','localization.yaml')
  map_yaml_file = os.path.join(pkg_share, 'maps', 'office_floor.yaml') # the map file path can be given in the yaml file, if the map file path mentioned there, no need to give in the nav2_map_server node
  rviz_file = os.path.join(pkg_share, 'rviz', 'localization.rviz')
  bring_up = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory(package_name),'launch','bringup_launch.py'
              )]),
              launch_arguments={'rviz_launch': 'false'}.items() 
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file],
    output='screen',
    parameters=[{'use_sim_time': True}],
    condition=IfCondition(localize_rviz_launch),
  )

  nav2_map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    respawn_delay=2.0,
    parameters=[localization_params_file, {'yaml_filename': map_yaml_file}, {'use_sim_true': True}],
    remappings=remappings,
  )
  nav2_amcl = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    respawn_delay=2.0,
    parameters=[localization_params_file, {'use_sim_true': True}],
    remappings=remappings,
  )
  nav2_lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_localization',
    output='screen',
    parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}, {'use_sim_true': True}],
  )

  localization = IncludeLaunchDescription(
                  PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                      [nav2_pkg_share, 'launch', 'localization_launch.py'])),
                  launch_arguments={'map': map_yaml_file,
                                    'use_sim_time': 'true',
                                    'params_file': localization_params_file}.items())
  
  return LaunchDescription([
    rviz_node,
    bring_up,
    nav2_map_server,
    nav2_amcl,
    nav2_lifecycle_manager
    #localization # - use this one or the three above
  ])



