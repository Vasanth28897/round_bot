from launch import LaunchDescription
import launch.actions
from launch.substitutions import Command,  LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'round_bot'
  pkg_share = get_package_share_directory(package_name)

  mapviz_config_file = os.path.join(pkg_share, "config", "gps_wpf_demo.mvc")

  mapviz_node = Node(
    package="mapviz",
    executable="mapviz",
    name="mapviz",
    output='screen',
    parameters=[{"config": mapviz_config_file},{"use_sim_time": True}]
  )

  initalize_origin = Node(
    package="swri_transform_util",
    executable="initialize_origin.py",
    name="initialize_origin",
    output='screen',
    parameters=[{'use_sim_time': True}],
    remappings=[
      ("fix", "navsat/fix"),
    ],
  )

  swri_transform = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="swri_transform",
    output='screen',
    parameters=[{'use_sim_time': True}],
    arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
  )

  return LaunchDescription([
    mapviz_node,
    initalize_origin,
    swri_transform,
  ])