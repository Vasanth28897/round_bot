import os
import xacro
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'round_bot'
  use_sim_time = True
  pkg_share = get_package_share_directory(package_name)

  xacro_file_path = os.path.join(pkg_share, 'description', 'round_bot.urdf.xacro')
  controller_params_file = os.path.join(get_package_share_directory(package_name),'config','control.yaml')
  robot_description = Command(['xacro ', xacro_file_path])
  
  params = {'robot_description': robot_description}
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', os.path.join(pkg_share, 'rviz', 'visualize.rviz')],
    output='screen'
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[params],
  )

  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='both',
    parameters=[{'robot_description': robot_description}],
  )

  joint_state_publisher_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='both',
    parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}],
  )

  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[controller_params_file],
    output='both',
  )

  delayed_controller_manager = TimerAction(
    period = 10.0,
    actions=[controller_manager]
  )

  diff_drive_controller = Node(
    package='controller_manager',
    executable='spawner',
    output='screen',
    arguments = ['diff_drive_base_controller', "--controller-manager", "/controller_manager"],
  )

  # delayed_diff_drive_controller = TimerAction(
  #   period=10.0,
  #   actions=[diff_drive_controller]
  # )

  # delayed_diff_drive_controller_handler = RegisterEventHandler(
  #   event_handler=OnProcessStart(
  #       target_action=controller_manager,
  #       on_start=[delayed_diff_drive_controller],
  #   )
  # ) #//same applies for joint_state_broadcaster and joint_state_publisher if contoller_manager is not delayed

  joint_state_broadcaster = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager"],
  )


  gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], #os.path.join(pkg_share, 'worlds', 'your_world_file.world')],
    output='screen'
  )

  spawn_entity = Node(
    package= 'gazebo_ros',
    executable= 'spawn_entity.py',
    name = 'urdf_spawner',
    output = 'screen',
    arguments = ['-topic', '/robot_description', '-entity', 'round_bot']
  )

  return LaunchDescription([
    rviz_node,
    robot_state_publisher,
    joint_state_publisher,
    joint_state_publisher_gui,
    delayed_controller_manager,
    diff_drive_controller,
    joint_state_broadcaster,
    #delayed_diff_drive_controller_handler,
    #delayed_joint_state_broadcaster_handler,
    gazebo,
    spawn_entity
  ])
