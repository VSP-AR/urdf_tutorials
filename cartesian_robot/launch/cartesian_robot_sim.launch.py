import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cartesian_robot')

    # Paths to your robot's URDF and MoveIt config
    robot_urdf = os.path.join(pkg_share, 'urdf', 'cartesian_robot.urdf.xacro')
    moveit_config_pkg = 'cartesian_robot_moveit_config'  # adjust if your MoveIt config is separate
    moveit_config_path = os.path.join(pkg_share, 'cartesian_robot_moveit_config')  # adjust accordingly

    # Launch Gazebo with your robot spawn launch file or directly spawn the model
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'cartesian_robot'],
        output='screen'
    )

    # Start the MoveIt move_group node with your MoveIt config
    # Path to MoveIt config package
    moveit_config_path = get_package_share_directory('cartesian_robot_moveit_config')

    # Include MoveIt move_group.launch.py
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_path, 'launch', 'move_group.launch.py')
        )
    )

    # RViz for MoveIt
    rviz_config_file = os.path.join(moveit_config_path, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Controller manager or specific controllers can be launched here if you have dedicated launch files

    return LaunchDescription([
        gazebo_launch,
        spawn_entity,
        move_group_launch,
        rviz_node,
    ])
