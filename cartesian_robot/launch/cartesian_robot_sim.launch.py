from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    # Configurations
    robot_description_package = 'cartesian_robot'
    moveit_config_package = 'cartesian_robot_moveit_config'
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world_file = LaunchConfiguration('world_file')

    # Robot Description (xacro)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', description_file
    ])
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description]
    )

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items(),
    )

    # Gazebo Spawn Entity
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content, '-name', 'cartesian_robot']
    )

    # Bridge /clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # MoveIt Move Group
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                'launch',
                'move_group.launch.py'
            ])
        )
    )

    return [
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        clock_bridge,
        moveit_launch,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'description_file',
            default_value='/home/vishnu/urdf_tutorial/src/cartesian_robot/urdf/cartesian_urdf.xacro',
            description='Path to robot description xacro file'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot'),
                'config',
                'controllers.yaml'
            ]),
            description='Controller YAML file'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot_moveit_config'),
                'launch',
                'moveit.rviz'
            ]),
            description='RViz config file'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.sdf',
            description='Gazebo world file'
        ),
        OpaqueFunction(function=launch_setup)
    ])
