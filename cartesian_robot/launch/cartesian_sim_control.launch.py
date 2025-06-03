# cartesian_sim_control.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    world_file = LaunchConfiguration('world_file')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        description_file
    ])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument('description_file', description='Path to XACRO'),
        DeclareLaunchArgument('controllers_file', description='Controllers YAML'),
        DeclareLaunchArgument('world_file', description='Gazebo world file'),

        # ros2_control_node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers_file, {'use_sim_time': True}],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': True}],
            output='screen'
        ),

        # Delay then spawn joint_state_broadcaster
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        ),

        # Delay then spawn cartesian controller
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['cartesian_robot_slider_controller', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        ),

        # Gazebo simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ),
            launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items()
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-string', robot_description_content, '-name', 'cartesian_robot']
        ),

        # Bridge clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
    ])
