from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # Add this import

def generate_launch_description():
    link_length_arg = DeclareLaunchArgument(
        name='link_length',
        default_value='1.0',
        description='Length of the robot links'
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare('scara_robot'),
        'urdf',
        'scara_urdf.xacro'
    ])

    # Command to process the xacro file
    robot_description = Command([
        'xacro', ' ', urdf_path, ' link_length:=', LaunchConfiguration('link_length')
    ])

    return LaunchDescription([
        link_length_arg,

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Robot State Publisher (robot_description param)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str)
            }],
            output='screen'
        ),

        # RViz visualization (delayed to ensure robot_state_publisher runs first)
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen'
                )
            ]
        )
    ])
