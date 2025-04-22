from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    link_length_arg = DeclareLaunchArgument(
        name='link_length',
        default_value='1.0',
        description='Length of the robot links'
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare('cartesian_robot'),
        'urdf',
        'cartesian_urdf.xacro'
    ])

    robot_description = Command([
        'xacro', ' ', urdf_path, ' link_length:=', LaunchConfiguration('link_length')
    ])

    return LaunchDescription([
        link_length_arg,

        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),       
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
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
