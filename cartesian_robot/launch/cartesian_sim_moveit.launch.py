from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


from launch.actions import TimerAction

def launch_setup(context, *args, **kwargs):
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world_file = LaunchConfiguration('world_file')
    moveit_launch_file = LaunchConfiguration('moveit_launch_file')
    launch_rviz = LaunchConfiguration('launch_rviz')

    control_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cartesian_robot"),
                "launch",
                "cartesian_sim_control.launch.py"
            ])
        ),
        launch_arguments={
            "description_file": description_file.perform(context),
            "controllers_file": controllers_file.perform(context),
            "world_file": world_file.perform(context),
        }.items()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file.perform(context)),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file.perform(context)],
        condition=IfCondition(launch_rviz)
    )

    # Delay MoveIt and RViz start by 5 seconds (to allow controllers to initialize)
    delayed_moveit_and_rviz = TimerAction(
        period=5.0,
        actions=[moveit_launch, rviz_node]
    )

    return [control_sim_launch, delayed_moveit_and_rviz]



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'description_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot'),
                'urdf',
                'cartesian_urdf.xacro'
            ]),
            description='Path to XACRO file'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot'),
                'config',
                'ros2_controllers.yaml'
            ]),
            description='ROS2 Control YAML config'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot_moveit_config'),
                'launch',
                'moveit.rviz'
            ]),
            description='RViz config for MoveIt'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.sdf',
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'moveit_launch_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot_moveit_config'),
                'launch',
                'move_group.launch.py'
            ]),
            description='MoveIt launch file'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        OpaqueFunction(function=launch_setup)
    ])
