from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Keep LaunchConfiguration objects (do NOT call .perform() here)
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world_file = LaunchConfiguration('world_file')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # Generate robot_description from xacro file
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        TextSubstitution(text=' '),
        description_file
    ])
    robot_description = {'robot_description': robot_description_content}

    # ros2_control_node running the controller manager
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file, {'use_sim_time': True}],
        output='screen',
    )

    # Spawner for joint_state_broadcaster (wait for ros2_control_node to start)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner for cartesian_robot_slider_controller
    cartesian_robot_slider_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_robot_slider_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Register event handlers for controller spawning sequencing
    spawn_controllers_sequence = [
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ros2_control_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[cartesian_robot_slider_controller_spawner],
            )
        ),
    ]

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    # Launch Gazebo (ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ' + world_file.perform(context)]  # note: perform() here to join string properly
        }.items(),
    )

    # Spawn robot in Gazebo (delayed to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=['-string', robot_description_content, '-name', 'cartesian_robot']
            )
        ]
    )

    # Bridge /clock topic Gazebo <-> ROS2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen',
    )

    # MoveIt move_group launch with sim time
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cartesian_robot_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # RViz node, only launched if launch_rviz == true
    from launch.conditions import IfCondition
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(launch_rviz),
    )

    # Return launch description entities
    return [
        ros2_control_node,
        *spawn_controllers_sequence,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        clock_bridge,
        moveit_launch,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'description_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot'),
                'urdf',
                'cartesian_urdf.xacro'
            ]),
            description='URDF/XACRO robot description file',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot'),
                'config',
                'ros2_controllers.yaml'
            ]),
            description='ROS2 Control controllers config file',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_robot_moveit_config'),
                'launch',
                'moveit.rviz'
            ]),
            description='RViz configuration file for MoveIt',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.sdf',
            description='Gazebo world SDF file',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Whether to launch RViz',
        ),
        OpaqueFunction(function=launch_setup),
    ])
