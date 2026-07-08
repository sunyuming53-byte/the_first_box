"""
ROS2 launch file for M65 chassis navigation with LIO odometry + Nav2.

Starts FAST-LIO2 mapping (with optional Livox driver and estopper), then
launches Nav2 planning and control stack for differential drive navigation.

Usage:
  ros2 launch omr_lio m65_lio_nav.launch.py
  ros2 launch omr_lio m65_lio_nav.launch.py map:=/path/to/map.yaml
  ros2 launch omr_lio m65_lio_nav.launch.py launch_livox_driver:=true
  ros2 launch omr_lio m65_lio_nav.launch.py launch_estopper:=true

Note: M65 controller_manager and robot_state_publisher are launched
separately (via omr_bringup bringup.launch.py launch_m65:=true).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Args ──
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    map_yaml = DeclareLaunchArgument('map', default_value='')
    launch_livox = DeclareLaunchArgument(
        'launch_livox_driver', default_value='true',
        description='Launch livox_ros_driver2 alongside LIO'
    )
    launch_estopper = DeclareLaunchArgument(
        'launch_estopper', default_value='true',
        description='Launch estopper node'
    )

    # ── Include LIO mapping launch ──
    lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('omr_lio'),
                                  'launch', 'lio_mapping.launch.py'])
        ]),
        launch_arguments={
            'launch_livox_driver': LaunchConfiguration('launch_livox_driver'),
            'launch_estopper': LaunchConfiguration('launch_estopper'),
        }.items(),
    )

    nav2_params = PathJoinSubstitution([
        FindPackageShare('omr_lio'), 'config', 'nav2_params.yaml'
    ])

    # ── Nav2 nodes ──
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[nav2_params],
        remappings=[('/cmd_vel', '/m65_controller_manager/diff_drive_controller/cmd_vel')],
    )

    inspection_sequencer = Node(
        package='omr_lio',
        executable='inspection_sequencer',
        name='inspection_sequencer',
        output='screen',
        parameters=[{'waypoints_file': ''}],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        use_sim_time,
        map_yaml,
        launch_livox,
        launch_estopper,
        lio_launch,
        controller_server,
        planner_server,
        bt_navigator,
        velocity_smoother,
        inspection_sequencer,
        lifecycle_mgr,
    ])
