from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    launch_moveit = LaunchConfiguration('launch_moveit')
    launch_cabinet = LaunchConfiguration('launch_cabinet')
    cabinet_x = LaunchConfiguration('cabinet_x')
    cabinet_y = LaunchConfiguration('cabinet_y')
    cabinet_z = LaunchConfiguration('cabinet_z')
    cabinet_roll = LaunchConfiguration('cabinet_roll')
    cabinet_pitch = LaunchConfiguration('cabinet_pitch')
    cabinet_yaw = LaunchConfiguration('cabinet_yaw')

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('omr_bringup'), 'urdf', 'sim_combined.urdf.xacro']),
    ])

    cabinet_model = PathJoinSubstitution([
        FindPackageShare('omr_bringup'), 'urdf', 'environment', 'cabinet.sdf'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('launch_moveit', default_value='true'),
        DeclareLaunchArgument('launch_cabinet', default_value='true'),
        # With yaw=-pi/2 and (x,y)=(1.60,-0.30), the corrected hinge is at
        # world (0.61575,-0.900). Initial and swept collision clearance must
        # be revalidated in Gazebo after geometry changes.
        DeclareLaunchArgument('cabinet_x', default_value='1.60'),
        DeclareLaunchArgument('cabinet_y', default_value='-0.30'),
        DeclareLaunchArgument('cabinet_z', default_value='0.0'),
        # cabinet.sdf normalizes the source CAD axes to Gazebo's Z-up frame.
        DeclareLaunchArgument('cabinet_roll', default_value='0.0'),
        DeclareLaunchArgument('cabinet_pitch', default_value='0.0'),
        # Gazebo uses positive counter-clockwise yaw, so -pi/2 rotates the
        # cabinet clockwise by 90 degrees in the world XY plane.
        DeclareLaunchArgument('cabinet_yaw', default_value='-1.57079632679'),

        # Gazebo Fortress resource path for package:// → model:// resolution
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               PathJoinSubstitution([FindPackageShare('omr_bringup'), '..'])),

        # ── Gazebo Sim (Fortress) ─────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ]),
            launch_arguments=[('gz_args', ['-r ', PathJoinSubstitution([FindPackageShare('omr_bringup'), 'worlds', 'empty_with_physics.sdf'])])],
        ),

        # ── Clock bridge (Gazebo → ROS2 /clock) ──────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),

        # ── robot_state_publisher ────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str), 'use_sim_time': True}],
        ),

        # ── Spawn robot ──────────────────────────────────
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_omr',
            arguments=['-topic', 'robot_description', '-name', 'omr', '-z', '0.075'],
        ),

        # ── Articulated cabinet as an independent Gazebo entity ──
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_cabinet',
            arguments=[
                '-file', cabinet_model,
                '-name', 'cabinet',
                '-x', cabinet_x,
                '-y', cabinet_y,
                '-z', cabinet_z,
                '-R', cabinet_roll,
                '-P', cabinet_pitch,
                '-Y', cabinet_yaw,
            ],
            condition=IfCondition(launch_cabinet),
        ),

        # Keep the cabinet joint separate from the robot's /joint_states.
        # This avoids presenting cabinet_door_joint as part of the MoveIt robot.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cabinet_joint_state_bridge',
            arguments=[
                '/cabinet/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model]',
            ],
            condition=IfCondition(launch_cabinet),
            output='screen',
        ),

        # ── Camera bridge (Fortress → ROS2) ─────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            remappings=[
                ('/camera', '/camera/color/image_raw'),
                ('/camera_info', '/camera/color/camera_info'),
            ],
            output='screen',
        ),

        # ── Controller spawners (delayed 3s for robot to load) ──
        TimerAction(period=3.0, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['dais_joint_state_broadcaster', '--controller-manager', '/controller_manager']),
            Node(package='controller_manager', executable='spawner',
                 arguments=['dais_joint_trajectory_controller', '--controller-manager', '/controller_manager']),
        ]),

        # ── MoveIt2 move_group (optional) ────────────────
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        Command([
                            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                            PathJoinSubstitution([FindPackageShare('omr_bringup'),
                                                  'urdf', 'omr.urdf.xacro']),
                        ]),
                        value_type=str,
                    ),
                    'robot_description_semantic': ParameterValue(
                        Command([
                            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                            PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                                  'config', 'rm65_full.srdf']),
                        ]),
                        value_type=str,
                    ),
                    'use_sim_time': True,
                    'publish_robot_description_semantic': True,
                },
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'kinematics.yaml']),
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'ompl_planning.yaml']),
                PathJoinSubstitution([FindPackageShare('rm65_moveit_config'),
                                      'config', 'sim_controllers.yaml']),
            ],
            condition=IfCondition(launch_moveit),
            output='screen',
        ),
    ])
