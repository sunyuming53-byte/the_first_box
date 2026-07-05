from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_ip', default_value='192.168.1.18',
                              description='IP address of the robot arm'),
        DeclareLaunchArgument('board_type', default_value='charuco',
                              description='Calibration board type: charuco, chessboard, aruco'),
        DeclareLaunchArgument('board_rows', default_value='7',
                              description='Number of inner corners (rows) on the board'),
        DeclareLaunchArgument('board_cols', default_value='5',
                              description='Number of inner corners (cols) on the board'),
        DeclareLaunchArgument('board_square_mm', default_value='40.0',
                              description='Square side length in millimetres'),
        DeclareLaunchArgument('board_marker_mm', default_value='30.0',
                              description='Aruco marker side length in millimetres (charuco/aruco only)'),
        DeclareLaunchArgument('mode', default_value='interactive',
                              description='Calibration mode: interactive, auto, batch'),
        DeclareLaunchArgument('session_dir', default_value='',
                              description='Directory to store/load calibration session data'),

        Node(
            package='realman_calibration',
            executable='calib_node',
            name='calib_node',
            parameters=[{
                'arm_ip': LaunchConfiguration('arm_ip'),
                'board_type': LaunchConfiguration('board_type'),
                'board_rows': LaunchConfiguration('board_rows'),
                'board_cols': LaunchConfiguration('board_cols'),
                'board_square_mm': LaunchConfiguration('board_square_mm'),
                'board_marker_mm': LaunchConfiguration('board_marker_mm'),
                'mode': LaunchConfiguration('mode'),
                'session_dir': LaunchConfiguration('session_dir'),
            }],
        ),
    ])
