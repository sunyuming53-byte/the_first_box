import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("omr_bringup")

    serial_port = LaunchConfiguration("serial_port")
    gate_count = LaunchConfiguration("gate_count")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("gate_count", default_value="1"),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    os.path.join(pkg_bringup, "config", "photogate_controllers.yaml"),
                    os.path.join(
                        pkg_bringup, "urdf", "photogate", "photogate.ros2_control.xacro"
                    ),
                    {"serial_port": serial_port},
                    {"gate_count": gate_count},
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            ),
        ]
    )
