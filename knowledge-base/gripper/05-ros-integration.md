# 05 — ROS Integration

## Source

`assets/ros/crt_ctag2f90d_gripper_visualization.zip` — ROS1 package (catkin) with URDF + STL meshes for visualization in RViz and Gazebo.

## Package Structure

```
crt_ctag2f90d_gripper_visualization/
├── package.xml          # catkin package (ROS1), depends on roslaunch, robot_state_publisher, rviz, gazebo
├── CMakeLists.txt
├── launch/
│   └── display.launch   # Launch: joint_state_publisher_gui + robot_state_publisher + rviz
├── urdf/
│   ├── crt_ctag2f90d_gripper_visualization_sync.urdf   # URDF model
│   └── crt_ctag2f90d_gripper_visualization.csv
├── meshes/              # STL files for each link
│   ├── base_link.STL
│   ├── Left_1_Link.STL
│   ├── Left_2_Link.STL
│   ├── Left_Support_Link.STL
│   ├── Right_1_Link.STL
│   ├── Right_2_Link.STL
│   └── Right_Support_Link.STL
├── config/
│   └── joint_names_crt_ctag2f90d_visualization.yaml
├── urdf.rviz            # RViz config
└── docs/                # README + screenshots
```

## Launch (ROS1)

```bash
roslaunch crt_ctag2f90d_gripper_visualization display.launch
```

This opens RViz with the gripper URDF loaded and `joint_state_publisher_gui` for manual joint control.

## Migrating to ROS2

The package is **catkin (ROS1)**. To use in this ROS2 Humble workspace:

1. Convert URDF — the URDF format is the same; just copy to a ROS2 package
2. Convert launch file from XML to Python
3. Replace `joint_state_publisher_gui` → `joint_state_publisher_gui` (ROS2 port available)
4. Replace `robot_state_publisher` → `robot_state_publisher` (ROS2 port available)
5. Use `ros2 run rviz2 rviz2` instead of `rosrun rviz rviz`

### Example ROS2 Launch (Python)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('crt_ctag2f90d_gripper_visualization')
    urdf = os.path.join(pkg_dir, 'urdf', 'crt_ctag2f90d_gripper_visualization_sync.urdf')

    return LaunchDescription([
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': open(urdf).read()}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg_dir, 'urdf.rviz')]),
    ])
```

## Gripper Joints

From the URDF, the gripper has a parallel jaw mechanism with:
- `base_link` → fixed base
- Left finger: `Left_Support_Link` → `Left_1_Link` → `Left_2_Link`
- Right finger: `Right_Support_Link` → `Right_1_Link` → `Right_2_Link`

For a pipeline, you likely only need a simplified single-DOF joint (open/close).
