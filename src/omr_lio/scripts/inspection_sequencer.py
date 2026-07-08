#!/usr/bin/env python3
"""
Inspection Sequencer — NavigateToPose per waypoint with task handshake.

Usage:
  ros2 run omr_lio inspection_sequencer --ros-args -p waypoints_file:=/path/to/points.yaml
  ros2 topic pub /go_to_waypoint std_msgs/String "data: cabinet_A"
"""

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class InspectionSequencer(Node):
    def __init__(self):
        super().__init__('inspection_sequencer')

        # Params
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('nav_timeout', 60.0)
        self.declare_parameter('task_timeout', 180.0)

        waypoints_file = self.get_parameter('waypoints_file').value
        self.global_frame = self.get_parameter('global_frame').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.task_timeout = self.get_parameter('task_timeout').value

        # Load waypoints from YAML
        self.waypoints = {}
        if waypoints_file:
            self.load_waypoints(waypoints_file)
        else:
            self.get_logger().warn('No waypoints_file specified')

        # Nav2 ActionClient
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Task handshake
        self.task_event_pub = self.create_publisher(String, '/chassis/task_event', 10)
        self.task_done_sub = self.create_subscription(
            String, '/chassis/task_done', self.task_done_callback, 10)
        self._task_done_received = False
        self._task_done_msg = ''

        # Trigger subscription
        self.go_sub = self.create_subscription(
            String, '~/go_to_waypoint', self.go_callback, 10)

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

    def load_waypoints(self, path):
        """Load YAML: {waypoints: [{name: str, pose: {x,y,z,qx,qy,qz,qw}, task: str?}]}"""
        with open(path) as f:
            data = yaml.safe_load(f)
        for wp in data.get('waypoints', []):
            self.waypoints[wp['name']] = wp

    def task_done_callback(self, msg):
        data = msg.data.strip()
        self._task_done_received = True
        self._task_done_msg = data

    def go_callback(self, msg):
        """Handle trigger: navigate to named waypoint."""
        name = msg.data.strip()
        wp = self.waypoints.get(name)
        if not wp:
            self.get_logger().error(f'Unknown waypoint: {name}')
            return

        self.get_logger().info(f'Navigating to {name}...')
        self.navigate_to(wp)

    def navigate_to(self, wp):
        """Send NavigateToPose goal, wait for result, then handle task."""
        pose = wp['pose']
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.pose.position = Point(x=pose['x'], y=pose['y'], z=pose['z'])
        goal_msg.pose.pose.orientation = Quaternion(
            x=pose['qx'], y=pose['qy'], z=pose['qz'], w=pose['qw'])

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'NavigateToPose rejected for {wp["name"]}')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.nav_timeout)

        result = result_future.result()
        if result and result.result.error_code == 0:
            self.get_logger().info(f'Arrived at {wp["name"]}')
            self.handle_task(wp)
        else:
            self.get_logger().error(f'Navigation failed for {wp["name"]}')

    def handle_task(self, wp):
        """If waypoint has a task, publish task_event and wait for task_done."""
        task = wp.get('task', '')
        if not task or task == 'none':
            return

        name = wp['name']
        self.get_logger().info(f'Starting task: {task} at {name}')
        self.task_event_pub.publish(String(data=f'start:{task}:{name}'))

        # Wait for task_done
        self._task_done_received = False
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=self.task_timeout)

        while not self._task_done_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now() - start > timeout:
                self.get_logger().warn(f'Task timeout for {name}')
                break

        self.task_event_pub.publish(String(data=f'done:{task}:{name}'))
        self.get_logger().info(f'Task {task} completed at {name}')


def main():
    rclpy.init()
    node = InspectionSequencer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
