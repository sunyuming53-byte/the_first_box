#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Waypoint Follower for omr_lio — ROS2 Humble.

Reads a CSV waypoint file (from record_waypoints) and navigates the robot
through all waypoints using Nav2's FollowWaypoints action. Handles task
waypoints (stop_Ns, detect, ext:xxx) by publishing to /chassis/task_event
and waiting for /chassis/task_done.

Usage:
  ros2 run omr_lio waypoint_follower --ros-args -p waypoints_file:=/path/to/waypoints.csv
"""

import csv
import math
import os
import re

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion


class WaypointFollower(Node):
    """Follow a CSV waypoint path via Nav2 FollowWaypoints, handling tasks inline."""

    def __init__(self):
        super().__init__('waypoint_follower')

        # ---- Parameters ----
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('odom_topic', '/lio/odom')
        self.declare_parameter('task_event_topic', '/chassis/task_event')
        self.declare_parameter('task_done_topic', '/chassis/task_done')
        self.declare_parameter('detect_pause_time', 2.0)
        self.declare_parameter('external_task_timeout', 180.0)
        self.declare_parameter('goal_tolerance', 0.30)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.task_event_topic = self.get_parameter('task_event_topic').value
        self.task_done_topic = self.get_parameter('task_done_topic').value
        self.detect_pause_time = self.get_parameter('detect_pause_time').value
        self.external_task_timeout = self.get_parameter('external_task_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Resolve CSV path
        if self.waypoints_file.strip() == '':
            self.waypoints_file = self._find_latest_csv()
        if not os.path.isfile(self.waypoints_file):
            raise FileNotFoundError(f'Waypoints file not found: {self.waypoints_file}')

        # ---- Load waypoints ----
        self.waypoints = self._load_waypoints(self.waypoints_file)
        if len(self.waypoints) == 0:
            raise RuntimeError('No waypoints loaded from CSV.')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {self.waypoints_file}')

        # ---- Find task waypoints ----
        self.task_indices = [
            i for i, wp in enumerate(self.waypoints) if wp['task'] != 'none'
        ]
        self.get_logger().info(f'{len(self.task_indices)} task waypoints: '
                               f'{[self.waypoints[i]["task"] for i in self.task_indices]}')

        # ---- Action client (Nav2 FollowWaypoints) ----
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._goal_handle = None
        self._result_future = None

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('FollowWaypoints action server not available — is Nav2 running?')
        self.get_logger().info('Nav2 FollowWaypoints action server connected.')

        # ---- Task event pub/sub ----
        self._task_event_pub = self.create_publisher(String, self.task_event_topic, 10)
        self._task_done_received = False
        self._task_done_sub = self.create_subscription(
            String, self.task_done_topic, self._task_done_cb, 10)

        # ---- Odom subscriber for progress ----
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._has_odom = False
        self._odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 100)

    # ================================================================
    # Internal helpers
    # ================================================================

    def _find_latest_csv(self):
        """Find the most recent CSV in the data directory."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.normpath(os.path.join(script_dir, '..', 'data'))
        if not os.path.isdir(data_dir):
            raise FileNotFoundError(f'Data directory not found: {data_dir}')
        csv_files = [os.path.join(data_dir, f) for f in os.listdir(data_dir)
                     if f.endswith('.csv')]
        if not csv_files:
            raise FileNotFoundError(f'No CSV files found in: {data_dir}')
        csv_files.sort(key=os.path.getmtime, reverse=True)
        return csv_files[0]

    def _load_waypoints(self, path):
        """Load waypoints from CSV into a list of dicts."""
        waypoints = []
        with open(path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    wp = {
                        'seq': int(row['seq']),
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'z': float(row.get('z', 0.0)),
                        'qx': float(row.get('qx', 0.0)),
                        'qy': float(row.get('qy', 0.0)),
                        'qz': float(row.get('qz', 0.0)),
                        'qw': float(row.get('qw', 1.0)),
                        'yaw': float(row.get('yaw', 0.0)),
                        'task': row.get('task', 'none').strip(),
                        'tol': float(row.get('tol', self.goal_tolerance)),
                        'frame_id': row.get('frame_id', 'map').strip(),
                    }
                    waypoints.append(wp)
                except (ValueError, KeyError) as e:
                    self.get_logger().warn(f'Skipping bad CSV row: {e}')
        return waypoints

    # ================================================================
    # Callbacks
    # ================================================================

    def _odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self._current_x = pose.position.x
        self._current_y = pose.position.y
        q = pose.orientation
        _, _, self._current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._has_odom = True

    def _task_done_cb(self, msg: String):
        self._task_done_received = True
        self.get_logger().debug(f'Task done signal received: "{msg.data}"')

    # ================================================================
    # FollowWaypoints goal lifecycle
    # ================================================================

    def _build_pose_stamped(self, wp):
        """Convert a waypoint dict to a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = wp['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=wp['x'], y=wp['y'], z=0.0)
        pose.pose.orientation = Quaternion(
            x=wp['qx'], y=wp['qy'], z=wp['qz'], w=wp['qw'])
        return pose

    def _send_segment(self, waypoints):
        """Send a list of waypoints as a single FollowWaypoints goal (blocking)."""
        if len(waypoints) == 0:
            return True

        goal = FollowWaypoints.Goal()
        goal.poses = [self._build_pose_stamped(wp) for wp in waypoints]

        self.get_logger().info(
            f'Sending FollowWaypoints goal with {len(goal.poses)} poses '
            f'(seq {waypoints[0]["seq"]} .. {waypoints[-1]["seq"]})')

        # Send goal (async, then block)
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.done():
            self.get_logger().error('Timeout waiting for goal acceptance')
            return False

        self._goal_handle = send_future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by FollowWaypoints server')
            return False

        self.get_logger().info('Goal accepted, navigating...')

        # Wait for result
        self._result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._result_future, timeout_sec=300.0)

        if not self._result_future.done():
            self.get_logger().warn('Navigation timed out, cancelling goal')
            self._cancel_goal()
            return False

        result = self._result_future.result()
        if result is None:
            self.get_logger().warn('Navigation returned no result')
            return False

        missed = result.result.missed_waypoints if result.result else []
        if missed:
            self.get_logger().warn(f'{len(missed)} waypoints were missed: {missed}')
        else:
            self.get_logger().info('All waypoints in segment reached.')

        return True

    def _cancel_goal(self):
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            self.get_logger().info('Goal cancelled')

    # ================================================================
    # Task execution
    # ================================================================

    def _publish_task_event(self, phase, task_name, idx):
        """Publish task event: {phase}:{task_name}:idx{idx}."""
        msg = String()
        msg.data = f'{phase}:{task_name}:idx{idx}'
        self._task_event_pub.publish(msg)
        self.get_logger().info(f'Task event: {msg.data}')

    def _sleep_with_spin(self, seconds):
        """Sleep while keeping ROS2 spinning."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        rate = self.create_rate(20)
        while rclpy.ok() and self.get_clock().now() < end:
            rate.sleep()

    def _execute_stop_task(self, task_name, stop_seconds, idx):
        """Handle a stop_Ns task — wait for N seconds."""
        self.get_logger().info(f'[TASK] stop {stop_seconds:.1f}s')
        self._publish_task_event('start', task_name, idx)
        self._sleep_with_spin(stop_seconds)
        self._publish_task_event('done', task_name, idx)

    def _execute_detect_task(self, idx):
        """Handle a detect task — pause for detect_pause_time seconds."""
        self.get_logger().info(f'[TASK] detect (pause {self.detect_pause_time:.1f}s)')
        self._publish_task_event('start', 'detect', idx)
        self._sleep_with_spin(self.detect_pause_time)
        self._publish_task_event('done', 'detect', idx)

    def _execute_external_task(self, ext_name, idx):
        """Handle an external (ext:xxx) task — wait for /chassis/task_done."""
        self.get_logger().info(
            f'[TASK] external {ext_name} (timeout {self.external_task_timeout:.0f}s)')
        self._task_done_received = False
        self._publish_task_event('start', ext_name, idx)

        deadline = self.get_clock().now() + Duration(seconds=self.external_task_timeout)
        self._task_done_received = False
        rate = self.create_rate(5)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self._task_done_received:
                self.get_logger().info(f'[TASK] external done: {ext_name}')
                break
            rate.sleep()
        else:
            if rclpy.ok():
                self.get_logger().warn(f'[TASK] external timeout: {ext_name}')

        self._publish_task_event('done', ext_name, idx)

    def _execute_task(self, task_name, idx):
        """Dispatch a task by its name."""
        if task_name == 'none' or task_name == '':
            return

        # stop_Ns — e.g. stop_5s, stop_10s
        stop_match = re.match(r'stop_(\d+(?:\.\d+)?)s?$', task_name)
        if stop_match:
            seconds = float(stop_match.group(1))
            self._execute_stop_task(task_name, seconds, idx)
            return

        # detect — pause for configurable time
        if task_name == 'detect':
            self._execute_detect_task(idx)
            return

        # ext:xxx — external task module
        if task_name.startswith('ext:'):
            external_name = task_name[4:].strip()
            if external_name == '':
                self.get_logger().warn('Empty external task name, skipping.')
                return
            self._execute_external_task(external_name, idx)
            return

        # Unknown task — skip with warning
        self.get_logger().warn(f'[TASK] unknown task type "{task_name}", skipping.')
        self._publish_task_event('skip', task_name, idx)

    # ================================================================
    # Main execution
    # ================================================================

    def _find_next_task_boundary(self, start_idx):
        """Find the index of the next task waypoint (inclusive), or last index."""
        for i in range(start_idx, len(self.waypoints)):
            if self.waypoints[i]['task'] != 'none':
                return i
        return len(self.waypoints) - 1

    def run(self):
        """Navigate through all waypoints, handling tasks at task waypoints.

        Waypoints are grouped into segments delimited by task waypoints.
        Each segment is sent as a single FollowWaypoints goal to Nav2.
        When a task waypoint is reached (the last pose in a segment),
        the task is executed before continuing to the next segment.
        """
        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints to follow.')
            return

        # Wait for odometry
        self.get_logger().info('Waiting for odometry...')
        for _ in range(50):  # ~5s timeout
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._has_odom:
                break

        if not self._has_odom:
            self.get_logger().warn('No odometry received — continuing without position monitoring')

        idx = 0
        while idx < len(self.waypoints) and rclpy.ok():
            # Find the next task waypoint boundary (inclusive)
            segment_end = self._find_next_task_boundary(idx)
            segment = self.waypoints[idx:segment_end + 1]

            if len(segment) == 0:
                break

            has_task = segment[-1]['task'] != 'none'

            # Navigate the segment
            success = self._send_segment(segment)
            if not success:
                self.get_logger().error(f'Navigation failed for segment starting at seq={segment[0]["seq"]}')
                break

            idx = segment_end + 1

            # Handle task at this waypoint
            if has_task:
                task_wp = segment[-1]
                self._execute_task(task_wp['task'], task_wp['seq'])

        self.get_logger().info('All waypoints completed!')

    def destroy_node(self):
        self._cancel_goal()
        self.get_logger().info('Waypoint follower shutdown.')
        super().destroy_node()


def main():
    rclpy.init()
    node = WaypointFollower()
    try:
        node.run()
    except Exception as e:
        node.get_logger().fatal(f'waypoint_follower error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
