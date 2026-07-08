#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Waypoint Recorder for omr_lio — ROS2 Humble.

Subscribes to LiDAR odometry (/lio/odom) and records poses as CSV waypoints
whenever the robot moves beyond a minimum distance threshold.

Usage:
  ros2 run omr_lio record_waypoints
  ros2 run omr_lio record_waypoints --ros-args -p min_distance:=0.5
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf_transformations import euler_from_quaternion


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('record_waypoints')

        # ---------- Parameters ----------
        self.declare_parameter('odom_topic', '/lio/odom')
        self.declare_parameter('task_topic', '/waypoint_task')
        self.declare_parameter('stop_seconds_topic', '/waypoint_stop_seconds')
        self.declare_parameter('min_distance', 0.30)
        self.declare_parameter('default_task', 'none')
        self.declare_parameter('default_tol', 0.30)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('file_name', '')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        self.stop_seconds_topic = self.get_parameter('stop_seconds_topic').value
        self.min_distance = self.get_parameter('min_distance').value
        self.default_task = self.get_parameter('default_task').value
        self.default_tol = self.get_parameter('default_tol').value
        self.output_dir_param = self.get_parameter('output_dir').value
        self.file_name = self.get_parameter('file_name').value

        # Resolve output directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_output_dir = os.path.normpath(os.path.join(script_dir, '..', 'data'))
        self.output_dir = self.output_dir_param if self.output_dir_param else default_output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        if self.file_name.strip() == '':
            time_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.file_name = f'waypoints_{time_str}.csv'

        self.csv_path = os.path.join(self.output_dir, self.file_name)

        # ---------- State ----------
        self.seq = 0
        self.last_saved_x = None
        self.last_saved_y = None
        self.last_saved_yaw = None
        self.first_msg_received = False

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_qx = None
        self.current_qy = None
        self.current_qz = None
        self.current_qw = None
        self.current_yaw = None
        self.current_frame_id = ''
        self.current_msg = None

        # ---------- Open CSV ----------
        self.csv_file = open(self.csv_path, mode='w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'seq', 'stamp', 'frame_id',
            'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw',
            'yaw', 'task', 'tol'
        ])
        self.csv_file.flush()

        # ---------- Subscription ----------
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 100)

        self.task_sub = self.create_subscription(
            String, self.task_topic, self.task_callback, 10)  # /waypoint_task

        self.stop_sub = self.create_subscription(
            Float32, self.stop_seconds_topic, self.stop_seconds_callback, 10)  # /waypoint_stop_seconds

        self.get_logger().info('=' * 50)
        self.get_logger().info('Waypoint recorder started.')
        self.get_logger().info(f'  Odom topic : {self.odom_topic}')
        self.get_logger().info(f'  Task topic : {self.task_topic}')
        self.get_logger().info(f'  Stop topic : {self.stop_seconds_topic}')
        self.get_logger().info(f'  Output csv : {self.csv_path}')
        self.get_logger().info(f'  Min dist   : {self.min_distance:.3f} m')
        self.get_logger().info('=' * 50)

    def odom_callback(self, msg: Odometry):
        self.current_msg = msg

        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])

        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.current_qx = qx
        self.current_qy = qy
        self.current_qz = qz
        self.current_qw = qw
        self.current_yaw = yaw
        self.current_frame_id = msg.header.frame_id

        if not self.first_msg_received:
            self.first_msg_received = True
            self.get_logger().info('First odometry message received.')

        should_save = False
        reason = ''

        if self.last_saved_x is None:
            should_save = True
            reason = 'first_point'
        else:
            dx = x - self.last_saved_x
            dy = y - self.last_saved_y
            dist = math.hypot(dx, dy)

            if dist >= self.min_distance:
                should_save = True
                reason = f'distance={dist:.3f}m'

        if should_save:
            self.save_waypoint(task=self.default_task, tol=self.default_tol,
                               reason=reason)

    def task_callback(self, msg: String):
        if self.current_msg is None:
            self.get_logger().warn('No odometry received yet, cannot mark task waypoint.')
            return

        task_name = msg.data.strip()
        if task_name == '':
            self.get_logger().warn('Received empty task mark, ignored.')
            return

        self.save_waypoint(task=task_name, tol=self.default_tol,
                           reason=f'manual_task={task_name}')

    def stop_seconds_callback(self, msg: Float32):
        if self.current_msg is None:
            self.get_logger().warn('No odometry received yet, cannot mark stop waypoint.')
            return

        self.save_waypoint(task=f'stop_{msg.data}s', tol=self.default_tol,
                           reason=f'manual_stop={msg.data}s')

    def save_waypoint(self, task, tol, reason=''):
        if self.current_msg is None:
            return

        x = self.current_x
        y = self.current_y
        z = 0.0  # Always save z=0 for waypoint following on 2D plane
        qx = self.current_qx
        qy = self.current_qy
        qz = self.current_qz
        qw = self.current_qw
        yaw = self.current_yaw

        if self.last_saved_x is not None:
            dx = x - self.last_saved_x
            dy = y - self.last_saved_y
            dist = math.hypot(dx, dy)
            dyaw = abs(wrap_to_pi(yaw - self.last_saved_yaw))
            if dist < 1e-6 and dyaw < 1e-6:
                return

        frame_id = self.current_frame_id
        stamp_sec = (float(self.current_msg.header.stamp.sec) +
                     float(self.current_msg.header.stamp.nanosec) * 1e-9)

        self.writer.writerow([
            self.seq,
            f'{stamp_sec:.6f}',
            frame_id,
            f'{x:.6f}',
            f'{y:.6f}',
            f'{z:.6f}',
            f'{qx:.6f}',
            f'{qy:.6f}',
            f'{qz:.6f}',
            f'{qw:.6f}',
            f'{yaw:.6f}',
            task,
            f'{tol:.3f}'
        ])

        self.seq += 1
        self.csv_file.flush()
        os.fsync(self.csv_file.fileno())

        self.last_saved_x = x
        self.last_saved_y = y
        self.last_saved_yaw = yaw

        self.get_logger().info(
            f'[{self.seq - 1:04d}] saved | x={x:.3f} y={y:.3f} yaw={yaw:.3f} | task={task} | {reason}')

    def destroy_node(self):
        try:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
        except Exception:
            pass

        self.get_logger().info('=' * 50)
        self.get_logger().info('Waypoint recorder stopped.')
        self.get_logger().info(f'  Saved csv: {self.csv_path}')
        self.get_logger().info(f'  Total waypoints: {self.seq}')
        self.get_logger().info('=' * 50)
        super().destroy_node()


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
