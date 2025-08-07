#!/usr/bin/env python3
"""
traj_csv_logger.py

A ROS2 node that listens on /pascal_cached_trajectory and /pascal_selected_trajectory,
then writes each pair of trajectories out to CSV files for offline comparison.
"""
import os
import csv
import datetime

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

class TrajCsvLogger(Node):
    def __init__(self):
        super().__init__('traj_csv_logger')
        self.cached_traj = None
        self.selected_traj = None

        # Subscribe to both topics
        self.create_subscription(
            JointTrajectory,
            '/pascal_cached_trajectory',
            self.cache_callback,
            10
        )
        self.create_subscription(
            JointTrajectory,
            '/pascal_selected_trajectory',
            self.selected_callback,
            10
        )
        self.get_logger().info('traj_csv_logger started, waiting for trajectories...')

    def cache_callback(self, msg: JointTrajectory):
        self.cached_traj = msg
        self.get_logger().info(f'Received cached trajectory ({len(msg.points)} points)')

    def selected_callback(self, msg: JointTrajectory):
        self.selected_traj = msg
        self.get_logger().info(f'Received selected trajectory ({len(msg.points)} points)')

        # Only write out when we have both
        if self.cached_traj is not None:
            self.write_csvs()
            # Reset so we only log once per pair
            self.cached_traj = None
            self.selected_traj = None

    def write_csvs(self):
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        # Filenames
        fn_cache = f'traj_{timestamp}_cached.csv'
        fn_selected = f'traj_{timestamp}_selected.csv'

        # Dump each trajectory
        self._dump_to_csv(self.cached_traj, fn_cache)
        self._dump_to_csv(self.selected_traj, fn_selected)

        self.get_logger().info(f'Wrote cached trajectory to {fn_cache}')
        self.get_logger().info(f'Wrote selected trajectory to {fn_selected}')

    def _dump_to_csv(self, traj: JointTrajectory, filename: str):
        # Ensure output directory exists
        out_dir = os.getcwd()
        path = os.path.join(out_dir, filename)
        with open(path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Header: joint names + time
            writer.writerow([*traj.joint_names, 'time_from_start_s'])
            # Rows: positions + time
            for pt in traj.points:
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                writer.writerow([*pt.positions, t])


def main(args=None):
    rclpy.init(args=args)
    node = TrajCsvLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
