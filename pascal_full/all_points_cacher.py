#!/usr/bin/env python3
import os
import csv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point

from pascal_full.trajectory_cacher import SphereMotionPlanner


class BatchPlanner(Node):
    def __init__(self):
        super().__init__('batch_planner')

        # initialize the trajectory planner
        self.planner = SphereMotionPlanner()

        # compute output CSV path
        script_dir = os.path.dirname(os.path.realpath(__file__))
        ws_root    = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))
        self.out_path = os.path.join(ws_root, 'refined_map.csv')

        # 1) truncate any existing file (erase old data)
        open(self.out_path, 'w').close()

        # track whether we've written the header yet
        self.header_written = False

        # subscribe to the voxel‑center cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/workspace_voxel_centers',
            self.callback,
            10
        )

    def callback(self, msg: PointCloud2):
        # extract all centers from the incoming PointCloud2
        points = []
        for p in point_cloud2.read_points(msg,
                                          field_names=('x','y','z'),
                                          skip_nans=True):
            points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))

        # append new trajectory rows on each callback
        with open(self.out_path, 'a', newline='') as f:
            writer = csv.writer(f)

            for vid, center in enumerate(points):
                self.get_logger().info(
                    f"Planning for voxel {vid}: center=({center.x:.3f},"
                    f"{center.y:.3f},{center.z:.3f})"
                )

                # now returns (approach_traj, center_traj, score)
                approach_traj, center_traj, score = self.planner.plan_for_center(center)
                if approach_traj is None or center_traj is None:
                    self.get_logger().warn(f"No valid full plan for voxel {vid}")
                    continue

                # write header once, now that we know the joint names
                if not self.header_written:
                    joint_names = approach_traj.joint_names
                    header = ['id', 'segment', *joint_names, 'time_from_start_s', 'score']
                    writer.writerow(header)
                    self.header_written = True

                # write the home->approach plan
                for pt in approach_traj.points:
                    t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    row = [vid, 'approach', *pt.positions, t, score]
                    writer.writerow(row)

                # write the approach->center Cartesian path
                for pt in center_traj.points:
                    t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    row = [vid, 'center', *pt.positions, t, score]
                    writer.writerow(row)

        self.get_logger().info(f"Appended planning results for {len(points)} voxels")


def main(args=None):
    rclpy.init(args=args)
    node = BatchPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
