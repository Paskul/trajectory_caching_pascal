#!/usr/bin/env python3
import os
import csv

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point

from pascal_full.trajectory_cacher import SphereMotionPlanner


class BatchPlanner(Node):
    def __init__(self):
        super().__init__('batch_planner')

        self.planner = SphereMotionPlanner()

        self.declare_parameter('csv_out_path',          '~/ros2_ws/refined_map.csv')
        self.declare_parameter('voxel_centers_topic',   '/workspace_voxel_centers')
        self.declare_parameter('cone_radius',     0.20)     # meters
        self.declare_parameter('cone_num_pts',    20)       # integer count

        self.csv_out_path =             self.get_parameter('csv_out_path').get_parameter_value().string_value
        self.voxel_centers_topic =      self.get_parameter('voxel_centers_topic').get_parameter_value().string_value
        self.cone_radius     =          self.get_parameter('cone_radius').get_parameter_value().double_value
        self.cone_num_pts    =          self.get_parameter('cone_num_pts').get_parameter_value().integer_value

        # truncate any existing file (erase old data)
        open(self.csv_out_path, 'w').close()

        # track whether we've written the header yet
        self.header_written = False

        # subscribe to the voxel‑center cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            self.voxel_centers_topic,
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
        with open(self.csv_out_path, 'a', newline='') as f:
            writer = csv.writer(f)

            for vid, center in enumerate(points):
                self.get_logger().info(
                    f"Planning for voxel {vid}: center=({center.x:.3f},"
                    f"{center.y:.3f},{center.z:.3f})"
                )

                approach_traj, center_traj, score = self.planner.plan_for_center(
                    center,
                    self.cone_num_pts,
                    self.cone_radius
                )
                
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
