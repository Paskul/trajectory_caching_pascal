#!/usr/bin/env python3

import os
import csv

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ClickPointListener(Node):
    """
    Subscribes to /clicked_point, maps click to voxel ID via voxel_map.csv,
    loads that ID's approach or center trajectory from refined_map.csv
    (depending on user choice), and publishes a JointTrajectory on
    /pascal_selected_trajectory.
    """
    def __init__(self):
        super().__init__('click_point_listener')

        # CSV paths (assume we're in workspace root)
        cwd = os.getcwd()
        self.voxel_map_path   = os.path.join(cwd, 'voxel_map.csv')
        self.refined_map_path = os.path.join(cwd, 'refined_map.csv')

        # load voxel coordinates
        self.voxels = []
        self._load_voxel_map()

        # ask once which segment to execute
        choice = None
        while choice not in ('approach', 'center'):
            choice = input("Select segment to execute ('approach' or 'center'): ").strip()
        self.segment = choice

        # publisher for selected trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/pascal_cached_trajectory',
            10
        )

        # subscribe to clicked_point
        self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.callback,
            10
        )

    def _load_voxel_map(self):
        if not os.path.isfile(self.voxel_map_path):
            self.get_logger().error(f"voxel_map.csv not found at {self.voxel_map_path}")
            return
        with open(self.voxel_map_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    vid = int(row['id'])
                    x = float(row['x']); y = float(row['y']); z = float(row['z'])
                    self.voxels.append({'id': vid, 'x': x, 'y': y, 'z': z})
                except Exception as e:
                    self.get_logger().warn(f"Skipping invalid voxel row {row}: {e}")
        self.get_logger().info(f"Loaded {len(self.voxels)} voxels")

    def callback(self, msg: PointStamped):
        cx, cy, cz = msg.point.x, msg.point.y, msg.point.z
        self.get_logger().info(f"Clicked point: ({cx:.3f},{cy:.3f},{cz:.3f})")

        # find exact or closest voxel
        voxel_id = None
        for v in self.voxels:
            if (abs(v['x']-cx)<1e-6 and
                abs(v['y']-cy)<1e-6 and
                abs(v['z']-cz)<1e-6):
                voxel_id = v['id']
                break
        if voxel_id is None:
            best = min(self.voxels,
                       key=lambda v: (v['x']-cx)**2 + (v['y']-cy)**2 + (v['z']-cz)**2)
            voxel_id = best['id']
            self.get_logger().warn(f"No exact match; using closest ID {voxel_id}")
        else:
            self.get_logger().info(f"Exact voxel ID {voxel_id}")

        # load and publish trajectory from refined_map.csv
        if not os.path.isfile(self.refined_map_path):
            self.get_logger().error(f"refined_map.csv not found at {self.refined_map_path}")
            return

        with open(self.refined_map_path, newline='') as f:
            reader = csv.DictReader(f)
            fields = reader.fieldnames or []
            # check required columns
            required = {'id','segment','time_from_start_s','score'}
            if not required.issubset(fields):
                self.get_logger().error(f"refined_map.csv missing required columns: {fields}")
                return
            # determine joint names: all except id, segment, time, score
            joint_names = [h for h in fields
                           if h not in ('id','segment','time_from_start_s','score')]

            traj = JointTrajectory()
            traj.joint_names = joint_names

            for row in reader:
                try:
                    rid = int(row['id'])
                except ValueError:
                    continue
                if rid != voxel_id:
                    continue
                if row['segment'] != self.segment:
                    continue

                # build one JointTrajectoryPoint
                positions = [float(row[j]) for j in joint_names]
                t = float(row['time_from_start_s'])
                pt = JointTrajectoryPoint()
                pt.positions = positions
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
                traj.points.append(pt)

        if not traj.points:
            self.get_logger().warn(f"No '{self.segment}' trajectory found for voxel ID {voxel_id}")
            return

        self.get_logger().info(
            f"Publishing '{self.segment}' trajectory with {len(traj.points)} points for ID {voxel_id}"
        )
        
        self.traj_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = ClickPointListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
