#!/usr/bin/env python3
"""
workspace_voxelizer.py

ROS2 node to voxelize a workspace around a UR5e base_link and
publish it as a MarkerArray every second in RViz, with apple‑occupied
voxels highlighted.

Uses TF2 to transform incoming apple PointCloud2 (in camera_link frame)
into base_link frame before quantization into the voxel grid.

Ensures stale voxel highlights are cleared each publish, so only current
apple points are shown.

Additionally publishes a PointCloud2 of all voxel-center midpoints on
'workspace_voxel_centers' and dumps a voxel_map.csv.
"""
import os
import csv
import math
import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

# Optional import for point-cloud transforms
try:
    from tf2_sensor_msgs import do_transform_cloud
except ImportError:
    try:
        from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
    except ImportError:
        do_transform_cloud = None


class WorkspaceVoxelizer(Node):
    def __init__(self):
        super().__init__('workspace_voxelizer')

        # ─── declare parameters ─────────────────────────────
        self.declare_parameter('base_frame',     'base_link')
        self.declare_parameter('camera_frame',   'camera_link')
        self.declare_parameter('voxel_size',     0.15) #0.125
        self.declare_parameter('radius_max',     1.0) #1.2
        self.declare_parameter('z_min',          0.0)
        self.declare_parameter('z_max',          1.2) #1.4
        self.declare_parameter('cut_angle',     -18.435)  # degrees
        self.declare_parameter('push_from_base', 0.6)    # meters forward

        self.declare_parameter('voxel_markerarray_topic', 'workspace_voxels')
        self.declare_parameter('voxel_pointcloud_topic', 'workspace_voxel_centers')
        self.declare_parameter('apple_point_topic', 'apple_cloud')
        self.declare_parameter('voxel_map_path', '~/ros2_ws/voxel_map.csv')

        # ─── load parameters ───────────────────────────────
        self.base_frame   = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.voxel_size   = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.r_max        = self.get_parameter('radius_max').get_parameter_value().double_value
        self.z_floor      = self.get_parameter('z_min').get_parameter_value().double_value
        self.z_ceiling    = self.get_parameter('z_max').get_parameter_value().double_value
        self.apple_point_topic    = self.get_parameter('apple_point_topic').get_parameter_value().string_value

        self.cut_angle = self.get_parameter('cut_angle').get_parameter_value().double_value
        self.tan_theta = math.tan(math.radians(self.cut_angle))

        self.push_y = self.get_parameter('push_from_base').get_parameter_value().double_value

        self.voxel_markerarray_topic = self.get_parameter('voxel_markerarray_topic').get_parameter_value().string_value
        self.voxel_pointcloud_topic = self.get_parameter('voxel_pointcloud_topic').get_parameter_value().string_value
        self.voxel_map_path = self.get_parameter('voxel_map_path').get_parameter_value().string_value

        # TF buffer & listener
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # build the voxel grid & publishable markers
        self._init_voxels()

        # publishers & state
        self.pub            = self.create_publisher(MarkerArray, self.voxel_markerarray_topic, 1)
        self.cloud_pub      = self.create_publisher(PointCloud2, self.voxel_pointcloud_topic, 1)
        self.apple_voxel_ids = set()

        # subscribe & timer
        self.create_subscription(PointCloud2,
                                 self.apple_point_topic,
                                 self.apple_callback,
                                 10)
        self.create_timer(1.0, self.publish_voxels)

    def _init_voxels(self):
        # grid ranges in base_link frame
        xs = np.arange(-self.r_max, self.r_max + self.voxel_size, self.voxel_size)
        ys = np.arange(-self.r_max, self.r_max + self.voxel_size, self.voxel_size)
        zs = np.arange(self.z_floor,  self.z_ceiling + self.voxel_size, self.voxel_size)

        self.ma = MarkerArray()
        self.index_to_vid = {}
        vid = 0

        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                for k, z in enumerate(zs):
                    # distance from base_link origin
                    dist = math.sqrt(x*x + y*y + z*z)
                    in_sphere = (dist <= self.r_max)

                    # slanted‑plane slice (hinge offset forward in +Y)
                    dz = z - self.z_floor
                    if dz < 0:
                        in_slice = False
                    else:
                        # slice begins at y = push_y on the floor, then rises at angle θ
                        in_slice = (y >= self.push_y + dz * self.tan_theta)

                    # intersection: only voxels inside both sphere AND slice
                    if not (in_sphere and in_slice):
                        continue

                    # create a semi‑transparent green cube at (x,y,z)
                    m = Marker()
                    m.header.frame_id = self.base_frame
                    m.ns  = 'workspace'
                    m.id  = vid
                    m.type   = Marker.CUBE
                    m.action = Marker.ADD
                    m.pose.position.x = float(x)
                    m.pose.position.y = float(y)
                    m.pose.position.z = float(z)
                    m.pose.orientation.w = 1.0
                    m.scale.x = self.voxel_size
                    m.scale.y = self.voxel_size
                    m.scale.z = self.voxel_size
                    m.color.r = 0.0
                    m.color.g = 1.0
                    m.color.b = 0.0
                    m.color.a = 0.02
                    m.lifetime = Duration(sec=0, nanosec=0)

                    self.ma.markers.append(m)
                    self.index_to_vid[(i, j, k)] = vid
                    vid += 1

        self.get_logger().info(f'Initialized {len(self.ma.markers)} workspace voxels')

        # stash all centers for the PointCloud2
        self._voxel_centers = [
            [m.pose.position.x, m.pose.position.y, m.pose.position.z]
            for m in self.ma.markers
        ]

        # dump the ID→center map to CSV
        #script_dir = os.path.dirname(os.path.realpath(__file__))
        #ws_root    = os.path.abspath(os.path.join(script_dir, '..','..','..'))
        #csv_path   = os.path.join(ws_root, 'cache_map.csv')
        csv_path = self.voxel_map_path

        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['id','x','y','z'])
            for m in self.ma.markers:
                writer.writerow([m.id,
                                 m.pose.position.x,
                                 m.pose.position.y,
                                 m.pose.position.z])
        self.get_logger().info(f'📄 Wrote voxel map to {csv_path}')

    def apple_callback(self, cloud_msg: PointCloud2):
        if do_transform_cloud is None:
            self.get_logger().error(
                'do_transform_cloud not available: install tf2_sensor_msgs.'
            )
            return

        # transform into base_frame
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1)
            )
            cloud_base = do_transform_cloud(cloud_msg, t)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return

        # map points → voxels
        self.apple_voxel_ids.clear()
        for x, y, z in pc2.read_points(cloud_base,
                                       ('x','y','z'),
                                       skip_nans=True):
            # clip by height
            if not (self.z_floor <= z <= self.z_ceiling):
                continue

            dist = math.sqrt(x*x + y*y + z*z)
            # enforce the outer radius
            if dist > self.r_max:
                continue

            i = int(round((x + self.r_max) / self.voxel_size))
            j = int(round((y + self.r_max) / self.voxel_size))
            k = int(round((z - self.z_floor) / self.voxel_size))

            vid = self.index_to_vid.get((i, j, k))
            if vid is not None:
                self.apple_voxel_ids.add(vid)

        self.get_logger().info(f'Apple voxels now: {len(self.apple_voxel_ids)}')

    def publish_voxels(self):
        current_ids = set(self.apple_voxel_ids)
        self.apple_voxel_ids.clear()

        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        for m in self.ma.markers:
            mm = Marker()
            mm.header.frame_id = m.header.frame_id
            mm.header.stamp    = now
            mm.ns   = m.ns
            mm.id   = m.id
            mm.type = m.type
            mm.action = m.action
            mm.pose   = m.pose
            mm.scale  = m.scale

            if m.id in current_ids:
                mm.color.r = 1.0
                mm.color.g = 0.0
                mm.color.b = 0.0
                mm.color.a = 1.0
            else:
                mm.color = m.color

            mm.lifetime = m.lifetime
            arr.markers.append(mm)

        self.pub.publish(arr)
        self.get_logger().info(
            f'Published {len(arr.markers)} markers; {len(current_ids)} highlighted'
        )

        # publish voxel‑center cloud
        header = Header()
        header.stamp     = now
        header.frame_id  = self.base_frame
        cloud_msg = pc2.create_cloud_xyz32(header, self._voxel_centers)
        self.cloud_pub.publish(cloud_msg)
        self.get_logger().info(
            f'Published voxel‑centroid cloud with {len(self._voxel_centers)} points'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceVoxelizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
