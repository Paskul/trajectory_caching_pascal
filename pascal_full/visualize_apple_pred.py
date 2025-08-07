#!/usr/bin/env python3
"""
visualize_apple_cloud.py

A ROS2 node that subscribes to a PointCloud2 of apple positions
and publishes a visualization_msgs/MarkerArray of red cube markers for RViz.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import sensor_msgs_py.point_cloud2 as pc2

class VisualizeAppleCloud(Node):
    def __init__(self):
        super().__init__('visualize_apple_cloud')
        # Subscribe to the apple PointCloud2
        self.sub = self.create_subscription(
            PointCloud2,
            'apple_cloud',
            self.cloud_callback,
            10
        )
        # Publisher for MarkerArray
        self.pub = self.create_publisher(MarkerArray, 'apple_pred_marker_arrays', 10)
        self.get_logger().info('Subscribed to apple_cloud; publishing MarkerArray on apple_pred_marker_arrays')

    def cloud_callback(self, cloud_msg: PointCloud2):
        # Read all points from the cloud
        points = list(pc2.read_points(cloud_msg, field_names=('x','y','z'), skip_nans=True))
        ma = MarkerArray()
        for idx, (x, y, z) in enumerate(points):
            marker = Marker()
            marker.header = cloud_msg.header
            marker.ns = 'apple_voxel'
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            # Position
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)
            marker.pose.orientation.w = 1.0
            # Cube size (e.g., 5cm)
            s = 0.05
            marker.scale.x = s
            marker.scale.y = s
            marker.scale.z = s
            # Red color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            # Lifetime so old markers expire automatically
            marker.lifetime = Duration(sec=1, nanosec=0)
            ma.markers.append(marker)

        # Publish all markers at once
        self.pub.publish(ma)
        self.get_logger().info(f'Published MarkerArray with {len(ma.markers)} apple voxels')


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeAppleCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
