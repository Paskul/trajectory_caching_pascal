#!/usr/bin/env python3
"""
apple_pred.py

A ROS2 node that listens to synchronized color, depth, and camera_info topics,
runs YOLO detection on each color frame to find apples, and for each
apple bounding box computes and publishes the 3D position in the camera frame.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs_py.point_cloud2 as pc2

class AppleDepthNode(Node):
    def __init__(self):
        super().__init__('apple_depth_node')
        # Declare parameters
        self.declare_parameter('color_topic', '/camera/color/image_rect')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect')
        self.declare_parameter('info_topic', '/camera/color/camera_info_rect')
        self.declare_parameter('model_path', '/home/pascal/ros2_ws/src/pascal_full/models/best-merged-apples-thlo-merged-wsu-v8n.pt')
        self.declare_parameter('apple_points_topic', 'apple_cloud')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('iou', 0.6)

        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic  = self.get_parameter('info_topic').get_parameter_value().string_value
        model_path  = self.get_parameter('model_path').get_parameter_value().string_value
        self.apple_points_topic  = self.get_parameter('apple_points_topic').get_parameter_value().string_value
        self.conf   = self.get_parameter('conf').get_parameter_value().double_value
        self.iou    = self.get_parameter('iou').get_parameter_value().double_value

        # Load YOLO model
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Publisher: PointCloud2 of apple points
        self.pub = self.create_publisher(PointCloud2, self.apple_points_topic, 10)

        # Subscribers with time sync
        c_sub = Subscriber(self, Image, color_topic)
        d_sub = Subscriber(self, Image, depth_topic)
        i_sub = Subscriber(self, CameraInfo, info_topic)
        ats   = ApproximateTimeSynchronizer([c_sub, d_sub, i_sub], queue_size=10, slop=0.05)
        ats.registerCallback(self.callback)

        self.get_logger().info(f"Subscribed to {color_topic}, {depth_topic}, {info_topic}")

    def callback(self, color_msg, depth_msg, info_msg):
        # Convert to CV
        try:
            bgr   = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")
            return

        # Camera intrinsics
        K = info_msg.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # Detect apples
        results = self.model(bgr, conf=self.conf, iou=self.iou)
        if not results or not results[0].boxes:
            return
        boxes = results[0].boxes.xyxy.cpu().numpy()

        # Collect 3D points
        points = []  # list of (x,y,z)
        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            crop = depth[y1:y2, x1:x2]
            valid = crop[crop>0]
            if valid.size == 0:
                continue
            z = float(np.median(valid)) / 1000.0
            # pixel closest to median
            flat = crop.flatten(); mask = flat>0
            med = np.median(flat[mask]); diff = np.abs(flat - med); diff[~mask] = np.inf
            idx_flat = int(np.argmin(diff)); ry, rx = np.unravel_index(idx_flat, crop.shape)
            u, v = rx + x1, ry + y1
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points.append((x, y, z))

        if not points:
            return

        # Create PointCloud2
        header = color_msg.header
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud)
        self.get_logger().info(f"Published PointCloud2 with {len(points)} points")


def main(args=None):
    rclpy.init(args=args)
    node = AppleDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
