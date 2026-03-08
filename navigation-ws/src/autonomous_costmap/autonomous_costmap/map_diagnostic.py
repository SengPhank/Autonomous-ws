#!/usr/bin/env python3
"""
Ultra-simple point cloud diagnostic.
Just prints the actual x,y,z coordinates to see if they change when you rotate.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

class PointCloudDiagnostic(Node):
    def __init__(self):
        super().__init__('pointcloud_diagnostic')
        self.declare_parameter('topic', '/zed/points')
        self.topic = str(self.get_parameter('topic').value)
        
        self.create_subscription(PointCloud2, self.topic, self.callback, 10)
        self.count = 0
        self.get_logger().info(f'Listening to {self.topic}')
        self.get_logger().info('Move robot forward, then rotate 90°, then move forward again.')
        self.get_logger().info('Watch if the X,Y coordinate ranges change.')

    def callback(self, msg: PointCloud2):
        self.count += 1
        
        # Extract points
        points = list(read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        
        if not points:
            return
        
        # Get min/max coordinates
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)
        
        # Print every 5th frame to reduce spam
        if self.count % 5 == 0:
            self.get_logger().info(
                f'Frame {self.count} | {len(points)} points | '
                f'X:[{min_x:.2f}, {max_x:.2f}] '
                f'Y:[{min_y:.2f}, {max_y:.2f}] '
                f'Z:[{min_z:.2f}, {max_z:.2f}] | '
                f'Frame: {msg.header.frame_id}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
Ultra-simple point cloud diagnostic.
Just prints the actual x,y,z coordinates to see if they change when you rotate.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

class PointCloudDiagnostic(Node):
    def __init__(self):
        super().__init__('pointcloud_diagnostic')
        self.declare_parameter('topic', '/zed/points')
        self.topic = str(self.get_parameter('topic').value)
        
        self.create_subscription(PointCloud2, self.topic, self.callback, 10)
        self.count = 0
        self.get_logger().info(f'Listening to {self.topic}')
        self.get_logger().info('Move robot forward, then rotate 90°, then move forward again.')
        self.get_logger().info('Watch if the X,Y coordinate ranges change.')

    def callback(self, msg: PointCloud2):
        self.count += 1
        
        # Extract points
        points = list(read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        
        if not points:
            return
        
        # Get min/max coordinates
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)
        
        # Print every 5th frame to reduce spam
        if self.count % 5 == 0:
            self.get_logger().info(
                f'Frame {self.count} | {len(points)} points | '
                f'X:[{min_x:.2f}, {max_x:.2f}] '
                f'Y:[{min_y:.2f}, {max_y:.2f}] '
                f'Z:[{min_z:.2f}, {max_z:.2f}] | '
                f'Frame: {msg.header.frame_id}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()