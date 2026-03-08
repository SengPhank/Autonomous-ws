#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class VelodyneToZedMimic(Node):
    def __init__(self):
        super().__init__('zed_mimic_converter')
        # Use your original topic names
        self.sub = self.create_subscription(PointCloud2, '/gazebo_ros_velodyne/out', self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/zed/points', 10)
        self.get_logger().info("ZED Mimic Node Started: Using NumPy for high-speed conversion.")

    def callback(self, v_msg):
        # 1. Convert the raw byte data to a numpy uint8 array
        # This avoids the "buffer size must be a multiple" error by treating it as bytes first
        raw_data = np.frombuffer(v_msg.data, dtype=np.uint8)

        # 2. Reshape the data so each row is one point (point_step bytes wide)
        num_points = len(raw_data) // v_msg.point_step
        reshaped_data = raw_data[:num_points * v_msg.point_step].reshape(num_points, v_msg.point_step)

        # 3. Extract the first 12 bytes (X, Y, Z floats) for all points at once
        # This replaces the 'for i in range...' loop with a single operation
        xyz_data = reshaped_data[:, :12].copy()

        # 4. Create the ZED-style message
        z_msg = PointCloud2()
        z_msg.header = v_msg.header
        # Keeping your original frame name
        z_msg.header.frame_id = "zed_left_camera_frame" 
        
        z_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        z_msg.is_bigendian = False
        z_msg.point_step = 12 
        z_msg.height = 1 # We treat it as an unorganized (flat) cloud
        z_msg.width = num_points
        z_msg.is_dense = v_msg.is_dense
        
        # 5. Pack the binary data back into bytes
        z_msg.data = xyz_data.tobytes()
        z_msg.row_step = z_msg.point_step * z_msg.width
        
        self.pub.publish(z_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelodyneToZedMimic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()