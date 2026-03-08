#!/usr/bin/env python3
"""
Global costmap - SIMPLIFIED VERSION.
Removed: distance-based boosting and ground filtering hacks
Rationale: Slope-based traversability + spatial collision checking is sufficient

How it works:
- Vertical walls (steep slopes) → naturally appear as RED (impassable)
- Flat ground (zero slope) → naturally appears as GREEN (traversable)
- Rover collision avoidance is path planner's job, not costmap's job
- No magic numbers, no hard-coded distances
"""
import math
from typing import Dict, Tuple
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_ros import TransformListener, Buffer

class GlobalCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__('global_costmap')
        
        # Parameters
        self.declare_parameter('input_pointcloud_topic', '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('height_map_topic', '/height_map')
        self.declare_parameter('traversability_topic', '/height_traversability_costmap')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('resolution', 0.20)
        self.declare_parameter('width_m', 100.0)
        self.declare_parameter('height_m', 100.0)
        self.declare_parameter('min_z', -1.0)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('min_points_per_cell', 2)
        self.declare_parameter('height_norm_min', -1.0)
        self.declare_parameter('height_norm_max', 1.5)
        self.declare_parameter('free_slope_deg', 5.0)
        self.declare_parameter('lethal_slope_deg', 25.0)
        self.declare_parameter('unknown_as_obstacle', True)
        
        # Load parameters
        self.input_topic = str(self.get_parameter('input_pointcloud_topic').value)
        self.height_map_topic = str(self.get_parameter('height_map_topic').value)
        self.traversability_topic = str(self.get_parameter('traversability_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_points = int(self.get_parameter('min_points_per_cell').value)
        self.height_norm_min = float(self.get_parameter('height_norm_min').value)
        self.height_norm_max = float(self.get_parameter('height_norm_max').value)
        self.free_slope_deg = float(self.get_parameter('free_slope_deg').value)
        self.lethal_slope_deg = float(self.get_parameter('lethal_slope_deg').value)
        self.unknown_as_obstacle = bool(self.get_parameter('unknown_as_obstacle').value)
        
        # Grid state
        self.sums: Dict[Tuple[int, int], float] = {}
        self.counts: Dict[Tuple[int, int], int] = {}
        self.grid_origin_x = None
        self.grid_origin_y = None
        self.grid_initialized = False
        
        # Publishers
        self.height_pub = self.create_publisher(OccupancyGrid, self.height_map_topic, 10)
        self.trav_pub = self.create_publisher(OccupancyGrid, self.traversability_topic, 10)
        
        # TF listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriber
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, 10)
        
        self.get_logger().info(
            f'global_costmap ready (slope-based, no distance hacks) | input={self.input_topic}'
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        """
        Process point cloud with pure slope-based traversability.
        No distance-based hacks, no ground filtering magic numbers.
        """
        
        source_frame = msg.header.frame_id
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                source_frame,
                rclpy.time.Time()
            )
        except Exception as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return
        
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # Convert quaternion to yaw angle
        q = rot
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Initialize grid on first cloud
        if not self.grid_initialized:
            self.grid_origin_x = trans.x - (self.width_m / 2.0)
            self.grid_origin_y = trans.y - (self.height_m / 2.0)
            self.grid_initialized = True
            self.get_logger().info(
                f'Grid initialized at origin: ({self.grid_origin_x:.2f}, {self.grid_origin_y:.2f})'
            )
        
        max_range_sq = self.max_range * self.max_range
        
        # Process each point - simple, no hacks
        for x, y, z in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            # Filter by height and distance (basic filtering only)
            if z < self.min_z or z > self.max_z:
                continue
            if (x * x + y * y) > max_range_sq:
                continue
            
            # Transform to odom frame (rotate + translate)
            x_rot = x * cos_yaw - y * sin_yaw
            y_rot = x * sin_yaw + y * cos_yaw
            
            point_odom_x = x_rot + trans.x
            point_odom_y = y_rot + trans.y
            
            # Bin into grid
            cell_x = int((point_odom_x - self.grid_origin_x) / self.resolution)
            cell_y = int((point_odom_y - self.grid_origin_y) / self.resolution)
            
            grid_width_cells = int(self.width_m / self.resolution)
            grid_height_cells = int(self.height_m / self.resolution)
            
            if 0 <= cell_x < grid_width_cells and 0 <= cell_y < grid_height_cells:
                cell_key = (cell_x, cell_y)
                if cell_key not in self.sums:
                    self.sums[cell_key] = 0.0
                    self.counts[cell_key] = 0
                
                self.sums[cell_key] += float(z)
                self.counts[cell_key] += 1
        
        # Publish maps
        self._publish_maps(msg)

    def _publish_maps(self, cloud_msg: PointCloud2) -> None:
        """Publish height and traversability maps."""
        
        if not self.grid_initialized or not self.sums:
            return
        
        grid_width_cells = int(self.width_m / self.resolution)
        grid_height_cells = int(self.height_m / self.resolution)
        cell_count = grid_width_cells * grid_height_cells
        
        # Build mean heights
        mean_heights = [math.nan] * cell_count
        for (cx, cy), sum_z in self.sums.items():
            count = self.counts[(cx, cy)]
            if count >= self.min_points:
                idx = cy * grid_width_cells + cx
                mean_heights[idx] = sum_z / count
        
        # Publish height map
        self._publish_height_map(cloud_msg, mean_heights, grid_width_cells, grid_height_cells)
        
        # Publish traversability map
        self._publish_traversability_map(cloud_msg, mean_heights, grid_width_cells, grid_height_cells)

    def _publish_height_map(self, cloud_msg: PointCloud2, mean_heights, width: int, height: int) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = cloud_msg.header.stamp
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = self.grid_origin_x
        grid.info.origin.position.y = self.grid_origin_y
        grid.info.origin.orientation.w = 1.0
        
        hmin = self.height_norm_min
        hmax = self.height_norm_max
        span = max(1e-6, hmax - hmin)
        
        data = [-1] * len(mean_heights)
        for idx, h in enumerate(mean_heights):
            if not math.isnan(h):
                normalized = int(round(((h - hmin) / span) * 100.0))
                data[idx] = max(0, min(100, normalized))
        
        grid.data = data
        self.height_pub.publish(grid)

    def _publish_traversability_map(self, cloud_msg: PointCloud2, mean_heights, width: int, height: int) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = cloud_msg.header.stamp
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = self.grid_origin_x
        grid.info.origin.position.y = self.grid_origin_y
        grid.info.origin.orientation.w = 1.0
        
        data = [-1] * len(mean_heights)
        free_slope = self.free_slope_deg
        lethal_slope = max(self.lethal_slope_deg, free_slope + 1e-6)
        neighbors = [
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),           (1, 0),
            (-1, 1),  (0, 1),  (1, 1),
        ]
        
        for my in range(height):
            for mx in range(width):
                idx = my * width + mx
                h = mean_heights[idx]
                
                if math.isnan(h):
                    data[idx] = 100 if self.unknown_as_obstacle else -1
                    continue
                
                # Compute max slope to neighbors (pure slope-based, no distance hacks)
                max_slope_deg = 0.0
                for dx, dy in neighbors:
                    nx, ny = mx + dx, my + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        nidx = ny * width + nx
                        nh = mean_heights[nidx]
                        if not math.isnan(nh):
                            dz = abs(h - nh)
                            dist_factor = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                            dist = self.resolution * dist_factor
                            slope_deg = math.degrees(math.atan2(dz, dist))
                            max_slope_deg = max(max_slope_deg, slope_deg)
                
                # Convert slope to cost (no magic numbers, pure traversability)
                if max_slope_deg <= free_slope:
                    cost = 0
                elif max_slope_deg >= lethal_slope:
                    cost = 100
                else:
                    ratio = (max_slope_deg - free_slope) / (lethal_slope - free_slope)
                    cost = int(round(ratio * 99.0))
                
                data[idx] = max(0, min(100, cost))
        
        grid.data = data
        self.trav_pub.publish(grid)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()