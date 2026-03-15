#!/usr/bin/env python3
"""
Global Costmap Node
===================
Publishes a single persistent /global_costmap OccupancyGrid in the odom
frame, suitable for Nav2 global path planning.

Key design decisions:
  - Single output topic (/global_costmap) — local costmap is Nav2's job
  - unknown_as_obstacle defaults FALSE — a 150x150m map is mostly unseen;
    painting unknown as lethal makes the whole map pink and unnavigable
  - Grid origin is anchored on the first *successful* TF lookup, not on
    the first cloud received (which may arrive before TF is ready)
  - Zero-depth guard rejects (0,0,0) points from ZED corrupted frames
  - Welford online variance with outlier rejection and observation cap
  - Full 3x3 rotation matrix (not yaw-only) for correct pitch/roll handling
  - Timestamp-matched TF with fallback to latest
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_ros import TransformException, TransformListener, Buffer


# ---------------------------------------------------------------------------#
# Geometry
# ---------------------------------------------------------------------------#

def quat_to_rotation_matrix(qx, qy, qz, qw):
    x2, y2, z2 = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [
        [1-2*(y2+z2),  2*(xy-wz),   2*(xz+wy)  ],
        [2*(xy+wz),    1-2*(x2+z2), 2*(yz-wx)  ],
        [2*(xz-wy),    2*(yz+wx),   1-2*(x2+y2)],
    ]

def apply_transform(px, py, pz, R, tx, ty, tz):
    return (
        R[0][0]*px + R[0][1]*py + R[0][2]*pz + tx,
        R[1][0]*px + R[1][1]*py + R[1][2]*pz + ty,
        R[2][0]*px + R[2][1]*py + R[2][2]*pz + tz,
    )


# ---------------------------------------------------------------------------#
# Per-cell accumulator
# ---------------------------------------------------------------------------#

CELL_OBS_CAP = 200   # stop updating after this many observations
MAX_Z_DELTA  = 1.0   # reject points whose Z deviates more than this from cell mean

class CellStats:
    __slots__ = ('n', 'mean', 'M2', 'z_min', 'z_max', 'last_seen')

    def __init__(self):
        self.n:         int   = 0
        self.mean:      float = 0.0
        self.M2:        float = 0.0
        self.z_min:     float = math.inf
        self.z_max:     float = -math.inf
        self.last_seen: float = 0.0

    def update(self, z: float, now_sec: float) -> None:
        if self.n >= CELL_OBS_CAP:
            return
        if self.n > 0 and abs(z - self.mean) > MAX_Z_DELTA:
            return  # outlier — corrupted frame remnant
        self.n += 1
        delta      = z - self.mean
        self.mean += delta / self.n
        self.M2   += delta * (z - self.mean)
        if z < self.z_min:
            self.z_min = z
        if z > self.z_max:
            self.z_max = z
        self.last_seen = now_sec

    @property
    def variance(self) -> float:
        return self.M2 / self.n if self.n > 1 else 0.0

    @property
    def z_range(self) -> float:
        return max(0.0, self.z_max - self.z_min)


# ---------------------------------------------------------------------------#
# Nav2 cost constants
# ---------------------------------------------------------------------------#
COST_FREE      = 0
COST_INSCRIBED = 99
COST_LETHAL    = 100
# -1 in OccupancyGrid.data (int8) → nav2 treats as unknown

NEIGHBORS = [(-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)]


# ---------------------------------------------------------------------------#
# Node
# ---------------------------------------------------------------------------#

class GlobalCostmapNode(Node):

    def __init__(self):
        super().__init__('global_costmap')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('input_pointcloud_topic',  '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('output_costmap_topic',    '/global_costmap')
        self.declare_parameter('frame_id',                'odom')

        self.declare_parameter('resolution',              0.10)
        self.declare_parameter('width_m',                 150.0)
        self.declare_parameter('height_m',                150.0)

        self.declare_parameter('min_z',                   -1.0)
        self.declare_parameter('max_z',                    2.5)
        self.declare_parameter('min_range',                0.3)   # ZED 2i min reliable depth
        self.declare_parameter('max_range',               12.0)
        self.declare_parameter('min_points_per_cell',      3)

        self.declare_parameter('free_slope_deg',           5.0)
        self.declare_parameter('lethal_slope_deg',        20.0)
        self.declare_parameter('max_variance',             0.05)
        self.declare_parameter('lethal_variance',          0.15)
        self.declare_parameter('vertical_obstacle_height', 0.15)

        # FALSE by default — a mostly-unseen 150x150m map painted lethal
        # is not useful. Unknown cells are left as -1 (nav2: unknown/free).
        self.declare_parameter('unknown_as_obstacle',      False)

        self.declare_parameter('robot_radius',             0.35)
        self.declare_parameter('cell_decay_sec',           0.0)
        self.declare_parameter('tf_timeout_sec',           0.1)

        p = self.get_parameter
        self.input_topic         = str(p('input_pointcloud_topic').value)
        self.output_topic        = str(p('output_costmap_topic').value)
        self.frame_id            = str(p('frame_id').value)

        self.resolution          = float(p('resolution').value)
        self.width_m             = float(p('width_m').value)
        self.height_m            = float(p('height_m').value)

        self.min_z               = float(p('min_z').value)
        self.max_z               = float(p('max_z').value)
        self.min_range           = float(p('min_range').value)
        self.max_range           = float(p('max_range').value)
        self.min_points          = int(p('min_points_per_cell').value)

        self.free_slope_deg      = float(p('free_slope_deg').value)
        self.lethal_slope_deg    = float(p('lethal_slope_deg').value)
        self.max_variance        = float(p('max_variance').value)
        self.lethal_variance     = float(p('lethal_variance').value)
        self.vert_obstacle_h     = float(p('vertical_obstacle_height').value)
        self.unknown_as_obstacle = bool(p('unknown_as_obstacle').value)

        self.robot_radius        = float(p('robot_radius').value)
        self.inflate_cells       = max(1, int(math.ceil(self.robot_radius / self.resolution)))

        raw_decay                = float(p('cell_decay_sec').value)
        self.cell_decay_sec      = raw_decay if raw_decay >= 1.0 else 0.0

        self.tf_timeout_sec      = float(p('tf_timeout_sec').value)

        self.min_range_sq        = self.min_range ** 2
        self.max_range_sq        = self.max_range ** 2

        # ---- State -----------------------------------------------------
        self.cells: Dict[Tuple[int, int], CellStats] = {}

        # Set on the first *successful* TF lookup, not on first cloud.
        # This prevents anchoring at a garbage position before TF is ready.
        self._global_origin: Optional[Tuple[int, int]] = None

        # ---- ROS -------------------------------------------------------
        self.costmap_pub = self.create_publisher(OccupancyGrid, self.output_topic, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, 10)

        self.get_logger().info(
            f'global_costmap ready | res={self.resolution}m | '
            f'{self.width_m}x{self.height_m}m | '
            f'inflate={self.inflate_cells} cells ({self.robot_radius}m) | '
            f'unknown={"obstacle" if self.unknown_as_obstacle else "free"} | '
            f'decay={"OFF" if self.cell_decay_sec == 0.0 else f"{self.cell_decay_sec}s"}'
        )

    # ------------------------------------------------------------------ #
    # Cloud callback
    # ------------------------------------------------------------------ #

    def cloud_callback(self, msg: PointCloud2) -> None:
        source_frame = msg.header.frame_id
        stamp        = msg.header.stamp

        # Try stamp-matched TF first, fall back to latest.
        # Either way, if TF fails we drop the cloud entirely — better to
        # skip frames than to anchor the map at a wrong position.
        tf = None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, source_frame, stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame_id, source_frame, rclpy.time.Time()
                )
            except TransformException as ex:
                self.get_logger().warn(
                    f'TF failed: {ex}', throttle_duration_sec=5.0
                )
                return  # drop this cloud

        t  = tf.transform.translation
        q  = tf.transform.rotation
        R  = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        tx, ty, tz = t.x, t.y, t.z

        now_sec = stamp.sec + stamp.nanosec * 1e-9

        # Anchor the map on the first *successful* TF, not on first cloud.
        if self._global_origin is None:
            half_w = int(math.floor((self.width_m  / 2.0) / self.resolution))
            half_h = int(math.floor((self.height_m / 2.0) / self.resolution))
            self._global_origin = (
                int(math.floor(tx / self.resolution)) - half_w,
                int(math.floor(ty / self.resolution)) - half_h,
            )
            self.get_logger().info(
                f'Map anchored at world '
                f'({self._global_origin[0]*self.resolution:.1f}, '
                f'{self._global_origin[1]*self.resolution:.1f}) | '
                f'robot at ({tx:.2f}, {ty:.2f})'
            )

        # Expire stale cells
        if self.cell_decay_sec > 0.0:
            cutoff = now_sec - self.cell_decay_sec
            dead   = [k for k, c in self.cells.items() if c.last_seen < cutoff]
            for k in dead:
                del self.cells[k]

        # Insert points
        for px, py, pz in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            range_sq = px*px + py*py
            if range_sq < self.min_range_sq or range_sq > self.max_range_sq:
                continue
            if pz < self.min_z or pz > self.max_z:
                continue

            wx, wy, wz = apply_transform(px, py, pz, R, tx, ty, tz)
            key = (int(math.floor(wx / self.resolution)),
                   int(math.floor(wy / self.resolution)))

            if key not in self.cells:
                self.cells[key] = CellStats()
            self.cells[key].update(wz, now_sec)

        self._publish(msg, tx, ty)

    # ------------------------------------------------------------------ #
    # Scoring
    # ------------------------------------------------------------------ #

    def _score_cell(self, cx: int, cy: int, cell: CellStats) -> int:
        # A: slope to 8-connected neighbours
        max_slope_deg = 0.0
        for dx, dy in NEIGHBORS:
            nc = self.cells.get((cx + dx, cy + dy))
            if nc and nc.n >= self.min_points:
                dz   = abs(cell.mean - nc.mean)
                dist = self.resolution * (math.sqrt(2.0) if dx and dy else 1.0)
                s    = math.degrees(math.atan2(dz, dist))
                if s > max_slope_deg:
                    max_slope_deg = s

        if max_slope_deg >= self.lethal_slope_deg:
            return COST_LETHAL
        if max_slope_deg <= self.free_slope_deg:
            slope_cost = COST_FREE
        else:
            ratio      = ((max_slope_deg - self.free_slope_deg) /
                          (self.lethal_slope_deg - self.free_slope_deg))
            slope_cost = int(round(ratio * 98.0))

        # B: vertical extent within cell
        z_cost = COST_FREE
        if cell.z_range >= self.vert_obstacle_h:
            ratio  = min(1.0, cell.z_range / (2.0 * self.vert_obstacle_h))
            z_cost = int(round(ratio * COST_LETHAL))

        # C: height variance
        v = cell.variance
        if v >= self.lethal_variance:
            v_cost = COST_LETHAL
        elif v >= self.max_variance:
            ratio  = (v - self.max_variance) / (self.lethal_variance - self.max_variance)
            v_cost = int(round(ratio * 98.0))
        else:
            v_cost = COST_FREE

        return max(slope_cost, z_cost, v_cost)

    # ------------------------------------------------------------------ #
    # Publish
    # ------------------------------------------------------------------ #

    def _publish(self, cloud_msg: PointCloud2, robot_x: float, robot_y: float) -> None:
        if self._global_origin is None:
            return

        res    = self.resolution
        gw     = int(self.width_m  / res)
        gh     = int(self.height_m / res)
        ox, oy = self._global_origin

        # Build raw cost array
        raw = [-1] * (gw * gh)
        for rel_y in range(gh):
            for rel_x in range(gw):
                key  = (ox + rel_x, oy + rel_y)
                cell = self.cells.get(key)
                idx  = rel_y * gw + rel_x
                if cell is None or cell.n < self.min_points:
                    raw[idx] = COST_LETHAL if self.unknown_as_obstacle else -1
                else:
                    raw[idx] = self._score_cell(key[0], key[1], cell)

        # Inflation pass
        data = list(raw)
        r    = self.inflate_cells
        for rel_y in range(gh):
            for rel_x in range(gw):
                if raw[rel_y * gw + rel_x] != COST_LETHAL:
                    continue
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if dx*dx + dy*dy > r*r:
                            continue
                        nx, ny = rel_x + dx, rel_y + dy
                        if 0 <= nx < gw and 0 <= ny < gh:
                            nidx = ny * gw + nx
                            if data[nidx] < COST_INSCRIBED:
                                data[nidx] = COST_INSCRIBED

        grid = OccupancyGrid()
        grid.header.stamp              = cloud_msg.header.stamp
        grid.header.frame_id           = self.frame_id
        grid.info.resolution           = res
        grid.info.width                = gw
        grid.info.height               = gh
        grid.info.origin.position.x    = ox * res
        grid.info.origin.position.y    = oy * res
        grid.info.origin.orientation.w = 1.0
        grid.data                      = data
        self.costmap_pub.publish(grid)


# ---------------------------------------------------------------------------#
# Entry point
# ---------------------------------------------------------------------------#

def main(args=None):
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