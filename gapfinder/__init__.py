"""Follow the deepest gap the car can fit through.

Supports two strategies:
- Follow-the-Gap (default): preprocess -> safety bubble -> max gap -> farthest point.
- Obstruction geometry: original arc-obstruction method from the base implementation.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class GapFinder(Node):
    """Move forward to the furthest area ahead, minding any obstacles in the way.

    All distances are meters.
    All angles are radians; the LaserScan measures counterclockwise from straight forward.
    """

    top_speed = 2.5
    max_accel = 0.25
    tti = 1  # time to impact (seconds) for speed scaling

    # attributes of the LaserScan which are assumed during pre-compute
    # I don't care to handle this changing at runtime
    angle_min = -2.356194496154785
    angle_max = 2.356194496154785
    angle_increment = 0.0043633230961859

    def __init__(
            self,
            car_size: float = 0.15,
            nsamples: int = None,
            use_follow_the_gap: bool = True,
            bubble_radius: float = 0.4,
            smooth_window: int = 5,
            flip_scan: bool = False,
            use_front_window: bool = False,
            front_window_degrees: float = 90.0,
            **kwargs,
    ) -> None:
        super().__init__('gap_finder')
        self.__dict__.update(kwargs)
        self.declare_parameter('use_follow_the_gap', use_follow_the_gap)
        self.declare_parameter('bubble_radius', bubble_radius)
        self.declare_parameter('smooth_window', smooth_window)
        self.declare_parameter('flip_scan', flip_scan)
        self.declare_parameter('use_front_window', use_front_window)
        self.declare_parameter('front_window_degrees', front_window_degrees)
        self.use_follow_the_gap = self.get_parameter('use_follow_the_gap').value
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.smooth_window = self.get_parameter('smooth_window').value
        self.flip_scan = self.get_parameter('flip_scan').value
        self.use_front_window = self.get_parameter('use_front_window').value
        self.front_window_degrees = self.get_parameter('front_window_degrees').value
        self.car_size = car_size
        self.nsamples = nsamples or abs(
            int((self.angle_max + self.angle_min) / self.angle_increment)
        )

        # we save time making arrays/matrices by precomputing static values
        # 2x2 matrix where [phi0, phi1] == phi - phi1
        step = np.arange(self.nsamples)
        self.delta_phi = np.abs(step[:, np.newaxis] - step)
        self.theta = np.linspace(self.angle_min, self.angle_max, self.nsamples)

        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.get_logger().info(
            f'GapFinder initialized (follow_the_gap={self.use_follow_the_gap})'
        )

    def preprocess_lidar(self, ranges, min_range=None, max_range=None):
        """Clean LiDAR: clip to [min, max] and replace NaN/inf with max_range."""
        if min_range is None:
            min_range = 0.05
        if max_range is None:
            max_range = 10.0
        processed = np.array(ranges, dtype=np.float64)
        processed = np.clip(processed, min_range, max_range)
        processed[np.isnan(processed)] = max_range
        processed[np.isinf(processed)] = max_range
        return processed

    def smooth_lidar(self, ranges, window_size=5):
        """Moving average to reduce noise. Returns a copy."""
        if window_size < 3:
            return np.array(ranges, dtype=np.float64)
        kernel = np.ones(window_size) / window_size
        return np.convolve(ranges, kernel, mode='same')

    def create_safety_bubble(self, ranges, bubble_radius=None):
        """Draw a bubble around the closest point; set those indices to 0 (in place)."""
        if bubble_radius is None:
            bubble_radius = self.bubble_radius
        closest_idx = int(np.argmin(ranges))
        closest_dist = ranges[closest_idx]
        if closest_dist <= 0:
            return ranges
        bubble_angle = math.atan(bubble_radius / closest_dist)
        bubble_points = int(bubble_angle / self.angle_increment)
        start_idx = max(closest_idx - bubble_points, 0)
        end_idx = min(closest_idx + bubble_points, len(ranges) - 1)
        ranges[start_idx:end_idx + 1] = 0.0
        return ranges

    def find_max_gap(self, ranges):
        """Find the longest run of non-zero elements. Returns (start_idx, end_idx)."""
        n = len(ranges)
        max_gap_size = 0
        max_start = 0
        max_end = 0
        current_start = None
        for i in range(n):
            if ranges[i] > 0:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    gap_size = i - current_start
                    if gap_size > max_gap_size:
                        max_gap_size = gap_size
                        max_start = current_start
                        max_end = i - 1
                    current_start = None
        if current_start is not None:
            gap_size = n - current_start
            if gap_size > max_gap_size:
                max_start = current_start
                max_end = n - 1
        return max_start, max_end

    def find_best_point(self, ranges, start_idx, end_idx):
        """Pick the farthest point in the gap (naive); can be improved per lecture."""
        gap_ranges = ranges[start_idx:end_idx + 1]
        best_rel = int(np.argmax(gap_ranges))
        return start_idx + best_rel

    def get_drive_vector_follow_the_gap(self, msg: LaserScan):
        """Find distance/angle by Follow-the-Gap pipeline."""
        ranges = np.array(msg.ranges, dtype=np.float64)
        if self.flip_scan:
            ranges = ranges[::-1]

        # Optional front-only view (e.g., +/-90 degrees), centered around forward.
        if self.use_front_window:
            half_window_points = int(
                math.radians(self.front_window_degrees) / msg.angle_increment
            )
            center_idx = len(ranges) // 2
            start = max(center_idx - half_window_points, 0)
            end = min(center_idx + half_window_points + 1, len(ranges))
            ranges = ranges[start:end]

        proc = self.preprocess_lidar(
            ranges,
            min_range=msg.range_min,
            max_range=msg.range_max,
        )
        if self.smooth_window >= 3:
            proc = self.smooth_lidar(proc, self.smooth_window)
        proc = self.create_safety_bubble(proc.copy())
        if np.all(proc <= 0):
            return 0.0, 0.0
        start_idx, end_idx = self.find_max_gap(proc)
        best_idx = self.find_best_point(proc, start_idx, end_idx)
        center_idx = (len(proc) - 1) / 2.0
        steering_angle = (best_idx - center_idx) * self.angle_increment
        distance = float(proc[best_idx])
        return distance, steering_angle

    def get_obstruction_matrix(self, msg: LaserScan):
        """Get a matrix of obstructed ranges (if any) for each angle, by each angle.

        matrix[i][j] is the range at which the ith angle is obstructed by the object at the jth
        angle. The matrix is null where the object at j does not obstruct driving towards i.

        The diagonal is how much i obstructs itself, which is just the range at i.
        """
        r = np.array(msg.ranges, dtype=np.float64)
        if self.flip_scan:
            r = r[::-1]
        # `nan`s _should_ propagate nicely as long as we use nanmin and nanmax
        r[(r < msg.range_min) | (r > msg.range_max)] = np.nan
        ratio = np.clip(self.car_size / r, -1.0, 1.0)
        phi = np.arcsin(ratio) / self.angle_increment  # in steps
        return np.where(self.delta_phi <= phi, r, np.nan)

    def get_obstructed_distance(self, msg: LaserScan):
        """Drivable distance for each angle, considering all obstructions."""
        return np.nanmin(self.get_obstruction_matrix(msg), axis=1)

    def get_drive_vector_obstruction(self, msg: LaserScan):
        """Find the distance and angle to the furthest drivable point (obstruction geometry)."""
        distance = self.get_obstructed_distance(msg)
        if np.all(np.isnan(distance)):
            return 0.0, 0.0
        dist = float(np.nanmax(distance))
        edge = np.diff((distance == dist).astype(int))  # +/-1 entering/exiting max region
        center_step = (edge.argmin() + edge.argmax() - 1) / 2.0
        center_step = int(np.clip(round(center_step), 0, self.nsamples - 1))
        steering_angle = float(self.theta[center_step])
        return dist, steering_angle

    def callback(self, msg: LaserScan) -> None:
        """Triggered whenever a LaserScan is received."""
        drive = AckermannDriveStamped()
        if self.use_follow_the_gap:
            distance, angle = self.get_drive_vector_follow_the_gap(msg)
        else:
            distance, angle = self.get_drive_vector_obstruction(msg)
        drive.drive.steering_angle = angle
        drive.drive.speed = speed = min([
            distance / self.tti,  # go no faster than time to impact
            self.top_speed,       # go no faster than top speed
            (abs(1 / (angle or 0.01)) * self.max_accel)**0.5,  # turn no faster than max_accel
        ])
        self.get_logger().info(f' /drive: angle={angle:.2f}, speed={speed:.2f}')
        self.publisher.publish(drive)


def main(args=None):
    """Run the gap follow node."""
    rclpy.init(args=args)
    rclpy.spin(node := GapFinder())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
