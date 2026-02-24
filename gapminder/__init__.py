"""Follow the deepest gap the car can fit through"""
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



class GapMinder(Node):
    """Move forward to the furthest area ahead, minding any obstacles in the way.

    All distances are meters.
    All angles are radians; the LaserScan measures counderclockwise from straight forward.
    """

    top_speed = 1.5
    max_accel = 0.25  # empirically derived to go around 0.5m/s@30*, 1m/s@15*, 1.5m/s@7*
    tti = 1.0  # desired time to impact, seconds

    # attributes of the LaserScan which are assumed during pre-compute
    # I don't care to handle this changing at runtime
    angle_min = -2.356194496154785
    angle_max = 2.356194496154785
    angle_increment = 0.0043633230961859

    def __init__(
            self,
            car_size: float = 0.25,  # meters
            # nsamples: int = None,
            **kwargs,  # constants about the scan, such as angle_max. etc.
    ) -> None:
        super().__init__('gap_minder')
        self.__dict__.update(kwargs)
        self.car_size = car_size
        self.nsamples = 1081 # FIXME, use real math

        # we save time making arrays/matrices by precomputing static values
        # 2x2 matrix where [phi0, phi1] == phi - phi1
        step = np.arange(self.nsamples)
        self.delta_phi = np.abs(step[:, np.newaxis] - step)
        self.theta = np.linspace(self.angle_min, self.angle_max, self.nsamples)
        self.cos = np.cos(self.theta)

        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.callback, 10)

        self.get_logger().info(f'Initialized {self.__class__.__name__}')

    def callback(self, msg: LaserScan) -> None:
        """Triggered whenever a LaserScan is reccieved."""
        drive = AckermannDriveStamped()
        distance, angle = self.get_drive_vector(msg)
        drive.drive.steering_angle = angle
        drive.drive.speed = speed = min([
            distance / self.tti,  # go no faster than time to impact
            self.top_speed,       # go no faster than top speed
            (abs(1 / (angle or 0.01)) * self.max_accel)**0.5,  # turn no faster than max_accel
        ])
        self.get_logger().info(f'🏎️ /drive: {angle=:5.02f}, {speed=:5.02f}')
        self.publisher.publish(drive)

    def get_drive_vector(self, msg: LaserScan):
        """Find the distance and angle to the furthest drivable point."""
        distance = self.get_obstructed_distance(msg) * self.cos
        dist = np.nanmax(distance)
        # edge = np.diff((distance == dist).astype(int))  # +/-1 entering/exiting max dist region
        # index = (edge.argmin() - edge.argmax() - 1) // 2
        index = distance.argmax()
        # dist = distance[index]
        theta = self.theta[index]
        self.get_logger().info(f'Best gap: theta[{index}] = {theta:5.02f}, {dist=:5.02f}')
        return dist, theta

    def get_obstructed_distance(self, msg: LaserScan):
        """Get the drivable distance at each angle."""
        return np.nanmin(self.get_obstruction_matrix(msg), axis=1)

    def get_obstruction_matrix(self, msg: LaserScan):
        """Get a matrix of obstrructed ranges (if any) for each angle, by each angle.
        
        matrix[i][j] is the range at which the ith angle is obstructed by the object at the jth
        angle. The matrix is null where the object at j does not obstruct driving towards i.

        The diagonal is how much i obstrusts itself, which is just the range at i.
        """
        # `nan`s _should_ propogate nicely as long as we use nanmin and nanmax
        r = np.array(msg.ranges)
        r[(r < msg.range_min) | (r > msg.range_max)] = np.nan

        phi = np.arcsin(self.car_size / r) / self.angle_increment  # in steps
        return np.where(self.delta_phi <= phi, r, np.nan)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    rclpy.spin(node := GapMinder())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
