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

    top_speed = 2.5
    max_accel = 0.25  # empirically derived to go around 0.5m/s@30*, 1m/s@15*, 1.5m/s@7*
    tti = 1  # desired time to impact, seconds

    # attributes of the LaserScan which are assumed during pre-compute
    # I don't care to handle this changing at runtime
    angle_min = -2.356194496154785
    angle_max = 2.356194496154785
    angle_increment = 0.0043633230961859

    def __init__(
            self,
            car_size: float = 0.15,  # meters
            nsamples: int = None,
            **kwargs,  # constants about the scan, such as angle_max. etc.
    ) -> None:
        super().__init__('gap_minder')
        self.__dict__.update(kwargs)
        self.car_size = car_size
        self.nsamples = nsamples or abs(int((self.angle_max + self.angle_min) / self.angle_increment))

        # we save time making arrays/matrices by precomputing static values
        # 2x2 matrix where [phi0, phi1] == phi - phi1
        step = np.arange(self.nsamples)
        self.delta_phi = np.abs(step[:, np.newaxis] - step)
        self.theta = np.linspace(self.angle_min, self.angle_max, self.nsamples)

        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.callback, 10)

        self.get_logger().info('Initialized %s', self.__class__.__name__)

    def callback(self, msg: LaserScan) -> None:
        drive = AckermannDriveStamped()
        distance, angle = self.get_drive_vector(msg)
        drive.drive.steering_angle = angle
        drive.drive.speed = min([
            distance / self.tti,  # go no faster than time to impact
            self.top_speed,       # go no faster than top speed
            (abs(1 / (angle or 0.01)) * self.max_accel)**0.5,  # turn no faster than max_accel
        ])
        self.get_logger().info('🏎️ /drive: angle = %5.02f, speed = %5.02f', angle, speed)
        self.drive_pub.publish(drive)

    def get_drive_vector(self, msg: LaserScan):
        dist = self.get_obstructed_distance(msg)
        dist_max = np.nanmax(dist)
        edge = np.diff((dist == dist_max).astype(int))  # +/-1 entering/exiting max dist region
        theta_max = (edge.argmin() + edge.argmax() - 1) / 2
        self.get_logger().debug('Best gap: theta=%5.02f, depth=%5.02f', theta_max, dist_max)
        return dist, theta_max

    def get_obstructed_distance(self, msg: LaserScan):
        return np.nanmin(self.get_obstruction_matrix(msg), axis=1)

    def get_obstruction_matrix(self, msg: LaserScan):
        # `nan`s _should_ propogate nicely as long as we use nanmin and nanmax
        r = np.array(msg.ranges)
        r[(r < msg.range_min) | (r > msg.range_max)] = np.nan

        phi = np.arcsin(self.car_size / r) / self.angle_increment  # in steps
        return np.where(self.delta_phi <= phi, r, np.nan)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(node := GapMinder())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
