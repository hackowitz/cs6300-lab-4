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
    min_speed = 0.5

    def __init__(self):
        super().__init__('gap_minder')
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.get_logger().info('Initialized %s', self.__class__.__name__)

    def callback(self, msg: LaserScan) -> None:
        angle = self.get_best_gap(msg)
        self.drive(angle)

    def get_best_gap(self, msg: LaserScan):
        # TODO
        return 0  # just go straight forward

    def drive(self, angle: float, speed: float = None):
        """Drive at the given angle."""
        if speed is None:
            speed = self.appropriate_speed_for(angle)
        drive = AckermannDriveStamped()
        drive.drive.steering_angle = angle
        drive.drive.speed = speed
        self.drive_pub.publish(drive)

    def get_appropriate_speed_for(angle: float):
        """Choose a safe speed, given the steerign angle."""
        theta = abs(np.degrees(angle))
        if theta < 10:
            return self.top_speed
        if theta < 20:
            # return (self.top_speed - self.min_speed) * (20 - theta) / 10
            return (self.top_speed - self.min_speed) / 2
        return self.min_speed


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(node := GapMinder())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
