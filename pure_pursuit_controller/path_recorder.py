#!/usr/bin/env python3

import csv
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class PathRecorder(Node):
    """Record vehicle odometry positions as reference waypoints."""

    def __init__(self):
        super().__init__('path_recorder')

        self.points = []
        self.last_x = None
        self.last_y = None
        self.min_distance = 0.05

        self.output_file = os.path.expanduser(
            '~/ros2_ws/data/waypoints.csv'
        )

        os.makedirs(
            os.path.dirname(self.output_file),
            exist_ok=True
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Path recorder started.'
        )

    def odom_callback(self, msg):
        """Store a waypoint when the vehicle moves enough."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None:
            self.points.append((x, y))
            self.last_x = x
            self.last_y = y
            return

        distance = (
            (x - self.last_x) ** 2
            + (y - self.last_y) ** 2
        ) ** 0.5

        if distance >= self.min_distance:
            self.points.append((x, y))
            self.last_x = x
            self.last_y = y

    def save_path(self):
        """Save recorded waypoints to a CSV file."""
        with open(
            self.output_file,
            'w',
            newline=''
        ) as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['x', 'y'])
            writer.writerows(self.points)

        self.get_logger().info(
            f'Saved {len(self.points)} waypoints to '
            f'{self.output_file}'
        )


def main(args=None):
    """Run the path recorder node."""
    rclpy.init(args=args)

    node = PathRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_path()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
