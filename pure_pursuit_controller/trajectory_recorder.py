#!/usr/bin/env python3

import csv
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class TrajectoryRecorder(Node):
    """Record the executed vehicle trajectory."""

    def __init__(self):
        super().__init__('trajectory_recorder')

        self.points = []

        self.output_file = os.path.expanduser(
            '~/ros2_ws/data/actual_trajectory.csv'
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
            'Trajectory recorder started.'
        )

    def odom_callback(self, msg):
        """Store the current vehicle position."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.points.append((x, y))

    def save_trajectory(self):
        """Save the executed trajectory to a CSV file."""
        with open(
            self.output_file,
            'w',
            newline=''
        ) as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['x', 'y'])
            writer.writerows(self.points)

        self.get_logger().info(
            f'Saved {len(self.points)} trajectory points to '
            f'{self.output_file}'
        )


def main(args=None):
    """Run the trajectory recorder node."""
    rclpy.init(args=args)

    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_trajectory()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
