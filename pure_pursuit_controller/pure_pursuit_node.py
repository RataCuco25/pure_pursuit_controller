#!/usr/bin/env python3

import csv
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


class PurePursuitController(Node):
    """Pure Pursuit controller for the simulated ground vehicle."""

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.declare_parameter('velocity', 1.0)
        self.declare_parameter('wheelbase', 2.7)
        self.declare_parameter('lookahead_gain', 1.0)
        self.declare_parameter('min_lookahead', 0.5)
        self.declare_parameter('max_angular_velocity', 1.5)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter(
            'waypoints_file',
            'data/waypoints.csv'
        )

        self.velocity = self.get_parameter(
            'velocity'
        ).value
        self.wheelbase = self.get_parameter(
            'wheelbase'
        ).value
        self.lookahead_gain = self.get_parameter(
            'lookahead_gain'
        ).value
        self.min_lookahead = self.get_parameter(
            'min_lookahead'
        ).value
        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value
        self.control_frequency = self.get_parameter(
            'control_frequency'
        ).value
        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value
        self.waypoints_file = self.get_parameter(
            'waypoints_file'
        ).value

        self.x = None
        self.y = None
        self.yaw = None
        self.target_index = 0

        self.waypoints = self.load_waypoints()

        if not self.waypoints:
            self.get_logger().error(
                'No waypoints were loaded.'
            )
            raise RuntimeError(
                'Reference trajectory is empty.'
            )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        timer_period = 1.0 / self.control_frequency

        self.control_timer = self.create_timer(
            timer_period,
            self.control_callback
        )

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints.'
        )

        self.get_logger().info(
            f'Controller: v={self.velocity:.2f} m/s, '
            f'L={self.wheelbase:.2f} m, '
            f'k={self.lookahead_gain:.2f}'
        )

    def load_waypoints(self):
        """Load x,y reference points from CSV."""
        path = os.path.expanduser(
            self.waypoints_file
        )

        if not os.path.isabs(path):
            package_share = get_package_share_directory(
                'pure_pursuit_controller'
            )
            path = os.path.join(
                package_share,
                path
            )

        waypoints = []

        try:
            with open(path, newline='') as csv_file:
                reader = csv.DictReader(csv_file)

                for row in reader:
                    waypoints.append(
                        (
                            float(row['x']),
                            float(row['y'])
                        )
                    )

        except (FileNotFoundError, KeyError, ValueError) as error:
            self.get_logger().error(
                f'Could not load waypoints: {error}'
            )

        return waypoints

    def odom_callback(self, msg):
        """Update vehicle pose from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy ** 2 + qz ** 2)
        )

    def find_lookahead_target(self, lookahead_distance):
        """Find a forward waypoint at the requested look-ahead distance."""
        if self.x is None or self.y is None:
            return None

        search_start = max(
            0,
            self.target_index - 10
        )

        closest_index = search_start
        closest_distance = float('inf')

        for index in range(
            search_start,
            len(self.waypoints)
        ):
            wx, wy = self.waypoints[index]
            distance = math.hypot(
                wx - self.x,
                wy - self.y
            )

            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

            if (
                index > closest_index + 10
                and distance > closest_distance
            ):
                break

        target_index = closest_index

        for index in range(
            closest_index,
            len(self.waypoints)
        ):
            wx, wy = self.waypoints[index]
            distance = math.hypot(
                wx - self.x,
                wy - self.y
            )

            if distance >= lookahead_distance:
                target_index = index
                break
        else:
            target_index = len(self.waypoints) - 1

        self.target_index = max(
            self.target_index,
            target_index
        )

        return self.waypoints[self.target_index]

    def control_callback(self):
        """Compute and publish the Pure Pursuit control command."""
        if self.x is None or self.yaw is None:
            return

        lookahead_distance = max(
            self.min_lookahead,
            self.lookahead_gain * abs(self.velocity)
        )

        target = self.find_lookahead_target(
            lookahead_distance
        )

        if target is None:
            self.stop_vehicle()
            return

        target_x, target_y = target

        final_x, final_y = self.waypoints[-1]
        distance_to_goal = math.hypot(
            final_x - self.x,
            final_y - self.y
        )

        if distance_to_goal <= self.goal_tolerance:
            self.stop_vehicle()

            if not hasattr(self, '_goal_reported'):
                self.get_logger().info(
                    f'Goal reached: distance={distance_to_goal:.3f} m'
                )
                self._goal_reported = True

            return

        dx = target_x - self.x
        dy = target_y - self.y

        target_heading = math.atan2(dy, dx)

        psi = math.atan2(
            math.sin(target_heading - self.yaw),
            math.cos(target_heading - self.yaw)
        )

        steering_angle = math.atan2(
            2.0 * self.wheelbase * math.sin(psi),
            lookahead_distance
        )

        yaw_rate = (
            self.velocity / self.wheelbase
        ) * math.tan(steering_angle)

        yaw_rate = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, yaw_rate)
        )

        command = Twist()
        command.linear.x = self.velocity
        command.angular.z = yaw_rate

        self.cmd_publisher.publish(command)

        if not hasattr(self, '_last_debug_time'):
            self._last_debug_time = self.get_clock().now()

        now = self.get_clock().now()
        elapsed = (
            now - self._last_debug_time
        ).nanoseconds / 1e9

        if elapsed >= 1.0:
            self.get_logger().info(
                f'Pose=({self.x:.2f}, {self.y:.2f}) '
                f'yaw={self.yaw:.2f} | '
                f'target={self.target_index} '
                f'({target_x:.2f}, {target_y:.2f}) | '
                f'psi={psi:.2f} '
                f'delta={steering_angle:.2f} | '
                f'wz={yaw_rate:.2f}'
            )

            self._last_debug_time = now

    def stop_vehicle(self):
        """Publish a zero velocity command."""
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0
        self.cmd_publisher.publish(command)

    def destroy_node(self):
        """Stop the vehicle before destroying the ROS node."""
        self.stop_vehicle()
        super().destroy_node()


def main(args=None):
    """Run the Pure Pursuit controller node."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = PurePursuitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
