#!/usr/bin/env python3
"""
Emergency anti-collision script — runs independently.

Monitors laser scan data and publishes zero velocity DIRECTLY
to the Ignition bridge topic when obstacles are too close.

Usage (separate terminal):
  source install/setup.bash
  python3 emergency_stop.py
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class EmergencyStop(Node):
    STOP_DISTANCE = 0.45  # meters

    def __init__(self):
        super().__init__('emergency_stop')
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, '/summit/merged_laser_scan', self.cb, scan_qos)
        self.create_subscription(LaserScan, '/scan', self.cb, scan_qos)
        self.pub = self.create_publisher(Twist, '/model/summit/cmd_vel', 10)
        self.stopping = False
        self.get_logger().info(f'EMERGENCY STOP active — threshold {self.STOP_DISTANCE}m')

    def cb(self, msg: LaserScan):
        min_r = float('inf')
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and not (math.isnan(r) or math.isinf(r)):
                if r < min_r:
                    min_r = r

        if min_r < self.STOP_DISTANCE:
            # Flood zero velocity to override any other commands
            zero = Twist()
            for _ in range(3):
                self.pub.publish(zero)
            if not self.stopping:
                self.get_logger().warn(f'EMERGENCY STOP! Obstacle at {min_r:.2f}m')
                self.stopping = True
        else:
            if self.stopping:
                self.get_logger().info(f'Clear — nearest obstacle at {min_r:.2f}m')
                self.stopping = False


def main():
    rclpy.init()
    node = EmergencyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
