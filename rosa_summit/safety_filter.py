import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SafetyFilter(Node):
    """
    Anti-collision safety filter — DIRECT to Ignition.

    Subscribes to:
      - /summit/cmd_vel       (Nav2 / agent output)
      - /summit/merged_laser_scan  (raw lidar from bridge)
      - /scan                      (republished lidar, fallback)

    Publishes to:
      - /model/summit/cmd_vel  (DIRECTLY to the Ignition bridge topic)

    NO relay needed. This node IS the relay + safety filter combined.
    """

    STOP_DISTANCE = 0.55   # meters — full stop
    SLOW_DISTANCE = 1.0    # meters — start slowing down

    def __init__(self):
        super().__init__('safety_filter')
        self.min_range = float('inf')
        self.last_scan_time = 0.0
        self.scan_count = 0

        # Flexible QoS for laser (works with both reliable and best-effort publishers)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscribe to BOTH scan topics for robustness
        self.create_subscription(
            LaserScan, '/summit/merged_laser_scan', self.scan_callback, scan_qos)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)

        # Subscribe to Nav2 / agent velocity commands
        self.create_subscription(
            Twist, '/summit/cmd_vel', self.cmd_callback, 10)

        # Publish DIRECTLY to the bridge topic (no relay needed)
        self.cmd_pub = self.create_publisher(Twist, '/model/summit/cmd_vel', 10)

        # Diagnostic timer (wall clock, not sim clock)
        self.diag_timer = self.create_timer(5.0, self.diagnostic_callback)

        self.get_logger().info(
            f'=== SAFETY FILTER ACTIVE ===\n'
            f'  stop  < {self.STOP_DISTANCE}m\n'
            f'  slow  < {self.SLOW_DISTANCE}m\n'
            f'  input:  /summit/cmd_vel\n'
            f'  output: /model/summit/cmd_vel (DIRECT)\n'
            f'  laser:  /summit/merged_laser_scan + /scan'
        )

    def scan_callback(self, msg: LaserScan):
        min_r = float('inf')
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and not (math.isnan(r) or math.isinf(r)):
                if r < min_r:
                    min_r = r
        self.min_range = min_r
        self.last_scan_time = time.monotonic()
        self.scan_count += 1

    def diagnostic_callback(self):
        if self.last_scan_time == 0:
            self.get_logger().warn('SAFETY: NO SCANS RECEIVED YET — filter inactive')
        else:
            age = time.monotonic() - self.last_scan_time
            self.get_logger().info(
                f'SAFETY: min_range={self.min_range:.2f}m, '
                f'scan_age={age:.1f}s, scans={self.scan_count}'
            )

    def cmd_callback(self, msg: Twist):
        out = Twist()
        d = self.min_range

        # If no scan data yet, pass through with warning
        if self.last_scan_time == 0:
            out = msg
            self.cmd_pub.publish(out)
            return

        if d <= self.STOP_DISTANCE:
            # STOP — only rotation allowed
            out.angular.z = msg.angular.z
            self.get_logger().warn(
                f'SAFETY STOP! obstacle={d:.2f}m (limit={self.STOP_DISTANCE}m)',
                throttle_duration_sec=1.0,
            )
        elif d < self.SLOW_DISTANCE:
            scale = (d - self.STOP_DISTANCE) / (self.SLOW_DISTANCE - self.STOP_DISTANCE)
            out.linear.x = msg.linear.x * scale
            out.linear.y = msg.linear.y * scale
            out.angular.z = msg.angular.z
        else:
            out.linear.x = msg.linear.x
            out.linear.y = msg.linear.y
            out.angular.z = msg.angular.z

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
