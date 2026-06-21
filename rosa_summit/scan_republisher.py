import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan


class ScanRepublisher(Node):
    """
    Republishes LaserScan from Ignition with a corrected frame_id.

    Ignition Gazebo names sensor frames as '<model>/<fused_link>/<sensor_name>'
    (e.g. 'summit/base_footprint/front_laser_sensor').  SLAM toolbox can't
    resolve that frame because it looks in /summit/tf_static.  This node
    rewrites the frame_id to 'base_footprint' so no additional TF lookup is
    required for the scan origin.
    """

    def __init__(self):
        super().__init__('scan_republisher')
        self.sub = self.create_subscription(
            LaserScan,
            '/summit/merged_laser_scan',
            self.callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(
            'scan_republisher: /summit/merged_laser_scan → /scan (frame_id=base_footprint)'
        )

    def callback(self, msg: LaserScan):
        msg.header.frame_id = 'base_footprint'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
