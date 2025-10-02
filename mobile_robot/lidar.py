import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFix(Node):
    def __init__(self):
        super().__init__('scan_frame_fix')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)

    def callback(self, msg):
        msg.header.frame_id = 'base_footprint'   # ✅ fix frame id
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
