# ~/ros2_ws/src/safety_detectors/safety_detectors/relay_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')
        self.publisher = self.create_publisher(
            CompressedImage, '/image/sub', 10)
        self.subscriber = self.create_subscription(
            CompressedImage, '/image/pub', self.callback, 10)

    def callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info('Relayed an image message.')

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
