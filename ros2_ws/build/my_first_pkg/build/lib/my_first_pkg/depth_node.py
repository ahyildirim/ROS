import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_node')

        self.publisher = self.create_publisher(Float32, '/depth/data', 10)
        self.timer = self.create_timer(0.1, self.publish_depth)

        self.depth = 0.0
        self.descent_rate = 0.1

        self.get_logger().info('Depth Node started')

    def publish_depth(self):
        self.depth += self.descent_rate + random.gauss(0.0, 0.01)

        msg = Float32()
        msg.data = self.depth

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    depth_node = DepthNode()
    rclpy.spin(depth_node)
    depth_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()