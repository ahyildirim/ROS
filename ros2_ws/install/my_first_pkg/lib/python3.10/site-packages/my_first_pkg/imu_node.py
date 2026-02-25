import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)

        self.get_logger().info('IMU Node started')

    def publish_imu(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.angular_velocity.x = random.gauss(0.0, 0.01)
        msg.angular_velocity.y = random.gauss(0.0, 0.01)
        msg.angular_velocity.z = random.gauss(0.0, 0.05)

        msg.linear_acceleration.x = random.gauss(0.0, 0.1)
        msg.linear_acceleration.y = random.gauss(0.0, 0.1)
        msg.linear_acceleration.z = 9.81 + random.gauss(0.0, 0.1)

        self.publisher.publish(msg)
    
def main(args=None):
        rclpy.init(args=args)
        imu_node = ImuNode()
        rclpy.spin(imu_node)
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()