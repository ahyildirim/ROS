import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import math

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.depth_sub = self.create_subscription(Float32, '/depth/data', self.depth_callback, 10)

        self.state_pub = self.create_publisher(Vector3, '/state_estimate', 10)

        self.current_yaw = 0.0
        self.current_depth = 0.0
        self.last_depth = 0.0
        self.vertical_velocity = 0.0

        self.timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info('Fusion Node started')

    def imu_callback(self, msg):
        q = msg.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def depth_callback(self, msg):
        self.current_depth = msg.data
        self.vertical_velocity = (self.current_depth - self.last_depth) / 0.1
        self.last_depth = self.current_depth

    def publish_state(self):
        state = Vector3()
        state.x = self.current_yaw
        state.y = self.current_depth
        state.z = self.vertical_velocity

        self.state_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = FusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()