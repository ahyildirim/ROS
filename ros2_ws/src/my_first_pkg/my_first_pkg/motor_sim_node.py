import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MotorSimNode(Node):
    def __init__(self):
        super().__init__('motor_sim_node')

        self.motor_sub = self.create_subscription(Float32, '/motor_cmd', self.motor_callback, 10)

        self.depth_pub = self.create_publisher(Float32, '/depth/data', 10)

        self.timer = self.create_timer(0.1, self.update_physics)

        self.depth = 0.0
        self.velocity = 0.0
        self.motor_cmd = 0.0

        self.thrust_gain = 2.0
        self.damping = 0.8
        self.dt = 0.1

        self.get_logger().info('Motor Sim Node started')

    def motor_callback(self, msg):
        self.motor_cmd = msg.data

    def update_physics(self):
        acceleration = self.motor_cmd * self.thrust_gain
        self.velocity += acceleration * self.dt

        self.velocity *= self.damping

        self.depth += self.velocity * self.dt

        if self.depth < 0.0:
            self.depth = 0.0
            self.velocity = 0.0

        msg = Float32()
        msg.data = self.depth

        self.depth_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    motor_sim_node = MotorSimNode()
    rclpy.spin(motor_sim_node)
    motor_sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()