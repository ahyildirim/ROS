import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

class DepthController(Node):
    def __init__(self):
        super().__init__('depth_controller')

        self.state_sub = self.create_subscription(Vector3, '/state_estimate', self.state_callback, 10)

        self.motor_pub = self.create_publisher(Float32, '/motor_cmd', 10)

        self.target_depth = 2.0
        self.kp = 5.0
        self.kd = 2.0

        self.last_error = 0.0

        self.get_logger().info('Depth Controller Node started')

    def state_callback(self, msg):
        current_depth = msg.y
        vertical_velocity = msg.z

        error = self.target_depth - current_depth
        derivative = -vertical_velocity
        output = self.kp * error + self.kd * derivative

        output = max(min(output, 1.0), -1.0)
        motor_msg = Float32()
        motor_msg.data = output

        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    depth_controller = DepthController()
    rclpy.spin(depth_controller)
    depth_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()