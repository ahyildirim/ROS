import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ListenerNode(Node):
    def __init__(self):
        super().__init__('display_node') #node oluşumu
 
        self.declare_parameter('max_temp_warning_threshold', 28.0) #max uyarı eşiği
        self.declare_parameter('min_temp_warning_threshold', 22.0) #min uyarı eşiği

        self.subscription = self.create_subscription( #Abone olunacak topic
            Float32, #msg tipi
            'temperature', #topic ismi
            self.listener_callback, #çalışacak fonksiyon
            10
        )
        self.subscription

    def listener_callback(self, msg):
        temp_max = self.get_parameter('max_temp_warning_threshold').value #değerler gerçek değişkene dönüştürülür
        temp_min = self.get_parameter('min_temp_warning_threshold').value
        if msg.data > temp_max: #eğer max tresholddan büyükse
            self.get_logger().warn(f'Yüksek sıcaklık: {msg.data:.2f} °C (eşik: {temp_max})') #uyarı bas
        elif msg.data < temp_min: #eğer min tresholddan büyükse
            self.get_logger().warn(f'Düşük sıcaklık: {msg.data:.2f} °C (eşik: {temp_min})') #uyarı bas
        else:
            self.get_logger().info(f'🟢 Normal sıcaklık: {msg.data:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()