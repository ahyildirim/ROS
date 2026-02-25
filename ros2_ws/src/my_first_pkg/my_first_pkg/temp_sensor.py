import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SensorNode(Node):
    def __init__(self):
        super().__init__('temp_sensor') #Node oluşturma
  
        self.declare_parameter('publish_frequency', 1.0) #yayınlama frekans hızı
        self.declare_parameter('temp_min', 20.0) #minimum sıcaklık
        self.declare_parameter('temp_max', 30.0) #maximum sıcaklık

        self.publisher_ = self.create_publisher(Float32, 'temperature', 10) #Topic oluşturma

        freq = self.get_parameter('publish_frequency').value #oluşturduğumuz değeri normal bir değişkene alıyoruz ki zaman hesaplaması yapalım. (T = 1/f)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback) #T = 1/f, zaman içerisinde çağırılacak fonksiyon

    def timer_callback(self):
        tmin = self.get_parameter('temp_min').value #min sıcaklık normal değişkene alınır
        tmax = self.get_parameter('temp_max').value #max sıcaklık alınır
        temperature = random.uniform(tmin, tmax) #min-max arası random sayı üretilir

        msg = Float32() #msg tipi float(32bit)
        msg.data = temperature #datası üretilen random ısıya eşitlenir(listener'da msg.data dediğimizde ısıya erişebilmek için)
        self.publisher_.publish(msg) #Mesaj basılır
        self.get_logger().info(f'🌡️ Sıcaklık: {temperature:.2f} °C') #Terminalde log olarak görmek için log basılır
 
def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()