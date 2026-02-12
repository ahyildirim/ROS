# ROS2 Eğitim Notları – Yapılandırılmış README

> Bu doküman, ROS2 öğrenme sürecimde tuttuğum **kişisel notlar**ın daha **okunabilir, düzenli ve erişilebilir** hale getirilmiş versiyonudur.
> **İçerik korunmuştur**, sadece başlıklandırma, bölümlendirme ve navigasyon iyileştirilmiştir.

---

# İçindekiler

* [1. ROS Nedir?](#1-ros-nedir)
* [2. ROS Kavramları](#2-ros-kavramları)

  * [2.1 Node](#21-node)
  * [2.2 Topic](#22-topic)
  * [2.3 Service](#23-service)
  * [2.4 Action](#24-action)
* [3. ROS Kurulumu](#3-ros-kurulumu)

  * [3.1 Locale Ayarları](#31-locale-ayarları)
  * [3.2 ROS Repository Ekleme](#32-ros-repository-ekleme)
  * [3.3 ROS2 Kurulumu](#33-ros2-kurulumu)
* [4. Workspace ve Proje Yapısı](#4-workspace-ve-proje-yapısı)
* [5. İlk ROS Node Uygulaması](#5-ilk-ros-node-uygulaması)

  * [5.1 Talker Node](#51-talker-node)
  * [5.2 Listener Node](#52-listener-node)
* [6. Gerçekçi Sensör Uygulaması](#6-gerçekçi-sensör-uygulaması)

  * [6.1 Temp Sensor Node](#61-temp-sensor-node)
  * [6.2 Display Node](#62-display-node)
* [7. Runtime Parametre Güncelleme](#7-runtime-parametre-güncelleme)
* [8. Launch Dosyası ile Tüm Sistemi Çalıştırma](#8-launch-dosyası-ile-tüm-sistemi-çalıştırma)
* [9. Faydalı ROS Komutları](#9-faydalı-ros-komutları)

---

# 1. ROS Nedir?

ROS, birden fazla modülün birbirleri ile haberleşmesini kolaylaştıran ve birbirleri ile senkronize çalışmasını sağlayan bir frameworkdür.

Örneğin elimizde bir araç olsun, bu aracın içinde LIDAR, görüntü işleme, tekerlek motorları ve engel tespiti yapan 4 ayrı modül olsun. Bu modüller birbirleri ile haberleşerek çalışacakları için ROS bir haberleşme protokolü kuruyor. Bahsettiğimiz bu modüllere **node** denir. ROS bu node'lar arasında veri alışverişini **topic, service** ve **action** kavramları ile düzenliyor.

---

# 2. ROS Kavramları

## 2.1 Node

ROS sistemindeki her bağımsız çalışan modül bir **node** olarak adlandırılır.

---

## 2.2 Topic

Topic, **yayın–abonelik (publish-subscribe)** sistemi ile mesaj aktarımı sağlar.

Bir talker olur, bu talker bir konu açar ve o konuya belirli mesajları yayınlar. Listener ise bu konuya abone olarak burada basılan mesajlara erişebilir, buna göre aksiyon gerçekleştirebilir.

---

## 2.3 Service

Service, anlık kritik sorgulara anında cevap alabilmek için kullanılır. Aynı bir web sunucusunun belirli bir isteğe belirli bir cevap döndürmesine benzer.

Örneğin, araçtaki engel tespit modülü bir engel ile karşılaştığında motorun durumunu sorgular. İstemci bu cevap gelene kadar askıya alınır.

---

## 2.4 Action

Action, **uzun süreli görevler** için kullanılır. Sürekli **feedback** gönderir, sonuç döner ve görev iptal edilebilir.

Örneğin: Otonom araçta hedefe gitme işlemi.

---

# 3. ROS Kurulumu

> Not: ROS2 Humble + Ubuntu 22.04 (Jammy) + Python 3.10.12

---

## 3.1 Locale Ayarları

```bash
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

---

## 3.2 ROS Repository Ekleme

```bash
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

---

## 3.3 ROS2 Kurulumu

```bash
sudo apt install ros-humble-desktop
```

Terminal ortamına ROS ekleme:

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

# 4. Workspace ve Proje Yapısı

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Paket oluşturma:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_pkg
```

---

# 5. İlk ROS Node Uygulaması

## 5.1 Talker Node

📄 **talker.py**
Her 0.5 saniyede bir `chatter` topic'ine mesaj yayınlar.

```python
import rclpy #ros kütüphanesi eklenir
from rclpy.node import Node #Node sınıfı importlanır
from std_msgs.msg import String #Yayınlayacağımız mesajın tipi importlanır

class MinimalPublisher(Node): #Kendi node sınıfıımzı tanımlıyoruz(ROS kütüphanesindeki Node sınıfını inherit eder)
    def __init__(self): #Constructor ilk çağırıldığında bu fonksiyon çalışır.
        super().__init__('talker') #inherit alınan node sınıfının constructoru çağırılarak sınıfa isim verilir.
        self.publisher_ = self.create_publisher(String, 'chatter', 10) #publisher oluşturulur
        self.timer = self.create_timer(0.5, self.timer_callback) #0.5 saniyede bir callback çağrılır
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 5.2 Listener Node

📄 **listener.py**
`chatter` topic'ini dinler ve mesajları terminale basar.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# 6. Gerçekçi Sensör Uygulaması

## 6.1 Temp Sensor Node

📄 **temp_sensor.py**
Parametreye bağlı sıcaklık simülasyonu üretir.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('temp_min', 20.0)
        self.declare_parameter('temp_max', 30.0)

        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)

        freq = self.get_parameter('publish_frequency').value
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)

    def timer_callback(self):
        tmin = self.get_parameter('temp_min').value
        tmax = self.get_parameter('temp_max').value
        temperature = random.uniform(tmin, tmax)

        msg = Float32()
        msg.data = temperature
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sıcaklık: {temperature:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6.2 Display Node

📄 **display_node.py**
Sıcaklık değerlerini okuyarak uyarı üretir.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')

        self.declare_parameter('max_temp_warning_threshold', 28.0)
        self.declare_parameter('min_temp_warning_threshold', 22.0)

        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        max_threshold = self.get_parameter('max_temp_warning_threshold').value
        min_threshold = self.get_parameter('min_temp_warning_threshold').value

        if msg.data > max_threshold:
            self.get_logger().warn(f'Yüksek sıcaklık: {msg.data:.2f} °C (eşik: {max_threshold})')
        elif msg.data < min_threshold:
            self.get_logger().warn(f'Düşük sıcaklık: {msg.data:.2f} °C (eşik: {min_threshold})')
        else:
            self.get_logger().info(f'Normal sıcaklık: {msg.data:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# 7. Runtime Parametre Güncelleme

```bash
ros2 param set /listener_node max_temp_warning_threshold 25.0
ros2 param set /sensor_node publish_frequency 5.0
```

---

# 8. Launch Dosyası ile Tüm Sistemi Çalıştırma

📂 **launch/system.launch.py**
Tek komutla tüm node’ları çalıştırmak için.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Terminalden alınabilecek parametreler
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency', default_value='2.0', description='Yayın frekansı (Hz)'
    )
    temp_min_arg = DeclareLaunchArgument(
        'temp_min', default_value='15.0', description='Minimum sıcaklık'
    )
    temp_max_arg = DeclareLaunchArgument(
        'temp_max', default_value='35.0', description='Maksimum sıcaklık'
    )
    max_threshold_arg = DeclareLaunchArgument(
        'max_temp_warning_threshold', default_value='28.0', description='Uyarı eşiği'
    )
    min_threshold_arg = DeclareLaunchArgument(
        'min_temp_warning_threshold', default_value='22.0', description='Uyarı eşiği'
    )

    # Node’lar
    temp_sensor = Node(
        package='my_first_pkg',
        executable='temp_sensor',
        name='temp_sensor',
        parameters=[{
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'temp_max': LaunchConfiguration('temp_max'),
            'temp_min': LaunchConfiguration('temp_min')
        }],
        output='screen'
    )

    display_node = Node(
        package='my_first_pkg',
        executable='display_node',
        name='display_node',
        parameters=[{
            'max_temp_warning_threshold': LaunchConfiguration('max_temp_warning_threshold'),
            'min_temp_warning_threshold': LaunchConfiguration('min_temp_warning_threshold')
        }],
        output='screen'
    )

    return LaunchDescription([
        publish_frequency_arg,
        temp_min_arg,
        temp_max_arg,
        min_threshold_arg,
        max_threshold_arg,
        display_node,
        temp_sensor
    ])

```

Çalıştırma:

```bash
ros2 launch my_first_pkg system.launch.py
```

---

# 9. Faydalı ROS Komutları

```bash
ros2 topic list
ros2 node list
ros2 topic info /topicadı
ros2 topic hz /topicadı
ros2 pkg list
ros2 topic echo /topicadı
```

---


amaçlı hazırlanmıştır.

> İleri ROS projeleri (SLAM, Navigation, Robot Control, Gazebo, RViz) için güçlü bir temel oluşturur.
