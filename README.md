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
<orijinal talker.py içeriği>
```

---

## 5.2 Listener Node

📄 **listener.py**
`chatter` topic'ini dinler ve mesajları terminale basar.

```python
<orijinal listener.py içeriği>
```

---

# 6. Gerçekçi Sensör Uygulaması

## 6.1 Temp Sensor Node

📄 **temp_sensor.py**
Parametreye bağlı sıcaklık simülasyonu üretir.

```python
<orijinal temp_sensor.py içeriği>
```

---

## 6.2 Display Node

📄 **display_node.py**
Sıcaklık değerlerini okuyarak uyarı üretir.

```python
<orijinal display_node.py içeriği>
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
<orijinal system.launch.py içeriği>
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

# Amaç

Bu README dosyası:

* Hızlı tekrar
* Hızlı referans
* Kolay navigasyon

amaçlı hazırlanmıştır.

> İleri ROS projeleri (SLAM, Navigation, Robot Control, Gazebo, RViz) için güçlü bir temel oluşturur.
