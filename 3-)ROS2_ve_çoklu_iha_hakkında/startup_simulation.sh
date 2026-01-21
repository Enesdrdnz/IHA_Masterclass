#!/bin/bash

# Tüm terminallerin açık kalmasını sağlamak için her komutun sonuna 'exec bash' eklenmiştir.
# Bu, komut bittikten sonra terminalin hemen kapanmasını önler.

echo "PX4 Simülasyonu ve Tüm Bağlantı Terminalleri Başlatılıyor..."

# --- 1. Terminal: PX4 SITL Araç 1 (Instance 1 - Port 14540) ---
gnome-terminal --title="1. PX4 SITL - Araç 1" --command='bash -c "
  cd ~/PX4-Autopilot/ &&
  export PX4_HOME_LAT=40.808556 &&
  export PX4_HOME_LON=29.354972 &&
  export PX4_HOME_ALT=10 &&
  echo \"PX4 SITL Araç 1 Başlatılıyor...\" &&
  PX4_SYS_AUTOSTART=4008 PX4_SIM_MODEL=gz_advanced_plane ./build/px4_sitl_default/bin/px4 -i 1;
  exec bash
"' &

# --- 2. Terminal: PX4 SITL Araç 2 (Instance 2 - Port 14541) ---
gnome-terminal --title="2. PX4 SITL - Araç 2" --command='bash -c "
  cd ~/PX4-Autopilot/ &&
  export PX4_HOME_LAT=40.808556 &&
  export PX4_HOME_LON=29.354972 &&
  export PX4_HOME_ALT=10 &&
  echo \"PX4 SITL Araç 2 Başlatılıyor...\" &&
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4008 PX4_GZ_MODEL_POSE=\"-2,0,0.25,0,0,4.7\" PX4_SIM_MODEL=gz_advanced_plane ./build/px4_sitl_default/bin/px4 -i 2;
  exec bash
"' &

# --- 3. Terminal: ROS 2 GZ Köprüsü ---
gnome-terminal --title="3. ROS 2 Bridge" --command='bash -c "
  cd ~/ws_sensor_combined &&
  source /opt/ros/humble/setup.bash &&
  source install/local_setup.bash &&
  echo \"ROS 2 Bridge Başlatılıyor...\" &&
  ros2 run ros_gz_bridge parameter_bridge /world/default/model/advanced_plane_1/link/camera_link/sensor/fpv_camera/image@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /world/default/model/advanced_plane_1/link/camera_link/sensor/fpv_camera/image:=/camera/image_ros_1;
  exec bash
"' &

# --- 4. Terminal: MicroXRCEAgent ---
gnome-terminal --title="4. MicroXRCEAgent" --command='bash -c "
  echo \"MicroXRCEAgent Başlatılıyor...\" &&
  MicroXRCEAgent udp4 -p 8888;
  exec bash
"' &

# --- 5. Terminal: QGroundControl ---
gnome-terminal --title="5. QGroundControl" --command='bash -c "
  cd ~/Downloads/ &&
  echo \"QGroundControl Başlatılıyor...\" &&
  ./QGroundControl-x86_64.AppImage;
  exec bash
"' &

# --- 6. Terminal: Mission Başlatıcı (25 Saniye Gecikmeli ve ROS Source İçerir) ---
#gnome-terminal --title="6. Python Mission Start" --command='bash -c "
#  echo \"PX4 ve Gazebo nun hazır olması için 25 saniye bekleniyor...\" &&
#  sleep 25 &&
#  cd ~/ws_sensor_combined &&
#  
#  # ROS 2 Environment Setup
#  source /opt/ros/humble/setup.bash &&
#  source install/local_setup.bash &&
#  
#  echo \"Görev komutu uçağa gönderiliyor...\" &&
#  python3 mission_start.py;
#  exec bash
#"' &

#echo "Tüm süreçler başlatıldı. 25 saniye sonra uçağınız göreve başlayacak."
