import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleAttitude

class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')
        self.bridge = CvBridge()
        
        # --- 1. Kamera Kalibrasyonu (Senin Verilerin) ---
        self.camera_matrix = np.array([[410.93925, 0.0, 640.0], [0.0, 410.93926, 360.0], [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))
        
        # --- 2. ArUco Ayarları ---
        self.marker_size = 1.0 
        self.obj_points = np.array([[-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, -0.5, 0]], dtype=np.float32)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())

        # --- 3. Kontrol Parametreleri ---
        self.base_P_gain = 0.6
        self.min_P_gain = 0.2
        self.descend_speed = 0.25
        self.land_alt = 1.2
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Yayıncılar ve Aboneler
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos)
        self.image_sub = self.create_subscription(Image, '/camera/image_0', self.image_cb, 10)

        self.nav_state = None
        self.roll, self.pitch = 0.0, 0.0
        self.marker_found = False
        self.error_x, self.error_y, self.error_z = 0.0, 0.0, 0.0
        
        self.timer = self.create_timer(0.05, self.run_loop)
        self.get_logger().info("XYZ Mesafe Bilgili Hassas İniş Modu Başlatıldı.")

    def attitude_cb(self, msg):
        q = msg.q
        self.roll = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1]**2 + q[2]**2))
        self.pitch = math.asin(2.0 * (q[0] * q[2] - q[3] * q[1]))

    def status_cb(self, msg):
        self.nav_state = msg.nav_state

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            corners, ids, _ = self.detector.detectMarkers(frame)
            
            if ids is not None:
                success, rvec, tvec = cv2.solvePnP(self.obj_points, corners[0], self.camera_matrix, self.dist_coeffs)
                if success:
                    self.marker_found = True
                    raw_x, raw_y, raw_z = tvec.flatten()

                    # Açı düzeltmesi (Drone eğimini kompanse et)
                    self.error_y = raw_y - (raw_z * math.tan(self.pitch))
                    self.error_x = raw_x + (raw_z * math.tan(self.roll))
                    self.error_z = raw_z
                    
                    # --- OpenCV Ekranına Yazdırma ---
                    cv2.rectangle(frame, (10, 10), (300, 160), (0, 0, 0), -1) # Bilgi kutusu
                    
                    # XYZ değerlerini ekrana bas (Düzeltilmiş değerler)
                    cv2.putText(frame, f"X (Right): {self.error_x:+.2f} m", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Y (Front): {self.error_y:+.2f} m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Z (Height): {self.error_z:+.2f} m", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    
                    # Marker üzerine eksenleri ve çerçeveyi çiz
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.5)
            else:
                self.marker_found = False
                cv2.putText(frame, "MARKER LOST", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.imshow("Precision Landing XYZ", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Hata: {e}")

    def run_loop(self):
        self.publish_offboard_heartbeat()
        if self.marker_found:
            if self.nav_state != 14: # NAVIGATION_STATE_OFFBOARD
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_velocity()
            # Terminale XYZ yazdır
            print(f"DEBUG -> X: {self.error_x:+.3f} | Y: {self.error_y:+.3f} | Z: {self.error_z:+.3f}")
        else:
            self.publish_zero_velocity()

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position, msg.velocity = False, True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_velocity(self):
        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.yaw = float('nan')

        # Dinamik Kazanç ve Hız Sınırlama (Önceki yumuşatma mantığı)
        current_P = self.min_P_gain + (self.base_P_gain - self.min_P_gain) * (np.clip(self.error_z, 0, 5) / 5.0)
        max_v = np.clip(0.25 * self.error_z, 0.2, 1.0) 

        vx = -self.error_y * current_P
        vy = self.error_x * current_P
        vz = self.descend_speed

        msg.velocity = [float(np.clip(vx, -max_v, max_v)), float(np.clip(vy, -max_v, max_v)), float(vz)]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

        if self.error_z < self.land_alt:
            self.get_logger().info("Hassas iniş bölgesine girildi. İniş başlatılıyor.")
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_zero_velocity(self):
        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.velocity = [0.0, 0.0, 0.0]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command, msg.param1, msg.param2 = command, p1, p2
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

def main():
    rclpy.init()
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
