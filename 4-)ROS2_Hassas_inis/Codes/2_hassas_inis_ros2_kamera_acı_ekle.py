import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')
        self.bridge = CvBridge()
        
        # Kamera Kalibrasyon
        self.camera_matrix = np.array([[410.93925, 0.0, 640.0], [0.0, 410.93926, 360.0], [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))
        
        # ArUco Tanımlama
        self.marker_size = 1.0 
        self.obj_points = np.array([[-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, -0.5, 0]], dtype=np.float32)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())

        # Kontrol Kazançları
        self.P_gain = 0.7         # Biraz artırıldı (Tepki için)
        self.descend_speed = 0.3  # m/s
        self.land_alt = 1.3
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.image_sub = self.create_subscription(Image, '/camera/image_0', self.image_cb, 10)

        self.nav_state = None
        self.marker_found = False
        self.error_x, self.error_y, self.error_z = 0.0, 0.0, 0.0
        
        self.timer = self.create_timer(0.05, self.run_loop)
        self.get_logger().info("Sistem Hazır. Marker Algılandığında Hareket Başlayacak.")

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
                    self.error_x, self.error_y, self.error_z = tvec.flatten()
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            else:
                self.marker_found = False
            cv2.imshow("Gimbal View", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Görüntü hatası: {e}")

    def run_loop(self):
        # 1. Daima Heartbeat gönder (Zorunlu!)
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        if self.marker_found:
            # Offboard'a geçiş komutu (Sadece geçmemişse)
            if self.nav_state != 14: # 14 = NAVIGATION_STATE_OFFBOARD
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            
            # Hız komutlarını gönder
            self.publish_velocity()
        else:
            # Marker yoksa olduğu yerde dur (Frenleme)
            self.publish_zero_velocity()

    def publish_velocity(self):
        msg = TrajectorySetpoint()
        # PX4 v1.14+ için NaN zorunluluğu
        msg.position = [float('nan')] * 3
        msg.yaw = float('nan')

        # Koordinat Dönüşümü (Kamera 180 derece ters takılı)
        # Kamera X -> Drone Sağ-Sol (Vy) | Kamera Y -> Drone İleri-Geri (Vx)
        vx = -self.error_y * self.P_gain
        vy = self.error_x * self.P_gain
        vz = self.descend_speed

        msg.velocity = [float(np.clip(vx, -1.5, 1.5)), float(np.clip(vy, -1.5, 1.5)), float(vz)]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

        if self.error_z < self.land_alt:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_zero_velocity(self):
        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.velocity = [0.0, 0.0, 0.0]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1, msg.param2 = p1, p2
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
