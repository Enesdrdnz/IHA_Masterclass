import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoCalibratedNode(Node):
    def __init__(self):
        super().__init__('aruco_calibrated_node')
        self.bridge = CvBridge()
        self.window_name = "ArUco Kalibreli Takip"
        
        # --- KALİBRASYON VERİLERİ (Girdiğin CameraInfo'ya göre) ---
        self.camera_matrix = np.array([
            [410.93925476, 0.0,          640.0],
            [0.0,          410.93926906, 360.0],
            [0.0,          0.0,          1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((5, 1)) # Distortion verilerin 0 olduğu için
        # ---------------------------------------------------------

        self.marker_size = 0.5  # Marker boyutu 1 metre

        # 3D Obje Noktaları
        self.obj_points = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        self.subscription = self.create_subscription(
            Image, '/camera/image_0', self.image_callback, 10)
        
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners, ids, _ = self.detector.detectMarkers(frame)

            if ids is not None:
                for i in range(len(ids)):
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, corners[i], 
                        self.camera_matrix, self.dist_coeffs
                    )
                    
                    if success:
                        # tvec[0]: X (sağ-sol), tvec[1]: Y (ileri-geri), tvec[2]: Z (yükseklik)
                        x, y, z = tvec.flatten()

                        # Görselleştirme
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.5)
                        
                        label = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m"
                        cv2.putText(frame, label, (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        self.get_logger().info(f"ID {ids[i][0]} -> Konum: {label}")

            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Hata: {e}")

def main():
    rclpy.init()
    node = ArucoCalibratedNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
