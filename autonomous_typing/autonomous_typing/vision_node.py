import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 1. Publicadores
        self.pose_publisher_ = self.create_publisher(Pose, '/keyboard_pose', 10)
        self.img_publisher_ = self.create_publisher(Image, '/camera/image_processed', 10)
        
        self.br = CvBridge()

        # 2. Configuración de la Cámara (Intenta índice 0, si falla prueba 1)
        # En Jetson, a veces la camara CSI es la 0 y la USB es la 1.
        self.cap = cv2.VideoCapture(0) 
        if not self.cap.isOpened():
             self.get_logger().warn("No se pudo abrir video0, intentando video1...")
             self.cap = cv2.VideoCapture(1)

        # Forzar resolución 720p (Logitech standard)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 3. Matriz de Cámara (Aproximada para 720p)
        self.camera_matrix = np.array([[1000.0, 0.0, 640.0], [0.0, 1000.0, 360.0], [0.0, 0.0, 1.0]], dtype=float)
        self.dist_coeffs = np.zeros((5, 1))

        # 4. Configuración ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # 5. Timer (Bucle principal a 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Vision Node Iniciado. Capturando video...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().error("No se pudo leer el frame de la cámara")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # Si detectamos al menos un marcador
        if ids is not None and len(ids) >= 1:
            # Dibujar los marcadores en la imagen para verlos en RQT
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimación de pose
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.02, self.camera_matrix, self.dist_coeffs)

            # Promedio de posiciones para hallar el centro
            x_sum = y_sum = z_sum = 0
            count = len(ids)
            
            for i in range(count):
                x_sum += tvecs[i][0][0]
                y_sum += tvecs[i][0][1]
                z_sum += tvecs[i][0][2]
                # Opcional: Dibujar ejes para debug visual
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.01)

            msg = Pose()
            msg.position.x = x_sum / count
            msg.position.y = y_sum / count
            msg.position.z = z_sum / count
            msg.orientation.w = 1.0 

            self.pose_publisher_.publish(msg)
            # Log de debug para que veas en la terminal si está calculando algo
            # self.get_logger().info(f"Teclado detectado en X:{msg.position.x:.2f} Y:{msg.position.y:.2f} Z:{msg.position.z:.2f}")

        # Publicar la imagen para RQT
        self.img_publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()