import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import time
# Asegúrate de importar tus configs
from .config import KEY_MAP, APPROACH_DISTANCE, PRESS_DEPTH, GRIPPER_ROLL, GRIPPER_PITCH

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.arm_pub = self.create_publisher(Pose, '/goal', 10)
        self.create_subscription(String, '/launch_key', self.launch_callback, 10)
        self.create_subscription(Pose, '/keyboard_pose', self.pose_callback, 10)

        self.keyboard_center = None
        self.state = "IDLE" 
        self.launch_code = ""

        self.get_logger().info("Controlador Listo. Esperando comando...")

    def pose_callback(self, msg):
        self.keyboard_center = msg

    def launch_callback(self, msg):
        if self.state == "IDLE":
            self.launch_code = msg.data.upper()
            self.get_logger().info(f"Comando recibido: {self.launch_code}. Iniciando...")
            self.state = "EXECUTING"
            # Ejecutamos la secuencia en un método aparte (idealmente debería ser en un thread si usas time.sleep)
            self.run_typing_sequence()

    def send_ik_goal(self, x, y, z, roll, pitch, delay=2.0):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.x = float(roll)
        msg.orientation.y = float(pitch)
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        self.arm_pub.publish(msg)
        time.sleep(delay) 

    def run_typing_sequence(self):
        # 1. VERIFICACIÓN INICIAL
        if self.keyboard_center is None:
            self.get_logger().error("¡ERROR! No veo los ArUcos. Abortando.")
            self.state = "IDLE"
            return

        # Capturamos la posición del teclado EN ESTE INSTANTE (snapshot)
        # Esto evita que si la cámara vibra, el objetivo se mueva mientras tecleas
        kb_ref_x = self.keyboard_center.position.x
        kb_ref_y = self.keyboard_center.position.y
        kb_ref_z = self.keyboard_center.position.z

        self.get_logger().info(f"Teclado fijado en: X={kb_ref_x:.2f}, Y={kb_ref_y:.2f}, Z={kb_ref_z:.2f}")

        # --- FASE 0: CENTRADO (LO QUE FALTABA) ---
        self.get_logger().info(">>> FASE 0: CENTRANDO BRAZO")
        
        # Calculamos el punto central "seguro" (frente al logo de Redragon, sin tocar teclas)
        # Usamos APPROACH_DISTANCE (ej. 5cm) hacia atrás en Z
        center_hover_x = kb_ref_x 
        center_hover_y = kb_ref_y
        center_hover_z = kb_ref_z - APPROACH_DISTANCE # Ajusta +/- según tu eje de profundidad

        # Enviamos al brazo al centro y esperamos MÁS tiempo para estabilizar
        self.send_ik_goal(center_hover_x, center_hover_y, center_hover_z, GRIPPER_ROLL, GRIPPER_PITCH, delay=4.0)

        # Aquí podrías agregar una lógica de "Verificación":
        # Si tuvieras feedback de los motores, checarías si llegaron.
        # Por ahora, confiamos en el delay de 4 segundos.

        # --- FASE 1: TECLEO ---
        self.get_logger().info(f">>> FASE 1: TECLEANDO '{self.launch_code}'")

        for char in self.launch_code:
            if char in KEY_MAP:
                self.get_logger().info(f"--> Letra: {char}")
                
                dx, dy = KEY_MAP[char]

                # Coordenadas objetivo
                target_x = kb_ref_x + dx
                target_y = kb_ref_y + dy
                
                # A. Moverse sobre la tecla (Hover)
                hover_z = kb_ref_z - APPROACH_DISTANCE
                self.send_ik_goal(target_x, target_y, hover_z, GRIPPER_ROLL, GRIPPER_PITCH, delay=1.5)

                # B. Presionar (Push)
                press_z = kb_ref_z + PRESS_DEPTH 
                self.send_ik_goal(target_x, target_y, press_z, GRIPPER_ROLL, GRIPPER_PITCH, delay=0.5)

                # C. Retraer (Pull back)
                self.send_ik_goal(target_x, target_y, hover_z, GRIPPER_ROLL, GRIPPER_PITCH, delay=0.5)
            
            else:
                self.get_logger().warn(f"Letra {char} no configurada. Saltando.")

        # --- FASE 2: RETIRADA ---
        self.get_logger().info(">>> FASE 2: TERMINADO. YENDO A HOME.")
        self.send_ik_goal(0.15, 0.0, 0.35, 0.0, 0.0) # Tu HOME definido en C++
        self.state = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()