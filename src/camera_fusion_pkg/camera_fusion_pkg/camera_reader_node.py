import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraReaderNode(Node):
    def __init__(self):
        super().__init__('camera_reader_node')

        # Declarar parámetros
        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        # Leer parámetros
        fps = self.get_parameter('fps').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value

        self.get_logger().info("Iniciando auto-descubrimiento de cámaras USB...")

        self.bridge = CvBridge()
        self.cameras = []      # Lista de diccionarios con {'cap', 'pub', 'name', 'device'}

        # Auto-descubrimiento (escanear de /dev/video0 a /dev/video9)
        cam_idx = 1
        for i in range(10):
            path = f'/dev/video{i}'
            if not os.path.exists(path):
                continue
            
            # Usamos explícitamente V4L2 para evitar que GStreamer colapse al cambiar resolución
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            
            # Ajustamos propiedades ANTES de leer
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            
            # Comprobar que proporciona frames de vídeo reales (y no solo metadata)
            valid_frames = 0
            for _ in range(3):
                ret, frame = cap.read()
                if ret and frame is not None and frame.sum() > 0:
                    valid_frames += 1
                    
            if valid_frames > 0:
                # Cámara válida detectada
                cam_name = f'cam_{cam_idx}'
                
                topic_name = f'{cam_name}/image_raw'
                pub = self.create_publisher(Image, topic_name, 10)
                
                self.cameras.append({
                    'cap': cap,
                    'pub': pub,
                    'name': cam_name,
                    'device': path
                })
                
                self.get_logger().info(f"✅ Cámara {cam_name} detectada y configurada en {path} -> {topic_name}")
                cam_idx += 1
            else:
                cap.release()

        if not self.cameras:
            self.get_logger().error("❌ No se ha detectado ninguna cámara válida en /dev/video*")
            return

        self.get_logger().info(f"Total de cámaras inicializadas: {len(self.cameras)}")

        # Timer para capturar y publicar frames de todas las cámaras a la vez
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 1. Hacer 'grab' en todas las cámaras primero para minimizar asincronía en la captura USB
        for cam in self.cameras:
            cam['grabbed'] = cam['cap'].grab()

        # Tiempo base para todos los frames del mismo ciclo
        timestamp = self.get_clock().now().to_msg()

        # 2. Hacer 'retrieve' y publicar secuencialmente
        for cam in self.cameras:
            if cam['grabbed']:
                ret, frame = cam['cap'].retrieve()
                if ret:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = timestamp
                    msg.header.frame_id = f"{cam['name']}_link"
                    cam['pub'].publish(msg)
                else:
                    self.get_logger().warning(f"Error al recuperar frame de {cam['name']}")
            else:
                self.get_logger().warning(f"Error al hacer grab de frame en {cam['name']}")


    def destroy_node(self):
        # Liberar todas las cámaras
        for cam in self.cameras:
            if cam['cap'].isOpened():
                cam['cap'].release()
                self.get_logger().info(f"Cámara {cam['name']} ({cam['device']}) liberada.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
