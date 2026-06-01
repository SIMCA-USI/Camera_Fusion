#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
import yaml
import os

class PanoramicFusionNode(Node):
    def __init__(self):
        super().__init__('panoramic_fusion_node')
        self.bridge = CvBridge()

        # 1. PARÁMETROS DINÁMICOS
        self.declare_parameter('overlap_start', 400)
        self.declare_parameter('overlap_end', 600)
        self.declare_parameter('canvas_w', 1100)
        self.declare_parameter('canvas_h', 480)
        self.declare_parameter('cam_w', 640)
        self.declare_parameter('cam_h', 480)

        self.overlap_start = self.get_parameter('overlap_start').value
        self.overlap_end = self.get_parameter('overlap_end').value
        self.canvas_w = self.get_parameter('canvas_w').value
        self.canvas_h = self.get_parameter('canvas_h').value
        self.cam_w = self.get_parameter('cam_w').value
        self.cam_h = self.get_parameter('cam_h').value

        # 2. CARGA DE ARCHIVOS YAML
        config_dir = os.path.expanduser('~/camera_fusion_ws/src/camera_fusion_pkg/config')
        path_cam1 = os.path.join(config_dir, 'cam_1_calibration.yaml')
        path_cam2 = os.path.join(config_dir, 'cam_2_calibration.yaml')
        path_homo = os.path.join(config_dir, 'board_homography.yaml')

        if not (os.path.exists(path_cam1) and os.path.exists(path_cam2) and os.path.exists(path_homo)):
            self.get_logger().error("Faltan archivos de calibración. Asegúrate de ejecutar calibraciones primero.")
            # Valores por defecto dummy para evitar crash inmediato si se lanza sin calibrar (para debugear)
            self.K1 = self.K2 = np.eye(3)
            self.D1 = self.D2 = np.zeros(5)
            self.H = np.eye(3)
        else:
            self.K1, self.D1 = self.load_intrinsics(path_cam1)
            self.K2, self.D2 = self.load_intrinsics(path_cam2)
            self.H = self.load_homography(path_homo)

        self.get_logger().info("Precomputando mapas de rectificación y máscara alpha...")

        # 3. PRECOMPUTACIÓN (MAPAS DE DISTORSIÓN)
        self.map1_x, self.map1_y = cv2.initUndistortRectifyMap(self.K1, self.D1, None, self.K1, (self.cam_w, self.cam_h), cv2.CV_16SC2)
        self.map2_x, self.map2_y = cv2.initUndistortRectifyMap(self.K2, self.D2, None, self.K2, (self.cam_w, self.cam_h), cv2.CV_16SC2)

        # 4. PRECOMPUTACIÓN (MÁSCARA DE BLENDING LINEAL)
        self.update_alpha_mask()

        # 5. PUBLICADOR Y SUSCRIPTORES SINCRONIZADOS
        self.fused_pub = self.create_publisher(Image, 'fused_panorama', 10)

        self.sub_cam1 = message_filters.Subscriber(self, Image, 'cam_1/image_raw')
        self.sub_cam2 = message_filters.Subscriber(self, Image, 'cam_2/image_raw')
        
        # Tolerancia estricta para mantener los 30 FPS sin lag visual
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_cam1, self.sub_cam2], 
            queue_size=10, 
            slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("✅ Nodo de Fusión Panorámica listo. Esperando imágenes...")

    def load_intrinsics(self, filepath):
        with open(filepath, 'r') as f:
            d = yaml.safe_load(f)
            return np.array(d['camera_matrix']['data']).reshape(3,3), np.array(d['distortion_coefficients']['data'])

    def load_homography(self, filepath):
        with open(filepath, 'r') as f:
            d = yaml.safe_load(f)
            return np.array(d['homography_matrix']['data']).reshape(3,3)

    def update_alpha_mask(self):
        # La máscara define qué porcentaje de la cámara 1 se usa.
        # 1.0 = Solo Cam 1. 0.0 = Solo Cam 2 (Warped).
        self.alpha_mask = np.ones((self.canvas_h, self.canvas_w, 3), dtype=np.float32)
        
        # Para evitar crashes si los parámetros son inválidos
        start = max(0, min(self.overlap_start, self.canvas_w - 1))
        end = max(start + 1, min(self.overlap_end, self.canvas_w))

        # Degradado en la zona de solape
        gradient = np.linspace(1.0, 0.0, end - start)
        for i in range(3):
            self.alpha_mask[:, start:end, i] = gradient
        
        # 0.0 para todo lo que esté a la derecha del solape
        self.alpha_mask[:, end:, :] = 0.0

    def sync_callback(self, msg1, msg2):
        try:
            # 1. Extracción de imágenes
            img1 = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
            img2 = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
            
            # 2. Rectificación (Aplanado de la lente)
            img1_rect = cv2.remap(img1, self.map1_x, self.map1_y, cv2.INTER_LINEAR)
            img2_rect = cv2.remap(img2, self.map2_x, self.map2_y, cv2.INTER_LINEAR)

            # 3. Warping (Alineación usando Homografía)
            # Doblamos la imagen 2 para que encaje en el espacio coordenado de la imagen 1
            img2_warped = cv2.warpPerspective(img2_rect, self.H, (self.canvas_w, self.canvas_h))

            # 4. Canvas y Blending (OPTIMIZADO)
            # Creamos un canvas directamente en uint8 para ahorrar memoria
            canvas = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
            
            start = max(0, min(self.overlap_start, self.cam_w))
            end = max(start + 1, min(self.overlap_end, self.cam_w))

            # a) Copiar la parte izquierda pura (Cámara 1) sin cálculos flotantes
            canvas[:, 0:start] = img1_rect[:, 0:start]
            
            # b) Copiar la parte derecha pura (Cámara 2 transformada)
            canvas[:, end:self.canvas_w] = img2_warped[:, end:self.canvas_w]
            
            # c) Calcular el degradado SOLO en la zona de solape (Ahorra un 80% de CPU)
            roi_1 = img1_rect[:, start:end].astype(np.float32)
            roi_2 = img2_warped[:, start:end].astype(np.float32)
            alpha = self.alpha_mask[:, start:end]
            
            blended_roi = (roi_1 * alpha) + (roi_2 * (1.0 - alpha))
            canvas[:, start:end] = blended_roi.astype(np.uint8)

            # 5. Publicación
            out_msg = self.bridge.cv2_to_imgmsg(canvas, "bgr8")
            out_msg.header = msg1.header
            out_msg.header.frame_id = 'panoramic_link'
            self.fused_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error procesando frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PanoramicFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
