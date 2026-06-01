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

class FeatureHomographyCalculator(Node):
    def __init__(self):
        super().__init__('feature_homography_calculator')
        self.bridge = CvBridge()
        
        self.get_logger().info("Esperando fotogramas para buscar puntos en común...")

        # Suscriptores sincronizados
        self.sub_cam1 = message_filters.Subscriber(self, Image, '/cam_1/image_raw')
        self.sub_cam2 = message_filters.Subscriber(self, Image, '/cam_2/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_cam1, self.sub_cam2], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.images_captured = False

    def sync_callback(self, msg1, msg2):
        if self.images_captured:
            return
            
        self.images_captured = True
        self.get_logger().info("¡Fotogramas capturados! Iniciando búsqueda de puntos en común...")
        
        img1 = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
        img2 = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
        
        self.compute_homography(img1, img2)
        
        # Una vez calculado, cerramos el nodo
        rclpy.shutdown()

    def compute_homography(self, img1, img2):
        # 1. Cargar calibraciones individuales (Opcional pero muy recomendado para quitar el ojo de pez primero)
        config_dir = os.path.expanduser('~/camera_fusion_ws/src/camera_fusion_pkg/config')
        path_cam1 = os.path.join(config_dir, 'cam_1_calibration.yaml')
        path_cam2 = os.path.join(config_dir, 'cam_2_calibration.yaml')
        
        if os.path.exists(path_cam1) and os.path.exists(path_cam2):
            self.get_logger().info("Aplicando aplanado de lentes...")
            with open(path_cam1, 'r') as f1, open(path_cam2, 'r') as f2:
                d1, d2 = yaml.safe_load(f1), yaml.safe_load(f2)
                K1, D1 = np.array(d1['camera_matrix']['data']).reshape(3,3), np.array(d1['distortion_coefficients']['data'])
                K2, D2 = np.array(d2['camera_matrix']['data']).reshape(3,3), np.array(d2['distortion_coefficients']['data'])
                img1 = cv2.undistort(img1, K1, D1)
                img2 = cv2.undistort(img2, K2, D2)

        # 2. Convertir a escala de grises
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        # 3. Extraer características (SIFT)
        self.get_logger().info("Buscando características (SIFT)...")
        sift = cv2.SIFT_create()
        kp1, des1 = sift.detectAndCompute(gray1, None)
        kp2, des2 = sift.detectAndCompute(gray2, None)

        # 4. Emparejar puntos (FLANN)
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des2, des1, k=2) # Match de img2 hacia img1

        # Filtro de Lowe para quedarse solo con los puntos buenos
        good_matches = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)

        self.get_logger().info(f"Puntos en común encontrados: {len(good_matches)}")

        if len(good_matches) > 10:
            src_pts = np.float32([kp2[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp1[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # 5. Calcular la Homografía robusta (RANSAC ignora los puntos erróneos)
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            
            # Guardar a archivo
            out_yaml = os.path.join(config_dir, 'board_homography.yaml')
            os.makedirs(config_dir, exist_ok=True)
            data = {
                'homography_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': [float(x) for x in H.flatten().tolist()]
                }
            }
            with open(out_yaml, 'w') as f:
                yaml.safe_dump(data, f)
            
            self.get_logger().info(f"✅ ¡Éxito! Homografía basada en puntos guardada en {out_yaml}")

            # 6. Mostrar visualización del resultado
            h, w = img1.shape[:2]
            img2_warped = cv2.warpPerspective(img2, H, (w + w, h))
            img2_warped[0:h, 0:w] = img1
            
            # Dibujar las líneas de coincidencia
            draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=mask.ravel().tolist(), flags=2)
            img_matches = cv2.drawMatches(img2, kp2, img1, kp1, good_matches, None, **draw_params)
            
            # Redimensionar para que quepa en pantalla
            img_matches_resized = cv2.resize(img_matches, (0,0), fx=0.5, fy=0.5)
            
            cv2.imshow("Puntos en Comun Encontrados", img_matches_resized)
            self.get_logger().info("Presiona cualquier tecla en la ventana de la imagen para salir...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        else:
            self.get_logger().error(f"❌ No hay suficientes puntos en común. Solo se encontraron {len(good_matches)}. Ajusta las cámaras o pon un escenario con más detalle delante.")

def main(args=None):
    rclpy.init(args=args)
    node = FeatureHomographyCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # No llamar destroy_node() aquí si ya se cerró en sync_callback
    
if __name__ == '__main__':
    main()
