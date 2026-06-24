#!/usr/bin/env python3
"""
Camera Reader Node — Arquitectura multi-hilo por cámara.

Cada cámara detectada corre en su propio hilo Python que hace cap.read()
en un bucle continuo. Esto elimina el cuello de botella de las llamadas
V4L2 grab() bloqueantes secuenciales que limitaban el FPS del timer.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import threading
import time


class CameraReaderNode(Node):
    def __init__(self):
        super().__init__('camera_reader_node')

        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self._fps    = self.get_parameter('fps').value
        self._width  = self.get_parameter('width').value
        self._height = self.get_parameter('height').value
        self._period = 1.0 / self._fps   # tiempo mínimo entre frames (rate limiter)

        self._running = False
        self._cameras = []
        self._threads = []
        self._bridge  = CvBridge()

        self.get_logger().info("Iniciando auto-descubrimiento de cámaras USB...")
        self._discover_cameras()

        if not self._cameras:
            self.get_logger().error("❌ No se ha detectado ninguna cámara válida.")
            return

        self.get_logger().info(f"Total de cámaras inicializadas: {len(self._cameras)}")

        # Arrancar un hilo de captura por cámara
        self._running = True
        for cam in self._cameras:
            t = threading.Thread(
                target=self._capture_loop,
                args=(cam,),
                daemon=True,
                name=f"capture-{cam['name']}"
            )
            t.start()
            self._threads.append(t)
            self.get_logger().info(
                f"✅ Hilo de captura arrancado para {cam['name']} ({cam['device']})")

    # ── DESCUBRIMIENTO ───────────────────────────────────────────────────────

    def _discover_cameras(self):
        cam_idx = 1
        for i in range(10):
            path = f'/dev/video{i}'
            if not os.path.exists(path):
                continue

            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue

            # 1. Force MJPG to avoid USB Bandwidth saturation (reduces stutters)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            cap.set(cv2.CAP_PROP_FPS,          self._fps)
            # 2. Force OpenCV to keep only the latest frame (prevents queue lag/stutter)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Validar que produce frames reales
            valid = 0
            for _ in range(3):
                ret, frame = cap.read()
                if ret and frame is not None and frame.sum() > 0:
                    valid += 1

            if valid > 0:
                cam_name  = f'cam_{cam_idx}'
                topic     = f'{cam_name}/image_raw'
                pub       = self.create_publisher(Image, topic, 10)

                self._cameras.append({
                    'cap':    cap,
                    'pub':    pub,
                    'name':   cam_name,
                    'device': path,
                })
                self.get_logger().info(
                    f"✅ Cámara {cam_name} en {path} → {topic}")
                cam_idx += 1
            else:
                cap.release()

    # ── BUCLE DE CAPTURA POR HILO ────────────────────────────────────────────

    def _capture_loop(self, cam: dict):
        """Captura y publica frames de una cámara de forma continua.

        El bucle corre en su propio hilo para no bloquear otras cámaras.
        Un rate-limiter suave evita publicar más rápido de lo configurado.
        """
        cap    = cam['cap']
        pub    = cam['pub']
        name   = cam['name']

        while self._running and rclpy.ok():
            ret, frame = cap.read()

            if not ret or frame is None:
                # Pequeña pausa antes de reintentar para no saturar CPU en error
                time.sleep(0.005)
                continue

            # Timestamp de captura real
            stamp = self.get_clock().now().to_msg()

            # Crear un MENSAJE NUEVO cada frame usando cv_bridge.
            # Reutilizar el mismo objeto msg causaba condiciones de carrera con el publicador ROS 2,
            # lo que generaba corrupción y tirones (stutters) esporádicos.
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = f'{name}_link'

            pub.publish(msg)

    # ── LIMPIEZA ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        # Esperar a que los hilos terminen (máx 1s)
        for t in self._threads:
            t.join(timeout=1.0)
        # Liberar capturas
        for cam in self._cameras:
            if cam['cap'].isOpened():
                cam['cap'].release()
                self.get_logger().info(f"Cámara {cam['name']} liberada.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraReaderNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
