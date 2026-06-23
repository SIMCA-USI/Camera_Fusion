#!/usr/bin/env python3
"""
Camera Reader Node — Arquitectura multi-hilo por cámara.

Cada cámara detectada corre en su propio hilo Python que hace cap.read()
en un bucle continuo. Esto elimina el cuello de botella de las llamadas
V4L2 grab() bloqueantes secuenciales que limitaban el FPS del timer.

Resolución de salida:
    Todos los frames publicados son SIEMPRE de tamaño (target_size × target_size),
    independientemente del modo nativo que soporte el hardware. Si la cámara
    devuelve una resolución diferente, se aplica cv2.resize antes de publicar.
    Esto garantiza un formato uniforme para todos los consumidores downstream
    (nodo de fusión, YOLO, etc.) sin necesidad de adaptar resoluciones en cada nodo.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import threading
import time


class CameraReaderNode(Node):
    def __init__(self):
        super().__init__('camera_reader_node')

        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 640)
        self.declare_parameter('target_size', 640)   # resolución cuadrada de salida garantizada

        self._fps         = self.get_parameter('fps').value
        self._width       = self.get_parameter('width').value
        self._height      = self.get_parameter('height').value
        self._target_size = self.get_parameter('target_size').value
        self._period      = 1.0 / self._fps

        self._running = False
        self._cameras = []
        self._threads = []

        self.get_logger().info(
            f"Iniciando auto-descubrimiento de cámaras USB "
            f"(salida garantizada: {self._target_size}×{self._target_size})...")
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

            # Solicitar la resolución deseada al driver (puede ser ignorado)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            cap.set(cv2.CAP_PROP_FPS,          self._fps)

            # Validar que produce frames reales
            valid = 0
            for _ in range(3):
                ret, frame = cap.read()
                if ret and frame is not None and frame.sum() > 0:
                    valid += 1

            if valid > 0:
                cam_name = f'cam_{cam_idx}'
                topic    = f'{cam_name}/image_raw'
                pub      = self.create_publisher(Image, topic, 10)

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
        """Captura, normaliza a target_size×target_size y publica frames.

        El bucle corre en su propio hilo para no bloquear otras cámaras.
        SIEMPRE se aplica cv2.resize al tamaño de salida garantizado,
        independientemente de la resolución real devuelta por el hardware.
        Esto hace el código agnóstico al tipo de cámara conectada.
        """
        cap    = cam['cap']
        pub    = cam['pub']
        name   = cam['name']
        sz     = self._target_size

        # Pre-construir el objeto Image con los campos fijos (evita alloc por frame).
        # Tamaño garantizado: target_size × target_size × BGR.
        msg              = Image()
        msg.height       = sz
        msg.width        = sz
        msg.encoding     = 'bgr8'
        msg.is_bigendian = False
        msg.step         = sz * 3
        msg.header.frame_id = f'{name}_link'

        while self._running and rclpy.ok():
            ret, frame = cap.read()

            if not ret or frame is None:
                time.sleep(0.005)
                continue

            # ── Normalización a resolución cuadrada garantizada ──────────────
            # Se aplica SIEMPRE para que el nodo sea independiente del hardware.
            # Si la cámara ya devuelve sz×sz, el resize es un no-op barato de OpenCV.
            h, w = frame.shape[:2]
            if h != sz or w != sz:
                frame = cv2.resize(frame, (sz, sz), interpolation=cv2.INTER_LINEAR)

            # Timestamp de captura (más preciso que el de publicación)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data         = frame.tobytes()
            pub.publish(msg)

    # ── LIMPIEZA ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
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
