#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
import cv2
import numpy as np
import yaml
import os
import threading
from threading import Lock

# ── Forzar a OpenCV a usar TODOS los hilos de la CPU disponibles.
# Las operaciones internas de OpenCV (remap, addWeighted...) son C++ puro
# y se ejecutan en sus propios threads, saltándose el GIL de Python.
cv2.setNumThreads(0)

# QoS para recepción de imágenes: BEST_EFFORT (sin reintentos) pero depth=4
# para absorber las ráfagas irregulares del capture USB sin descartar frames.
_QOS_SENSOR_SUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=4
)
# Publisher: depth=1 es suficiente (el consumidor quiere el más reciente)
_QOS_SENSOR_PUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class PanoramicFusionNode(Node):
    def __init__(self):
        super().__init__('panoramic_fusion_node')

        # 1. PARÁMETROS
        self.declare_parameter('overlap_start', 400)
        self.declare_parameter('overlap_end', 600)
        self.declare_parameter('canvas_w', 1100)
        self.declare_parameter('canvas_h', 480)
        self.declare_parameter('cam_w', 640)
        self.declare_parameter('cam_h', 480)
        self.declare_parameter('slop_ms', 150)  # 150ms: absorbe irregularidades USB
        # INTER_NEAREST (0) es ~4x mas rapido que INTER_LINEAR (1)
        # a costa de un poco de calidad en el borde de la imagen
        self.declare_parameter('interp', cv2.INTER_LINEAR)

        self.overlap_start = self.get_parameter('overlap_start').value
        self.overlap_end   = self.get_parameter('overlap_end').value
        self.canvas_w      = self.get_parameter('canvas_w').value
        self.canvas_h      = self.get_parameter('canvas_h').value
        self.cam_w         = self.get_parameter('cam_w').value
        self.cam_h         = self.get_parameter('cam_h').value
        self.slop_seconds  = self.get_parameter('slop_ms').value / 1000.0
        self.interp        = self.get_parameter('interp').value

        # 2. CALIBRACIÓN
        config_dir = os.path.expanduser(
            '~/camera_fusion_ws/src/camera_fusion_pkg/config')
        path_cam1 = os.path.join(config_dir, 'cam_1_calibration.yaml')
        path_cam2 = os.path.join(config_dir, 'cam_2_calibration.yaml')
        path_homo = os.path.join(config_dir, 'board_homography.yaml')

        if not (os.path.exists(path_cam1) and
                os.path.exists(path_cam2) and
                os.path.exists(path_homo)):
            self.get_logger().error(
                "Faltan archivos de calibración. Usando identidad como fallback.")
            K = np.eye(3, dtype=np.float64)
            D = np.zeros(5, dtype=np.float64)
            self.K1, self.D1 = K.copy(), D.copy()
            self.K2, self.D2 = K.copy(), D.copy()
            self.H = np.eye(3, dtype=np.float64)
        else:
            self.K1, self.D1 = self._load_intrinsics(path_cam1)
            self.K2, self.D2 = self._load_intrinsics(path_cam2)
            self.H           = self._load_homography(path_homo)

        # 3. PRECOMPUTACIÓN DE MAPAS
        self.get_logger().info("Precomputando mapas de rectificación...")
        size_cam  = (self.cam_w, self.cam_h)
        size_cnvs = (self.canvas_w, self.canvas_h)

        # Cam 1: mapa de distorsión (CV_16SC2 → formato óptimo de OpenCV)
        self.map1_x, self.map1_y = cv2.initUndistortRectifyMap(
            self.K1, self.D1, None, self.K1, size_cam, cv2.CV_16SC2)

        # Cam 2: mapa combinado distorsión + homografía en UN SOLO remap
        m2x, m2y = cv2.initUndistortRectifyMap(
            self.K2, self.D2, None, self.K2, size_cam, cv2.CV_32FC1)
        cx = cv2.warpPerspective(m2x, self.H, size_cnvs, flags=cv2.INTER_LINEAR)
        cy = cv2.warpPerspective(m2y, self.H, size_cnvs, flags=cv2.INTER_LINEAR)
        self.map2_x, self.map2_y = cv2.convertMaps(cx, cy, cv2.CV_16SC2)

        # Calcular pesos de blending (preallocado, no se recalcula por frame)
        self._blend_start = max(0, min(self.overlap_start, self.cam_w - 1))
        self._blend_end   = max(self._blend_start + 1,
                                min(self.overlap_end, self.canvas_w))
        blend_w = self._blend_end - self._blend_start
        # Pesos en uint16 escalados a 256 — aritmética entera, sin float por frame
        # alpha_w16[i] = round(alpha_i * 256), shape [1, blend_w, 1]
        alpha_f = np.linspace(256.0, 0.0, blend_w, dtype=np.float32)
        self._alpha_w   = alpha_f.round().astype(np.uint16)[np.newaxis, :, np.newaxis]
        self._alpha_inv_w = (256 - self._alpha_w).astype(np.uint16)
        # Mantener float32 como fallback por si acaso
        self._alpha     = (self._alpha_w / 256.0).astype(np.float32)
        self._alpha_inv = 1.0 - self._alpha

        # Thread-local storage: cada hilo del executor tiene sus propios buffers.
        # Se asignan una sola vez por hilo (no malloc por frame).
        # Esto ELIMINA la condicion de carrera en out1/out2 y evita malloc continuo.
        self._local = threading.local()

        # Objeto Image reutilizable: solo se crea una vez, los campos fijos
        # no cambian nunca. Solo se actualizan .data y .header cada frame.
        self._out_msg             = Image()
        self._out_msg.height      = self.canvas_h
        self._out_msg.width       = self.canvas_w
        self._out_msg.encoding    = 'bgr8'
        self._out_msg.is_bigendian = False
        self._out_msg.step        = self.canvas_w * 3

        # 5. BÚFERES DE SINCRONIZACIÓN Y LOCK
        self.cam1_buffer   = []
        self.cam2_buffer   = []
        self.max_buffer_sz = 8   # más buffer para absorber ráfagas USB
        self.fusion_lock   = Lock()

        # 6. PUBLISHER Y SUSCRIPTORES
        self.callback_group = ReentrantCallbackGroup()
        self.fused_pub = self.create_publisher(Image, 'fused_panorama', _QOS_SENSOR_PUB)

        self.create_subscription(Image, 'cam_1/image_raw',
                                 self._cam1_cb, _QOS_SENSOR_SUB,
                                 callback_group=self.callback_group)
        self.create_subscription(Image, 'cam_2/image_raw',
                                 self._cam2_cb, _QOS_SENSOR_SUB,
                                 callback_group=self.callback_group)

        self.get_logger().info(
            "✅ Nodo de Fusión Panorámica (High-Performance) listo.")

    # ── HELPERS DE CARGA ────────────────────────────────────────────────────

    def _load_intrinsics(self, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        K = np.array(d['camera_matrix']['data'],
                     dtype=np.float64).reshape(3, 3)
        D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)
        return K, D

    def _load_homography(self, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        return np.array(d['homography_matrix']['data'],
                        dtype=np.float64).reshape(3, 3)

    # ── CONVERSIÓN ROS → NUMPY SIN COPIA EXTRA ──────────────────────────────

    @staticmethod
    def _msg_to_bgr(msg):
        """Convierte un sensor_msgs/Image BGR8 a numpy array evitando copias."""
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)

    # ── CALLBACKS DE CÁMARA ─────────────────────────────────────────────────

    def _cam1_cb(self, msg):
        self.cam1_buffer.append(msg)
        if len(self.cam1_buffer) > self.max_buffer_sz:
            self.cam1_buffer.pop(0)
        self._try_fuse()

    def _cam2_cb(self, msg):
        self.cam2_buffer.append(msg)
        if len(self.cam2_buffer) > self.max_buffer_sz:
            self.cam2_buffer.pop(0)
        self._try_fuse()

    # ── SINCRONIZADOR SUAVE ─────────────────────────────────────────────────

    def _best_match(self):
        """Publica el par de frames más recientes si ambas cámaras tienen datos.

        Estrategia:
        1. Necesitamos al menos 1 frame en cada búfer.
        2. Tomamos el más reciente de cada uno.
        3. Si la diferencia de tiempo entre ellos supera max_pair_diff (500ms),
           descartamos el más antiguo de los dos (puede ser un frame fantasma
           por fallo de una cámara).
        4. Si la diferencia es aceptable, publicamos y limpiamos ambos búferes.
        """
        if not self.cam1_buffer or not self.cam2_buffer:
            return None, None

        m1 = self.cam1_buffer[-1]
        m2 = self.cam2_buffer[-1]

        t1 = m1.header.stamp.sec + m1.header.stamp.nanosec * 1e-9
        t2 = m2.header.stamp.sec + m2.header.stamp.nanosec * 1e-9
        diff = abs(t1 - t2)

        # Si la diferencia entre cámaras es muy grande, el frame más antiguo
        # es un residuo de cuando una cámara falló. Descartarlo y esperar.
        if diff > self.slop_seconds:
            if t1 < t2:
                self.cam1_buffer.clear()
            else:
                self.cam2_buffer.clear()
            return None, None

        # Par válido: publicar y limpiar búferes
        self.cam1_buffer.clear()
        self.cam2_buffer.clear()
        return m1, m2

    # ── FUSIÓN ───────────────────────────────────────────────────────────────

    def _get_thread_bufs(self):
        """Devuelve buffers preallocados para el hilo actual (sin malloc por frame)."""
        loc = self._local
        if not hasattr(loc, 'out1'):
            loc.out1   = np.zeros((self.cam_h, self.cam_w,      3), dtype=np.uint8)
            loc.out2   = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
            loc.canvas = np.empty((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
            bw = self._blend_end - self._blend_start
            # uint16 para aritemética entera de blending — ~2x más rápido que float32
            loc.roi1u  = np.empty((self.canvas_h, bw, 3), dtype=np.uint16)
            loc.roi2u  = np.empty((self.canvas_h, bw, 3), dtype=np.uint16)
        return loc.out1, loc.out2, loc.canvas, loc.roi1u, loc.roi2u

    def _try_fuse(self):
        # Sección crítica mínima: solo lectura/limpieza de búferes
        with self.fusion_lock:
            msg1, msg2 = self._best_match()

        if msg1 is None:
            return

        # Obtener buffers del hilo actual (sin malloc, sin race condition)
        out1, out2, canvas, roi1u, roi2u = self._get_thread_bufs()

        try:
            raw1 = self._msg_to_bgr(msg1)
            raw2 = self._msg_to_bgr(msg2)

            # Remap con interpolacion configurable
            cv2.remap(raw1, self.map1_x, self.map1_y, self.interp, dst=out1)
            cv2.remap(raw2, self.map2_x, self.map2_y, self.interp, dst=out2)

            s = self._blend_start
            e = self._blend_end

            # Zonas puras (copia directa, sin aritmetica)
            canvas[:, :s] = out1[:, :s]
            canvas[:, e:] = out2[:, e:]

            # Blending en uint16: evita conversion float32 -> 2x mas rapido
            # Formula: (roi1 * alpha_w + roi2 * alpha_inv_w) >> 8
            np.copyto(roi1u, out1[:, s:e], casting='unsafe')  # uint8->uint16 sin copia extra
            np.copyto(roi2u, out2[:, s:e], casting='unsafe')
            roi1u *= self._alpha_w
            roi2u *= self._alpha_inv_w
            roi1u += roi2u
            canvas[:, s:e] = (roi1u >> 8).astype(np.uint8)

            # Publicar reutilizando el objeto Image (solo actualizar data y header)
            self._out_msg.header          = msg1.header
            self._out_msg.header.frame_id = 'panoramic_link'
            self._out_msg.data            = canvas.tobytes()
            self.fused_pub.publish(self._out_msg)

        except Exception as ex:
            self.get_logger().error(f"Error en fusión: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = PanoramicFusionNode()
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
