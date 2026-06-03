#!/usr/bin/env python3
"""
Camera Fusion Node — Modo ASÍNCRONO optimizado.

Mejoras vs versión anterior:
- Sin timer: el procesamiento se dispara en el instante que llega un frame nuevo
  → latencia ~0ms en lugar de hasta 33ms del timer.
- Hilo de procesamiento dedicado con cola de tamaño 1: el executor ROS nunca
  se bloquea durante el remap/blend.
- Dos remaps en paralelo: OpenCV libera el GIL → paralelismo real en dos hilos.
- Reutilización del objeto Image de salida: sin malloc por frame.
- Skip de frames duplicados: si llegó el mismo par que el ciclo anterior, no
  se reprocesa.
"""
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
import queue
from concurrent.futures import ThreadPoolExecutor

# Reducir threads de OpenCV a la mitad para que dos remaps paralelos
# no compitan por todos los cores (cada uno usa ~N/2 cores).
_N_CORES = os.cpu_count() or 4
cv2.setNumThreads(max(1, _N_CORES // 2))

_QOS_SUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=2
)
_QOS_PUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class AsyncFusionNode(Node):
    def __init__(self):
        super().__init__('panoramic_fusion_node')

        self.declare_parameter('overlap_start', 400)
        self.declare_parameter('overlap_end',   600)
        self.declare_parameter('canvas_w',     1100)
        self.declare_parameter('canvas_h',      480)
        self.declare_parameter('cam_w',         640)
        self.declare_parameter('cam_h',         480)
        self.declare_parameter('interp', cv2.INTER_LINEAR)

        self.overlap_start = self.get_parameter('overlap_start').value
        self.overlap_end   = self.get_parameter('overlap_end').value
        self.canvas_w      = self.get_parameter('canvas_w').value
        self.canvas_h      = self.get_parameter('canvas_h').value
        self.cam_w         = self.get_parameter('cam_w').value
        self.cam_h         = self.get_parameter('cam_h').value
        self.interp        = self.get_parameter('interp').value

        # ── Calibración ──────────────────────────────────────────────────────
        cfg = os.path.expanduser(
            '~/camera_fusion_ws/src/camera_fusion_pkg/config')
        p1 = os.path.join(cfg, 'cam_1_calibration.yaml')
        p2 = os.path.join(cfg, 'cam_2_calibration.yaml')
        ph = os.path.join(cfg, 'board_homography.yaml')

        if os.path.exists(p1) and os.path.exists(p2) and os.path.exists(ph):
            self.K1, self.D1 = self._load_intrinsics(p1)
            self.K2, self.D2 = self._load_intrinsics(p2)
            self.H           = self._load_homography(ph)
        else:
            self.get_logger().warn("Calibración no encontrada — usando identidad.")
            I = np.eye(3, dtype=np.float64)
            Z = np.zeros(5, dtype=np.float64)
            self.K1 = self.K2 = I
            self.D1 = self.D2 = Z
            self.H = I

        # ── Mapas precomputados ──────────────────────────────────────────────
        sc = (self.cam_w, self.cam_h)
        sv = (self.canvas_w, self.canvas_h)

        self.map1x, self.map1y = cv2.initUndistortRectifyMap(
            self.K1, self.D1, None, self.K1, sc, cv2.CV_16SC2)
        m2x, m2y = cv2.initUndistortRectifyMap(
            self.K2, self.D2, None, self.K2, sc, cv2.CV_32FC1)
        cx = cv2.warpPerspective(m2x, self.H, sv, flags=cv2.INTER_LINEAR)
        cy = cv2.warpPerspective(m2y, self.H, sv, flags=cv2.INTER_LINEAR)
        self.map2x, self.map2y = cv2.convertMaps(cx, cy, cv2.CV_16SC2)

        # ── Alpha precomputado en uint16 ─────────────────────────────────────
        s = max(0, min(self.overlap_start, self.cam_w - 1))
        e = max(s + 1, min(self.overlap_end, self.canvas_w))
        self._s, self._e = s, e
        bw = e - s
        aw = np.linspace(256, 0, bw, dtype=np.float32).round().astype(np.uint16)
        self._alpha_w     = aw[np.newaxis, :, np.newaxis]
        self._alpha_inv_w = (256 - self._alpha_w).astype(np.uint16)

        # ── Buffers del hilo de procesamiento (único hilo → sin race) ────────
        self._out1   = np.zeros((self.cam_h, self.cam_w,      3), dtype=np.uint8)
        self._out2   = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
        self._canvas = np.empty((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
        self._roi1   = np.empty((self.canvas_h, bw, 3), dtype=np.uint16)
        self._roi2   = np.empty((self.canvas_h, bw, 3), dtype=np.uint16)

        # Mensaje de salida reutilizable (seguro: solo lo escribe el hilo proc)
        self._out_msg              = Image()
        self._out_msg.height       = self.canvas_h
        self._out_msg.width        = self.canvas_w
        self._out_msg.encoding     = 'bgr8'
        self._out_msg.is_bigendian = False
        self._out_msg.step         = self.canvas_w * 3

        # ── Estado compartido callbacks ↔ hilo de procesamiento ──────────────
        self._latest1   = None
        self._latest2   = None
        self._lock1     = threading.Lock()
        self._lock2     = threading.Lock()

        # Cola de señales de tamaño 1: si ya hay una señal pendiente,
        # descartamos la nueva (el hilo usará el frame más reciente al despertar)
        self._trigger_q = queue.Queue(maxsize=1)

        # Pool de 2 workers para remap paralelo (OpenCV libera el GIL)
        self._remap_pool = ThreadPoolExecutor(max_workers=2)

        # Timestamps del último par procesado (para skip de duplicados)
        self._last_t1 = self._last_t2 = -1.0

        # ── Hilo de procesamiento dedicado ───────────────────────────────────
        self._running = True
        self._proc_thread = threading.Thread(
            target=self._proc_loop, daemon=True, name='fusion-proc')
        self._proc_thread.start()

        # ── ROS ──────────────────────────────────────────────────────────────
        cg = ReentrantCallbackGroup()
        self.fused_pub = self.create_publisher(Image, 'fused_panorama', _QOS_PUB)
        self.create_subscription(Image, 'cam_1/image_raw',
                                 self._cb1, _QOS_SUB, callback_group=cg)
        self.create_subscription(Image, 'cam_2/image_raw',
                                 self._cb2, _QOS_SUB, callback_group=cg)

        self.get_logger().info(
            f"✅ AsyncFusion (event-driven, remap paralelo, {_N_CORES} cores) listo.")

    # ── Carga ────────────────────────────────────────────────────────────────

    def _load_intrinsics(self, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        return (np.array(d['camera_matrix']['data'],
                         dtype=np.float64).reshape(3, 3),
                np.array(d['distortion_coefficients']['data'], dtype=np.float64))

    def _load_homography(self, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        return np.array(d['homography_matrix']['data'],
                        dtype=np.float64).reshape(3, 3)

    # ── Callbacks de cámara ──────────────────────────────────────────────────

    def _cb1(self, msg):
        with self._lock1:
            self._latest1 = msg
        self._trigger()

    def _cb2(self, msg):
        with self._lock2:
            self._latest2 = msg
        self._trigger()

    def _trigger(self):
        """Señaliza al hilo de procesamiento que hay un nuevo frame disponible.
        Si ya hay una señal en cola, la descartamos: el hilo siempre leerá
        el frame MÁS RECIENTE cuando despierte."""
        try:
            self._trigger_q.put_nowait(True)
        except queue.Full:
            pass  # señal ya pendiente, el hilo la atenderá

    # ── Hilo de procesamiento ─────────────────────────────────────────────────

    def _proc_loop(self):
        """Bucle del hilo dedicado de fusión. Espera señales del trigger,
        lee los frames más recientes y procesa."""
        while self._running:
            # Bloquear hasta recibir señal (o timeout para poder salir limpio)
            try:
                self._trigger_q.get(timeout=0.5)
            except queue.Empty:
                continue

            # Leer frames más recientes bajo sus respectivos locks
            with self._lock1:
                msg1 = self._latest1
            with self._lock2:
                msg2 = self._latest2

            if msg1 is None or msg2 is None:
                continue

            # Calcular timestamps del par actual
            t1 = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
            t2 = msg2.header.stamp.sec + msg2.header.stamp.nanosec * 1e-9

            # Skip si el par no ha cambiado desde el último procesamiento
            if t1 == self._last_t1 and t2 == self._last_t2:
                continue

            self._fuse(msg1, msg2)
            self._last_t1, self._last_t2 = t1, t2

            # Re-señalizar si llegaron nuevos frames DURANTE el procesamiento
            with self._lock1:
                new_t1 = (self._latest1.header.stamp.sec +
                          self._latest1.header.stamp.nanosec * 1e-9
                          if self._latest1 else -1.0)
            with self._lock2:
                new_t2 = (self._latest2.header.stamp.sec +
                          self._latest2.header.stamp.nanosec * 1e-9
                          if self._latest2 else -1.0)

            if new_t1 != self._last_t1 or new_t2 != self._last_t2:
                self._trigger()

    # ── Fusión ───────────────────────────────────────────────────────────────

    def _fuse(self, msg1, msg2):
        try:
            raw1 = np.frombuffer(msg1.data, dtype=np.uint8).reshape(
                msg1.height, msg1.width, 3)
            raw2 = np.frombuffer(msg2.data, dtype=np.uint8).reshape(
                msg2.height, msg2.width, 3)

            # Remap en paralelo: OpenCV libera el GIL → dos cores en paralelo
            f1 = self._remap_pool.submit(
                cv2.remap, raw1, self.map1x, self.map1y,
                self.interp, self._out1)
            f2 = self._remap_pool.submit(
                cv2.remap, raw2, self.map2x, self.map2y,
                self.interp, self._out2)
            f1.result(); f2.result()  # esperar ambos

            s, e = self._s, self._e

            self._canvas[:, :s] = self._out1[:, :s]
            self._canvas[:, e:] = self._out2[:, e:]

            np.copyto(self._roi1, self._out1[:, s:e], casting='unsafe')
            np.copyto(self._roi2, self._out2[:, s:e], casting='unsafe')
            self._roi1 *= self._alpha_w
            self._roi2 *= self._alpha_inv_w
            self._roi1 += self._roi2
            self._canvas[:, s:e] = (self._roi1 >> 8).astype(np.uint8)

            # Reutilizar el objeto Image (hilo único → sin race)
            self._out_msg.header          = msg1.header
            self._out_msg.header.frame_id = 'panoramic_link'
            self._out_msg.data            = self._canvas.tobytes()
            self.fused_pub.publish(self._out_msg)

        except Exception as ex:
            self.get_logger().error(f"Error en fusión: {ex}")

    # ── Limpieza ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._trigger()           # desbloquear el hilo si está en get()
        self._proc_thread.join(timeout=2.0)
        self._remap_pool.shutdown(wait=False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AsyncFusionNode()
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
