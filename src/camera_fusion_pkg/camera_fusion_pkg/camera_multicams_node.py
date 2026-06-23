#!/usr/bin/env python3
"""
Multi-Camera Fusion Node — Escalable a N cámaras, salida 640×640.

Arquitectura:
- Sin timer: el procesamiento se dispara cuando llega cualquier frame nuevo.
- Hilo de procesamiento dedicado con cola de tamaño 1.
- Remaps en paralelo (un worker por cámara): OpenCV libera el GIL.
- Buffer de publicación pre-reservado: sin malloc por frame.
- Slop check: descarta sets de frames con diferencia temporal excesiva.

Resolución de salida: 640×640 (canvas cuadrado, compatible con YOLO).

Calibración:
- K se escala automáticamente si la resolución del stream (cam_w × cam_h)
  difiere de la resolución de calibración del YAML.
  Escalado exacto: fx *= sw, fy *= sh, cx *= sw, cy *= sh.
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


class MulticamsFusionNode(Node):
    def __init__(self):
        super().__init__('multicams_fusion_node')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('camera_topics',
            ['cam_1/image_raw', 'cam_2/image_raw', 'cam_3/image_raw'])
        self.declare_parameter('camera_calibrations',
            ['cam_1_calibration.yaml', 'cam_2_calibration.yaml', 'cam_3_calibration.yaml'])
        self.declare_parameter('camera_homographies',
            ['cam1_to_cam2_homography.yaml', 'identity', 'cam3_to_cam2_homography.yaml'])
        # overlaps: [s0, e0, s1, e1, ...] — un par (start, end) por costura
        # Valores por defecto proporcionales al canvas 640px
        self.declare_parameter('overlaps', [250, 320, 320, 390])

        self.declare_parameter('canvas_w',  640)
        self.declare_parameter('canvas_h',  640)
        self.declare_parameter('cam_w',     640)
        self.declare_parameter('cam_h',     640)
        self.declare_parameter('interp',    cv2.INTER_NEAREST)  # ~2× más rápido que INTER_LINEAR
        self.declare_parameter('slop_ms',   150)

        self.camera_topics  = self.get_parameter('camera_topics').value
        self.camera_calibs  = self.get_parameter('camera_calibrations').value
        self.camera_homos   = self.get_parameter('camera_homographies').value
        self.overlaps_raw   = self.get_parameter('overlaps').value

        self.canvas_w       = self.get_parameter('canvas_w').value
        self.canvas_h       = self.get_parameter('canvas_h').value
        self.cam_w          = self.get_parameter('cam_w').value
        self.cam_h          = self.get_parameter('cam_h').value
        self.interp         = self.get_parameter('interp').value
        self.slop_seconds   = self.get_parameter('slop_ms').value / 1000.0

        self.num_cams = len(self.camera_topics)

        if len(self.camera_calibs) != self.num_cams or len(self.camera_homos) != self.num_cams:
            self.get_logger().error(
                "Las listas de configuración (topics, calibs, homos) deben tener la misma longitud.")
            return

        if len(self.overlaps_raw) != 2 * (self.num_cams - 1):
            self.get_logger().error(
                f"Se esperan {2 * (self.num_cams - 1)} valores en overlaps (start, end por costura).")
            return

        # ── Parse overlaps ────────────────────────────────────────────────────
        self.overlaps = []
        for i in range(self.num_cams - 1):
            s = max(0, min(self.overlaps_raw[2 * i],     self.canvas_w - 1))
            e = max(s + 1, min(self.overlaps_raw[2 * i + 1], self.canvas_w))
            self.overlaps.append((s, e))

        # ── Mapas precomputados ──────────────────────────────────────────────
        self.maps_x = []
        self.maps_y = []
        cfg_dir = os.path.expanduser('~/camera_fusion_ws/src/camera_fusion_pkg/config')
        sc = (self.cam_w, self.cam_h)
        sv = (self.canvas_w, self.canvas_h)

        for i in range(self.num_cams):
            calib_path = os.path.join(cfg_dir, self.camera_calibs[i])
            homo_file  = self.camera_homos[i]

            K, D = (self._load_intrinsics(calib_path, self.cam_w, self.cam_h)
                    if os.path.exists(calib_path)
                    else (np.eye(3, dtype=np.float64), np.zeros(5, dtype=np.float64)))

            mx, my = cv2.initUndistortRectifyMap(K, D, None, K, sc, cv2.CV_32FC1)

            if homo_file.lower() == 'identity':
                H = np.eye(3, dtype=np.float64)
            else:
                homo_path = os.path.join(cfg_dir, homo_file)
                H = (self._load_homography(homo_path)
                     if os.path.exists(homo_path)
                     else np.eye(3, dtype=np.float64))

            cx = cv2.warpPerspective(mx, H, sv, flags=cv2.INTER_LINEAR)
            cy = cv2.warpPerspective(my, H, sv, flags=cv2.INTER_LINEAR)
            mx_cv16, my_cv16 = cv2.convertMaps(cx, cy, cv2.CV_16SC2)
            self.maps_x.append(mx_cv16)
            self.maps_y.append(my_cv16)

        # ── Buffers de blending ───────────────────────────────────────────────
        self._alpha_w     = []
        self._alpha_inv_w = []
        self._roi1        = []
        self._roi2        = []
        for i in range(self.num_cams - 1):
            s, e = self.overlaps[i]
            bw = e - s
            aw = np.linspace(256, 0, bw, dtype=np.float32).round().astype(np.uint16)
            aw_3d = aw[np.newaxis, :, np.newaxis]
            self._alpha_w.append(aw_3d)
            self._alpha_inv_w.append((256 - aw_3d).astype(np.uint16))
            self._roi1.append(np.empty((self.canvas_h, bw, 3), dtype=np.uint16))
            self._roi2.append(np.empty((self.canvas_h, bw, 3), dtype=np.uint16))

        # ── Buffers del hilo ─────────────────────────────────────────────────
        # Un buffer de remap por cámara (canvas completo, se usa como destino de remap)
        self._outs = [
            np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
            for _ in range(self.num_cams)
        ]

        # Buffer de publicación pre-reservado: _canvas apunta al backing de _pub_buf.
        # Escribir en _canvas escribe en _pub_buf sin copia adicional.
        self._pub_buf = bytearray(self.canvas_h * self.canvas_w * 3)
        self._canvas  = np.frombuffer(self._pub_buf, dtype=np.uint8).reshape(
            self.canvas_h, self.canvas_w, 3)

        # Mensaje de salida reutilizable.
        self._out_msg              = Image()
        self._out_msg.height       = self.canvas_h
        self._out_msg.width        = self.canvas_w
        self._out_msg.encoding     = 'bgr8'
        self._out_msg.is_bigendian = False
        self._out_msg.step         = self.canvas_w * 3
        self._out_msg.data         = self._pub_buf  # sin malloc por frame

        # ── Estado compartido ────────────────────────────────────────────────
        self._latest_msgs  = [None] * self.num_cams
        self._locks        = [threading.Lock() for _ in range(self.num_cams)]
        self._last_stamps  = [-1.0] * self.num_cams
        self._trigger_q    = queue.Queue(maxsize=1)
        self._remap_pool   = ThreadPoolExecutor(max_workers=self.num_cams)

        self._running     = True
        self._proc_thread = threading.Thread(
            target=self._proc_loop, daemon=True, name='multicams-proc')
        self._proc_thread.start()

        # ── ROS ──────────────────────────────────────────────────────────────
        cg = ReentrantCallbackGroup()
        self.fused_pub = self.create_publisher(Image, 'fused_panorama', _QOS_PUB)

        self.subs = []
        for i, topic in enumerate(self.camera_topics):
            cb  = self._make_cb(i)
            sub = self.create_subscription(Image, topic, cb, _QOS_SUB, callback_group=cg)
            self.subs.append(sub)

        self.get_logger().info(
            f"✅ MulticamsFusion ({self.num_cams} cámaras, INTER_NEAREST, "
            f"canvas {self.canvas_w}×{self.canvas_h}) listo.")

    # ── Carga ────────────────────────────────────────────────────────────────

    def _load_intrinsics(self, path: str, target_w: int, target_h: int):
        """Carga K y D desde YAML y escala K si la resolución difiere de la calibración.

        El escalado es matemáticamente exacto para un resize puro:
            fx_new = fx * (target_w / calib_w)
            fy_new = fy * (target_h / calib_h)
            cx_new = cx * (target_w / calib_w)
            cy_new = cy * (target_h / calib_h)
        """
        with open(path) as f:
            d = yaml.safe_load(f)

        K = np.array(d['camera_matrix']['data'],
                     dtype=np.float64).reshape(3, 3)
        D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)

        calib_w = int(d.get('image_width',  target_w))
        calib_h = int(d.get('image_height', target_h))

        if calib_w != target_w or calib_h != target_h:
            sx = target_w / calib_w
            sy = target_h / calib_h
            K = K.copy()
            K[0, 0] *= sx   # fx
            K[1, 1] *= sy   # fy
            K[0, 2] *= sx   # cx
            K[1, 2] *= sy   # cy
            self.get_logger().info(
                f"  Calibración escalada {calib_w}×{calib_h} → {target_w}×{target_h} "
                f"(sx={sx:.4f}, sy={sy:.4f})")

        return K, D

    def _load_homography(self, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        return np.array(d['homography_matrix']['data'],
                        dtype=np.float64).reshape(3, 3)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _make_cb(self, idx):
        def cb(msg):
            with self._locks[idx]:
                self._latest_msgs[idx] = msg
            self._trigger()
        return cb

    def _trigger(self):
        try:
            self._trigger_q.put_nowait(True)
        except queue.Full:
            pass

    # ── Hilo de procesamiento ─────────────────────────────────────────────────

    def _proc_loop(self):
        while self._running:
            try:
                self._trigger_q.get(timeout=0.5)
            except queue.Empty:
                continue

            msgs = []
            for i in range(self.num_cams):
                with self._locks[i]:
                    msgs.append(self._latest_msgs[i])

            if any(m is None for m in msgs):
                continue

            stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs]

            # Slop check: descartar si los frames no están suficientemente sincronizados
            if (max(stamps) - min(stamps)) > self.slop_seconds:
                continue

            # Skip si el conjunto de frames no ha cambiado
            if all(stamps[i] == self._last_stamps[i] for i in range(self.num_cams)):
                continue

            self._fuse(msgs)
            self._last_stamps = stamps

    # ── Fusión ───────────────────────────────────────────────────────────────

    def _fuse(self, msgs):
        try:
            # Remaps en paralelo (un worker por cámara, OpenCV libera GIL)
            futures = []
            for i in range(self.num_cams):
                raw = np.frombuffer(msgs[i].data, dtype=np.uint8).reshape(
                    msgs[i].height, msgs[i].width, 3)
                f = self._remap_pool.submit(
                    cv2.remap, raw, self.maps_x[i], self.maps_y[i],
                    self.interp, self._outs[i])
                futures.append(f)

            for f in futures:
                f.result()

            # ── Composición del canvas ────────────────────────────────────────
            # Zona pura izquierda (cámara 0)
            s0 = self.overlaps[0][0] if self.overlaps else self.canvas_w
            self._canvas[:, :s0] = self._outs[0][:, :s0]

            for i in range(self.num_cams - 1):
                s, e = self.overlaps[i]

                # Blend en la zona de costura
                np.copyto(self._roi1[i], self._outs[i][:, s:e],   casting='unsafe')
                np.copyto(self._roi2[i], self._outs[i+1][:, s:e], casting='unsafe')
                self._roi1[i] *= self._alpha_w[i]
                self._roi2[i] *= self._alpha_inv_w[i]
                self._roi1[i] += self._roi2[i]
                self._canvas[:, s:e] = (self._roi1[i] >> 8).astype(np.uint8)

                # Zona pura entre esta costura y la siguiente
                next_start = e
                next_end   = (self.overlaps[i + 1][0]
                              if i + 1 < len(self.overlaps)
                              else self.canvas_w)
                self._canvas[:, next_start:next_end] = self._outs[i+1][:, next_start:next_end]

            # Publicar reutilizando el objeto Image y el bytearray pre-reservado.
            self._out_msg.header          = msgs[0].header
            self._out_msg.header.frame_id = 'panoramic_link'
            self.fused_pub.publish(self._out_msg)

        except Exception as ex:
            self.get_logger().error(f"Error en fusión: {ex}")

    # ── Limpieza ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._trigger()
        self._proc_thread.join(timeout=2.0)
        self._remap_pool.shutdown(wait=False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MulticamsFusionNode()
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
