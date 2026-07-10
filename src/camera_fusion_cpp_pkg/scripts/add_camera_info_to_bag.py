#!/usr/bin/env python3
"""
add_camera_info_to_bag.py

Lee un bag MCAP existente y crea uno nuevo añadiendo el topic
  /carrito/followme/camera_info
sincronizado con cada mensaje de /carrito/followme/video_frames.

La K de la imagen panorámica se calcula a partir de cam_1_calibration.yaml:
  - fx, fy  →  sin cambio (el crop no escala)
  - cx_new  =  cx − crop_x
  - cy_new  =  cy − crop_y
  - D       =  zeros (imagen ya undistorsionada)
  - width/height se leen automáticamente del primer frame fusionado.

Uso:
  source ~/fusiones_ws/install/setup.bash
  python3 add_camera_info_to_bag.py \\
      --input  bags/calibracion_lidar_camaras/ \\
      --output bags/calibracion_con_camera_info/ \\
      --crop-x 30 --crop-y 10
"""

import argparse
import os
import sys
import yaml

import rclpy.serialization
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image
from builtin_interfaces.msg import Time


# ──────────────────────────────────────────────────────────────────────────────
# Utilidades
# ──────────────────────────────────────────────────────────────────────────────

def load_cam1_intrinsics(yaml_path: str):
    """Devuelve (fx, fy, cx, cy) desde cam_1_calibration.yaml."""
    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)
    data = cfg["camera_matrix"]["data"]
    # K aplanada por filas: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    return data[0], data[4], data[2], data[5]   # fx, fy, cx, cy


def find_config_dir():
    """Localiza el directorio config del paquete camera_fusion_pkg."""
    candidates = [
        os.path.expanduser("~/fusiones_ws/src/camera_fusion_pkg/config"),
        os.path.expanduser("~/fusiones_ws/install/camera_fusion_pkg"
                           "/share/camera_fusion_pkg/config"),
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    return None


def build_camera_info_msg(fx, fy, cx_pan, cy_pan, width, height,
                           frame_id="panoramic_link") -> CameraInfo:
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width  = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [
        fx,   0.0, cx_pan,
        0.0,  fy,  cy_pan,
        0.0,  0.0,  1.0,
    ]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [
        fx,   0.0, cx_pan, 0.0,
        0.0,  fy,  cy_pan, 0.0,
        0.0,  0.0,  1.0,   0.0,
    ]
    return msg


def ns_to_time(nanoseconds: int) -> Time:
    t = Time()
    t.sec     = nanoseconds // 1_000_000_000
    t.nanosec = nanoseconds %  1_000_000_000
    return t


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Añade /carrito/followme/camera_info a un bag MCAP existente."
    )
    parser.add_argument("--input",   required=True,
                        help="Directorio del bag de entrada")
    parser.add_argument("--output",  required=True,
                        help="Directorio del bag de salida (no debe existir)")
    parser.add_argument("--crop-x",  type=int, default=30,
                        help="Píxeles recortados por la izquierda (default: 30)")
    parser.add_argument("--crop-y",  type=int, default=10,
                        help="Píxeles recortados por arriba (default: 10)")
    parser.add_argument("--config-dir", default=None,
                        help="Ruta al directorio config con cam_1_calibration.yaml")
    parser.add_argument("--frame-id", default="panoramic_link",
                        help="frame_id de la CameraInfo (default: panoramic_link)")
    parser.add_argument("--fused-topic", default="/carrito/followme/video_frames",
                        help="Topic de la imagen fusionada en el bag")
    parser.add_argument("--info-topic",  default="/carrito/followme/camera_info",
                        help="Topic de salida para la CameraInfo")
    args = parser.parse_args()

    # ── Validaciones ──────────────────────────────────────────────────────────
    if not os.path.isdir(args.input):
        print(f"[ERROR] No existe el bag de entrada: {args.input}")
        sys.exit(1)
    if os.path.exists(args.output):
        print(f"[ERROR] El directorio de salida ya existe: {args.output}")
        sys.exit(1)

    config_dir = args.config_dir or find_config_dir()
    if config_dir is None:
        print("[ERROR] No se encontró el directorio config. "
              "Usa --config-dir para especificarlo.")
        sys.exit(1)

    cam1_yaml = os.path.join(config_dir, "cam_1_calibration.yaml")
    if not os.path.isfile(cam1_yaml):
        print(f"[ERROR] No se encontró {cam1_yaml}")
        sys.exit(1)

    # ── Cargar intrínsecos ────────────────────────────────────────────────────
    fx, fy, cx, cy = load_cam1_intrinsics(cam1_yaml)
    cx_pan = cx - args.crop_x
    cy_pan = cy - args.crop_y

    print(f"[INFO] Calibración cam_1: fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}")
    print(f"[INFO] Crop: x={args.crop_x} y={args.crop_y}")
    print(f"[INFO] K panorámica:  fx={fx:.3f} fy={fy:.3f} cx={cx_pan:.3f} cy={cy_pan:.3f}")

    # ── Abrir bag de entrada ──────────────────────────────────────────────────
    storage_in = rosbag2_py.StorageOptions(uri=args.input, storage_id="mcap")
    converter_in = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_in, converter_in)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # ── Detectar dimensiones del frame fusionado ──────────────────────────────
    # Necesitamos leer el primer mensaje de video_frames para saber width/height.
    # Lo haremos en el primer pase y luego reabriremos.
    if args.fused_topic not in type_map:
        print(f"[ERROR] Topic '{args.fused_topic}' no encontrado en el bag.")
        print(f"        Topics disponibles: {list(type_map.keys())}")
        sys.exit(1)

    print(f"[INFO] Buscando dimensiones en '{args.fused_topic}'...")
    pan_w, pan_h = None, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == args.fused_topic:
            img_msg = rclpy.serialization.deserialize_message(data, Image)
            pan_w = img_msg.width
            pan_h = img_msg.height
            print(f"[INFO] Imagen panorámica detectada: {pan_w}x{pan_h}")
            break

    if pan_w is None:
        print(f"[ERROR] No se encontraron mensajes en '{args.fused_topic}'.")
        sys.exit(1)

    # ── Construir mensaje CameraInfo (plantilla estática) ─────────────────────
    cam_info_template = build_camera_info_msg(
        fx, fy, cx_pan, cy_pan, pan_w, pan_h, args.frame_id
    )
    print(f"[INFO] CameraInfo: {pan_w}x{pan_h} | frame_id='{args.frame_id}'")

    # ── Reabrir bag de entrada desde el principio ─────────────────────────────
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_in, converter_in)
    topic_types = reader.get_all_topics_and_types()

    # ── Abrir bag de salida ───────────────────────────────────────────────────
    storage_out = rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap")
    converter_out = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_out, converter_out)

    # Registrar todos los topics originales
    for t in topic_types:
        writer.create_topic(t)

    # Registrar el nuevo topic de CameraInfo
    # En ROS 2 Jazzy, TopicMetadata requiere id como primer argumento posicional
    info_topic_meta = rosbag2_py.TopicMetadata(
        id=len(topic_types),
        name=args.info_topic,
        type="sensor_msgs/msg/CameraInfo",
        serialization_format="cdr"
    )
    writer.create_topic(info_topic_meta)

    # ── Copiar todos los mensajes + insertar CameraInfo ───────────────────────
    total = 0
    inserted = 0

    print("[INFO] Procesando bag...")
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        writer.write(topic, data, timestamp_ns)
        total += 1

        # Por cada frame fusionado, insertar CameraInfo con el mismo timestamp
        if topic == args.fused_topic:
            cam_info_msg = CameraInfo()
            cam_info_msg.header.frame_id = cam_info_template.header.frame_id
            cam_info_msg.header.stamp    = ns_to_time(timestamp_ns)
            cam_info_msg.width           = cam_info_template.width
            cam_info_msg.height          = cam_info_template.height
            cam_info_msg.distortion_model = cam_info_template.distortion_model
            cam_info_msg.d = cam_info_template.d
            cam_info_msg.k = cam_info_template.k
            cam_info_msg.r = cam_info_template.r
            cam_info_msg.p = cam_info_template.p

            serialized = rclpy.serialization.serialize_message(cam_info_msg)
            writer.write(args.info_topic, serialized, timestamp_ns)
            inserted += 1

        if total % 1000 == 0:
            print(f"  → {total} mensajes copiados, {inserted} CameraInfo insertados...")

    print(f"\n[OK] Bag de salida: {args.output}")
    print(f"     Mensajes originales copiados : {total}")
    print(f"     CameraInfo insertados        : {inserted}")
    print(f"\nVerifica con:")
    print(f"  ros2 bag info {args.output}")


if __name__ == "__main__":
    main()
