# Multi-Camera Panoramic Fusion (ROS 2)

Sistema en ROS 2 para la adquisición, calibración y fusión en tiempo real de múltiples cámaras (webcams USB), generando una vista panorámica unificada a **640×640 px** compatible con inferencia YOLO. El procesamiento es totalmente **asíncrono y event-driven**, garantizando ≥15 FPS.

---

## Fundamentos Teóricos

La fusión de cámaras requiere un proceso de calibración geométrica en dos fases para corregir aberraciones de lente y alinear los sistemas de coordenadas.

### 1. Calibración Intrínseca (Corrección de Lente)
Las lentes de bajo coste introducen distorsiones (efecto barril) que curvan las líneas rectas. Mediante un tablero de ajedrez, el algoritmo estima la **Matriz de la Cámara ($K$)** y los **Coeficientes de Distorsión ($D$)**. El nodo de fusión los usa para rectificar matemáticamente cada imagen antes de unirlas.

![Calibración Intrínseca](docs/images/calibracion_intrinseca.png)

### 2. Calibración Estéreo → Homografía
Con ambas cámaras viendo el mismo tablero simultáneamente, se obtiene la rotación relativa ($R_{rel}$) entre ellas. A partir de ahí se calcula la **Matriz de Homografía**:

$$H = K_1 \cdot R_{rel} \cdot K_2^{-1}$$

Esta proyecta la imagen de cam_2 sobre el plano de cam_1 (`cv2.warpPerspective`). En la zona de solapamiento se aplica **Linear Blending** — un degradado suave calculado únicamente sobre la ROI de costura.

![Fusión y Blending](docs/images/fusion_panoramica.png)

---

## Arquitectura del Software

```
/dev/video*  ──►  camera_reader_node  ──►  cam_1/image_raw  ─┐
                                      ──►  cam_2/image_raw  ─┤─►  camera_fusion_async_node  ──►  /ravo/followme/video_frames
                                      ──►  cam_N/image_raw  ─┘         (640×640, ≥15 FPS)
```

### Nodos ROS 2

| Nodo | Fichero | Descripción |
|---|---|---|
| `camera_reader_node` | `camera_reader_node.py` | Auto-descubre cámaras USB vía V4L2. Publica **siempre a 640×640** aplicando `cv2.resize`, independientemente del modo nativo del hardware. Un hilo de captura por cámara. |
| `camera_fusion_async_node` | `camera_fusion_async_node.py` | Fusión de **2 cámaras** event-driven. Remap paralelo en 2 cores (OpenCV libera el GIL). Buffer de publicación pre-reservado (sin `malloc` por frame). |
| `camera_multicams_node` | `camera_multicams_node.py` | Fusión escalable a **N cámaras**. Misma arquitectura asíncrona, remap paralelo con un worker por cámara. |

### Diseño asíncrono (event-driven)

```
cam_1 llega → señal ─┐
cam_2 llega → señal ─┘→ [hilo fusión]: remap‖ → blend → publica
                         └── executor ROS siempre libre ──────┘
```

- **Sin timer**: el procesamiento arranca en cuanto llega un frame nuevo → latencia ~0 ms.
- **Cola de tamaño 1**: si el hilo está procesando, la señal siguiente se descarta y el hilo leerá el frame más reciente al despertar → nunca acumula backlog.
- **Remap paralelo**: `INTER_NEAREST` en ThreadPoolExecutor de 2 workers → OpenCV libera el GIL, dos cores en paralelo.
- **Buffer pre-reservado**: `_canvas` respaldado por un `bytearray` fijo. El objeto `Image` apunta directamente a él → `publish()` sin malloc extra.

### Calibración y escalado de K

Los mapas de undistort se calculan con `K` escalado automáticamente si la resolución del stream (`cam_h=640`) difiere del YAML de calibración (`image_height=480`):

```
fy_new = fy × (640 / 480)  →  662.11 → 882.81
cy_new = cy × (640 / 480)  →  240.17 → 320.22
```

Este escalado es **matemáticamente exacto** para un resize puro. No se requiere recalibrar.

### Scripts de Soporte (`scripts/`)

| Script | Descripción |
|---|---|
| `listar_camaras.sh` | Escanea `/dev/video*` con `v4l2-ctl` y OpenCV para identificar cámaras válidas. |
| `calibrate_camera.sh` | Wrapper del calibrador ROS 2 (`camera_calibration`) para calibración monocular. |
| `calibrar_estereo.sh` | Wrapper para calibración estéreo par a par (modo `--approximate`). |
| `extraer_estereo.py` | Procesa los `.tar.gz` de ROS 2, extrae $H$ de `ost.txt` y exporta `board_homography.yaml`. |

---

## Despliegue

### Requisitos

- ROS 2 Jazzy / Humble
- `python3-opencv`, `python3-numpy`, `python3-yaml`

### 1. Compilación

```bash
cd ~/camera_fusion_ws
colcon build --packages-select camera_fusion_pkg
source install/setup.bash
```

### 2. Calibración

> **Nota:** si no existen los ficheros `config/cam_X_calibration.yaml` y `config/board_homography.yaml`, el nodo de fusión usará matrices identidad (sin corrección de lente).

*Nota: Es importante cambiar el tamaño del tablero y la longitud de los lados de cada cuadrado al calibrar.*

**A. Intrínseca (una vez por cámara):**
```bash
./scripts/calibrate_camera.sh cam_1 /cam_1/image_raw FilasxColumnas "Tamaño_lado_en_metros"
./scripts/calibrate_camera.sh cam_2 /cam_2/image_raw FilasxColumnas "Tamaño_lado_en_metros"
```
El script extrae automáticamente los `.yaml` a `config/`.

**B. Estéreo (Homografía de alineación):**
*Nota: Es importante cambiar el tamaño del tablero y la longitud de los lados de cada cuadrado*
Lanza ambas cámaras (puedes usar el launch file) y ejecuta:
```bash
./scripts/calibrar_estereo.sh FilasxColumnas "Tamaño_lado_en_metros" /cam_1/image_raw /cam_2/image_raw
```
Al pulsar SAVE, el script genera `config/board_homography.yaml` automáticamente.

### 3. Ejecución

```bash
ros2 launch camera_fusion_pkg multicams.launch.py
```

| Topic | Resolución | Descripción |
|---|---|---|
| `/autobus/camaras/cam_1/image_raw` | 640×640 | Stream cámara 1 (sin fusión) |
| `/autobus/camaras/cam_2/image_raw` | 640×640 | Stream cámara 2 (sin fusión) |
| `/ravo/followme/video_frames` | 640×640 | Vista panorámica fusionada |

### 4. Verificación

```bash
# FPS del resultado fusionado (objetivo: ≥15 Hz)
ros2 topic hz /ravo/followme/video_frames

# Comprobar resolución de salida
ros2 topic echo /ravo/followme/video_frames --once | grep -E 'height|width'
# Esperado → height: 640, width: 640
```

---

## Parámetros configurables

### `camera_reader_node`

| Parámetro | Default | Descripción |
|---|---|---|
| `fps` | `30` | FPS solicitados al driver V4L2 |
| `width` / `height` | `640` / `640` | Resolución solicitada a V4L2 (puede ignorarse por el hardware) |
| `target_size` | `640` | Resolución de salida **garantizada** — resize siempre aplicado |

### `camera_fusion_async_node` / `camera_multicams_node`

| Parámetro | Default | Descripción |
|---|---|---|
| `canvas_w` / `canvas_h` | `640` / `640` | Resolución del canvas de salida |
| `cam_w` / `cam_h` | `640` / `640` | Resolución esperada de los frames de entrada |
| `overlap_start` / `overlap_end` | `280` / `360` | Columnas de inicio/fin de la zona de blending |
| `interp` | `INTER_NEAREST` | Interpolación del remap (`INTER_NEAREST` ~2× más rápido que `INTER_LINEAR`) |
| `slop_ms` | `50` / `150` | Diferencia máxima de timestamp entre frames para considerarlos válidos |
