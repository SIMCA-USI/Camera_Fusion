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

## 🚀 Despliegue y Ejecución

### 1. Ejecución Rápida (Launch)
El sistema incluye varios archivos `launch` para facilitar la ejecución. Si no has calibrado las cámaras aún, el sistema usará matrices de identidad y funcionará igualmente sin distorsiones corregidas.

**Opción A: Fusión Multi-Cámara (N cámaras, recomendado)**:
Lanza el sistema escalable y dinámico.
```bash
ros2 launch camera_fusion_pkg multicams.launch.py
```

**Opción B: Fusión Dual (Solo 2 cámaras)**:
Puedes elegir el modo síncrono o asíncrono (event-driven, más rápido).
```bash
ros2 launch camera_fusion_pkg fusion.launch.py mode:=async
```

### 🛠️ 2. Requisitos y Compilación
Si acabas de descargar el código, compílalo primero:
- **Dependencias**: ROS 2 Jazzy / Humble, `python3-opencv`, `python3-numpy`, `python3-yaml`

```bash
cd ~/camera_fusion_ws
colcon build --packages-select camera_fusion_pkg
source install/setup.bash
```

### 🎯 3. Calibración de Cámaras (Opcional pero recomendado)
> **Nota:** Si no existen los archivos YAML en `config/`, el sistema omitirá este paso. *Importante: al usar los scripts, asegúrate de escribir correctamente las `FilasxColumnas` de tu tablero y el `"Tamaño_lado_en_metros"`.*

**A. Intrínseca (una vez por cámara):**
```bash
./scripts/calibrate_camera.sh cam_1 /cam_1/image_raw FilasxColumnas "Tamaño_lado_en_metros"
./scripts/calibrate_camera.sh cam_2 /cam_2/image_raw FilasxColumnas "Tamaño_lado_en_metros"
```
*(El script guarda automáticamente los `.yaml` en `config/`)*

**B. Estéreo (Homografía de alineación):**
Abre las cámaras con el script launch y luego en otra terminal ejecuta:
```bash
./scripts/calibrar_estereo.sh FilasxColumnas "Tamaño_lado_en_metros" /cam_1/image_raw /cam_2/image_raw
```
*(Haz clic en SAVE para generar `config/board_homography.yaml`)*

### 🔍 4. Verificación y Tópicos

| Topic | Resolución | Descripción |
|---|---|---|
| `/autobus/camaras/cam_1/image_raw` | 640×640 | Stream cámara 1 (sin fusión) |
| `/autobus/camaras/cam_2/image_raw` | 640×640 | Stream cámara 2 (sin fusión) |
| `/ravo/followme/video_frames` | 640×640 | Vista panorámica fusionada |

**Comandos útiles de comprobación:**
```bash
# FPS del resultado fusionado (objetivo: ≥15 Hz)
ros2 topic hz /ravo/followme/video_frames

# Comprobar resolución de salida (Esperado → height: 640, width: 640)
ros2 topic echo /ravo/followme/video_frames --once | grep -E 'height|width'
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
