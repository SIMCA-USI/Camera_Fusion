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

## Despliegue y Ejecución

Sigue estos pasos en orden. Si el repositorio es nuevo para ti, empieza por el paso 1 aunque ya tengas ROS 2 instalado.

---

### Paso 1: Requisitos previos

**Sistema operativo y ROS 2**

El paquete es compatible con ROS 2 Humble y ROS 2 Jazzy. Asegurate de tener el entorno configurado:

```bash
# Sustituye 'humble' por 'jazzy' si corresponde
source /opt/ros/humble/setup.bash
```

**Dependencias de Python**

```bash
sudo apt install python3-opencv python3-numpy python3-yaml
```

**Estructura del espacio de trabajo**

El paquete debe estar dentro de un espacio de trabajo de colcon:

```
camera_fusion_ws/
└── src/
    └── camera_fusion_pkg/   ← contenido de este repositorio
```

Si acabas de clonar el repositorio, crea el espacio de trabajo y coloca el paquete dentro:

```bash
mkdir -p ~/camera_fusion_ws/src
cd ~/camera_fusion_ws/src
git clone <url-del-repositorio> camera_fusion_pkg
```

---

### Paso 2: Compilacion

```bash
cd ~/camera_fusion_ws
colcon build --packages-select camera_fusion_pkg
source install/setup.bash
```

> Es necesario ejecutar `source install/setup.bash` cada vez que abras una terminal nueva, o añadirlo al fichero `~/.bashrc`.

---

### Paso 3: Identificar las camaras conectadas

Antes de calibrar o lanzar el sistema, identifica los indices de dispositivo de cada camara USB:

```bash
./src/camera_fusion_pkg/scripts/listar_camaras.sh
```

El script escanea `/dev/video*` y muestra que dispositivos son camaras validas. Anota el indice de cada camara (por ejemplo, `/dev/video0`, `/dev/video2`).

---

### Paso 4: Calibracion de camaras

La calibracion corrige la distorsion optica de cada lente y calcula la alineacion geometrica entre camaras. Sin calibracion el sistema funciona, pero las imagenes presentaran distorsion y el cosido entre camaras no sera preciso.

Los archivos resultantes se guardan en `src/camera_fusion_pkg/config/`.

#### 4.1 Calibracion intrinseca (una vez por camara)

Necesitas un tablero de ajedrez impreso. El script lanza el calibrador de ROS 2 y guarda el resultado automaticamente.

Parametros del comando: `<nombre_camara> <topico_imagen> <FilasxColumnas> <tamano_cuadrado_metros>`

```bash
# Calibrar camara 1
./src/camera_fusion_pkg/scripts/calibrate_camera.sh \
    cam_1 /cam_1/image_raw 8x6 0.025

# Calibrar camara 2
./src/camera_fusion_pkg/scripts/calibrate_camera.sh \
    cam_2 /cam_2/image_raw 8x6 0.025
```

El calibrador muestra una ventana en tiempo real. Mueve el tablero lentamente por el campo de vision hasta que la barra de progreso se complete. Entonces hace clic en **CALIBRATE** y despues en **SAVE**. Los archivos `.yaml` se guardaran en `config/`.

#### 4.2 Calibracion estereo (homografia de alineacion)

Este paso calcula la rotacion relativa entre las dos camaras y genera la matriz de homografia que el nodo de fusion utiliza para alinear las imagenes.

Primero arranca las camaras en una terminal:

```bash
ros2 launch camera_fusion_pkg fusionasincrona.launch.py
```

Luego, en otra terminal, ejecuta el calibrador estereo:

```bash
./src/camera_fusion_pkg/scripts/calibrar_estereo.sh \
    8x6 0.025 /cam_1/image_raw /cam_2/image_raw
```

Mueve el tablero de forma que sea visible simultaneamente por ambas camaras. Cuando el calibrador tenga suficientes muestras, haz clic en **SAVE**.

Por ultimo, extrae la homografia del paquete guardado:

```bash
python3 ./src/camera_fusion_pkg/scripts/extraer_estereo.py
```

Esto genera `config/board_homography.yaml`, que el nodo de fusion cargara automaticamente al arrancar.

---

### Paso 5: Lanzar el sistema

Existen tres archivos launch segun el caso de uso:

| Launch file | Nodo de fusion | Cuando usarlo |
|---|---|---|
| `multicams.launch.py` | `camera_multicams_node` | **Recomendado.** Soporta N camaras de forma escalable. |
| `fusionasincrona.launch.py` | `camera_fusion_async_node` | Solo 2 camaras, procesamiento event-driven sin timer. |
| `fusionsincrona.launch.py` | `camera_fusion_node` | Solo 2 camaras, modo sincrono con sincronizador de timestamps. Experimental. |

**Opcion recomendada: multicams (N camaras)**

```bash
ros2 launch camera_fusion_pkg multicams.launch.py
```

**Opcion para exactamente 2 camaras (asincrona)**

```bash
ros2 launch camera_fusion_pkg fusionasincrona.launch.py
```

**Opcion para exactamente 2 camaras (sincrona, experimental)**

```bash
ros2 launch camera_fusion_pkg fusionsincrona.launch.py
```

---

### Paso 6: Verificacion

Una vez el sistema este en marcha, comprueba que los topicos se estan publicando correctamente.

**Topicos disponibles:**

| Topico | Resolucion | Descripcion |
|---|---|---|
| `/autobus/camaras/cam_1/image_raw` | 640x640 | Stream de la camara 1 (sin fusion) |
| `/autobus/camaras/cam_2/image_raw` | 640x640 | Stream de la camara 2 (sin fusion) |
| `/ravo/followme/video_frames` | 640x640 | Vista panoramica fusionada |

**Comprobar frecuencia de publicacion (objetivo: >=15 Hz):**

```bash
ros2 topic hz /ravo/followme/video_frames
```

**Comprobar resolucion de salida:**

```bash
ros2 topic echo /ravo/followme/video_frames --once | grep -E 'height|width'
# Esperado: height: 640 / width: 640
```

**Ver la imagen fusionada en RViz:**

```bash
rviz2
# Añadir un display de tipo Image y suscribirlo a /ravo/followme/video_frames
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
