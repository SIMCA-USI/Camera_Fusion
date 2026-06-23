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

Sigue estos pasos en orden. Si el repositorio es nuevo para ti, empieza por el Paso 1 aunque ya tengas ROS 2 instalado.

---

### Paso 1: Requisitos previos

**ROS 2**

El paquete es compatible con ROS 2 Humble y ROS 2 Jazzy. Asegurate de tener el entorno activo en cada terminal que abras:

```bash
# Sustituye 'humble' por 'jazzy' si corresponde
source /opt/ros/humble/setup.bash
```

**Dependencias de Python y V4L2**

```bash
sudo apt install python3-opencv python3-numpy python3-yaml v4l-utils
```

---

### Paso 2: Clonar y compilar

El paquete debe estar dentro de un espacio de trabajo colcon estandar:

```
ros2_ws/
└── src/
    └── camera_fusion_pkg/   ← contenido de este repositorio
```

Clona el repositorio y compila:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <url-del-repositorio> camera_fusion_pkg

cd ~/ros2_ws
colcon build --packages-select camera_fusion_pkg
source install/setup.bash
```

> Ejecuta `source install/setup.bash` cada vez que abras una terminal nueva, o añadelo al fichero `~/.bashrc`.

---

### Paso 3: Identificar las camaras conectadas

Antes de calibrar o lanzar el sistema, identifica que dispositivos USB estan disponibles.

Ejecuta el siguiente script **desde la raiz del workspace**:

```bash
cd ~/ros2_ws
./src/camera_fusion_pkg/scripts/listar_camaras.sh
```

El script comprueba todos los `/dev/video*` con `v4l2-ctl` y OpenCV, e indica cuales devuelven imagen valida (`OK`) y cuales son dispositivos de metadatos inutilizables.

Anota el indice de cada camara valida (por ejemplo `/dev/video0` y `/dev/video2`) para usarlo en los pasos siguientes.

---

### Paso 4: Calibracion de camaras

La calibracion corrige la distorsion optica de cada lente y calcula la homografia de alineacion entre camaras adyacentes. Sin calibracion el sistema funciona, pero las imagenes mostraran distorsion de barril y el cosido entre camaras no sera geometricamente preciso.

Los archivos YAML resultantes se guardan automaticamente en `src/camera_fusion_pkg/config/`.

Necesitas un **tablero de ajedrez impreso** para ambas fases.

---

#### 4.1 Calibracion intrinseca (una vez por camara)

Este paso mide la distorsion de lente de cada camara de forma independiente. Ejecutalo con las camaras desconectadas del nodo de ROS para que el calibrador acceda directamente al dispositivo.

Parametros: `<nombre_camara>  <topico_imagen>  <FilasxColumnas>  <tamano_cuadrado_en_metros>`

```bash
cd ~/ros2_ws

# Camara 1
./src/camera_fusion_pkg/scripts/calibrate_camera.sh cam_1 /cam_1/image_raw 8x6 0.025

# Camara 2
./src/camera_fusion_pkg/scripts/calibrate_camera.sh cam_2 /cam_2/image_raw 8x6 0.025
```

El script lanza el calibrador oficial de ROS 2 con una ventana grafica en tiempo real:

1. Mueve el tablero despacio por todo el campo de vision de la camara hasta que las barras X, Y, Size y Skew se pongan en verde.
2. Haz clic en **CALIBRATE** (el proceso tarda unos segundos).
3. Haz clic en **SAVE** y cierra la ventana.

El script recoge el archivo guardado en `/tmp` y lo mueve automaticamente a `config/<nombre_camara>_calibration.yaml`.

---

#### 4.2 Calibracion estereo (homografia de alineacion)

Este paso calcula la rotacion relativa entre las dos camaras y a partir de ella genera la matriz de homografia que el nodo de fusion usa para proyectar y alinear las imagenes.

Para esta fase el calibrador necesita suscribirse a los topicos de imagen de ambas camaras. Arranca primero el nodo de captura en una terminal:

```bash
cd ~/ros2_ws
ros2 run camera_fusion_pkg camera_reader_node
```

A continuacion, en otra terminal, lanza el calibrador estereo:

```bash
cd ~/ros2_ws
./src/camera_fusion_pkg/scripts/calibrar_estereo.sh 8x6 0.025 /cam_1/image_raw /cam_2/image_raw
```

1. Coloca el tablero en la zona de solapamiento de ambas camaras para que sea visible por las dos simultaneamente.
2. Muevelo despacio hasta que las barras esten verdes.
3. Haz clic en **CALIBRATE** y luego en **SAVE**. Cierra la ventana.

Al cerrar, el script procesa automaticamente el paquete guardado, extrae la homografia y genera `config/board_homography.yaml`. No es necesario ejecutar `extraer_estereo.py` manualmente.

---

### Paso 5: Lanzar el sistema

Una vez compilado (y con los YAML de calibracion en `config/` si los tienes), lanza el sistema con el archivo launch correspondiente a tu configuracion:

| Launch file | Nodo de fusion | Cuando usarlo |
|---|---|---|
| `multicams.launch.py` | `camera_multicams_node` | **Recomendado.** Soporta N camaras de forma escalable. |
| `fusionasincrona.launch.py` | `camera_fusion_async_node` | Solo 2 camaras, procesamiento event-driven sin timer. |
| `fusionsincrona.launch.py` | `camera_fusion_node` | Solo 2 camaras, modo sincrono con sincronizador de timestamps. Experimental. |

**Opcion recomendada: N camaras**

```bash
ros2 launch camera_fusion_pkg multicams.launch.py
```

**Solo 2 camaras, modo asincrono**

```bash
ros2 launch camera_fusion_pkg fusionasincrona.launch.py
```

**Solo 2 camaras, modo sincrono (experimental)**

```bash
ros2 launch camera_fusion_pkg fusionsincrona.launch.py
```

---

### Paso 6: Verificacion

Una vez el sistema esta en marcha, comprueba que los topicos se publican correctamente.

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
# Esperado: height: 640, width: 640
```

**Ver la imagen fusionada en RViz2:**

```bash
rviz2
# Anadir un display de tipo Image y suscribirlo a /ravo/followme/video_frames
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
