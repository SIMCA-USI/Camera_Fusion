# Multi-Camera Panoramic Fusion (ROS 2)

Este repositorio contiene un sistema completo en ROS 2 diseñado para la adquisición, calibración y fusión en tiempo real de múltiples cámaras (webcams USB) con el objetivo de generar una única vista panorámica sin interrupciones visuales (seamless stitching). El proyecto está desarrollado bajo un estándar de investigación.

## Fundamentos Teóricos y Flujo de Trabajo

La fusión de cámaras requiere un proceso riguroso de calibración geométrica para corregir las aberraciones de las lentes y alinear los sistemas de coordenadas espaciales. El sistema se divide en tres fases conceptuales:

### 1. Calibración Intrínseca (Corrección de Lente)
**¿Por qué se hace?** Las lentes de las cámaras de bajo coste introducen distorsiones severas (efecto barril o acerico) que curvan las líneas rectas. Además, cada cámara tiene un "centro óptico" y una "distancia focal" únicos. 
Mediante la observación de un patrón conocido (tablero de ajedrez), el algoritmo estima la **Matriz de la Cámara ($K$)** y los **Coeficientes de Distorsión ($D$)**. El nodo de fusión utiliza estos datos para "aplanar" (rectificar) matemáticamente cada imagen antes de intentar unirlas.

<!-- TODO: Insertar imagen representativa de una cámara mostrando el tablero de calibración intrínseca -->
![Calibración Intrínseca](docs/images/calibracion_intrinseca.png)

### 2. Calibración Estéreo (Alineación Espacial)
**¿Por qué se hace?** Para unir dos imágenes, necesitamos saber exactamente dónde está físicamente una cámara con respecto a la otra. La calibración estéreo analiza cómo ambas cámaras ven el mismo patrón en el espacio 3D simultáneamente.
A partir de esto, se extrae la matriz de rotación relativa ($R_{rel}$) entre ambas cámaras.

<!-- TODO: Insertar imagen representativa de ambas cámaras viendo el mismo tablero en el área de solape -->
![Calibración Estéreo](docs/images/calibracion_estereo.png)

### 3. Fusión por Homografía y Blending
Con las cámaras rectificadas y su relación rotacional conocida, calculamos una **Matriz de Homografía ($H$)** mediante la ecuación:
$H = K_1 \cdot R_{rel} \cdot K_2^{-1}$

Esta matriz proyecta la imagen de la cámara derecha sobre el plano coordenado de la cámara izquierda (`cv2.warpPerspective`). Finalmente, para evitar un "corte duro" visible en la unión de las dos imágenes, el nodo aplica un algoritmo de **Linear Blending (Degradado)** enfocado exclusivamente en la Región de Interés (ROI) de solape, optimizando dramáticamente el rendimiento computacional.

<!-- TODO: Insertar imagen mostrando el resultado final de la vista panorámica fusionada -->
![Fusión y Blending](docs/images/fusion_panoramica.png)

---

## Despliegue y Uso

### Compilación
Asegúrate de tener ROS 2 (Jazzy/Humble) instalado.
```bash
cd ~/camera_fusion_ws
colcon build --packages-select camera_fusion_pkg
source install/setup.bash
```

### 1. Detección de Cámaras
Para comprobar qué dispositivos de vídeo están disponibles y descartar los nodos de metadatos del kernel de Linux:
```bash
./scripts/listar_camaras.sh
```

### 2. Calibración
*Nota: Si las cámaras no se han calibrado, el nodo de fusión no arrancará para evitar errores de segmentación.*

**A. Intrínseca (Por cada cámara):**
```bash
./scripts/calibrate_camera.sh cam_1 /cam_1/image_raw 9x6 0.0404
./scripts/calibrate_camera.sh cam_2 /cam_2/image_raw 9x6 0.0404
```
*(Extrae los archivos generados a la carpeta `config` renombrándolos a `cam_1_calibration.yaml` y `cam_2_calibration.yaml`)*

**B. Estéreo (Matriz de Homografía):**
Lanza ambas cámaras (puedes usar el launch file) y ejecuta:
```bash
./scripts/calibrar_estereo.sh 9x6 0.0404 /cam_1/image_raw /cam_2/image_raw
```
Una vez guardada la calibración, genera el archivo YAML final usando el script de extracción:
```bash
python3 ./scripts/extraer_estereo.py
```

### 3. Ejecución del Sistema
El sistema completo se levanta mediante un único archivo Launch que orquesta la lectura de cámaras y la fusión de forma sincronizada.

```bash
ros2 launch camera_fusion_pkg fusion.launch.py
```
El resultado panorámico en tiempo real se publicará en el tópico: `fused_panorama`.

---

## Arquitectura del Software

### Nodos ROS 2 (`src/camera_fusion_pkg/`)
* **`camera_reader_node.py`**: Nodo encargado de la ingesta de vídeo. Posee una lógica de auto-descubrimiento que escanea V4L2 en busca de cámaras válidas, las inicializa a `640x480` y utiliza sincronización forzada (`cap.grab()` simultáneo) para minimizar la latencia del bus USB antes de publicar en los tópicos `cam_X/image_raw`.
* **`camera_fusion_node.py`**: Nodo principal de visión artificial. Se suscribe a las cámaras usando filtros de tiempo aproximado (`message_filters`). En cada iteración, rectifica las distorsiones, aplica la homografía a la segunda cámara, y ejecuta un algoritmo de fusión (*blending*) localizado espacialmente para unir ambos fotogramas maximizando los FPS (reducción del 80% de operaciones en coma flotante respecto a un blending de lienzo completo).

### Scripts de Soporte (`scripts/`)
* **`listar_camaras.sh`**: Script en Bash y Python que interactúa con `v4l2-ctl` y OpenCV para depurar dispositivos `/dev/video*`.
* **`calibrate_camera.sh`**: Wrapper automatizado que invoca el calibrador oficial de ROS 2 (`camera_calibration`) inyectando los parámetros correctos para la corrección monocular.
* **`calibrar_estereo.sh`**: Wrapper para invocar el nodo de calibración en modo estéreo (`--approximate`, ignorando servicios inexistentes).
* **`extraer_estereo.py`**: Script de post-procesamiento. Abre los tarballs (`.tar.gz`) generados por la GUI de ROS 2, extrae los parámetros estéreo ocultos en `ost.txt`, procesa el álgebra lineal para obtener $H$ y exporta el `board_homography.yaml` limpio para el nodo de fusión.

### Launch (`launch/`)
* **`fusion.launch.py`**: Orquestador oficial del paquete. Lanza el lector de cámaras y el motor de fusión simultáneamente, gestionando el ciclo de vida de los nodos.
