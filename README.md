# Multi-Camera Panoramic Fusion (ROS 2)

Este repositorio contiene un sistema completo en ROS 2 diseñado para la adquisición, calibración y fusión en tiempo real de múltiples cámaras (webcams USB) con el objetivo de generar una única vista panorámica sin interrupciones visuales (seamless stitching). El proyecto está desarrollado bajo un estándar de investigación en C++ para garantizar alto rendimiento y cero latencia.

## Fundamentos Teóricos y Flujo de Trabajo

La fusión de cámaras requiere un proceso riguroso de calibración geométrica para corregir las aberraciones de las lentes y alinear los sistemas de coordenadas espaciales. El sistema se divide en tres fases conceptuales:

### 1. Calibración Intrínseca (Corrección de Lente)
**¿Por qué se hace?** Las lentes introducen distorsiones que curvan las líneas rectas. Además, cada cámara tiene un "centro óptico" y una "distancia focal" únicos. Mediante la observación de un patrón conocido (tablero de ajedrez), el algoritmo estima la **Matriz de la Cámara ($K$)** y los **Coeficientes de Distorsión ($D$)**. El nodo de fusión utiliza estos datos para rectificar matemáticamente cada imagen antes de intentar unirlas.

<!-- TODO: Insertar imagen representativa de una cámara mostrando el tablero de calibración intrínseca -->
![Calibración Intrínseca](docs/images/calibracion_intrinseca.png)

### 2. Calibración Estéreo (Alineación Espacial)
**¿Por qué se hace?** Para unir dos imágenes, necesitamos saber exactamente dónde está físicamente una cámara con respecto a la otra. La calibración estéreo analiza cómo ambas cámaras ven el mismo patrón en el espacio 3D simultáneamente, extrayendo la matriz de rotación relativa.

<!-- TODO: Insertar imagen representativa de ambas cámaras viendo el mismo tablero en el área de solape -->
![Calibración Estéreo](docs/images/calibracion_estereo.png)

### 3. Fusión por Homografía y Blending
Con las cámaras rectificadas y su relación rotacional conocida, calculamos una **Matriz de Homografía ($H$)**. Esta matriz proyecta la imagen de la cámara derecha sobre el plano coordenado de la cámara izquierda. Finalmente, para evitar un corte duro en la unión, el nodo aplica un algoritmo de **Linear Blending** enfocado en la Región de Interés (ROI) de solape.

<!-- TODO: Insertar imagen mostrando el resultado final de la vista panorámica fusionada -->
![Fusión y Blending](docs/images/fusion_panoramica.png)

---

## Configuración del Entorno y Alias

Para facilitar el uso repetitivo de las herramientas, es muy recomendable configurar unos alias en el archivo `.bashrc` y añadir los comandos de entorno correspondientes de tus *workspaces*. 

Puedes añadir todo automáticamente ejecutando el siguiente bloque en tu terminal (asegúrate de que las rutas coinciden con la ubicación real de tus directorios):

```bash
echo "alias calibrar='bash ~/ros2_ws/src/camera_fusion_pkg/scripts/calibrate_camera.sh'" >> ~/.bashrc
echo "alias calibrar_estereo='bash ~/ros2_ws/src/camera_fusion_pkg/scripts/calibrar_estereo.sh'" >> ~/.bashrc
echo "alias listar_camaras='bash ~/ros2_ws/src/camera_fusion_pkg/scripts/listar_camaras.sh'" >> ~/.bashrc
source ~/.bashrc
```

## Rutas de Calibración y Archivos YAML

El nodo principal de fusión lee los parámetros de calibración desde archivos `.yaml`. Por defecto, el programa asume que estos archivos se encuentran en:
`/home/$USER/ros2_ws/src/camera_fusion_pkg/config/`

Si despliegas este proyecto en otra máquina, otro usuario, o en un directorio distinto, **debes actualizar la ruta de configuración**. 
Para ello, el nodo en C++ expone el parámetro `config_dir`. Al lanzar el nodo o al modificar el archivo de lanzamiento (launch file), debes configurar el parámetro `config_dir` pasándole la ruta absoluta donde se generen y guarden los archivos `.yaml` de calibración.

---

## Tabla de Resoluciones de Referencia

La resolución final de la panorámica **depende de toda la cadena desde el principio**: la calibración intrínseca y la calibración estéreo deben hacerse a la misma resolución que se configure en el nodo de fusión. Si se cambia la resolución, hay que **recalibrar todo desde cero**.

Los parámetros del nodo de fusión que dependen de la resolución son:

| Parámetro | Descripción |
|---|---|
| `cam_w` / `cam_h` | Resolución física de captura de cada cámara por USB (debe coincidir con la resolución de calibración) |
| `canvas_w` / `canvas_h` | Tamaño del lienzo interno de trabajo (aprox. el doble de ancho que la cámara) |
| `overlap_start` / `overlap_end` | Zona de blending en píxeles X sobre el lienzo (zona central donde se solapan ambas cámaras) |
| `margin_x` / `margin_y` | Márgenes en píxeles para recortar los bordes negros del resultado final |

### Configuraciones Probadas

| Parámetro | **480p** *(defecto)* | **720p** | **1080p** |
|---|---|---|---|
| `cam_w` × `cam_h` | 640 × 480 | 1280 × 720 | 1920 × 1080 |
| `canvas_w` × `canvas_h` | 1100 × 480 | 2200 × 720 | 3300 × 1080 |
| `overlap_start` | 475 | 950 | 1425 |
| `overlap_end` | 525 | 1050 | 1575 |
| Panorámica aprox. | ~1040 × 460 px | ~2100 × 700 px | ~3150 × 1060 px |

> **⚠️ Importante**: Los valores de `overlap` y `margin` son aproximados de partida. El solapamiento real depende de la colocación física de las cámaras, por lo que puede ser necesario ajustarlos manualmente tras la calibración. Los valores de `margin_x` y `margin_y` (por defecto `30` y `10`) no escalan de forma crítica con la resolución.

> **⚠️ Recuerda**: Si cambias la resolución de captura, los archivos `.yaml` de calibración (`cam_1_calibration.yaml`, `cam_2_calibration.yaml` y `board_homography.yaml`) **deben ser recalculados a esa nueva resolución**. Reutilizar calibraciones de otra resolución romperá la fusión.

---

## Despliegue y Uso

### Compilación
Asegúrate de tener ROS 2 (Jazzy/Humble), OpenCV y la librería `yaml-cpp` instalados en la máquina de despliegue (`sudo apt update && sudo apt install libyaml-cpp-dev v4l-utils`).
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 1. Detección de Cámaras
Para comprobar qué dispositivos de vídeo están disponibles y detectar el hardware:
```bash
listar_camaras
```

### 2. Calibración

**A. Intrínseca (Por cada cámara):**
```bash
calibrar cam_1 /cam_1/image_raw 9x6 0.0404
calibrar cam_2 /cam_2/image_raw 9x6 0.0404
```

**B. Estéreo (Matriz de Homografía):**
Lanza ambas cámaras individualmente y ejecuta:
```bash
calibrar_estereo 9x6 0.0404 /cam_1/image_raw /cam_2/image_raw
```

### 3. Ejecución del Sistema

**Visualización Recomendada:**
Para visualizar correctamente el resultado de la fusión o los topics de vídeo individuales sin pérdida de rendimiento, se recomienda utilizar **RViz2**.

**Lectura Individual de Cámaras:**
Si necesitas revisar las cámaras por separado (por ejemplo, para depurar antes de la calibración), puedes lanzar el nodo lector nativo en C++:
```bash
ros2 run camera_fusion_cpp_pkg camera_reader_cpp_node
```

**Fusión Panorámica:**
El sistema completo ha sido refactorizado a un nodo monolítico en C++ que realiza la captura desde hardware y la fusión en un único proceso, consiguiendo evitar los cuellos de botella de red DDS (Zero-Copy).
```bash
ros2 launch camera_fusion_pkg camfusion.launch.py
```

**Cambio de Cámara (GUI):**
En caso de necesitar intercambiar el orden de las cámaras en caliente, puedes lanzar la interfaz gráfica ejecutando el siguiente comando:
```bash
python3 src/camera_fusion_cpp_pkg/scripts/camera_swap_gui.py
```

### 4. Despliegue Experimental: Múltiples Cámaras (N-Cámaras)
*Nota importante: Este modo se encuentra actualmente en proceso de pruebas experimentales.*

Además de la fusión estándar de 2 cámaras, el repositorio incluye un modo en desarrollo para fusionar dinámicamente 3 o más cámaras mediante `multicams.launch.py`. La arquitectura utiliza una cámara "ancla" central hacia la que se proyectan las demás mediante calibraciones estéreo par a par.

---

## Arquitectura del Software

### Nodo Principal C++ (`camera_fusion_cpp_pkg`)
* **`camera_fusion_cpp_node`**: Nodo de alto rendimiento en C++ 17/20. Sustituye a la antigua arquitectura distribuida en Python. 
  - **Lectura Hardware Dedicada**: Implementa hilos de captura individuales por cámara, utilizando el patrón de lectura asíncrona V4L2 (`grab` y `retrieve` separados) para evitar la saturación del bus USB.
  - **Detección Dinámica**: Identifica el hardware leyendo `/sys/class/video4linux/videoX/name` e ignora automáticamente las cámaras integradas para evitar conflictos de asignación.
  - **Zero-Copy Fusion**: Los hilos de captura comparten memoria directamente con el bucle de fusión principal, eliminando la sobrecarga de publicación/suscripción.
  - **Control Constante**: Publica estrictamente a la tasa configurada (por defecto 15 FPS) limitando el uso de CPU y asegurando determinismo en la red de comunicaciones.

### Scripts de Soporte (`scripts/`)
* **`listar_camaras.sh`**: Interactúa con el kernel para depurar dispositivos V4L2.
* **`calibrate_camera.sh`**: Invocador automatizado del calibrador oficial de ROS 2 para corrección monocular.
* **`calibrar_estereo.sh`**: Wrapper para invocar el nodo de calibración en modo estéreo asíncrono.
