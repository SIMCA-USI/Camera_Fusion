<div align="center">
  <h1>📸 Multi-Camera Panoramic Fusion (ROS 2)</h1>
  <p>Fusión panorámica sin interrupciones (seamless stitching) en tiempo real para múltiples cámaras web USB.</p>
</div>

## 📖 Explicación del Paquete

Este paquete en **C++ para ROS 2** proporciona un sistema de adquisición de vídeo de alto rendimiento. Mediante el uso de matrices de calibración intrínseca (corrección de lente) y calibración estéreo (matrices de homografía), el sistema une múltiples flujos de vídeo solapados para generar una única imagen panorámica continua.

### Arquitectura de Software
- **Lectura Hardware Dedicada:** Hilos de captura individuales por cámara separan la orden de captura (`grab`) de la lectura (`retrieve`) en el driver V4L2. Esto evita la saturación del bus USB.
- **Zero-Copy Fusion:** Los hilos de lectura escriben la imagen en una memoria compartida directamente accesible por el bucle de fusión principal (OpenCV), eliminando por completo la latencia de enviar imágenes entre nodos a través de la red ROS (DDS).
- **Adaptabilidad Dinámica:** Admite funcionar con una sola cámara (simplemente rectifica y publica) o con múltiples cámaras aplicando *alpha-blending* para ocultar los cortes.
- **Preservación Total de Campo de Visión ($\alpha = 1.0$):**
  Flujo: *Calibrar $\rightarrow$ Fusionar $\rightarrow$ Recortar*. Retiene el 100% de los píxeles capturados por el sensor óptico mediante matrices de cámara óptimas (`cv::getOptimalNewCameraMatrix`). Las dos cámaras se proyectan completas en el lienzo panorámico y se realiza un único recorte de márgenes válidos (`crop`) al final sobre la panorámica combinada, evitando la pérdida de ángulos. Cuenta con `auto_adjust_homography: true` para adaptar analíticamente la homografía de calibraciones previas sin requerir recalibrar.

---

## ⚙️ Parámetros y Resoluciones

Toda la configuración se realiza desde el archivo [`config/params.yaml`](config/params.yaml) sin necesidad de recompilar el código.

### Resoluciones de Referencia
Dependiendo del hardware disponible, puedes ajustar la resolución del sistema basándote en esta tabla. **Nota:** el `canvas_w` (lienzo) debe ser lo bastante grande para albergar las imágenes proyectadas de todas las cámaras.

| Modo de Calidad | `cam_w` × `cam_h` (Captura) | `canvas_w` × `canvas_h` (Lienzo) | Rango de Solape (`overlap_start` - `end`) |
|-----------------|-----------------------------|----------------------------------|-------------------------------------------|
| **480p** (Bajo) | `640` × `480`               | `1100` × `480`                   | `475` - `525`                             |
| **720p** (Medio)| `1280` × `720`              | `2200` × `720`                   | `950` - `1050`                            |
| **1080p** (Alto)| `1920` × `1080`             | `3300` × `1080`                  | `1425` - `1575`                           |

> [!WARNING]  
> **Requisito crítico:** Las matrices matemáticas de calibración ($K$, $D$ y $H$) están estrictamente atadas a la resolución en píxeles. **Si cambias la resolución física de captura (`cam_w` / `cam_h`), es obligatorio realizar de nuevo todo el proceso de calibración.**

---

## 🚀 Despliegue

### 0. Alias de Terminal
Añade a `~/.bashrc` (una sola vez) y ejecuta `source ~/.bashrc`:
```bash
alias calibrar='bash ~/ROCIN-FollowMe/followme_ws/src/camera_fusion_pkg/scripts/calibrate_camera.sh'
alias calibrar_estereo='bash ~/ROCIN-FollowMe/followme_ws/src/camera_fusion_pkg/scripts/calibrar_estereo.sh'
alias listar_camaras='bash ~/ROCIN-FollowMe/followme_ws/src/camera_fusion_pkg/scripts/listar_camaras.sh'
```
Los scripts guardan siempre los resultados en `camera_fusion_pkg/config/`, sin importar desde qué directorio se lancen. Usa `listar_camaras` para ver qué `/dev/video*` funciona.

### 1. Fase de Calibración
Antes de ejecutar el sistema de fusión, es necesario calibrar las lentes de cada cámara y su posición relativa espacial mediante un patrón de tablero de ajedrez (*checkerboard*).

> [!NOTE]
> **Sobre las dimensiones del tablero:**
> En los comandos a continuación verás los argumentos `<INTERSECCIONES>` y `<TAMAÑO_M>`. 
> - **Intersecciones (Ej. `9x6`)**: Es el número de *vértices internos* del tablero (cuadrados totales menos uno por lado).
> - **Tamaño (Ej. `0.0404`)**: Es la longitud del lado de un solo cuadrado impreso, medida en metros (40.4 mm en este caso).
> **Debes cambiar estos dos valores** en los comandos de abajo para que coincidan matemáticamente con tu tablero impreso, de lo contrario las distancias espaciales serán erróneas.

#### A. Calibración Intrínseca (Individual)
Permite corregir las aberraciones de ojo de pez o deformaciones de cada lente. **Debe ejecutarse una vez por cámara.**

1. Lanza el orquestador de calibración monocular para la `cam_1`:
   ```bash
   # Sintaxis: script <nombre_cam> <topic> <intersecciones> <tamaño_m>
   calibrar cam_1 /cam_1/image_raw 9x6 0.0404
   ```
2. Mueve el tablero delante de la cámara en varios ángulos hasta que los medidores (X, Y, Size, Skew) estén en verde.
3. Haz clic en **CALIBRATE** y luego en **SAVE**.
4. Repite el proceso exacto cambiando `cam_1` a `cam_2` (y el topic a `/cam_2/image_raw`).

#### B. Calibración Estéreo (Homografía)
Establece la relación geométrica entre las dos cámaras. **Ambas cámaras deben estar ya ancladas a su soporte final, sin posibilidad de moverse.**

1. Inicia el script de cálculo de homografía:
   ```bash
   # Sintaxis: script <intersecciones> <tamaño_m> <topic_izq> <topic_der>
   calibrar_estereo 9x6 0.0404 /cam_1/image_raw /cam_2/image_raw
   ```
2. Coloca el tablero de ajedrez en la zona central donde se solapan los campos de visión de ambas cámaras. **Ambas deben ver el tablero completo a la vez.**
3. El sistema tomará muestras sincronizadas y generará un archivo `board_homography.yaml` en el directorio de configuración.

---

### 2. Fase de Lanzamiento (Producción)

Con todos los archivos YAML generados (`cam_1_calibration.yaml`, `cam_2_calibration.yaml`, `board_homography.yaml`), el sistema está listo para arrancar:

```bash
ros2 launch camera_fusion_pkg camfusion.launch.py
```

#### Visualización
Para visualizar el resultado final sin saturar la red DDS local:
```bash
rviz2
```
*Suscríbete mediante un display tipo `Image` al topic:* `/carrito/followme/video_frames` (o `/autobus/camaras/panoramic_fusion/image_raw` según el namespace configurado).
