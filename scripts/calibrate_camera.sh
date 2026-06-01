#!/bin/bash

# Uso: ./calibrate_camera.sh [NOMBRE_CAMARA] [TOPICO_IMAGEN] [TAMAÑO_CUADRICULA] [TAMAÑO_CUADRADO]
# Ejemplo: ./calibrate_camera.sh cam_1 /cam_1/image_raw 8x6 0.108

CAMERA_NAME=${1:-"cam_1"}
IMAGE_TOPIC=${2:-"/${CAMERA_NAME}/image_raw"}
GRID_SIZE=${3:-"9x6"}
SQUARE_SIZE=${4:-"0.04"}

# Directorio donde se guardarán las configuraciones (dentro del paquete)
CONFIG_DIR="$(pwd)/src/camera_fusion_pkg/config"
mkdir -p "${CONFIG_DIR}"

echo "================================================="
echo "Iniciando calibración de cámara"
echo "Cámara: ${CAMERA_NAME}"
echo "Tópico: ${IMAGE_TOPIC}"
echo "Cuadrícula: ${GRID_SIZE}"
echo "Tamaño del cuadrado: ${SQUARE_SIZE}m"
echo "================================================="
echo "Instrucciones:"
echo "1. Mueve el tablero frente a la cámara hasta que las barras X, Y, Size, Skew estén en verde."
echo "2. Haz clic en el botón 'CALIBRATE' (tardará unos segundos)."
echo "3. Haz clic en el botón 'SAVE' (se guardará en /tmp)."
echo "4. Cierra la ventana. El script automáticamente moverá el archivo a la carpeta config."
echo "================================================="

# Eliminar datos de calibración anteriores en /tmp si existen
rm -f /tmp/calibrationdata.tar.gz

# Ejecutar el nodo oficial de calibración de ROS 2
ros2 run camera_calibration cameracalibrator \
    --size ${GRID_SIZE} \
    --square ${SQUARE_SIZE} \
    --no-service-check \
    --ros-args -r image:=${IMAGE_TOPIC}

# Una vez que se cierra la ventana, buscar el archivo en /tmp
if [ -f "/tmp/calibrationdata.tar.gz" ]; then
    echo "¡Datos de calibración encontrados! Procesando..."
    
    # Extraer en una carpeta temporal
    TEMP_EXTRACT=$(mktemp -d)
    tar -xzf /tmp/calibrationdata.tar.gz -C "${TEMP_EXTRACT}"
    
    if [ -f "${TEMP_EXTRACT}/ost.yaml" ]; then
        # Copiar y renombrar el archivo de calibración a la carpeta config
        FINAL_PATH="${CONFIG_DIR}/${CAMERA_NAME}_calibration.yaml"
        cp "${TEMP_EXTRACT}/ost.yaml" "${FINAL_PATH}"
        
        echo "✅ ¡ÉXITO! La calibración ha sido guardada en:"
        echo "📂 ${FINAL_PATH}"
    else
        echo "❌ Error: No se encontró el archivo ost.yaml en el paquete de calibración."
    fi
    
    # Limpieza
    rm -rf "${TEMP_EXTRACT}"
    rm /tmp/calibrationdata.tar.gz
else
    echo "⚠️ No se encontró el archivo de calibración en /tmp."
    echo "Asegúrate de haber hecho clic en el botón 'SAVE' antes de cerrar la ventana."
fi
