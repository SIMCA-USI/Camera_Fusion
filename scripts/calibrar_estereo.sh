#!/bin/bash
# =============================================================================
# calibrar_estereo.sh — Ejecuta la calibración estéreo para dos cámaras
# =============================================================================

# Parámetros por defecto
GRID_SIZE=${1:-"9x6"}
SQUARE_SIZE=${2:-"0.04"}
LEFT_TOPIC=${3:-"/cam_1/image_raw"}
RIGHT_TOPIC=${4:-"/cam_2/image_raw"}

echo "============================================================"
echo "    LANZANDO CALIBRADOR ESTÉREO OFICIAL DE ROS 2"
echo "============================================================"
echo "Cuadrícula: ${GRID_SIZE}"
echo "Tamaño del cuadrado: ${SQUARE_SIZE}m"
echo "Tópico Izquierdo: ${LEFT_TOPIC}"
echo "Tópico Derecho: ${RIGHT_TOPIC}"
echo "============================================================"
echo "Instrucciones:"
echo "1. Coloca el tablero en el área de SOLAPAMIENTO de ambas cámaras."
echo "2. Mueve el tablero hasta que las barras estén verdes."
echo "3. Pulsa CALIBRATE y luego SAVE."
echo "4. Cierra la ventana. El script procesará automáticamente la Homografía."
echo "============================================================"

# Eliminar archivo anterior para evitar confusiones
rm -f /tmp/calibrationdata.tar.gz

# Ejecutar el calibrador oficial en modo estéreo (--approximate permite ligera asincronía)
ros2 run camera_calibration cameracalibrator \
    --size ${GRID_SIZE} \
    --square ${SQUARE_SIZE} \
    --approximate 0.1 \
    --camera_name stereo_forward \
    --disable_calib_cb_fast_check \
    --no-service-check \
    --ros-args \
    -r left:=${LEFT_TOPIC} \
    -r right:=${RIGHT_TOPIC} \
    -r left_camera:=/cam_1 \
    -r right_camera:=/cam_2

# Una vez cerrado, ejecutamos el script de extracción en Python
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
python3 "${SCRIPT_DIR}/extraer_estereo.py"
