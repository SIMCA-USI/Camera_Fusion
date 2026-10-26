#!/bin/bash
# =============================================================================
# listar_camaras.sh — Muestra las cámaras USB disponibles en el sistema
# =============================================================================
# Uso: ./listar_camaras.sh
# =============================================================================

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║     Dispositivos de cámara disponibles    ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# Verificar que v4l2-ctl está instalado
if ! command -v v4l2-ctl &>/dev/null; then
    echo "⚠  v4l2-ctl no encontrado. Instalando..."
    sudo apt-get install -y v4l-utils
fi

# Listar dispositivos con nombre
v4l2-ctl --list-devices 2>/dev/null | sed 's/^/  /'

echo ""
echo "─── Detalle por dispositivo ───────────────────────────────"
for dev in /dev/video*; do
    # Filtrar sólo los que capturan vídeo (no metadata)
    if v4l2-ctl -d "$dev" --info 2>/dev/null | grep -q "Video Capture"; then
        node_name=$(v4l2-ctl -d "$dev" --info 2>/dev/null | grep "Card type" | awk -F: '{print $2}' | xargs)
        echo "  $dev  →  $node_name"
    fi
done

echo ""
echo "─── Prueba rápida con OpenCV ──────────────────────────────"
python3 - <<'EOF'
import cv2, os

found = []
for i in range(10):
    path = f'/dev/video{i}'
    if not os.path.exists(path):
        continue
    cap = cv2.VideoCapture(i)
    if not cap.isOpened():
        cap.release()
        continue
    valid_frames = 0
    for _ in range(5):
        ret, frame = cap.read()
        if ret and frame is not None and frame.sum() > 0:
            valid_frames += 1
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    status = "✓ OK" if valid_frames > 0 else "✗ Pantalla negra (metadata-only)"
    found.append((i, path, w, h, status))

if not found:
    print("  No se encontraron dispositivos /dev/video*")
else:
    for idx, path, w, h, status in found:
        print(f"  [{idx}] {path}  {w}x{h}  {status}")
EOF

echo ""
echo "💡 Útil para la calibración:"
echo "Verifica qué dispositivo muestra '✓ OK' (por ejemplo /dev/video0 o /dev/video2)."
echo "Podrás usar ese índice al lanzar los nodos de captura de ROS 2."
echo ""
