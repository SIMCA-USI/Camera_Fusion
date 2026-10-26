#!/usr/bin/env python3
import os
import tarfile
import yaml
import numpy as np
import cv2
import sys

def extract_stereo_homography():
    alpha = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config'))
    tarball_path = os.path.join(config_dir, 'calibrationdataestereo.tar.gz')
    
    if not os.path.exists(tarball_path):
        print(f"❌ No se encontró {tarball_path}. Asegúrate de haber presionado 'SAVE' en la calibración estéreo.")
        return

    print(f"Extrayendo calibración estéreo con alpha={alpha}...")
    tmp_dir = "/tmp/stereo_extract"
    os.makedirs(tmp_dir, exist_ok=True)

    with tarfile.open(tarball_path, 'r:gz') as tar:
        tar.extractall(path=tmp_dir)

    ost_txt = os.path.join(tmp_dir, 'ost.txt')
    if not os.path.exists(ost_txt):
        print("❌ No se encontró ost.txt en el archivo guardado.")
        return

    print("Analizando matrices...")
    matrices = {}
    current_cam = None
    width = 640
    height = 480
    
    with open(ost_txt, 'r') as f:
        lines = f.readlines()
        
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line == 'width' and i + 1 < len(lines):
            try:
                width = int(lines[i+1].strip())
            except ValueError:
                pass
        elif line == 'height' and i + 1 < len(lines):
            try:
                height = int(lines[i+1].strip())
            except ValueError:
                pass
        elif line.endswith('left]'):
            current_cam = 'left'
        elif line.endswith('right]'):
            current_cam = 'right'
        elif line.startswith('camera matrix'):
            if current_cam:
                data = []
                for j in range(1, 4):
                    data.extend([float(x) for x in lines[i+j].strip().split()])
                if current_cam not in matrices: matrices[current_cam] = {}
                matrices[current_cam]['K'] = np.array(data).reshape(3,3)
                i += 3
        elif line.startswith('distortion'):
            if current_cam:
                data = [float(x) for x in lines[i+1].strip().split()]
                if current_cam not in matrices: matrices[current_cam] = {}
                matrices[current_cam]['D'] = np.array(data)
                i += 1
        elif line.startswith('rectification'):
            if current_cam:
                data = []
                for j in range(1, 4):
                    data.extend([float(x) for x in lines[i+j].strip().split()])
                if current_cam not in matrices: matrices[current_cam] = {}
                matrices[current_cam]['R'] = np.array(data).reshape(3,3)
                i += 3
        i += 1

    if 'left' not in matrices or 'right' not in matrices:
        print("❌ Faltan datos estéreo en el archivo. ¿Calibraste en modo estéreo?")
        return
        
    # Calcular Rotación relativa y Homografía entre ambas cámaras con alpha óptimo
    K1 = matrices['left']['K']
    K2 = matrices['right']['K']
    D1 = matrices['left'].get('D', np.zeros(5))
    D2 = matrices['right'].get('D', np.zeros(5))
    R1 = matrices['left']['R']
    R2 = matrices['right']['R']
    
    R_rel = R1.T @ R2
    
    # Calcular matrices óptimas para alpha (alpha=1.0 retiene todo el FOV)
    new_K1, _ = cv2.getOptimalNewCameraMatrix(K1, D1, (width, height), alpha)
    new_K2, _ = cv2.getOptimalNewCameraMatrix(K2, D2, (width, height), alpha)
    
    H = new_K1 @ R_rel @ np.linalg.inv(new_K2)
    H /= H[2, 2]
    
    os.makedirs(config_dir, exist_ok=True)
    out_yaml = os.path.join(config_dir, 'board_homography.yaml')
    
    data = {
        'homography_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [float(x) for x in H.flatten().tolist()]
        }
    }
    
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(data, f)
        
    print(f"✅ ¡Éxito! Matriz de Homografía con alpha={alpha} guardada en:\n📂 {out_yaml}")

    import shutil
    shutil.rmtree(tmp_dir)

if __name__ == '__main__':
    extract_stereo_homography()
