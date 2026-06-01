#!/usr/bin/env python3
import os
import tarfile
import yaml
import numpy as np

def extract_stereo_homography():
    config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'camera_fusion_pkg', 'config'))
    tarball_path = os.path.join(config_dir, 'calibrationdataestereo.tar.gz')
    config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'camera_fusion_pkg', 'config'))
    
    if not os.path.exists(tarball_path):
        print(f"❌ No se encontró {tarball_path}. Asegúrate de haber presionado 'SAVE'.")
        return

    print("Extrayendo calibración estéreo...")
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
    
    with open(ost_txt, 'r') as f:
        lines = f.readlines()
        
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.endswith('left]'):
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
        elif line.startswith('rectification'):
            if current_cam:
                data = []
                for j in range(1, 4):
                    data.extend([float(x) for x in lines[i+j].strip().split()])
                matrices[current_cam]['R'] = np.array(data).reshape(3,3)
                i += 3
        i += 1

    if 'left' not in matrices or 'right' not in matrices:
        print("❌ Faltan datos estéreo en el archivo. ¿Calibraste en modo estéreo?")
        return
        
    # Calcular Rotación relativa y Homografía entre ambas cámaras
    # H = K1 * R_rel * K2_inv (Asumiendo traslación pura paralela o rotación pura)
    K1 = matrices['left']['K']
    K2 = matrices['right']['K']
    R1 = matrices['left']['R']
    R2 = matrices['right']['R']
    
    R_rel = R1.T @ R2
    H = K1 @ R_rel @ np.linalg.inv(K2)
    
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
        
    print(f"✅ ¡Éxito! Matriz de Homografía generada y guardada en:\n📂 {out_yaml}")

    # Limpieza de archivos temporales
    import shutil
    shutil.rmtree(tmp_dir)

if __name__ == '__main__':
    extract_stereo_homography()
