#!/usr/bin/env python3
import yaml
import numpy as np
import scipy.linalg
import os

def main():
    config_dir = os.path.expanduser("~/fusiones_ws/src/camera_fusion_pkg/config")
    input_file = os.path.join(config_dir, "board_homography.yaml")
    output_file = os.path.join(config_dir, "virtual_homography.yaml")

    print(f"Leyendo {input_file}...")
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)

    H_data = data['homography_matrix']['data']
    H = np.array(H_data).reshape((3, 3))

    print("Calculando raices de la matriz (H^1/2 y H^-1/2)...")
    H_half = scipy.linalg.sqrtm(H)
    H_invhalf = np.linalg.inv(H_half)

    # Normalizar para que el valor inferior derecho sea 1.0
    H_half /= H_half[2, 2]
    H_invhalf /= H_invhalf[2, 2]

    out_data = {
        'H_half': {
            'rows': 3,
            'cols': 3,
            'data': [float(x) for x in H_half.real.flatten()]
        },
        'H_invhalf': {
            'rows': 3,
            'cols': 3,
            'data': [float(x) for x in H_invhalf.real.flatten()]
        }
    }

    with open(output_file, 'w') as f:
        yaml.dump(out_data, f)

    print(f"Guardado exitosamente en {output_file}")

if __name__ == "__main__":
    main()
