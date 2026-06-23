import cv2, numpy as np, time

cv2.setNumThreads(0)
W, H = 640, 480
CW, CH = 1100, 480

K = np.eye(3, dtype=np.float64); K[0,0]=K[1,1]=500; K[0,2]=320; K[1,2]=240
D = np.zeros(5)
H_mat = np.eye(3, dtype=np.float64)
H_mat[0,2] = 460

map1x, map1y = cv2.initUndistortRectifyMap(K, D, None, K, (W,H), cv2.CV_16SC2)
m2x, m2y = cv2.initUndistortRectifyMap(K, D, None, K, (W,H), cv2.CV_32FC1)
cx = cv2.warpPerspective(m2x, H_mat, (CW,CH), flags=cv2.INTER_LINEAR)
cy = cv2.warpPerspective(m2y, H_mat, (CW,CH), flags=cv2.INTER_LINEAR)
map2x, map2y = cv2.convertMaps(cx, cy, cv2.CV_16SC2)

raw1 = np.random.randint(0,255,(H,W,3),dtype=np.uint8)
raw2 = np.random.randint(0,255,(H,W,3),dtype=np.uint8)
out1 = np.zeros((H,W,3),dtype=np.uint8)
out2 = np.zeros((CH,CW,3),dtype=np.uint8)
alpha = np.linspace(1,0,200,dtype=np.float32)[np.newaxis,:,np.newaxis]
canvas = np.empty((CH,CW,3),dtype=np.uint8)

N = 200

ops = {
  'frombuffer reshape':        lambda: np.frombuffer(bytes(raw1), dtype=np.uint8).reshape(H,W,3),
  'remap cam1 INTER_LINEAR':   lambda: cv2.remap(raw1, map1x, map1y, cv2.INTER_LINEAR, dst=out1),
  'remap cam2 INTER_LINEAR':   lambda: cv2.remap(raw2, map2x, map2y, cv2.INTER_LINEAR, dst=out2),
  'remap cam1 INTER_NEAREST':  lambda: cv2.remap(raw1, map1x, map1y, cv2.INTER_NEAREST, dst=out1),
  'remap cam2 INTER_NEAREST':  lambda: cv2.remap(raw2, map2x, map2y, cv2.INTER_NEAREST, dst=out2),
  'float32 blend 200cols':     lambda: (out1[:,400:600].astype(np.float32)*alpha + out2[:,400:600].astype(np.float32)*(1-alpha)).astype(np.uint8),
  'copy left zone':            lambda: canvas.__setitem__((slice(None),slice(None,400)), out1[:,0:400]),
  'copy right zone':           lambda: canvas.__setitem__((slice(None),slice(600,None)), out2[:,600:]),
  'canvas.tobytes()':          lambda: canvas.tobytes(),
}

for name, fn in ops.items():
    # warmup
    for _ in range(10): fn()
    t = time.perf_counter()
    for _ in range(N): fn()
    ms = (time.perf_counter()-t)/N*1000
    print(f'{name:<40} {ms:.3f} ms')
