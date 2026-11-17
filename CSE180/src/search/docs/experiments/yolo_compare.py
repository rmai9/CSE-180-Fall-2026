"""Compare models/thresholds on one frame and pin the numpy channel convention."""
import os
import time

os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/yolo_cfg')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/mpl_cfg')

import numpy as np
import torch
from PIL import Image as PILImage
from ultralytics import YOLO

FRAME = '/tmp/cap/p1_d35_rgb.png'
rgb = np.array(PILImage.open(FRAME).convert('RGB'))
bgr = rgb[:, :, ::-1].copy()

print('=== channel convention: file vs numpy RGB vs numpy BGR ===')
m = YOLO('yolo26n.pt')
for label, src in [('file', FRAME), ('numpy RGB', rgb), ('numpy BGR', bgr)]:
    r = m.predict(src, imgsz=640, conf=0.01, verbose=False)[0]
    top = max([float(b.conf[0]) for b in r.boxes], default=0.0)
    n = len(r.boxes)
    print('  %-10s boxes=%d top_conf=%.3f' % (label, n, top))

print('=== model comparison on %s (imgsz=640) ===' % os.path.basename(FRAME))
for w in ['yolo26n.pt', 'yolo11n.pt', 'yolov8n.pt']:
    mm = YOLO(w)
    r = mm.predict(FRAME, imgsz=640, conf=0.01, verbose=False)[0]
    persons = [(float(b.conf[0]), [round(float(v)) for v in b.xyxy[0]]) for b in r.boxes if r.names[int(b.cls[0])] == 'person']
    others = [(r.names[int(b.cls[0])], round(float(b.conf[0]), 3)) for b in r.boxes if r.names[int(b.cls[0])] != 'person']
    persons.sort(key=lambda t: -t[0])
    print('  %-12s person=%s others=%s' % (w, [(round(c, 3), b) for c, b in persons], others))

print('=== confidence vs imgsz for yolo26n (file input) ===')
for imgsz in [320, 480, 640, 960]:
    r = m.predict(FRAME, imgsz=imgsz, conf=0.01, verbose=False)[0]
    persons = sorted([float(b.conf[0]) for b in r.boxes if r.names[int(b.cls[0])] == 'person'], reverse=True)
    print('  imgsz=%3d person_confs=%s' % (imgsz, [round(c, 3) for c in persons]))

print('=== torch threads ===')
print('  default threads=%d' % torch.get_num_threads())
for nt in [1, 2, 4, 8]:
    torch.set_num_threads(nt)
    for _ in range(2):
        m.predict(FRAME, imgsz=640, conf=0.25, verbose=False)
    ts = []
    for _ in range(4):
        t0 = time.perf_counter()
        m.predict(FRAME, imgsz=640, conf=0.25, verbose=False)
        ts.append((time.perf_counter() - t0) * 1000)
    ts.sort()
    print('  threads=%d imgsz=640 median %.0f ms (min %.0f)' % (nt, ts[len(ts) // 2], ts[0]))
for nt in [1, 4]:
    torch.set_num_threads(nt)
    for _ in range(2):
        m.predict(FRAME, imgsz=320, conf=0.25, verbose=False)
    ts = []
    for _ in range(4):
        t0 = time.perf_counter()
        m.predict(FRAME, imgsz=320, conf=0.25, verbose=False)
        ts.append((time.perf_counter() - t0) * 1000)
    ts.sort()
    print('  threads=%d imgsz=320 median %.0f ms (min %.0f)' % (nt, ts[len(ts) // 2], ts[0]))
