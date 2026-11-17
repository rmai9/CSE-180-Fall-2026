"""Test pretrained YOLO on captured simulator frames.

Usage: python3 yolo_test.py <image> [<image> ...]
"""
import glob
import json
import os
import sys
import time

os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/yolo_cfg')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/mpl_cfg')

import numpy as np
import torch
import torchvision
import ultralytics

print('=== versions ===')
print('ultralytics %s | torch %s | torchvision %s | numpy %s' % (
    ultralytics.__version__, torch.__version__, torchvision.__version__, np.__version__))
print('threads=%d cuda=%s' % (torch.get_num_threads(), torch.cuda.is_available()))

base = os.path.join(os.path.dirname(ultralytics.__file__), 'cfg', 'models')
families = sorted(os.path.basename(p) for p in glob.glob(os.path.join(base, '*')))
print('=== model families shipped in this release ===')
print(', '.join(families))
for want in ['yolo26', 'yolo11', 'yolov8']:
    ys = sorted(os.path.basename(p) for p in glob.glob(os.path.join(base, want, '*.yaml')))[:4]
    print('  %-8s -> %s' % (want, ys if ys else 'NOT PRESENT'))

from ultralytics import YOLO  # noqa: E402

print('=== weight availability ===')
loaded = None
loaded_name = None
for name in ['yolo26n.pt', 'yolo11n.pt', 'yolov8n.pt']:
    try:
        m = YOLO(name)
        print('  %-14s LOADED (%d classes, task=%s)' % (name, len(m.names), m.task))
        if loaded is None:
            loaded, loaded_name = m, name
    except Exception as ex:  # noqa: BLE001
        print('  %-14s FAILED: %s' % (name, str(ex).splitlines()[0][:150]))

if loaded is None:
    print('no weights could be loaded')
    sys.exit(1)

print('=== how numpy/array input is handled by the installed version ===')
pred_src = os.path.join(os.path.dirname(ultralytics.__file__), 'engine', 'predictor.py')
with open(pred_src) as f:
    lines = f.readlines()
for i, ln in enumerate(lines):
    if 'ndarray' in ln or 'ndim ==' in ln:
        print('  predictor.py:%d: %s' % (i + 1, ln.rstrip()[:120]))

images = sys.argv[1:]
print('=== detections (model=%s, imgsz=640, conf shown at 0.25) ===' % loaded_name)
results_summary = {}
for path in images:
    tag = os.path.basename(path)
    r = loaded.predict(path, imgsz=640, conf=0.25, verbose=False)[0]
    names = r.names
    n_person = 0
    rows = []
    for b in r.boxes:
        cls = int(b.cls[0])
        score = float(b.conf[0])
        xyxy = [round(float(v), 1) for v in b.xyxy[0]]
        w = xyxy[2] - xyxy[0]
        h = xyxy[3] - xyxy[1]
        rows.append((names[cls], score, xyxy, round(w, 1), round(h, 1)))
        if names[cls] == 'person':
            n_person += 1
    rows.sort(key=lambda t: -t[1])
    print('  %s: %d raw detection(s), %d person-class' % (tag, len(rows), n_person))
    for name, score, xyxy, w, h in rows:
        print('      %-12s conf=%.3f box=%s size=%.0fx%.0f px' % (name, score, xyxy, w, h))
    results_summary[tag] = {'raw': len(rows), 'person': n_person,
                            'top': rows[0] if rows else None}

print('=== timing (model=%s) ===' % loaded_name)
first = images[0]
for imgsz in [640, 320]:
    for i in range(2):  # warmup
        loaded.predict(first, imgsz=imgsz, conf=0.25, verbose=False)
    times = []
    for i in range(5):
        t0 = time.perf_counter()
        loaded.predict(first, imgsz=imgsz, conf=0.25, verbose=False)
        times.append((time.perf_counter() - t0) * 1000.0)
    times.sort()
    print('  imgsz=%d: median %.1f ms  min %.1f ms  -> %.1f FPS (single thread of control, CPU)'
          % (imgsz, times[len(times) // 2], times[0], 1000.0 / times[len(times) // 2]))

print('=== channel-order check (same frame, file vs numpy) ===')
from PIL import Image as PILImage  # noqa: E402
arr_rgb = np.array(PILImage.open(first).convert('RGB'))
r_file = loaded.predict(first, imgsz=640, conf=0.25, verbose=False)[0]
r_np = loaded.predict(arr_rgb, imgsz=640, conf=0.25, verbose=False)[0]
print('  file: %d boxes, numpy(RGB): %d boxes' % (len(r_file.boxes), len(r_np.boxes)))
if len(r_file.boxes) and len(r_np.boxes):
    print('  file top conf %.3f vs numpy top conf %.3f' % (
        float(r_file.boxes.conf.max()), float(r_np.boxes.conf.max())))
json.dump(results_summary, open('/tmp/yolo_result.json', 'w'), indent=1)
