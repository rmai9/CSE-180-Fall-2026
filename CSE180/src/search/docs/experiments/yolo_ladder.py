"""Detection vs distance for the simulator person, with a depth cross-check."""
import os
import time

os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/yolo_cfg')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/mpl_cfg')

import numpy as np
from ultralytics import YOLO

CAP = '/tmp/cap'
FRAMES = [('3.5 m', 'p1_d35'), ('4.5 m', 'p1_d45'), ('6.0 m', 'p1_d60'), ('8.0 m', 'p1_d80')]
MODELS = ['yolo26n.pt', 'yolo11n.pt', 'yolov8n.pt']

models = {w: YOLO(w) for w in MODELS}

print('=== person detections (conf>0.05 reported; box in 320x240 px) ===')
for imgsz in [320, 480]:
    print('--- imgsz=%d ---' % imgsz)
    for label, tag in FRAMES:
        path = os.path.join(CAP, tag + '_rgb.png')
        row = []
        for w in MODELS:
            r = models[w].predict(path, imgsz=imgsz, conf=0.01, verbose=False)[0]
            persons = sorted([(float(b.conf[0]), [float(v) for v in b.xyxy[0]]) for b in r.boxes
                              if r.names[int(b.cls[0])] == 'person' and float(b.conf[0]) > 0.05],
                             key=lambda t: -t[0])
            if persons:
                c, xy = persons[0]
                row.append('%s=%.3f(%dx%d px, %d box)' % (
                    w.replace('.pt', ''), c, xy[2] - xy[0], xy[3] - xy[1], len(persons)))
            else:
                row.append('%s=none' % w.replace('.pt', ''))
        print('  %-6s %s' % (label, ' | '.join(row)))

print('=== depth cross-check at the detected person box (yolo11n, imgsz=480) ===')
for label, tag in FRAMES:
    dframe = np.load(os.path.join(CAP, tag + '_depth.npy'))
    r = models['yolo11n.pt'].predict(os.path.join(CAP, tag + '_rgb.png'), imgsz=480,
                                     conf=0.05, verbose=False)[0]
    persons = sorted([(float(b.conf[0]), [float(v) for v in b.xyxy[0]]) for b in r.boxes
                      if r.names[int(b.cls[0])] == 'person'], key=lambda t: -t[0])
    if not persons:
        print('  %-6s no detection' % label)
        continue
    conf, xy = persons[0]
    cx, cy = int((xy[0] + xy[2]) / 2), int((xy[1] + xy[3]) / 2)
    patch = dframe[max(0, cy - 8):cy + 8, max(0, cx - 4):cx + 4]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    med = float(np.median(valid)) if valid.size else float('nan')
    print('  %-6s conf=%.3f box_center=(%d,%d) depth_median=%.3f m valid_px=%d/%d  (expected ~%.2f m + 0.06 camera offset)'
          % (label, conf, cx, cy, med, valid.size, patch.size, float(label.split()[0])))

print('=== timing at imgsz=320/480 (sim running in background) ===')
for w in MODELS:
    for imgsz in [320, 480]:
        for _ in range(2):
            models[w].predict(os.path.join(CAP, 'p1_d45_rgb.png'), imgsz=imgsz, conf=0.25, verbose=False)
        ts = []
        for _ in range(4):
            t0 = time.perf_counter()
            models[w].predict(os.path.join(CAP, 'p1_d45_rgb.png'), imgsz=imgsz, conf=0.25, verbose=False)
            ts.append((time.perf_counter() - t0) * 1000)
        ts.sort()
        print('  %-12s imgsz=%-4d median %5.0f ms  (%4.1f FPS)  min %.0f ms'
              % (w, imgsz, ts[len(ts) // 2], 1000.0 / ts[len(ts) // 2], ts[0]))
