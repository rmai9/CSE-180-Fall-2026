import os
os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/yolo_cfg')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/mpl_cfg')

import cv2
import numpy as np
from ultralytics import YOLO

CAP = '/tmp/cap'
FRAMES = [('4.0 m', 'p2_d45'), ('6.0 m', 'p2_d60')]
model = YOLO('yolo11n.pt')

tiles = []
for label, tag in FRAMES:
    img = cv2.imread(os.path.join(CAP, tag + '_rgb.png'))
    r = model.predict(os.path.join(CAP, tag + '_rgb.png'), imgsz=480, conf=0.05, verbose=False)[0]
    persons = sorted([(float(b.conf[0]), [int(v) for v in b.xyxy[0]]) for b in r.boxes
                      if r.names[int(b.cls[0])] == 'person'], key=lambda t: -t[0])
    tile = img.copy()
    for conf, (x1, y1, x2, y2) in persons:
        cv2.rectangle(tile, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(tile, 'person %.2f' % conf, (max(0, x1 - 6), max(14, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    zoom = cv2.resize(tile, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_NEAREST)
    cv2.putText(zoom, '%s  top person conf %.2f' % (label, persons[0][0] if persons else 0.0),
                (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
    tiles.append(zoom)
    print('%s: %d person box(es) %s' % (label, len(persons), [round(c, 3) for c, _ in persons]))

grid = np.hstack(tiles)
cv2.imwrite('/tmp/cap/contact_p2.png', grid)
print('wrote', grid.shape)
