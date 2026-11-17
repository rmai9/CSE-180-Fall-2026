"""Build an annotated contact sheet of the distance-ladder frames."""
import os

os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/yolo_cfg')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/mpl_cfg')

import cv2
import numpy as np
from ultralytics import YOLO

CAP = '/tmp/cap'
FRAMES = [('3.5 m', 'p1_d35'), ('4.5 m', 'p1_d45'), ('6.0 m', 'p1_d60'), ('8.0 m', 'p1_d80')]
model = YOLO('yolo11n.pt')

tiles = []
for label, tag in FRAMES:
    path = os.path.join(CAP, tag + '_rgb.png')
    img = cv2.imread(path)  # BGR, same as the ROS rgb8 -> BGR conversion
    r = model.predict(path, imgsz=480, conf=0.05, verbose=False)[0]
    persons = [(float(b.conf[0]), [int(v) for v in b.xyxy[0]]) for b in r.boxes
               if r.names[int(b.cls[0])] == 'person']
    tile = img.copy()
    for conf, (x1, y1, x2, y2) in persons:
        cv2.rectangle(tile, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(tile, 'person %.2f' % conf, (max(0, x1 - 5), max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
    # zoom x2 so detail survives visual inspection
    zoom = cv2.resize(tile, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    cv2.putText(zoom, '%s  (%d detection)' % (label, len(persons)), (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
    tiles.append(zoom)
    print('%s: %d person box(es) %s' % (label, len(persons), [c for c, _ in persons]))

grid = np.vstack([np.hstack(tiles[0:2]), np.hstack(tiles[2:4])])
cv2.imwrite('/tmp/cap/contact.png', grid)
print('wrote /tmp/cap/contact.png', grid.shape)

# also save the raw 8 m frame zoomed for inspection
img = cv2.imread(os.path.join(CAP, 'p1_d80_rgb.png'))
cv2.imwrite('/tmp/cap/zoom_d80.png', cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
img = cv2.imread(os.path.join(CAP, 'p1_d35_rgb.png'))
cv2.imwrite('/tmp/cap/zoom_d35.png', cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
print('wrote zoom_d80.png / zoom_d35.png')
