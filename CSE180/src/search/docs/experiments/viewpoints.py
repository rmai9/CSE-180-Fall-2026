"""Pick robot poses with a clear line of sight to a person, from the occupancy map.

Usage: python3 /tmp/viewpoints.py <person_x> <person_y> [distance ...]
"""
import math
import sys
import numpy as np
from PIL import Image
from scipy import ndimage

PATH = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.pgm'
RES = 0.03
OX, OY = -15.1, -25.0
CLEAR_NEED = 0.30  # robot radius 0.22 + margin

a = np.array(Image.open(PATH))
ROWS, COLS = a.shape
occ_prob = (255.0 - a) / 255.0
occupied = occ_prob > 0.65
free = occ_prob < 0.10
clearance = ndimage.distance_transform_edt(~occupied) * RES
traversable = free & (clearance >= CLEAR_NEED)


def xy_to_rc(x, y):
    return ROWS - 1 - int(round((y - OY) / RES)), int(round((x - OX) / RES))


def sample(x, y):
    r, c = xy_to_rc(x, y)
    if not (0 <= r < ROWS and 0 <= c < COLS):
        return None
    return r, c


def los_ok(x0, y0, x1, y1, stop_short=0.45):
    """True if the straight segment is free of occupied/unknown cells (stopping short of target)."""
    L = math.hypot(x1 - x0, y1 - y0)
    if L <= stop_short:
        return False
    n = int(L / 0.02)
    worst = 1e9
    for i in range(n + 1):
        t = i / n
        d = t * L
        if d > L - stop_short:
            break
        r, c = sample(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        if r is None or not free[r, c]:
            return False
        worst = min(worst, clearance[r, c])
    return worst


def pose_report(x, y, px, py):
    rc = sample(x, y)
    if rc is None:
        return 'OUT OF MAP'
    r, c = rc
    worst = los_ok(x, y, px, py)
    return 'traversable=%-5s clearance=%.2f LOS=%s' % (
        bool(traversable[r, c]), clearance[r, c],
        ('clear (min %.2f m)' % worst) if worst is not False else 'BLOCKED')


def candidates(px, py, dist):
    out = []
    for k in range(180):
        ang = 2 * math.pi * k / 180.0
        x = px + dist * math.cos(ang)
        y = py + dist * math.sin(ang)
        rc = sample(x, y)
        if rc is None:
            continue
        r, c = rc
        if not traversable[r, c]:
            continue
        worst = los_ok(x, y, px, py)
        if worst is False:
            continue
        yaw = math.atan2(py - y, px - x)
        out.append((worst, clearance[r, c], x, y, yaw, worst * 0.5 + clearance[r, c] * 0.5))
    out.sort(key=lambda t: -t[5])
    return out


if __name__ == '__main__':
    px, py = float(sys.argv[1]), float(sys.argv[2])
    dists = [float(d) for d in sys.argv[3:]] or [4.0]
    print('person at (%.2f, %.2f)' % (px, py))
    print('\n=== due-north bearing ladder (robot south->north of person, +y side) ===')
    for d in [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0]:
        x, y = px, py + d
        print('  d=%5.1f m pose(%.2f,%.2f) yaw=%.3f : %s' % (d, x, y, -math.pi / 2, pose_report(x, y, px, py)))
    for d in dists:
        cands = candidates(px, py, d)
        print('\n--- distance %.1f m: %d clear-line-of-sight poses ---' % (d, len(cands)))
        for worst, clear, x, y, yaw, score in cands[:4]:
            print('  spawn x=%.2f y=%.2f yaw=%.3f   minLOSclearance=%.2f poseclearance=%.2f'
                  % (x, y, yaw, worst, clear))
