"""Inspect the warehouse occupancy map: size, free space, coarse ASCII view."""
import math
import numpy as np
from PIL import Image

PATH = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.pgm'
RES = 0.03
OX, OY = -15.1, -25.0

im = Image.open(PATH)
a = np.array(im)
print('pgm mode=%s size=%s dtype=%s' % (im.mode, a.shape, a.dtype))
H, W = a.shape
print('cells: W=%d H=%d -> %.1f m x %.1f m' % (W, H, W * RES, H * RES))
print('map x range: %.2f .. %.2f   y range: %.2f .. %.2f' % (
    OX, OX + W * RES, OY, OY + H * RES))

occupied = a < 100
free = a > 200
unknown = ~occupied & ~free
print('occupied=%d free=%d unknown=%d (of %d)' % (occupied.sum(), free.sum(), unknown.sum(), a.size))

# world -> pixel: col = (x - OX)/RES ; row = H-1 - (y - OY)/RES
def xy_to_rc(x, y):
    c = int((x - OX) / RES)
    r = H - 1 - int((y - OY) / RES)
    return r, c

def describe(name, x, y):
    r, c = xy_to_rc(x, y)
    if 0 <= r < H and 0 <= c < W:
        val = a[r, c]
        kind = 'occupied' if occupied[r, c] else ('free' if free[r, c] else 'unknown')
        # nearest free cell distance
        print('%-22s map(%7.2f,%7.2f) pixel(r=%4d,c=%4d) value=%3d -> %s' % (name, x, y, r, c, val, kind))
    else:
        print('%-22s map(%7.2f,%7.2f) OUT OF BOUNDS pixel(%d,%d)' % (name, x, y, r, c))

describe('robot start', 2.12, -21.3)
describe('person 1 (standing)', 1.0, -1.0)
describe('person 2 (walking)', -12.0, 15.0)
describe('map origin', OX, OY)
describe('map center', OX + W * RES / 2, OY + H * RES / 2)

# ASCII view, ~0.45 m per char
step = 15
rows = []
marks = {'S': (2.12, -21.3), '1': (1.0, -1.0), '2': (-12.0, 15.0)}
grid = {}
for ch, (x, y) in marks.items():
    r, c = xy_to_rc(x, y)
    grid[(r // step, c // step)] = ch
for r in range(0, H, step):
    line = []
    for c in range(0, W, step):
        blk_occ = occupied[r:r + step, c:c + step].mean()
        blk_free = free[r:r + step, c:c + step].mean()
        if (r // step, c // step) in grid:
            line.append(grid[(r // step, c // step)])
        elif blk_occ > 0.12:
            line.append('#')
        elif blk_free > 0.5:
            line.append('.')
        else:
            line.append(' ')
    rows.append(''.join(line))
print('ASCII: each char = %.2f m, y increases upward (top row = y max = %.1f)' % (step * RES, OY + H * RES))
for i, line in enumerate(rows):
    y = OY + H * RES - i * step * RES
    print('%7.1f %s' % (y, line))
print('x axis starts at %.1f m, each char %.2f m' % (OX, step * RES))
