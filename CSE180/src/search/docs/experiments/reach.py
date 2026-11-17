"""Reachability analysis of the warehouse map with a robot-radius clearance constraint."""
import numpy as np
from PIL import Image
from scipy import ndimage

PATH = '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.pgm'
RES = 0.03
OX, OY = -15.1, -25.0
ROBOT_RADIUS = 0.22
SAFETY = 0.08

a = np.array(Image.open(PATH))
H, W = a.shape
vals, counts = np.unique(a, return_counts=True)
print('pixel values (top 6 by count): %s' % sorted(zip(counts, vals), reverse=True)[:6])

occ_prob = (255.0 - a) / 255.0
occupied = occ_prob > 0.65
free = occ_prob < 0.10
print('occupied=%.0f free=%.0f unknown=%.0f' % (occupied.sum(), free.sum(), (~occupied & ~free).sum()))

clearance = ndimage.distance_transform_edt(~occupied) * RES
print('clearance stats: max=%.2f m' % clearance.max())

def rc(x, y):
    return H - 1 - int(round((y - OY) / RES)), int(round((x - OX) / RES))

need = ROBOT_RADIUS + SAFETY
traversable = free & (clearance >= need)
print('traversable cells=%d (free=%d) -> %.1f%% of free space survives a %.2f m clearance' % (
    traversable.sum(), free.sum(), 100.0 * traversable.sum() / free.sum(), need))

lab, n = ndimage.label(traversable, structure=np.ones((3, 3)))
print('connected traversable components=%d' % n)
sizes = ndimage.sum(np.ones_like(lab), lab, index=range(1, n + 1))
order = np.argsort(sizes)[::-1]
for i in order[:8]:
    comp = i + 1
    ys, xs = np.where(lab == comp)
    cx = OX + xs.mean() * RES
    cy = OY + (H - 1 - ys.mean()) * RES
    print('  comp %2d: area=%7.1f m^2 bbox x[%.1f,%.1f] y[%.1f,%.1f] centroid(%.1f,%.1f)' % (
        comp, sizes[i] * RES * RES,
        OX + xs.min() * RES, OX + xs.max() * RES,
        OY + (H - 1 - ys.max()) * RES, OY + (H - 1 - ys.min()) * RES, cx, cy))

sr, sc = rc(2.12, -21.3)
print('start cell traversable=%s comp=%d' % (bool(traversable[sr, sc]), lab[sr, sc]))
reach = lab == lab[sr, sc]
print('reachable from start: %.1f m^2 (%.1f%% of traversable)' % (
    reach.sum() * RES * RES, 100.0 * reach.sum() / max(traversable.sum(), 1)))

def check(name, x, y):
    r, c = rc(x, y)
    if not (0 <= r < H and 0 <= c < W):
        print('%-24s OUT OF MAP' % name)
        return
    # nearest reachable-free cell within 3 m
    win = 100
    r0, r1 = max(0, r - win), min(H, r + win + 1)
    c0, c1 = max(0, c - win), min(W, c + win + 1)
    sub = reach[r0:r1, c0:c1]
    if sub.any():
        ys, xs = np.where(sub)
        d = np.hypot((ys + r0 - r) * RES, (xs + c0 - c) * RES)
        k = d.argmin()
        print('%-24s map(%.2f,%.2f) clearance=%.2f free=%s reachable=%s nearest reachable %.2f m away at (%.2f,%.2f)' % (
            name, x, y, clearance[r, c], bool(free[r, c]), bool(reach[r, c]), d[k],
            OX + (xs[k] + c0) * RES, OY + (H - 1 - (ys[k] + r0)) * RES))
    else:
        print('%-24s map(%.2f,%.2f) clearance=%.2f free=%s reachable=%s NO reachable cell within 3 m' % (
            name, x, y, clearance[r, c], bool(free[r, c]), bool(reach[r, c])))

check('person 1 (1,-1)', 1.0, -1.0)
check('person 2 (-12,15)', -12.0, 15.0)
check('robot start', 2.12, -21.3)
check('center aisle', 0.0, 0.0)
check('aisle x=-8.3,y=-10', -8.3, -10.0)

# Fine view (0.3 m/char) of reachable vs traversable vs wall
step = 10
lines = []
marks = {'S': (2.12, -21.3), '1': (1.0, -1.0), '2': (-12.0, 15.0)}
grid = {rc(x, y)[0] // step * 1: None for x, y in []}
grid = {}
for ch, (x, y) in marks.items():
    r, c = rc(x, y)
    grid[(r // step, c // step)] = ch
for r in range(0, H, step):
    line = []
    for c in range(0, W, step):
        blk_r = reach[r:r + step, c:c + step]
        blk_o = occupied[r:r + step, c:c + step]
        blk_t = traversable[r:r + step, c:c + step]
        ch = '.'
        if blk_o.mean() > 0.2:
            ch = '#'
        elif blk_r.mean() > 0.5:
            ch = 'R'
        elif blk_t.mean() > 0.5:
            ch = 'x'
        elif free[r:r + step, c:c + step].mean() > 0.5:
            ch = 'o'
        else:
            ch = ' '
        if (r // step, c // step) in grid:
            ch = grid[(r // step, c // step)]
        line.append(ch)
    lines.append(''.join(line))
print('FINE MAP: 1 char = %.2f m ; R=reachable from start, x=traversable-but-unreachable, o=free-but-too-narrow, #=wall/obstacle' % (step * RES))
for i, line in enumerate(lines):
    y = OY + H * RES - i * step * RES
    print('%7.1f %s' % (y, line))
print('x from %.1f m, 1 char = %.2f m' % (OX, step * RES))
