#!/usr/bin/env python3
"""Extract an ordered centerline + track widths from an occupancy map.

Topology-based (no clean lap required):
  1. seal wall gaps (dilate occupied), flood-fill corridor from trajectory
  2. skeletonize corridor, prune spurs -> the remaining cycle IS the centerline
  3. trace the cycle, smooth with a periodic spline, resample uniformly
  4. cast normals to the corridor boundary for left/right widths
  5. orient to match the recorded driving direction

Output: TUMFTM-format CSV (x_m,y_m,w_tr_right_m,w_tr_left_m) + QC plot.

Usage: .venv/bin/python tools/extract_centerline.py data/maps/track_v1 [out_prefix]
"""

import math
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splprep, splev
from skimage.morphology import skeletonize

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_map(prefix):
    with open(prefix + '.yaml') as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(prefix + '.pgm'))
    img = np.flipud(img)
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    return img >= 250, img <= 50, res, ox, oy


def load_traj(csv_path):
    d = np.genfromtxt(csv_path, delimiter=',', names=True)
    return d['x'], d['y']


def prune_to_cycle(skel):
    """Iteratively remove endpoints until only cycles remain."""
    k = np.ones((3, 3))
    s = skel.copy()
    for _ in range(500):
        nb = ndimage.convolve(s.astype(np.uint8), k, mode='constant') - s
        endpoints = s & (nb <= 1)
        if not endpoints.any():
            break
        s = s & ~endpoints
    return s


def trace_via_waypoints(skel, tx, ty, ox, oy, res, n_wp=10):
    """Order the centerline by chaining skeleton shortest paths through
    waypoints taken from the driven trajectory (in driving order).

    Robust to spurs (shortest paths ignore them) and to leak shortcuts
    (waypoints force the path around the real circuit).
    """
    from scipy.sparse import lil_matrix
    from scipy.sparse.csgraph import dijkstra
    from scipy.spatial import cKDTree

    ys, xs = np.nonzero(skel)
    pts = np.stack([ys, xs], axis=1)
    idx_of = {tuple(p): i for i, p in enumerate(pts)}
    n = len(pts)
    g = lil_matrix((n, n))
    for i, (y, x) in enumerate(pts):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                j = idx_of.get((y + dy, x + dx))
                if j is not None:
                    g[i, j] = math.hypot(dy, dx)
    g = g.tocsr()
    tree = cKDTree(pts[:, ::-1])            # (x, y) order for queries

    # Waypoints: spread along the FIRST LAP of the trajectory by cumulative
    # path length, skipping teleport jumps. Lap end = first return near the
    # start after having travelled away.
    seg = np.hypot(np.diff(tx), np.diff(ty))
    seg[seg > 0.5] = 0.0
    s = np.concatenate([[0], np.cumsum(seg)])
    total = s[-1]
    lap_end = total
    away = False
    for i in range(1, len(tx)):
        d0 = math.hypot(tx[i] - tx[0], ty[i] - ty[0])
        if d0 > 3.0:
            away = True
        elif away and d0 < 0.6 and s[i] > 5.0:
            lap_end = s[i]
            break
    print(f'trajectory: {total:.1f} m total, first lap ≈ {lap_end:.1f} m')
    wp_idx = [int(np.searchsorted(s, lap_end * k / n_wp)) for k in range(n_wp)]
    wp_nodes = []
    for i in wp_idx:
        px = (tx[min(i, len(tx) - 1)] - ox) / res
        py = (ty[min(i, len(ty) - 1)] - oy) / res
        _, node = tree.query([px, py])
        if not wp_nodes or node != wp_nodes[-1]:
            wp_nodes.append(node)

    order = []
    hops = wp_nodes + [wp_nodes[0]]
    for a, b in zip(hops[:-1], hops[1:]):
        _, pred = dijkstra(g, indices=a, return_predecessors=True)
        if pred[b] < 0:
            print(f'warning: no skeleton path between waypoints, skipping hop')
            continue
        path = [b]
        while path[-1] != a:
            path.append(pred[path[-1]])
        order.extend(pts[path[::-1][:-1]])
    return np.array(order, dtype=float)


def centerline_from_trajectory(tx, ty, corridor, res, ox, oy, iters=6):
    """Centre the driven trajectory inside the corridor.

    A global skeleton assumes the drivable area is a simple loop. On a track
    with an inner passage the flood-filled corridor merges regions that are not
    connected in the driving sense, and the skeleton happily cuts across a wall
    (seen on the ICRA circuit: the extracted line crossed the infield and the
    car dutifully drove into it). The driven trajectory, by contrast, is
    evidence of what is actually drivable, so use it as the anchor and only
    push each point sideways towards the middle of its own corridor.
    """
    solid = ~corridor

    def wall_dist(x, y, nx, ny, limit=3.0):
        d = 0.0
        while d < limit:
            d += res
            ix = int(round((x + nx * d - ox) / res))
            iy = int(round((y + ny * d - oy) / res))
            if not (0 <= iy < solid.shape[0] and 0 <= ix < solid.shape[1]) \
                    or solid[iy, ix]:
                return d
        return limit

    # One clean lap, resampled uniformly
    seg = np.hypot(np.diff(tx), np.diff(ty))
    seg[seg > 0.5] = 0.0
    s_cum = np.concatenate([[0], np.cumsum(seg)])
    lap_end, away = s_cum[-1], False
    for i in range(1, len(tx)):
        d0 = math.hypot(tx[i] - tx[0], ty[i] - ty[0])
        if d0 > 3.0:
            away = True
        elif away and d0 < 0.6 and s_cum[i] > 5.0:
            lap_end = s_cum[i]
            break
    n_pts = max(80, int(lap_end / 0.15))
    want = np.linspace(0, lap_end, n_pts, endpoint=False)
    P = np.column_stack([np.interp(want, s_cum, tx), np.interp(want, s_cum, ty)])

    for _ in range(iters):
        d = np.gradient(P, axis=0)
        t = np.hypot(d[:, 0], d[:, 1])
        nx, ny = -d[:, 1] / (t + 1e-9), d[:, 0] / (t + 1e-9)
        shift = np.zeros(len(P))
        for i, (x, y) in enumerate(P):
            wl = wall_dist(x, y, nx[i], ny[i])
            wr = wall_dist(x, y, -nx[i], -ny[i])
            shift[i] = 0.5 * (wl - wr)
        # Half-correction, then smooth. Correcting harder or skipping the
        # smoothing centres the *points* better but makes the line jagged, and
        # a jagged line swings the car's body corners into the wall: measured
        # worst-corner clearance got worse, not better (Porto 0.325 -> 0.152 m,
        # ICRA 0.215 -> 0.071 m). Body sweep, not point position, is what the
        # error budget actually spends.
        P[:, 0] += 0.5 * shift * nx
        P[:, 1] += 0.5 * shift * ny
        # circular smoothing keeps it from ratcheting into corners
        w = 9
        k = np.ones(w) / w
        for c in range(2):
            P[:, c] = np.convolve(np.concatenate([P[-w:, c], P[:, c], P[:w, c]]),
                                  k, mode='same')[w:-w]

    return P


def main():
    prefix = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else prefix + '_centerline'

    free, occ, res, ox, oy = load_map(prefix)
    tx, ty = load_traj(prefix + '_traj.csv')

    # Seal wall gaps and flood the drivable corridor from trajectory seeds
    solid = ndimage.binary_dilation(occ, iterations=3)
    corridor_all = free & ~solid
    lbl, n = ndimage.label(corridor_all)
    seeds = []
    for x, y in zip(tx[::10], ty[::10]):
        ix, iy = int((x - ox) / res), int((y - oy) / res)
        if 0 <= iy < lbl.shape[0] and 0 <= ix < lbl.shape[1] and lbl[iy, ix]:
            seeds.append(lbl[iy, ix])
    if not seeds:
        sys.exit('trajectory does not intersect any free component')
    corridor = lbl == np.bincount(seeds).argmax()
    # Light smoothing of the corridor edge (does not bridge sealed walls)
    corridor = ndimage.binary_closing(corridor, iterations=1)
    print(f'corridor cells: {corridor.sum()} '
          f'({corridor.sum() * res * res:.1f} m^2)')

    if '--skeleton' in sys.argv:
        skel = skeletonize(corridor)
        cyc = prune_to_cycle(skel)
        print(f'skeleton px: {skel.sum()}, after spur pruning: {cyc.sum()}')
        order = trace_via_waypoints(cyc, tx, ty, ox, oy, res)
        pts = np.stack([ox + order[:, 1] * res, oy + order[:, 0] * res], axis=1)
        print(f'centerline (skeleton): {len(pts)} pts')
    else:
        pts = centerline_from_trajectory(tx, ty, corridor, res, ox, oy)
        print(f'centerline (trajectory-anchored): {len(pts)} pts '
              f'(~{np.hypot(*np.diff(pts, axis=0).T).sum():.1f} m loop)')
    win = 15
    kernel = np.ones(win) / win
    for c in range(2):
        pts[:, c] = np.convolve(
            np.concatenate([pts[-win:, c], pts[:, c], pts[:win, c]]),
            kernel, mode='same')[win:-win]
    pts = pts[::3]
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=len(pts) * 0.0008, per=True)
    u = np.linspace(0, 1, 400, endpoint=False)
    cx, cy = splev(u, tck)
    dcx, dcy = splev(u, tck, der=1)

    # Match recorded driving direction
    i_near = int(np.argmin((np.array(cx) - tx[0]) ** 2 +
                           (np.array(cy) - ty[0]) ** 2))
    j = min(20, len(tx) - 1)
    drive = np.array([tx[j] - tx[0], ty[j] - ty[0]])
    tang = np.array([dcx[i_near], dcy[i_near]])
    if float(np.dot(drive, tang)) < 0:
        cx, cy = cx[::-1], cy[::-1]
        dcx, dcy = -dcx[::-1], -dcy[::-1]
        print('flipped direction to match driving direction')

    boundary = ~corridor
    def wall_dist(x, y, nx, ny):
        d = 0.0
        while d < 3.0:
            d += res
            ix = int(round((x + nx * d - ox) / res))
            iy = int(round((y + ny * d - oy) / res))
            if not (0 <= iy < boundary.shape[0] and 0 <= ix < boundary.shape[1]) \
                    or boundary[iy, ix]:
                return d
        return d

    rows = []
    for x, y, dx_, dy_ in zip(cx, cy, dcx, dcy):
        t = math.hypot(dx_, dy_)
        nx, ny = -dy_ / t, dx_ / t
        rows.append([x, y, wall_dist(x, y, -nx, -ny), wall_dist(x, y, nx, ny)])
    rows = np.array(rows)
    print(f'widths: left {rows[:,3].mean():.2f}±{rows[:,3].std():.2f}  '
          f'right {rows[:,2].mean():.2f}±{rows[:,2].std():.2f}  '
          f'min corridor {(rows[:,2]+rows[:,3]).min():.2f} m')

    np.savetxt(out + '.csv', rows, delimiter=',', fmt='%.4f',
               header='x_m,y_m,w_tr_right_m,w_tr_left_m', comments='# ')

    H, W = corridor.shape
    plt.figure(figsize=(6, 10))
    plt.imshow(corridor, origin='lower', cmap='gray',
               extent=[ox, ox + W * res, oy, oy + H * res])
    plt.plot(tx, ty, 'c.', ms=0.8, label='trajectory')
    plt.plot(cx, cy, 'r-', lw=1.4, label='centerline')
    step = len(rows) // 30
    for i in range(0, len(rows), step):
        x, y, wr, wl = rows[i]
        t = math.hypot(dcx[i], dcy[i])
        nx, ny = -dcy[i] / t, dcx[i] / t
        plt.plot([x - nx * wr, x + nx * wl], [y - ny * wr, y + ny * wl],
                 'y-', lw=0.7)
    plt.plot(cx[0], cy[0], 'g*', ms=12, label='s=0')
    plt.legend()
    plt.title('centerline + widths QC')
    plt.savefig(out + '_qc.png', dpi=130, bbox_inches='tight')
    print(f'wrote {out}.csv and QC plot')


if __name__ == '__main__':
    main()
