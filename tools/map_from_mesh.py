#!/usr/bin/env python3
"""Build an exact occupancy grid straight from the simulator's track mesh.

Every map we have raced against so far came from a recon lap: drive the car
round, ray-trace the LiDAR from the IPS poses, accumulate log-odds
(`src/tools/build_map_from_bag.py`). That map is only as good as the recon --
it inherits the pose error, the teleports a colliding recon lap produces, and
it only knows the walls the LiDAR happened to see. This tool skips all of that
and rasterises the *actual collision geometry* out of the Unity scene, which is
ground truth by construction.

What it does
------------
1. Copies `level0`, `sharedassets0.assets(.resS)` out of the sim image with a
   throwaway `docker create` + `docker cp` (never touches a running container),
   caching them so repeat runs are free.
2. Finds the `<name> Track` GameObject in the scene, composes the **full parent
   transform chain** -- the tracks live under an `Infrastructure` root at Unity
   (-10, 0, -14), so ignoring it puts every wall 10 by 14 m out of place -- and
   applies translation, rotation and scale to the mesh vertices.
3. Maps Unity to the ROS/IPS world frame the racelines live in:
   `ros_x = unity_z`, `ros_y = -unity_x`, height = `unity_y`.
4. Projects the wall triangles onto the ground plane and rasterises them at
   0.025 m into the same `.pgm` / `.yaml` map_server pair the recon maps use
   (0 occupied, 254 free, 205 unknown; row 0 = max y).
5. Flood-fills the drivable corridor from the track's spawn point so the inside
   of the loop and the outside of it stay `unknown`, exactly like a recon map.

What counts as a wall
---------------------
All of it. Each of the six circuits in the 2026 image is a single mesh made of
**two closed tubes** -- the inner and outer boundary, radius 0.127 m (Porto,
Berlin) or 0.152 m (the 2024/2025 circuits), resting on the ground with their
axis at the track root's y. There is no road surface in the mesh at all; the
car drives on the separate scene-wide `Floor` plane. So there are no floor
faces to reject, and the correct collision footprint is the tube's full
silhouette -- its widest horizontal section, which is what projecting every
triangle gives you. `--band lo hi` slices a height band instead if a future
track ever does carry a floor (sanity-check `--report` first: it prints the
mesh's height extent and the near-horizontal face fraction).

Usage
-----
  .venv/bin/python tools/map_from_mesh.py porto data/maps/porto_mesh
  .venv/bin/python tools/map_from_mesh.py "2025 ICRA" data/maps/icra2025_mesh
  .venv/bin/python tools/map_from_mesh.py --list

Writes `<out>.pgm`, `<out>.yaml` and `<out>_checkpoints.csv` (the lap
definition and spawn poses, in the ROS frame).
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

import UnityPy
from UnityPy.helpers.MeshHelper import MeshHandler

IMAGE = 'autodriveecosystem/autodrive_roboracer_sim'
SIM_DIR = '/home/autodrive_simulator/Data'
FILES = ('level0', 'sharedassets0.assets', 'sharedassets0.assets.resS')

RES = 0.025          # m / cell, same as every recon map
PAD = 0.5            # m of margin around the mesh bounding box
SUPER = 4            # subsamples per cell per axis when rasterising
COVER = 0.5          # cell is occupied when this fraction of it is solid

FREE, OCC, UNKNOWN = 254, 0, 205

# A mesh map is true geometry, so a particle filter running against it should
# cast its rays from where the LiDAR physically is (pit_crew.vehicle:
# LIDAR_FROM_POSE). Recon maps stamp whatever offset they were *built* with;
# this one stamps the physically correct value.
LIDAR_FROM_POSE = 0.1933


# --------------------------------------------------------------------------
# asset extraction
# --------------------------------------------------------------------------

def extract(tag, dest):
    """Copy the Unity data files out of the image without running it.

    `docker create` makes a container that is never started, so this cannot
    disturb a live sim; the container is removed again immediately.
    """
    out = os.path.join(dest, tag)
    os.makedirs(out, exist_ok=True)
    if all(os.path.exists(os.path.join(out, f)) for f in FILES):
        return out
    cid = subprocess.check_output(
        ['docker', 'create', '--entrypoint', '/bin/bash', f'{IMAGE}:{tag}'],
        text=True).strip()
    try:
        for f in FILES:
            subprocess.run(['docker', 'cp', f'{cid}:{SIM_DIR}/{f}',
                            os.path.join(out, f)],
                           check=True, capture_output=True)
    finally:
        subprocess.run(['docker', 'rm', '-f', cid], capture_output=True)
    return out


# --------------------------------------------------------------------------
# scene graph
# --------------------------------------------------------------------------

def load_scene(path):
    env = UnityPy.load(os.path.join(path, 'level0'))
    idx = {}
    for obj in env.objects:
        try:
            idx.setdefault(obj.type.name, {})[obj.path_id] = obj.read_typetree()
        except Exception:
            pass
    return idx


def go_of(idx, transform):
    return idx['GameObject'].get(transform['m_GameObject']['m_PathID'], {})


def name_of(idx, transform):
    return go_of(idx, transform).get('m_Name', '?')


def quat_matrix(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def local_matrix(t):
    """The 4x4 TRS of one transform, Unity convention (scale, then rot, then pos)."""
    m = np.eye(4)
    s = t['m_LocalScale']
    m[:3, :3] = quat_matrix(t['m_LocalRotation']) @ np.diag(
        [s['x'], s['y'], s['z']])
    p = t['m_LocalPosition']
    m[:3, 3] = [p['x'], p['y'], p['z']]
    return m


def world_matrix(idx, path_id):
    """Compose every transform from the scene root down to `path_id`."""
    chain = []
    pid = path_id
    while pid and pid in idx['Transform']:
        t = idx['Transform'][pid]
        chain.append(t)
        pid = t['m_Father']['m_PathID']
    m = np.eye(4)
    for t in reversed(chain):
        m = m @ local_matrix(t)
    return m


def transform_of(idx, go_path_id):
    for pid, t in idx['Transform'].items():
        if t['m_GameObject']['m_PathID'] == go_path_id:
            return pid
    return None


def find_tracks(idx):
    """Every `... Track` GameObject in the scene, by name."""
    return {g['m_Name']: pid for pid, g in idx['GameObject'].items()
            if g.get('m_Name', '').endswith('Track')}


def mesh_id_of(idx, go_path_id):
    """The mesh the object collides with (MeshCollider), else the one it draws."""
    g = idx['GameObject'][go_path_id]
    collider = renderer = None
    for c in g.get('m_Component', []):
        cid = c['component']['m_PathID'] if 'component' in c else c['m_PathID']
        if cid in idx.get('MeshCollider', {}):
            collider = idx['MeshCollider'][cid]['m_Mesh']['m_PathID']
        elif cid in idx.get('MeshFilter', {}):
            renderer = idx['MeshFilter'][cid]['m_Mesh']['m_PathID']
    return collider or renderer


def read_mesh(path, mesh_path_id):
    env = UnityPy.load(os.path.join(path, 'sharedassets0.assets'))
    for obj in env.objects:
        if obj.type.name == 'Mesh' and obj.path_id == mesh_path_id:
            h = MeshHandler(obj.read())
            h.process()
            verts = np.asarray(h.m_Vertices, dtype=np.float64).reshape(-1, 3)
            tris = np.asarray([t for sub in h.get_triangles() for t in sub],
                              dtype=np.int64)
            return verts, tris
    raise SystemExit(f'mesh {mesh_path_id} not found in sharedassets0.assets')


# --------------------------------------------------------------------------
# frames
# --------------------------------------------------------------------------

def unity_to_ros(p):
    """ROS x = Unity z, ROS y = -Unity x. Height (Unity y) is returned third."""
    out = np.empty_like(p)
    out[..., 0] = p[..., 2]
    out[..., 1] = -p[..., 0]
    out[..., 2] = p[..., 1]
    return out


def ros_yaw(mat):
    """Heading of a Unity transform, expressed in the ROS frame."""
    f = mat[:3, :3] @ np.array([0.0, 0.0, 1.0])     # Unity forward
    return math.atan2(-f[0], f[2])


def pose_rows(idx, root_name, kind):
    """(name, x, y, yaw, width) for every child of a Checkpoints/Spawn root."""
    rows = []
    for pid, t in idx['Transform'].items():
        if name_of(idx, t) != root_name:
            continue
        for c in t.get('m_Children', []):
            cid = c['m_PathID']
            if cid not in idx['Transform']:
                continue
            m = world_matrix(idx, cid)
            p = unity_to_ros(m[:3, 3])
            s = idx['Transform'][cid]['m_LocalScale']
            rows.append((kind, name_of(idx, idx['Transform'][cid]),
                         p[0], p[1], ros_yaw(m), s['x']))
    return rows


# --------------------------------------------------------------------------
# rasterisation
# --------------------------------------------------------------------------

def rasterise(xy_tris, xmin, ymin, W, H, res, sup, cover):
    """Fill the projected triangles into a WxH occupancy mask.

    Drawn `sup` times oversampled and then reduced by coverage, so a cell is
    occupied when at least `cover` of its area is inside the geometry. That
    keeps wall pixels on the real surface rather than dilating them by up to a
    cell, which matters because clearance is read off this map.
    """
    img = Image.new('1', (W * sup, H * sup), 0)
    draw = ImageDraw.Draw(img)
    gx = (xy_tris[:, :, 0] - xmin) / res * sup
    gy = (xy_tris[:, :, 1] - ymin) / res * sup
    for i in range(len(gx)):
        pts = [(gx[i, 0], gy[i, 0]), (gx[i, 1], gy[i, 1]), (gx[i, 2], gy[i, 2])]
        draw.polygon(pts, fill=1, outline=1)
    fine = np.array(img, dtype=np.uint8)
    frac = fine.reshape(H, sup, W, sup).mean(axis=(1, 3))
    return frac >= cover


def build_grid(pts_ros, tris, spawn_xy, res, pad, sup, cover):
    xy = pts_ros[:, :2]
    xmin = math.floor((xy[:, 0].min() - pad) / res) * res
    ymin = math.floor((xy[:, 1].min() - pad) / res) * res
    W = int(math.ceil((xy[:, 0].max() + pad - xmin) / res))
    H = int(math.ceil((xy[:, 1].max() + pad - ymin) / res))

    occ = rasterise(xy[tris], xmin, ymin, W, H, res, sup, cover)

    # Free space = the connected open region the car actually starts in. The
    # inside of the loop and the world outside it stay unknown, which is what a
    # recon map looks like and what the downstream tools expect.
    img = np.full((H, W), UNKNOWN, dtype=np.uint8)
    img[occ] = OCC
    lbl, n = ndimage.label(~occ)
    sx = int((spawn_xy[0] - xmin) / res)
    sy = int((spawn_xy[1] - ymin) / res)
    seed = lbl[sy, sx] if (0 <= sx < W and 0 <= sy < H) else 0
    if seed == 0:
        print('  WARNING: spawn point is not in open space; leaving all free',
              file=sys.stderr)
        img[~occ] = FREE
    else:
        img[lbl == seed] = FREE
    return img, xmin, ymin, W, H


def write_map(out, img, xmin, ymin, res):
    W, H = img.shape[1], img.shape[0]
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out + '.pgm', 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        f.write(np.flipud(img).tobytes())          # pgm row 0 = max y
    with open(out + '.yaml', 'w') as f:
        f.write(f"image: {os.path.basename(out)}.pgm\n"
                f"resolution: {res}\n"
                f"origin: [{xmin}, {ymin}, 0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
                f"lidar_offset: {LIDAR_FROM_POSE}\n"
                f"source: mesh\n")


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('track', nargs='?', help='substring of the track name')
    ap.add_argument('out', nargs='?', help='output prefix, e.g. data/maps/porto_mesh')
    ap.add_argument('--tag', default='2026-iros-practice')
    ap.add_argument('--res', type=float, default=RES)
    ap.add_argument('--pad', type=float, default=PAD)
    ap.add_argument('--supersample', type=int, default=SUPER)
    ap.add_argument('--cover', type=float, default=COVER,
                    help='cell area fraction that must be solid (default 0.5)')
    ap.add_argument('--band', nargs=2, type=float, metavar=('LO', 'HI'),
                    help='keep only faces whose height is within [LO, HI] '
                         'above the ground (default: the whole mesh)')
    ap.add_argument('--cache', default=os.path.join(tempfile.gettempdir(),
                                                    'simassets'))
    ap.add_argument('--list', action='store_true', help='list tracks and exit')
    ap.add_argument('--report', action='store_true',
                    help='print mesh geometry statistics')
    args = ap.parse_args()

    path = extract(args.tag, args.cache)
    idx = load_scene(path)
    tracks = find_tracks(idx)

    if args.list:
        for name, pid in sorted(tracks.items()):
            tid = transform_of(idx, pid)
            m = world_matrix(idx, tid)
            print(f'  {name:26s} active={idx["GameObject"][pid].get("m_IsActive")} '
                  f'world={np.round(m[:3, 3], 4).tolist()} '
                  f'mesh={mesh_id_of(idx, pid)}')
        return
    if not args.track or not args.out:
        ap.error('need a track name substring and an output prefix')

    hits = [n for n in tracks if args.track.lower() in n.lower()]
    if len(hits) != 1:
        sys.exit(f'{args.track!r} matches {hits or "nothing"}; be more specific')
    name = hits[0]
    stem = name.rsplit(' ', 1)[0]           # "SRL 2025 ICRA Track" -> "SRL 2025 ICRA"
    go_id = tracks[name]
    tid = transform_of(idx, go_id)
    mat = world_matrix(idx, tid)
    print(f'{name}: world translation {np.round(mat[:3, 3], 4).tolist()}, '
          f'rotation {"identity" if np.allclose(mat[:3, :3], np.diag(np.diag(mat[:3, :3]))) and np.allclose(np.diag(mat[:3, :3]), 1) else "non-trivial"}')

    verts, tris = read_mesh(path, mesh_id_of(idx, go_id))
    world = verts @ mat[:3, :3].T + mat[:3, 3]
    ros = unity_to_ros(world)
    print(f'  mesh: {len(verts)} vertices, {len(tris)} triangles; '
          f'height {ros[:, 2].min():.3f}..{ros[:, 2].max():.3f} m, '
          f'ROS x {ros[:, 0].min():.2f}..{ros[:, 0].max():.2f}, '
          f'y {ros[:, 1].min():.2f}..{ros[:, 1].max():.2f}')

    keep = np.ones(len(tris), dtype=bool)
    if args.band:
        h = ros[tris][:, :, 2]
        keep = (h.max(axis=1) >= args.band[0]) & (h.min(axis=1) <= args.band[1])
        print(f'  height band {args.band}: kept {keep.sum()}/{len(tris)} faces')
    if args.report:
        p = ros[tris]
        n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
        ln = np.linalg.norm(n, axis=1)
        up = np.abs(n[:, 2]) / np.maximum(ln, 1e-12)
        print(f'  faces: {np.mean(up > 0.9):.1%} near-horizontal, '
              f'{np.mean(up < 0.1):.1%} near-vertical, '
              f'{np.mean(ln < 1e-9):.1%} degenerate')

    spawns = pose_rows(idx, f'{stem} Spawn Points', 'spawn')
    checks = pose_rows(idx, f'{stem} Checkpoints', 'checkpoint')
    if not spawns:
        sys.exit(f'no "{stem} Spawn Points" in the scene')
    spawn = spawns[0]
    print(f'  spawn: ROS ({spawn[2]:.3f}, {spawn[3]:.3f}) yaw {spawn[4]:.3f} rad; '
          f'{len(checks)} checkpoints')

    img, xmin, ymin, W, H = build_grid(ros, tris[keep], (spawn[2], spawn[3]),
                                       args.res, args.pad, args.supersample,
                                       args.cover)
    write_map(args.out, img, xmin, ymin, args.res)
    print(f'  grid {W}x{H} @ {args.res} m, origin [{xmin}, {ymin}]: '
          f'{np.mean(img == OCC):.2%} occupied, {np.mean(img == FREE):.2%} free, '
          f'{np.mean(img == UNKNOWN):.2%} unknown')

    cp = args.out + '_checkpoints.csv'
    with open(cp, 'w') as f:
        f.write('kind,name,x,y,yaw,width\n')
        for k, nm, x, y, yaw, w in spawns + sorted(
                checks, key=lambda r: (r[1].isdigit() and int(r[1])) or 0):
            f.write(f'{k},{nm},{x:.4f},{y:.4f},{yaw:.5f},{w:.3f}\n')
    print(f'  wrote {args.out}.pgm/.yaml and {cp}')


if __name__ == '__main__':
    main()
