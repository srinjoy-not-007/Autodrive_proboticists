"""Compare two AutoDRIVE simulator Docker images asset-by-asset.

The published lap times we benchmark against were set on a *different* build of
the simulator than the one we develop on, so "6.53 s" is only a target if the
vehicle, the physics settings and the track geometry are the same. This answers
that from the Unity assets themselves rather than by inference.

It extracts the serialized scene (`level0`) and project settings
(`globalgamemanagers`) out of each image and compares:

  * PhysicsManager / TimeManager  — gravity, solver iterations, fixed timestep
  * Rigidbody + WheelCollider     — mass, drag, wheel radius, friction curves
  * the vehicle transform subtree — every part, position and rotation
  * the active track's mesh       — by MD5 of the raw serialized mesh
  * checkpoints and spawn points  — the lap definition and the start pose

Usage (needs UnityPy in the host venv, and the images pulled):
  .venv/bin/python tools/compare_sim_builds.py 2025-cdc-tf-practice 2026-iros-practice

Result on those two, 2026-08-22: everything identical except the vehicle's
editor placement (z 0.74 -> 0.80, overridden by the spawn point at runtime) and
the extra tracks each release adds. The benchmark is directly comparable.
"""

import hashlib
import json
import os
import subprocess
import sys
import tempfile

import UnityPy

IMAGE = 'autodriveecosystem/autodrive_roboracer_sim'
SIM_DIR = '/home/autodrive_simulator/Data'
FILES = ('level0', 'globalgamemanagers', 'sharedassets0.assets')


def extract(tag, dest):
    """Copy the Unity data files out of the image (no need to run it)."""
    out = os.path.join(dest, tag)
    os.makedirs(out, exist_ok=True)
    cid = subprocess.check_output(
        ['docker', 'create', '--entrypoint', '/bin/bash', f'{IMAGE}:{tag}'],
        text=True).strip()
    try:
        for f in FILES:
            subprocess.run(['docker', 'cp', f'{cid}:{SIM_DIR}/{f}', f'{out}/{f}'],
                           check=True, capture_output=True)
    finally:
        subprocess.run(['docker', 'rm', '-f', cid], capture_output=True)
    return out


def strip_pointers(o):
    """PPtr file/path ids are assigned per build and differ by construction."""
    if isinstance(o, dict):
        if set(o.keys()) <= {'m_FileID', 'm_PathID'}:
            return '<PPtr>'
        return {k: strip_pointers(v) for k, v in o.items()
                if k not in ('m_GameObject', 'm_PrefabInstance',
                             'm_PrefabAsset', 'm_CorrespondingSourceObject')}
    if isinstance(o, list):
        return [strip_pointers(x) for x in o]
    return o


def scene(path):
    """Index a level file by object type."""
    env = UnityPy.load(os.path.join(path, 'level0'))
    idx = {}
    for obj in env.objects:
        try:
            idx.setdefault(obj.type.name, {})[obj.path_id] = obj.read_typetree()
        except Exception:
            pass
    return idx


def settings(path):
    env = UnityPy.load(os.path.join(path, 'globalgamemanagers'))
    out = {}
    for obj in env.objects:
        if obj.type.name in ('PhysicsManager', 'TimeManager'):
            out[obj.type.name] = strip_pointers(obj.read_typetree())
    return out


def name_of(idx, transform):
    go = idx['GameObject'].get(transform['m_GameObject']['m_PathID'], {})
    return go.get('m_Name', '?')


def subtree(idx, root_name):
    """Every transform under `root_name`, keyed by path, with its pose."""
    starts = [p for p, t in idx['Transform'].items()
              if name_of(idx, t) == root_name]
    rows = {}
    for s in starts:
        stack = [(s, '')]
        while stack:
            pid, prefix = stack.pop()
            t = idx['Transform'][pid]
            full = f'{prefix}/{name_of(idx, t)}'
            rows[full] = (
                tuple(round(t['m_LocalPosition'][k], 5) for k in 'xyz'),
                tuple(round(t['m_LocalRotation'][k], 5) for k in 'xyzw'),
                tuple(round(t['m_LocalScale'][k], 5) for k in 'xyz'))
            for c in t.get('m_Children', []):
                if c['m_PathID'] in idx['Transform']:
                    stack.append((c['m_PathID'], full))
    return rows


def active_track(idx):
    for d in idx['GameObject'].values():
        n = d.get('m_Name', '')
        if n.endswith('Track') and d.get('m_IsActive'):
            return n
    return None


def track_mesh_md5(path, idx, track_name):
    """MD5 of the raw serialized mesh the active track renders and collides with."""
    go_id = next((p for p, g in idx['GameObject'].items()
                  if g.get('m_Name') == track_name), None)
    if go_id is None:
        return None, None
    wanted = set()
    for c in idx['GameObject'][go_id]['m_Component']:
        cid = c['component']['m_PathID'] if 'component' in c else c['m_PathID']
        for kind in ('MeshFilter', 'MeshCollider'):
            d = idx.get(kind, {}).get(cid)
            if d:
                wanted.add(d['m_Mesh']['m_PathID'])
    env = UnityPy.load(os.path.join(path, 'sharedassets0.assets'))
    for obj in env.objects:
        if obj.path_id in wanted and obj.type.name == 'Mesh':
            raw = obj.get_raw_data()
            return hashlib.md5(raw).hexdigest(), len(raw)
    return None, None


def friction_limits(wheel):
    """Unity scales friction-curve values by wheel load, so value*stiffness is mu."""
    lat, lon = wheel['m_SidewaysFriction'], wheel['m_ForwardFriction']
    peak = lat['m_ExtremumValue'] * lat['m_Stiffness']
    tail = lat['m_AsymptoteValue'] * lat['m_Stiffness']
    return {
        'mu_lat_peak': round(peak, 3),
        'ay_peak_m_s2': round(peak * 9.81, 2),
        'peak_at_slip': lat['m_ExtremumSlip'],
        'mu_lat_asymptote': round(tail, 3),
        'ay_asymptote_m_s2': round(tail * 9.81, 2),
        'asymptote_at_slip': lat['m_AsymptoteSlip'],
        'mu_lon_peak': round(lon['m_ExtremumValue'] * lon['m_Stiffness'], 3),
    }


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    tags = sys.argv[1:3]
    tmp = tempfile.mkdtemp(prefix='simdiff_')
    paths, idxs = {}, {}
    for t in tags:
        print(f'extracting {t} ...')
        paths[t] = extract(t, tmp)
        idxs[t] = scene(paths[t])

    def compare(label, values):
        a, b = (json.dumps(values[t], sort_keys=True, default=str) for t in tags)
        print(f'  {label:24s} {"IDENTICAL" if a == b else "DIFFERS":10s}')
        return a == b

    print('\n=== project settings ===')
    compare('physics + time', {t: settings(paths[t]) for t in tags})

    print('\n=== vehicle ===')
    for kind in ('Rigidbody', 'WheelCollider'):
        compare(kind, {t: sorted(json.dumps(strip_pointers(d), sort_keys=True)
                                 for d in idxs[t].get(kind, {}).values())
                       for t in tags})
    subs = {t: subtree(idxs[t], 'Roboracer') for t in tags}
    keys = set(subs[tags[0]]) | set(subs[tags[1]])
    moved = [k for k in keys if subs[tags[0]].get(k) != subs[tags[1]].get(k)]
    print(f'  {"transform subtree":24s} {len(keys)} parts, {len(moved)} differing')
    for k in sorted(moved):
        print(f'      {k}')
        for t in tags:
            print(f'        {t}: {subs[t].get(k)}')

    print('\n=== grip implied by the tyre model ===')
    for t in tags:
        wheel = next(iter(idxs[t]['WheelCollider'].values()))
        print(f'  {t}: {friction_limits(wheel)}')

    print('\n=== track ===')
    for t in tags:
        name = active_track(idxs[t])
        md5, size = track_mesh_md5(paths[t], idxs[t], name)
        print(f'  {t}: active={name!r} mesh md5={md5} ({size} B)')
    for what in ('Checkpoints', 'Spawn Points'):
        name = active_track(idxs[tags[0]])
        root = name.replace('Track', what.split()[0] if what == 'Checkpoints' else 'Spawn Points')
        root = f'{name.rsplit(" ", 1)[0]} {what}'
        compare(what, {t: subtree(idxs[t], root) for t in tags})

    print(f'\n(extracted assets left in {tmp})')


if __name__ == '__main__':
    main()
