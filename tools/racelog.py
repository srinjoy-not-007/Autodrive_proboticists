#!/usr/bin/env python3
"""One loader for race logs and racelines, with unambiguous names.

Two column conventions in these files have produced wrong conclusions more than
once, so every tool that reads a log should come through here rather than
indexing the CSV by hand.

**Race log columns** (`src/tools/race_logger.py`):

* ``x, y`` are the **IPS GROUND TRUTH** pose, published by the simulator. This
  is the opposite of the natural reading of the header, where ``px, py`` look
  like "position"; they are not.
* ``px, py, pyaw, pstamp`` are the **LOCALIZATION ESTIMATE** (``/pit_crew/pose``)
  and the stamp of *when that estimate was true*. Each row carries the last
  received pose, i.e. about one sensor frame of staleness: compare the estimate
  with the truth interpolated at ``pstamp``, never at ``t``.
* ``speed`` is the logger's own filtered **wheel-encoder** speed, not truth.
  Truth speed is the IPS trajectory differentiated (``truth_speed`` below).
* ``steering`` is the simulator's **actuator feedback in RADIANS**; ``cmd`` is
  the **published command, normalized to [-1, 1]** (multiply by
  ``vehicle.MAX_STEER`` = 0.5236 to compare them). Verified two ways: our
  controllers publish ``vehicle.steer_to_cmd()``, which divides by MAX_STEER,
  and the devkit bridge forwards the [-1, 1] value to Unity while republishing
  ``/steering`` from the sim's own ``V1 Steering`` state. Measured on
  ``data/logs/race_20260912_070432.csv``: max |cmd| 0.705 against max |steering|
  0.369 rad = 0.705 * 0.5236, with a per-sample ratio of 0.494 (below MAX_STEER
  only because the rack lags the command).

**Collision attribution -- the c-2 rule.** The logger writes one row from
``on_col`` when ``collision_count`` increments, but the simulator has already
teleported the car back to its checkpoint and published that pose by then. So
for a collision whose counter steps at row ``c``:

* row ``c``   -- the collision row itself, holding the *teleported* IPS pose;
* row ``c-1`` -- the IPS row that first carried the teleported pose;
* row ``c-2`` -- the last **pre-impact** sample, and the only one that says
  where the car actually hit.

Reading ``c-1`` clusters every collision at the checkpoint (this produced a
bogus "all collisions at s 0-1.6" result once). Attribution must also run on the
RAW rows: de-duplicating repeated IPS poses deletes row ``c`` entirely.

**Raceline columns** (``data/racelines/*.csv``): ``s_m, x_m, y_m, psi, kappa,
vx, ax``. So ``line[:, :2]`` is ``(s, x)`` and NOT a position; the xy pair is
``line[:, 1:3]``.
"""

import csv

import numpy as np

MAX_STEER = 0.5236          # rad; see src/pit_crew/vehicle.py
TELEPORT_S = 1.5            # default guard either side of a collision

# CSV column -> name in the returned dict. Required first, then optional ones
# that only newer logs carry.
_REQUIRED = {'t': 't', 'lap_count': 'lap', 'collision_count': 'coll'}
_OPTIONAL = {'speed': 'enc_speed', 'steering': 'steer_fb_rad', 'cmd': 'cmd',
             'throttle': 'throttle', 'lap_time': 'lap_time',
             'last_lap_time': 'last_lap_time', 'best_lap_time': 'best_lap_time',
             'collision_event': 'coll_event', 'cte': 'cte', 'pstamp': 'pstamp',
             'pyaw': 'est_yaw'}


def load_log(path):
    """Race-log columns as numpy arrays under unambiguous names.

    Always present: ``t``, ``truth_xy`` (N, 2 -- the IPS truth), ``truth_x``,
    ``truth_y``, ``lap``, ``coll``, ``n``, and ``columns`` (the CSV header, so a
    caller can tell an absent column from a column of NaNs).

    Present when the log has them: ``est_xy`` (N, 2 -- the localization
    estimate), ``est_yaw``, ``pstamp``, ``enc_speed``, ``steer_fb_rad``, ``cmd``
    (normalized [-1, 1]), ``cmd_rad`` (``cmd * MAX_STEER``), ``throttle``,
    ``lap_time``, ``last_lap_time``, ``best_lap_time``, ``coll_event``, ``cte``.

    Rows are returned RAW and in file order: no de-duplication and no teleport
    filtering, because collision attribution needs the rows the filters delete.
    See ``dedupe_ips_mask`` and ``teleport_mask``.
    """
    with open(path) as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'empty log: {path}')
    header = set(rows[0])

    def col(key):
        out = np.empty(len(rows))
        for i, row in enumerate(rows):
            try:
                out[i] = float(row[key])
            except (TypeError, ValueError):
                out[i] = np.nan
        return out

    missing = [k for k in _REQUIRED if k not in header]
    if missing:
        raise ValueError(f'{path}: not a race log (missing {missing})')
    log = {'columns': header, 'n': len(rows)}
    for key, name in _REQUIRED.items():
        log[name] = col(key)
    log['lap'] = log['lap'].astype(int)
    log['coll'] = log['coll'].astype(int)
    # The IPS ground truth. NOT px/py.
    log['truth_x'], log['truth_y'] = col('x'), col('y')
    log['truth_xy'] = np.column_stack([log['truth_x'], log['truth_y']])
    for key, name in _OPTIONAL.items():
        if key in header:
            log[name] = col(key)
    if 'coll_event' in log:
        log['coll_event'] = log['coll_event'].astype(int)
    if 'cmd' in log:
        log['cmd_rad'] = log['cmd'] * MAX_STEER
    if 'px' in header and 'py' in header:
        # The localization estimate. NOT the truth.
        log['est_xy'] = np.column_stack([col('px'), col('py')])
    return log


def load_line(path):
    """A raceline CSV as ``(s, xy, psi, kappa, v)``; columns s,x,y,psi,kappa,vx,ax."""
    raw = np.loadtxt(path, delimiter=',')
    if raw.ndim != 2 or raw.shape[1] < 6:
        raise ValueError(f'{path}: expected s_m,x_m,y_m,psi,kappa,vx[,ax]')
    return raw[:, 0], raw[:, 1:3], raw[:, 3], raw[:, 4], raw[:, 5]


def project(xy, points):
    """Nearest line-point index for every sample (chunked to stay small)."""
    points = np.asarray(points, float).reshape(-1, 2)
    idx = np.empty(len(points), int)
    for a in range(0, len(points), 2000):
        b = min(a + 2000, len(points))
        d = np.hypot(points[a:b, None, 0] - xy[None, :, 0],
                     points[a:b, None, 1] - xy[None, :, 1])
        idx[a:b] = d.argmin(1)
    return idx


def line_yaw(xy):
    """Travel direction at each line point, by central difference."""
    fwd = np.roll(xy, -1, 0) - np.roll(xy, 1, 0)
    return np.arctan2(fwd[:, 1], fwd[:, 0])


def signed_offset(xy, points, idx=None):
    """Lateral offset of points from the line, + = LEFT of travel."""
    points = np.asarray(points, float).reshape(-1, 2)
    if idx is None:
        idx = project(xy, points)
    yaw = line_yaw(xy)[idx]
    dx, dy = points[:, 0] - xy[idx, 0], points[:, 1] - xy[idx, 1]
    return -np.sin(yaw) * dx + np.cos(yaw) * dy


def dedupe_ips_mask(log):
    """Rows carrying a fresh IPS pose. The logger writes faster than the bridge.

    Drops the collision row too (it repeats the teleported pose), so never use
    this before attributing collisions.
    """
    keep = np.ones(log['n'], bool)
    keep[1:] = ((np.diff(log['truth_x']) != 0)
                | (np.diff(log['truth_y']) != 0))
    return keep


def collision_rows(log):
    """Row indices where ``collision_count`` steps -- the ``c`` of the c-2 rule."""
    return (np.flatnonzero(np.diff(log['coll']) > 0) + 1).astype(int)


def teleport_mask(log, window=TELEPORT_S):
    """True where a sample is within ``window`` seconds of a collision."""
    bad = np.zeros(log['n'], bool)
    for i in collision_rows(log):
        bad |= np.abs(log['t'] - log['t'][i]) < window
    return bad


def truth_speed(log):
    """Speed from IPS differences, held across the logger's duplicated rows.

    The encoder column is a filtered estimate; this is the ground truth. Rows
    that merely repeat the previous pose would read zero, so each takes the
    speed of the last real IPS step.
    """
    fresh = np.flatnonzero(dedupe_ips_mask(log))
    v = np.zeros(log['n'])
    if len(fresh) < 2:
        return v
    x, y, t = log['truth_x'], log['truth_y'], log['t']
    step = np.hypot(np.diff(x[fresh]), np.diff(y[fresh]))
    dt = np.maximum(np.diff(t[fresh]), 1e-3)
    vf = np.concatenate([[step[0] / dt[0]], step / dt])
    # forward-fill onto every row
    src = np.searchsorted(fresh, np.arange(log['n']), 'right') - 1
    return vf[np.clip(src, 0, len(vf) - 1)]


def collisions(log, line=None, line_path=None):
    """One record per collision, attributed to its PRE-IMPACT row (c-2).

    ``line`` is ``(s, xy, ...)`` from ``load_line`` (or just ``(s, xy)``); pass
    ``line_path`` instead to load it here. Without either, the geometric fields
    are omitted.

    Each record has: ``n`` (1-based), ``row`` (c), ``pre_row`` (c-2), ``t``,
    ``lap`` (the lap counter at c-2, i.e. the lap being driven when it hit),
    ``truth_xy``, ``truth_speed`` (from IPS differences), ``enc_speed``,
    ``cte`` (the controller's own reported error, if logged) and, with a line,
    ``s`` and ``offset`` (+ = left of travel).
    """
    if line is None and line_path is not None:
        line = load_line(line_path)
    s = xy = None
    if line is not None:
        s, xy = line[0], line[1]
    v_truth = truth_speed(log)
    out = []
    for k, c in enumerate(collision_rows(log), 1):
        pre = int(max(c - 2, 0))
        rec = dict(n=k, row=int(c), pre_row=pre, t=float(log['t'][pre]),
                   lap=int(log['lap'][pre]),
                   truth_xy=(float(log['truth_x'][pre]),
                             float(log['truth_y'][pre])),
                   truth_speed=float(v_truth[pre]))
        if 'enc_speed' in log:
            rec['enc_speed'] = float(log['enc_speed'][pre])
        if 'cte' in log:
            rec['cte'] = float(log['cte'][pre])
        if xy is not None:
            p = np.array([[log['truth_x'][pre], log['truth_y'][pre]]])
            j = int(project(xy, p)[0])
            rec['s'] = float(s[j])
            rec['offset'] = float(signed_offset(xy, p, np.array([j]))[0])
        out.append(rec)
    return out


def format_collisions(records):
    """The per-collision table as text lines (shared by the CLI tools)."""
    head = (f'{"#":>2} {"lap":>4} {"row":>6} {"s":>7} {"offset":>7} '
            f'{"cte":>7} {"v_true":>7} {"v_enc":>7}')
    lines = [head, '-' * len(head)]
    for r in records:
        lines.append(
            f'{r["n"]:2d} {r["lap"]:4d} {r["pre_row"]:6d} '
            f'{r.get("s", float("nan")):7.2f} '
            f'{r.get("offset", float("nan")):+7.3f} '
            f'{r.get("cte", float("nan")):+7.3f} '
            f'{r["truth_speed"]:7.2f} '
            f'{r.get("enc_speed", float("nan")):7.2f}')
    return lines


def main():
    import argparse
    ap = argparse.ArgumentParser(description='Per-collision attribution (c-2).')
    ap.add_argument('log')
    ap.add_argument('line', nargs='?')
    args = ap.parse_args()
    log = load_log(args.log)
    recs = collisions(log, line_path=args.line)
    if not recs:
        print('no collisions in this log')
        return 0
    print('\n'.join(format_collisions(recs)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
