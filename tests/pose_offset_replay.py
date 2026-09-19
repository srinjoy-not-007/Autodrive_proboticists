#!/usr/bin/env python3
"""Closed replay of the particle filter against a REAR-AXLE truth trajectory.

Not a pytest module (nothing here is named `test_*`): this is the measurement
harness behind the `pose_along_offset_m` / `lidar_offset_override` knobs, kept
next to `test_localization_audit.py` because it reuses that file's `Caster` and
node plumbing.  `tests/test_pose_offset.py` is the regression suite.

What it does, and why it is the only offline way to answer the question:

* the race logs carry no scans, so the only replay possible is a CLOSED one --
  ray-cast the map the filter localizes in.  That cannot see a mesh/simulator
  mismatch, but it *can* see the lever arm, the likelihood field and the map
  discretisation, which is exactly the trio the Sep 19 audit blamed for the
  corner-outward lateral bias;
* the geometry is set up the way the real vehicle is, which the audit's own
  replay was not: the truth point is the **rear axle** (that is what the IPS
  topic publishes -- measured, see `test_localization_audit`'s docstring) and
  the LiDAR is cast from `truth + LIDAR_FROM_REAR_AXLE (0.2733 m)` along the
  heading.  A filter told the lever arm is 0.1933 m therefore converges on a
  point 0.08 m ahead of the truth, which is the live finding reproduced.

Variants compared, all on the same stream and the same RNG seed:

    base   the shipping filter                 (lever arm 0.1933, no shift)
    shift  `pose_along_offset_m:=0.08`         (output moved back 0.08 m)
    lever  `lidar_offset_override:=0.2733`     (the filter's own lever arm)

Usage (host rule: pin it):

    .venv/bin/python tests/pose_offset_replay.py \
        --log data/logs/race_20260919_044701.csv --laps 6
"""

import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, 'tests')
SRC = os.environ.get('PIT_CREW_SRC') or os.path.join(ROOT, 'src')
DATA = os.environ.get('PIT_CREW_DATA') or os.path.join(ROOT, 'data')
TOOLS = os.path.join(ROOT, 'tools')

sys.path.insert(0, TESTS)
sys.path.insert(0, TOOLS)
import rclpy_stub                                             # noqa: E402

rclpy_stub.install()
sys.path.insert(0, SRC)

from rclpy_stub import Imu, JointState, LaserScan, Time       # noqa: E402
from pit_crew import vehicle                                  # noqa: E402
import racelog                                                # noqa: E402

MAP_PREFIX = os.path.join(DATA, 'maps', 'finals_mesh')
LINE = os.path.join(DATA, 'racelines',
                    'finals_mesh_exitm_m10t6_h3v86_race.csv')

# The real sensor, not the audit's coarse stand-in: the filter subsamples
# 1080 -> 60 beams exactly as it does live, so the beam geometry that feeds
# the likelihood field is the shipped one.
N_BEAM = vehicle.LIDAR_N
A_MIN = vehicle.LIDAR_ANGLE_MIN
A_INC = vehicle.LIDAR_ANGLE_INC
R_MIN, R_MAX = vehicle.LIDAR_RANGE_MIN, vehicle.LIDAR_RANGE_MAX
WHEEL_R = vehicle.WHEEL_RADIUS

# Shipping localization arguments (submission/autodrive_devkit.sh).
SHIP = dict(max_step_mps=9.0, publish_on_reloc=False)

VARIANTS = {
    'base': {},
    'shift': dict(pose_along_offset_m=vehicle.IPS_FROM_REAR_AXLE),
    'lever': dict(lidar_offset_override=vehicle.LIDAR_FROM_REAR_AXLE),
    'both': dict(lidar_offset_override=vehicle.LIDAR_FROM_REAR_AXLE,
                 pose_along_offset_m=vehicle.IPS_FROM_REAR_AXLE),
}

# Segments named the way the corner audit names them (s on the shipping line).
SEGMENTS = [
    ('straight s 2-15', 2.0, 15.0),
    ('hairpin s 18-21', 17.5, 21.5),
    ('corner s 24-27', 23.0, 27.5),
    ('  apex s 24-25', 24.0, 25.0),
    ('kinks s 30-35', 29.0, 35.5),
    ('corner s 36-40', 35.5, 40.0),
    ('hairpin s 42-45', 41.5, 44.9),
    ('  apex s 43.2-44.2', 43.2, 44.2),
]


class Caster:
    """Range image against the occupancy grid the filter itself loads."""

    def __init__(self, prefix):
        import yaml
        from PIL import Image
        with open(prefix + '.yaml') as f:
            meta = yaml.safe_load(f)
        img = np.flipud(np.array(Image.open(prefix + '.pgm')))
        self.res = float(meta['resolution'])
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        self.occ = img <= 50
        self.H, self.W = img.shape
        self.t = np.arange(int(R_MAX / (self.res * 0.5)) + 1) * self.res * 0.5
        self.ang = A_MIN + np.arange(N_BEAM) * A_INC

    def cast(self, x, y, yaw):
        a = yaw + self.ang
        px = x + np.cos(a)[:, None] * self.t[None, :]
        py = y + np.sin(a)[:, None] * self.t[None, :]
        gx = np.floor((px - self.ox) / self.res).astype(np.int64)
        gy = np.floor((py - self.oy) / self.res).astype(np.int64)
        bad = (gx < 0) | (gx >= self.W) | (gy < 0) | (gy >= self.H)
        np.clip(gx, 0, self.W - 1, out=gx)
        np.clip(gy, 0, self.H - 1, out=gy)
        hit = self.occ[gy, gx] | bad
        hit[:, self.t < R_MIN] = False
        first = np.argmax(hit, axis=1)
        r = np.where(hit[np.arange(N_BEAM), first], self.t[first], np.inf)
        return r.tolist()


def truth_stream(log_path, laps, skip_laps=2):
    """Rear-axle truth (t, x, y, yaw, v) at the log's own sensor frames.

    Only rows carrying a FRESH IPS pose are kept -- the logger writes faster
    than the bridge, so the duplicated rows are not sensor frames and their
    intervals are not the sensor period.
    """
    log = racelog.load_log(log_path)
    fresh = np.flatnonzero(racelog.dedupe_ips_mask(log))
    lap = log['lap'][fresh]
    first = int(lap.min()) + skip_laps
    sel = fresh[(lap >= first) & (lap < first + laps)]
    if len(sel) < 50:
        raise SystemExit(f'{log_path}: only {len(sel)} frames in laps '
                         f'{first}..{first + laps}')
    t = log['t'][sel]
    x, y = log['truth_x'][sel], log['truth_y'][sel]
    # Heading from the path tangent. The audit measured ~0 sideslip at the
    # truth point through both hairpins, which is what a rear-axle reference
    # gives, so the tangent IS the heading to within the noise.
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    v = np.concatenate([[0.0], np.hypot(np.diff(x), np.diff(y))
                        / np.maximum(np.diff(t), 1e-3)])
    v[0] = v[1]
    return t, x, y, yaw, v


def replay(log_path, laps, seed=7, cast_offset=None, **over):
    """Drive the real node over the closed stream; return published poses.

    `cast_offset` is where the SCANS come from, measured from the truth point
    along its heading.  The physical answer is `LIDAR_FROM_REAR_AXLE` (the
    default), because the truth point is the rear axle.  Passing 0.1933
    instead is the CONTROL: it makes the truth point the IPS point, i.e. the
    geometry the shipping filter assumes, so whatever error survives there is
    the harness's own (likelihood field, grid discretisation, the ray caster's
    quantisation) and not the lever-arm mismatch under test.
    """
    from pit_crew.localization import ParticleFilter
    if cast_offset is None:
        cast_offset = vehicle.LIDAR_FROM_REAR_AXLE
    caster = Caster(MAP_PREFIX)
    t, xs, ys, yaws, vs = truth_stream(log_path, laps)
    params = dict(SHIP)
    params.update(over)
    params.setdefault('map_prefix', MAP_PREFIX)
    params.setdefault('centerline_csv',
                      os.path.join(DATA, 'racelines',
                                   'finals_mesh_centerline.csv'))
    params.setdefault('init_x', float(xs[0]))
    params.setdefault('init_y', float(ys[0]))
    rclpy_stub.reset_clock()
    rclpy_stub.set_overrides(params)
    n = ParticleFilter()
    rclpy_stub.set_overrides({})
    np.random.seed(seed)
    enc = {'L': 0.0, 'R': 0.0}
    out = np.full((len(t), 3), np.nan)
    for k in range(len(t)):
        rclpy_stub.reset_clock()
        rclpy_stub.advance(float(t[k]) - rclpy_stub.now_s() + 0.001)
        stamp = Time(sec=int(t[k]), nanosec=int((t[k] % 1) * 1e9))
        if k:
            for w in ('L', 'R'):
                enc[w] += vs[k] * (t[k] - t[k - 1]) / WHEEL_R
        for w in ('L', 'R'):
            m = JointState()
            m.header.stamp = stamp
            m.position = [enc[w]]
            n.on_enc(w, m)
        im = Imu()
        im.orientation.z = math.sin(yaws[k] / 2.0)
        im.orientation.w = math.cos(yaws[k] / 2.0)
        n.on_imu(im)
        sc = LaserScan()
        sc.header.stamp = stamp
        sc.angle_min = A_MIN
        sc.angle_increment = A_INC
        sc.range_min = R_MIN
        sc.range_max = R_MAX
        # The TRUE LiDAR position: the truth point is the rear axle and the
        # sensor sits 0.2733 m ahead of it.
        sc.ranges = caster.cast(xs[k] + cast_offset * math.cos(yaws[k]),
                                ys[k] + cast_offset * math.sin(yaws[k]),
                                yaws[k])
        before = len(n.pub_pose.msgs)
        n.on_scan(sc)
        if len(n.pub_pose.msgs) > before:
            p = n.pub_pose.msgs[-1]
            out[k] = (p.pose.position.x, p.pose.position.y,
                      2.0 * math.asin(max(-1.0, min(1.0,
                                                    p.pose.orientation.z))))
    return out, (t, xs, ys, yaws, vs)


def errors(out, truth, warm=40):
    """Along-track and lateral error of the published pose vs the truth."""
    t, xs, ys, yaws, vs = truth
    dx, dy = out[:, 0] - xs, out[:, 1] - ys
    along = dx * np.cos(yaws) + dy * np.sin(yaws)
    lat = -dx * np.sin(yaws) + dy * np.cos(yaws)
    ok = np.isfinite(along)
    ok[:warm] = False
    return along, lat, ok


def segment_table(along, lat, ok, s_of_sample):
    rows = []
    for name, s0, s1 in SEGMENTS:
        m = ok & (s_of_sample >= s0) & (s_of_sample < s1)
        if m.sum() < 5:
            continue
        rows.append((name, int(m.sum()),
                     float(np.median(along[m])), float(np.mean(along[m])),
                     float(np.median(lat[m])),
                     float(np.percentile(np.abs(lat[m]), 95))))
    m = ok
    rows.append(('ALL', int(m.sum()), float(np.median(along[m])),
                 float(np.mean(along[m])), float(np.median(lat[m])),
                 float(np.percentile(np.abs(lat[m]), 95))))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--log', action='append', required=True)
    ap.add_argument('--laps', type=int, default=6)
    ap.add_argument('--variants', default='base,shift,lever')
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--cast-offset', type=float, default=None,
                    help='where the scans come from, m ahead of the truth '
                         'point (default: the physical 0.2733; 0.1933 is the '
                         'control that isolates the harness\'s own bias)')
    args = ap.parse_args()

    s_line, xy_line = racelog.load_line(LINE)[:2]
    for log_path in args.log:
        print(f'\n=== {os.path.basename(log_path)}')
        for name in args.variants.split(','):
            out, truth = replay(log_path, args.laps, seed=args.seed,
                                cast_offset=args.cast_offset,
                                **VARIANTS[name])
            along, lat, ok = errors(out, truth)
            idx = racelog.project(xy_line, np.column_stack(truth[1:3]))
            s_of = s_line[idx]
            dt = float(np.median(np.diff(truth[0])))
            print(f'--- {name}  (dt {dt:.4f} s, {int(ok.sum())} poses)')
            print(f'{"segment":>18} {"n":>5} {"along p50":>10} '
                  f'{"along mean":>11} {"lat p50":>9} {"|lat| p95":>10}')
            for row in segment_table(along, lat, ok, s_of):
                print(f'{row[0]:>18} {row[1]:5d} {row[2]:+10.3f} '
                      f'{row[3]:+11.3f} {row[4]:+9.3f} {row[5]:10.3f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
