"""Offline Follow-the-Gap harness: the real node against a ray-cast LiDAR.

Follow-the-Gap is the supervisor's last-resort fallback (`autodrive_devkit.sh`
falls back to it after the race stack dies `MAX_RESTARTS` times), and it is the
one node in the stack that has never been exercised without the simulator: it
needs a LiDAR, and every host test so far has fed it a synthetic scan that is
not a track.  Its two known failure modes -- circulating BACKWARDS (the sim
counts zero laps) and 24 collisions on a narrow circuit -- are both closed-loop
and neither can be reproduced from a canned scan.

What this does:

  * ray-casts `vehicle.LIDAR_N` beams over 270 deg from the LiDAR origin
    (`vehicle.LIDAR_FROM_POSE` = 0.1933 m ahead of the pose point) against the
    map's distance field, by sphere tracing -- the field is the distance to the
    nearest wall, so stepping by it is exact and never tunnels;
  * drives `tools.sim_tracker.Plant`, the same kinematic-bicycle plant with the
    measured actuator (3.2 rad/s slew, transport dead time, speed servo) that
    the MPC harness uses;
  * runs the REAL `pit_crew.follow_the_gap.FollowTheGap` node on top of
    `tests/rclpy_stub`, callback for callback, with the stub clock frozen to
    simulated time so the node's own staleness and timer logic is exercised;
  * counts laps and DIRECTION from the scene's numbered checkpoints (the same
    file the simulator resets to), and collisions from the car footprint
    against the distance field, resetting to the last passed checkpoint the way
    the simulator does.

It is not the simulator: no tyre slip beyond `sim_tracker`'s grip model, no
sensor noise, no render jitter.  It is a geometry and control-loop test, and
its job is to tell us which of the fallback's profiles gets round an unseen
track in the right direction without hitting anything.

Usage:
    .venv/bin/python tools/ftg_sim.py porto_mesh --params src/config/ftg.yaml
    .venv/bin/python tools/ftg_sim.py --all --profile mapping --profile racing
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import rclpy_stub                                              # noqa: E402

rclpy_stub.install()

sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

import sim_tracker as ST                                       # noqa: E402
from pit_crew import vehicle                                   # noqa: E402
from pit_crew.follow_the_gap import FollowTheGap               # noqa: E402

MAPS = os.path.join(ROOT, 'data', 'maps')
FRAME = vehicle.RATE_NOMINAL       # s, the telemetry period FTG was tuned at
DEAD = 0.135                       # s, bridge -> actuator transport delay
TIMER_DT = 0.05                    # the node's own watchdog timer period
SCAN_TRANSPORT = 0.001             # s, stamp -> callback, so `now > stamp`
HALF_DIAG = 0.5 * math.hypot(vehicle.CAR_LENGTH, vehicle.CAR_WIDTH)


# --------------------------------------------------------------- stub clock

_SIM_T = [0.0]


def _now():
    return _SIM_T[0]


rclpy_stub.now_s = _now            # `_Clock.now` resolves this at call time


def _set_time(t):
    _SIM_T[0] = float(t)


def _stamp(msg, t):
    """Stamp a message at `t`, FLOORED to the nanosecond.

    Rounding it up instead costs the whole harness: the node rejects any scan
    whose stamp is in its future (`0 <= now - stamp <= 0.35`), and a stamp one
    nanosecond ahead of the clock made it drop 124 of 288 scans and stand still
    for seconds at a time.  Scans are therefore stamped a millisecond of bridge
    transport BEFORE they are delivered.
    """
    ns = int(t * 1e9)
    msg.header.stamp.sec = ns // 1000000000
    msg.header.stamp.nanosec = ns % 1000000000
    return msg


# ------------------------------------------------------------------- lidar

def raycast(dist, res, ox, oy, x, y, world_angles, max_range, iters=96):
    """Sphere-trace `world_angles` from (x, y); returns ranges in metres.

    `dist` is the Euclidean distance to the nearest occupied cell, so a step of
    `dist` can never cross a wall.  Outside the map the field is unknown, so a
    beam that leaves the grid is free to max range -- every one of our maps is
    a closed circuit with a wide margin, so this only affects beams that have
    already escaped through a gap the real track does not have.
    """
    H, W = dist.shape
    c = np.cos(world_angles)
    s = np.sin(world_angles)
    t = np.zeros(world_angles.size)
    live = np.ones(world_angles.size, bool)
    hit_eps = 0.75 * res
    for _ in range(iters):
        px = x + t * c
        py = y + t * s
        ix = ((px - ox) / res).astype(np.int32)
        iy = ((py - oy) / res).astype(np.int32)
        inside = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        d = np.where(inside, dist[np.clip(iy, 0, H - 1), np.clip(ix, 0, W - 1)],
                     max_range)
        live &= d > hit_eps
        live &= t < max_range
        if not live.any():
            break
        t = np.where(live, t + np.maximum(d, 0.5 * res), t)
    return np.clip(t, vehicle.LIDAR_RANGE_MIN, max_range)


# -------------------------------------------------------------- reference

def load_reference(prefix):
    """Closed reference polyline from the scene's NUMBERED checkpoints.

    These are in racing order by construction (`tools/map_from_mesh.py` writes
    the Unity scene's checkpoint objects out with their names), which is what
    makes direction measurable: progress along this polyline is signed, so a
    car circulating the wrong way scores negative.
    """
    pts = ST.load_checkpoints(prefix)[:, :2]
    pts = np.vstack([pts, pts[:1]])
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s_nodes = np.concatenate([[0.0], np.cumsum(seg)])
    total = s_nodes[-1]
    n = max(int(total / 0.05), 64)
    s_dense = np.linspace(0.0, total, n, endpoint=False)
    xs = np.interp(s_dense, s_nodes, pts[:, 0])
    ys = np.interp(s_dense, s_nodes, pts[:, 1])
    return np.column_stack([xs, ys]), s_dense, total


def spawn_pose(prefix):
    with open(prefix + '_checkpoints.csv') as f:
        for r in csv.DictReader(f):
            if r['kind'] == 'spawn' and r['name'].endswith('1'):
                return float(r['x']), float(r['y']), float(r['yaw'])
    raise SystemExit(f'no spawn pose in {prefix}_checkpoints.csv')


class Progress:
    """Signed arc-length progress around the checkpoint polyline."""

    def __init__(self, prefix):
        self.xy, self.s, self.total = load_reference(prefix)
        self.last = None
        self.net = 0.0
        self.lo = 0.0

    def s_of(self, x, y):
        i = int(np.argmin((self.xy[:, 0] - x) ** 2 + (self.xy[:, 1] - y) ** 2))
        return float(self.s[i])

    def update(self, x, y):
        s = self.s_of(x, y)
        if self.last is None:
            self.last = s
            return 0.0
        d = s - self.last
        half = 0.5 * self.total
        if d > half:
            d -= self.total
        elif d < -half:
            d += self.total
        self.last = s
        self.net += d
        self.lo = min(self.lo, self.net)
        return d


# -------------------------------------------------------------------- run

def load_params(path):
    with open(path) as f:
        doc = yaml.safe_load(f)
    return dict(doc['follow_the_gap']['ros__parameters'])


def load_map(prefix):
    """(distance-to-wall in m, meta), with UNKNOWN cells treated as WALL.

    `pf_core.load_map` counts unknown (205) as free, which is right for a
    particle filter -- an unobserved cell should not veto a particle -- and
    catastrophic here.  The grey outside the circuit and the infield the recon
    lap never saw are unknown, so a ray-cast LiDAR built on that convention
    sees a way out of the track wherever two wall meshes meet at an angle, and
    a reactive planner WILL drive through it: the first version of this harness
    had the car leave the ICRA-2026 circuit at the bottom-left corner and
    circle in the grey for the rest of the run.  Free here means observed free.
    """
    from PIL import Image
    from scipy import ndimage
    with open(prefix + '.yaml') as f:
        meta = yaml.safe_load(f)
    img = np.flipud(np.array(Image.open(prefix + '.pgm')))
    free = img >= 250
    dist = ndimage.distance_transform_edt(free) * float(meta['resolution'])
    return dist, img, meta


def wall_normal(gx, gy, res, ox, oy, x, y):
    """Unit gradient of the distance field: points AWAY from the nearest wall."""
    H, W = gx.shape
    ix = int(np.clip((x - ox) / res, 0, W - 1))
    iy = int(np.clip((y - oy) / res, 0, H - 1))
    nx, ny = float(gx[iy, ix]), float(gy[iy, ix])
    n = math.hypot(nx, ny)
    if n < 1e-6:
        return 0.0, 0.0
    return nx / n, ny / n


def run(prefix, params, laps=10, max_time=600.0, frame=FRAME, dead=DEAD,
        collide_reset=False, verbose=False, trace=None):
    dist, _img, meta = load_map(prefix)
    grad_y, grad_x = np.gradient(dist)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    prog = Progress(prefix)
    checks = ST.load_checkpoints(prefix)
    check_s = np.array([prog.s_of(c[0], c[1]) for c in checks])

    x0, y0, yaw0 = spawn_pose(prefix)
    _set_time(0.0)
    rclpy_stub.set_overrides(params)
    node = FollowTheGap()
    plant = ST.Plant(x0, y0, yaw0, v=0.0, dead=dead, frame=frame,
                     **ST.PLANT_REAL)

    angles = node.angles.copy()
    max_range = vehicle.LIDAR_RANGE_MAX
    from sensor_msgs.msg import Imu, LaserScan

    def deliver_commands(t):
        st = node.stub_published('steering_command')
        th = node.stub_published('throttle_command')
        if not st or not th:
            return
        plant.accept_frame(t, float(st[-1].data) * vehicle.MAX_STEER,
                           float(th[-1].data))
        del st[:-1]
        del th[:-1]

    t = 0.0
    next_frame = 0.0
    next_timer = TIMER_DT
    lap_times = []
    lap_s = 0.0                      # sim time the current lap started
    collisions = 0
    collide_until = -1.0
    yr = 0.0
    last_progress_t = 0.0
    best_net = 0.0
    still = 0.0
    min_clear = 9.9
    rows = []
    stalled = False
    step = 0
    while t < max_time and len(lap_times) < laps:
        if t >= next_frame - 1e-9:
            _set_time(t + SCAN_TRANSPORT)
            lx = plant.x + vehicle.LIDAR_FROM_POSE * math.cos(plant.yaw)
            ly = plant.y + vehicle.LIDAR_FROM_POSE * math.sin(plant.yaw)
            r = raycast(dist, res, ox, oy, lx, ly, angles + plant.yaw,
                        max_range)
            scan = _stamp(LaserScan(), t)
            scan.angle_min = vehicle.LIDAR_ANGLE_MIN
            scan.angle_increment = vehicle.LIDAR_ANGLE_INC
            scan.range_min = vehicle.LIDAR_RANGE_MIN
            scan.range_max = max_range
            scan.ranges = r.tolist()
            imu = _stamp(Imu(), t)
            imu.angular_velocity.z = float(yr)
            node.stub_deliver('/imu', imu)
            node.stub_deliver('/lidar', scan)
            deliver_commands(t)
            next_frame += frame
            if trace is not None:
                rows.append((t, plant.x, plant.y, plant.yaw, plant.v,
                             plant.delta, prog.net, float(np.min(r))))
            if abs(plant.v) < 0.05:
                still += frame
        if t >= next_timer - 1e-9:
            _set_time(t)
            node.stub_fire_timers()
            deliver_commands(t)
            next_timer += TIMER_DT

        prev = (plant.x, plant.y)
        yr = plant.step(t)
        t += ST.DT
        step += 1

        # Contact is resolved EVERY physics step, but only where it can
        # happen: if the car's centre is further from a wall than its own half
        # diagonal, no part of it can be touching.  Checking every fifth step
        # instead let the car creep through a wall four steps out of five --
        # the first version of this harness drove the ICRA-2025 circuit
        # straight through the barriers and out of the map.
        cix = int((plant.x - ox) / res)
        ciy = int((plant.y - oy) / res)
        near = (not (0 <= cix < dist.shape[1] and 0 <= ciy < dist.shape[0])
                or dist[ciy, cix] <= HALF_DIAG)
        if near:
            clear = ST.footprint_clear(dist, meta, plant.x, plant.y, plant.yaw)
            min_clear = min(min_clear, clear)
            if clear <= 0.0:
                fresh = t >= collide_until
                if fresh:
                    collisions += 1
                    collide_until = t + 0.5
                if not collide_reset:
                    # CONTACT, the default: the simulator does not teleport a
                    # car that hits a wall, it leaves it against the wall, and
                    # that is exactly the situation the node's stuck/escape
                    # keys exist for.  Cancel the component of the motion that
                    # goes INTO the wall, keep the tangential part, and take
                    # the speed off -- the car scrapes and can reverse out.
                    nx, ny = wall_normal(grad_x, grad_y, res, ox, oy,
                                         plant.x, plant.y)
                    dx, dy = plant.x - prev[0], plant.y - prev[1]
                    into = dx * nx + dy * ny
                    if into < 0.0:
                        plant.x = prev[0] + dx - into * nx
                        plant.y = prev[1] + dy - into * ny
                    else:
                        plant.x, plant.y = prev
                    if fresh:
                        plant.v *= 0.3      # impact, once per contact episode
                if collide_reset:
                    # The simulator resets to the last checkpoint PASSED, not
                    # the nearest one, so a car that hits a wall on entry does
                    # not get a free ride to the exit.
                    s_now = prog.s_of(plant.x, plant.y)
                    behind = np.where(check_s <= s_now, check_s, -1e9)
                    j = int(np.argmax(behind)) if behind.max() > -1e8 \
                        else int(np.argmax(check_s))
                    cx, cy, cyaw = checks[j]
                    plant.x = cx - 0.15 * math.cos(cyaw)
                    plant.y = cy - 0.15 * math.sin(cyaw)
                    plant.yaw = cyaw
                    plant.v = 0.0
                    plant.delta = 0.0
                    plant.cmd_queue.clear()
                    prog.last = prog.s_of(plant.x, plant.y)

        if step % 20 == 0:
            prog.update(plant.x, plant.y)
            if prog.net > best_net + 0.5:
                best_net = prog.net
                last_progress_t = t
            elif t - last_progress_t > 45.0:
                stalled = True
                break
            while prog.net >= (len(lap_times) + 1) * prog.total:
                lap_times.append(t - lap_s)
                lap_s = t
                if verbose:
                    print(f'    lap {len(lap_times)}: {lap_times[-1]:.2f} s '
                          f'({collisions} collisions so far)')

    if trace is not None and rows:
        np.savetxt(trace, np.array(rows), delimiter=',',
                   header='t,x,y,yaw,v,delta,s,min_range', comments='')
    escapes = sum(1 for lvl, m in node.get_logger().records
                  if m.startswith('stuck: reversing'))
    return dict(laps=len(lap_times), lap_times=lap_times,
                collisions=collisions, sim_time=t, stalled=stalled,
                net=prog.net, min_net=prog.lo, length=prog.total,
                still=still, escapes=escapes, min_clear=min_clear,
                x=plant.x, y=plant.y,
                errors=node.get_logger().errors())


def plot(prefix, trace_csv, out_png, scale=2):
    """Map (observed-free in white) with the driven trace in red."""
    from PIL import Image, ImageDraw
    dist, img, meta = load_map(prefix)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    H, W = img.shape
    # `img` is already flipped to map orientation (row 0 = y min), so the
    # picture has to be flipped BACK to be drawn.  Drawing the trace on the
    # unflipped array is what made an earlier version of this look as though
    # the car drove through every wall.
    base = np.flipud(np.where(img >= 250, 255, 0).astype(np.uint8))
    rgb = Image.fromarray(np.dstack([base] * 3)).convert('RGB')
    d = np.genfromtxt(trace_csv, delimiter=',', names=True)
    ImageDraw.Draw(rgb).line(
        [((x - ox) / res, H - 1 - (y - oy) / res)
         for x, y in zip(d['x'], d['y'])], fill=(255, 0, 0), width=2)
    rgb.resize((W // scale, H // scale)).save(out_png)
    return out_png


TRACKS = ['porto_mesh', 'icra2026_mesh', 'cdctf2025_mesh', 'icra2025_mesh',
          'finals_rehearsal_mesh']
PROFILES = {'mapping': os.path.join(ROOT, 'src', 'config', 'ftg_mapping.yaml'),
            'racing': os.path.join(ROOT, 'src', 'config', 'ftg.yaml')}


def fmt(name, profile, res):
    lt = res['lap_times']
    med = float(np.median(lt)) if lt else float('nan')
    best = min(lt) if lt else float('nan')
    flag = ''
    if res['min_net'] < -1.0:
        flag += ' BACKWARDS(%.1f m)' % res['min_net']
    if res['stalled']:
        flag += ' STALLED'
    return (f'{name:<22} {profile:<10} laps {res["laps"]:>2}  '
            f'coll {res["collisions"]:>3}  med {med:6.2f}s  best {best:6.2f}s  '
            f'sim {res["sim_time"]:6.1f}s  esc {res["escapes"]:>3}  '
            f'still {res["still"]:5.1f}s{flag}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track', nargs='?', help='map slug under data/maps')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--params', help='a follow_the_gap params yaml')
    ap.add_argument('--profile', action='append', default=[],
                    choices=sorted(PROFILES))
    ap.add_argument('--set', action='append', default=[],
                    metavar='K=V', help='override one parameter')
    ap.add_argument('--laps', type=int, default=10)
    ap.add_argument('--max-time', type=float, default=600.0)
    ap.add_argument('--frame', type=float, default=FRAME)
    ap.add_argument('--dead', type=float, default=DEAD)
    ap.add_argument('--collide-reset', action='store_true',
                    help='teleport to the last checkpoint after a collision '
                         'instead of modelling contact with the wall')
    ap.add_argument('--trace', help='write a CSV trace (single track only)')
    ap.add_argument('--plot', help='render the trace over the map to this PNG')
    ap.add_argument('--tag', default='', help='label for the output lines')
    ap.add_argument('-v', '--verbose', action='store_true')
    a = ap.parse_args()

    tracks = TRACKS if a.all else [a.track]
    if not tracks or tracks == [None]:
        ap.error('give a track slug or --all')
    jobs = []
    if a.params:
        jobs.append((a.tag or os.path.basename(a.params), load_params(a.params)))
    for p in a.profile or ([] if a.params else ['mapping', 'racing']):
        jobs.append((p, load_params(PROFILES[p])))
    extra = {}
    for kv in a.set:
        k, v = kv.split('=', 1)
        try:
            extra[k] = float(v)
        except ValueError:
            extra[k] = v
    for name, base in jobs:
        base.update(extra)
    for track in tracks:
        prefix = os.path.join(MAPS, track)
        for name, params in jobs:
            t0 = time.time()
            res = run(prefix, params, laps=a.laps, max_time=a.max_time,
                      frame=a.frame, dead=a.dead,
                      collide_reset=a.collide_reset, verbose=a.verbose,
                      trace=a.trace)
            print(fmt(track, name, res), f' [{time.time() - t0:.0f}s wall]')
            if a.plot and a.trace:
                plot(prefix, a.trace, a.plot)
            if res['errors']:
                print('   node errors:', res['errors'][:3])
            sys.stdout.flush()


if __name__ == '__main__':
    main()
