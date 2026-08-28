"""Replay a bag through the particle-filter core and score it against IPS.

Deterministic and as fast as the CPU allows — no real-time message drops — so
parameter sweeps are meaningful and repeatable.

Usage:
  python3 offline_pf.py <bag_dir> <map_prefix> [--centerline CSV]
                        [--seed N] [--param key=value ...] [--dump out.csv]
"""

import argparse
import math
import os
import sqlite3
import sys

import numpy as np

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pit_crew.pf_core import ParticleFilterCore, load_map   # noqa: E402
from pit_crew import vehicle                                 # noqa: E402
from pit_crew.scan_odometry import ScanSpeed, scan_to_points  # noqa: E402


TOPICS = {
    '/autodrive/roboracer_1/lidar': LaserScan,
    '/autodrive/roboracer_1/imu': Imu,
    '/autodrive/roboracer_1/ips': Point,
    '/autodrive/roboracer_1/left_encoder': JointState,
    '/autodrive/roboracer_1/right_encoder': JointState,
    '/autodrive/roboracer_1/throttle': Float32,
}


def read_bag(bag_dir):
    db = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
    conn = sqlite3.connect(os.path.join(bag_dir, db[0]))
    tmap = {tid: name for tid, name, _ in
            conn.execute('SELECT id, name, type FROM topics')}
    rows = conn.execute(
        'SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp')
    out = []
    for tid, ts, data in rows:
        name = tmap.get(tid)
        cls = TOPICS.get(name)
        if cls is not None:
            out.append((ts * 1e-9, name, deserialize_message(data, cls)))
    conn.close()
    return out


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('map_prefix')
    ap.add_argument('--centerline', default=None)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--param', action='append', default=[])
    ap.add_argument('--speed-source', default='encoder',
                    choices=['encoder', 'none', 'imu', 'throttle', 'scan'])
    ap.add_argument('--throttle-to-mps', type=float, default=25.0,
                    help='1/ff_gain; scale the calibrated throttle->speed curve')
    ap.add_argument('--dump', default=None)
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    np.random.seed(args.seed)
    params = {}
    for kv in args.param:
        k, v = kv.split('=')
        params[k] = int(v) if v.isdigit() else float(v)

    dist, img, meta = load_map(args.map_prefix)
    centerline = None
    if args.centerline and os.path.exists(args.centerline):
        centerline = np.loadtxt(args.centerline, delimiter=',')[:, :2]

    pf = ParticleFilterCore(dist, img, meta, centerline, **params)
    speed_est = vehicle.SpeedEstimator(alpha=float(os.environ.get("PF_ALPHA","0.5")))

    msgs = read_bag(args.bag)
    imu_yaw = None
    truth = None
    last_t = None
    rows = []

    imu_speed = 0.0
    imu_last_t = None
    thr_speed = 0.0
    scan_speed = ScanSpeed(alpha=float(os.environ.get('SCAN_ALPHA', '0.5')))
    prev_yaw = None

    for t, name, m in msgs:
        if name.endswith('imu'):
            imu_yaw = yaw_from_quat(m.orientation)
            if args.speed_source == 'imu':
                if imu_last_t is not None:
                    d = min(max(t - imu_last_t, 0.0), 0.2)
                    imu_speed += m.linear_acceleration.x * d
                    imu_speed *= 0.995          # leak against integrator drift
                    imu_speed = float(np.clip(imu_speed, -1.0, 20.0))
                imu_last_t = t
        elif name.endswith('ips'):
            truth = (m.x, m.y)
        elif name.endswith('throttle'):
            thr_speed = max(0.0, float(m.data)) * args.throttle_to_mps
        elif name.endswith('encoder'):
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if m.position:
                # name[-13] happens to yield '/' for left and 'r' for
                # right -- two distinct keys, so it worked by accident.
                # Say what is meant instead of relying on that.
                speed_est.update('L' if 'left' in name else 'R',
                                 stamp, m.position[0])
        elif name.endswith('lidar'):
            if imu_yaw is None or truth is None:
                continue
            if not pf.initialized:
                pf.init_near(truth[0], truth[1])
                last_t = t
                continue
            dt = min(max(t - last_t, 0.0), 0.5)
            last_t = t
            if args.speed_source == 'encoder':
                if speed_est.jumped:
                    speed_est.jumped = False
                    pf.relocalize()
                speed = speed_est.speed
            elif args.speed_source == 'imu':
                speed = imu_speed
            elif args.speed_source == 'throttle':
                speed = thr_speed
            elif args.speed_source == 'scan':
                pts = scan_to_points(m.ranges, m.angle_min, m.angle_increment,
                                     m.range_min, m.range_max)
                dyaw = 0.0 if prev_yaw is None else \
                    math.atan2(math.sin(imu_yaw - prev_yaw),
                               math.cos(imu_yaw - prev_yaw))
                prev_yaw = imu_yaw
                speed = scan_speed.update(pts, dyaw, dt)
            else:
                speed = 0.0
            pf.predict(speed, dt, imu_yaw)
            pf.update(m.ranges, m.angle_min, m.angle_increment,
                      m.range_min, m.range_max, imu_yaw)
            ex, ey, _ = pf.estimate(imu_yaw)
            rows.append((t, ex, ey, truth[0], truth[1],
                         math.hypot(ex - truth[0], ey - truth[1]), pf.score))

    if not rows:
        print('no estimates produced')
        return
    err = np.array([r[5] for r in rows])
    settled = err[int(len(err) * 0.1):]
    tag = ' '.join(f'{k}={v}' for k, v in sorted(params.items())) or 'defaults'
    print(f'{tag:60s} n={len(err):5d} '
          f'median={np.median(err):6.3f} mean={err.mean():7.3f} '
          f'p95={np.percentile(err, 95):7.3f} max={err.max():8.3f} '
          f'<0.25m={100*np.mean(err < 0.25):5.1f}% '
          f'<0.5m={100*np.mean(settled < 0.5):5.1f}% '
          f'reloc={pf.relocalizations}')

    if args.dump:
        np.savetxt(args.dump, np.array(rows), delimiter=',', fmt='%.4f',
                   header='t,est_x,est_y,true_x,true_y,err,score', comments='# ')


if __name__ == '__main__':
    main()
