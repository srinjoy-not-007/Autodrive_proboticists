#!/usr/bin/env python3
"""Occupancy-grid mapping with known poses from a recorded bag.

Reads LiDAR + IPS + IMU from a rosbag2 sqlite bag and ray-traces scans from
the known poses. Robust to collision teleports (every scan has an absolute
pose), unlike odometry-based SLAM. Offline use only.

Pose: position from IPS (x, y); heading from IMU yaw + a constant offset
estimated by aligning IMU yaw with the IPS velocity direction while driving.

Usage:
  python3 tools/build_map_from_bag.py data/bags/mapping_XXX [out_prefix]
Outputs out_prefix.pgm + out_prefix.yaml (ROS map_server format) and
out_prefix_traj.csv with the per-scan poses.
"""

import math
import os
import sqlite3
import sys

import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan, Imu
    from geometry_msgs.msg import Point
except ImportError:
    sys.exit('run inside the devkit container (needs rclpy + msg defs)')

# The LiDAR sits 0.1933 m ahead of the IPS point the poses report (measured from
# the simulator scene; see pit_crew/vehicle.py). Correcting for it is physically
# right and measurably sharpens the map — Porto wall pixels 6.53% -> 4.66%, ICRA
# 7.03% -> 4.19% — and it is worth 1.6 s a race on Porto.
#
# It is OFF by default anyway. The sharper ICRA map changes what the
# skeletonizer sees as the track's topology: the traced loop goes from 56.8 m to
# 88.5 m with two genuine direction reversals, and no drivable centerline comes
# out at all. Fixing the extractor is the prerequisite for turning this on, and
# whatever builds the map and whatever localizes against it must agree.
LIDAR_FROM_POSE = float(os.environ.get('PIT_CREW_LIDAR_OFFSET', '0.0'))
RES = 0.025          # m / cell
PAD = 1.0            # m padding around trajectory bounds
L_OCC = 0.85         # log-odds-ish increments (probability weights)
L_FREE = -0.3


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_bag(bag_dir):
    db = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
    if not db:
        sys.exit(f'no .db3 in {bag_dir}')
    conn = sqlite3.connect(os.path.join(bag_dir, db[0]))
    topics = {name: (tid, typ) for tid, name, typ in
              conn.execute('SELECT id, name, type FROM topics')}
    out = {}
    for want, cls in [('/autodrive/roboracer_1/lidar', LaserScan),
                      ('/autodrive/roboracer_1/imu', Imu),
                      ('/autodrive/roboracer_1/ips', Point)]:
        tid = topics[want][0]
        rows = conn.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? '
            'ORDER BY timestamp', (tid,)).fetchall()
        out[want] = [(t * 1e-9, deserialize_message(d, cls)) for t, d in rows]
    conn.close()
    return out


def interp_series(times, values, t):
    """Linear interp of a 1D series at time t (values may be angles: no)."""
    i = np.searchsorted(times, t)
    if i <= 0:
        return values[0]
    if i >= len(times):
        return values[-1]
    a = (t - times[i - 1]) / (times[i] - times[i - 1] + 1e-12)
    return values[i - 1] * (1 - a) + values[i] * a


def main():
    bag_dir = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else \
        os.path.join('/home/data/maps', 'track_knownpose')

    data = read_bag(bag_dir)
    ips = data['/autodrive/roboracer_1/ips']
    imu = data['/autodrive/roboracer_1/imu']
    scans = data['/autodrive/roboracer_1/lidar']
    print(f'bag: {len(scans)} scans, {len(ips)} ips, {len(imu)} imu')

    t_ips = np.array([t for t, _ in ips])
    x_ips = np.array([m.x for _, m in ips])
    y_ips = np.array([m.y for _, m in ips])
    t_imu = np.array([t for t, _ in imu])
    yaw_imu = np.unwrap(np.array([yaw_from_quat(m.orientation.x, m.orientation.y,
                                                m.orientation.z, m.orientation.w)
                                  for _, m in imu]))

    # Estimate constant IMU-yaw -> world-yaw offset from motion direction
    offsets = []
    for i in range(1, len(ips)):
        dt = t_ips[i] - t_ips[i - 1]
        dx, dy = x_ips[i] - x_ips[i - 1], y_ips[i] - y_ips[i - 1]
        d = math.hypot(dx, dy)
        if dt <= 0 or d < 0.03 or d / dt < 0.4 or d > 1.0:   # skip slow/teleport
            continue
        world = math.atan2(dy, dx)
        imu_y = interp_series(t_imu, yaw_imu, 0.5 * (t_ips[i] + t_ips[i - 1]))
        offsets.append((world - imu_y + math.pi) % (2 * math.pi) - math.pi)
    if not offsets:
        sys.exit('no motion segments found for yaw alignment')
    off = math.atan2(np.mean(np.sin(offsets)), np.mean(np.cos(offsets)))
    spread = float(np.std(offsets))
    print(f'yaw offset {off:.3f} rad (n={len(offsets)}, std {spread:.3f})')

    # Grid bounds from trajectory + lidar range headroom
    xmin, xmax = x_ips.min() - PAD - 2, x_ips.max() + PAD + 2
    ymin, ymax = y_ips.min() - PAD - 2, y_ips.max() + PAD + 2
    W = int((xmax - xmin) / RES)
    H = int((ymax - ymin) / RES)
    grid = np.zeros((H, W), dtype=np.float32)
    print(f'grid {W}x{H} @ {RES} m')

    traj_rows = []
    used = 0
    for t, s in scans:
        # Pose at scan time
        x = interp_series(t_ips, x_ips, t)
        y = interp_series(t_ips, y_ips, t)
        yaw = interp_series(t_imu, yaw_imu, t) + off
        # Skip scans right around a teleport (pose interp is invalid there)
        i = np.searchsorted(t_ips, t)
        if 0 < i < len(t_ips):
            jump = math.hypot(x_ips[i] - x_ips[i - 1], y_ips[i] - y_ips[i - 1])
            if jump > 0.5:
                continue
        traj_rows.append((t, x, y, yaw))
        used += 1

        r = np.asarray(s.ranges, dtype=np.float64)
        n = r.size
        ang = s.angle_min + np.arange(n) * s.angle_increment + yaw
        valid = np.isfinite(r) & (r > s.range_min) & (r < s.range_max * 0.98)
        # Occupied endpoints
        # The LiDAR is 0.1933 m ahead of the IPS point these poses report (see
        # pit_crew/vehicle.py). Mapping and localization must agree about this
        # or the filter is matching a map built with a different sensor origin.
        sx = x + LIDAR_FROM_POSE * math.cos(yaw)
        sy = y + LIDAR_FROM_POSE * math.sin(yaw)
        ex = sx + r[valid] * np.cos(ang[valid])
        ey = sy + r[valid] * np.sin(ang[valid])
        gx = ((ex - xmin) / RES).astype(int)
        gy = ((ey - ymin) / RES).astype(int)
        ok = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        grid[gy[ok], gx[ok]] += L_OCC
        # Free space along rays (subsampled for speed)
        sub = np.flatnonzero(valid)[::6]
        for j in sub:
            steps = int(r[j] / (RES * 3))
            if steps < 2:
                continue
            rr = np.linspace(0.05, r[j] - 0.05, steps)
            fx = ((sx + rr * np.cos(ang[j]) - xmin) / RES).astype(int)
            fy = ((sy + rr * np.sin(ang[j]) - ymin) / RES).astype(int)
            ok = (fx >= 0) & (fx < W) & (fy >= 0) & (fy < H)
            grid[fy[ok], fx[ok]] += L_FREE

    print(f'used {used}/{len(scans)} scans')

    # Classify: unknown until evidence
    img = np.full((H, W), 205, dtype=np.uint8)      # unknown grey
    img[grid < -0.5] = 254                           # free
    img[grid > 2.0] = 0                              # occupied
    img = np.flipud(img)                             # pgm row 0 = top (max y)

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out + '.pgm', 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        f.write(img.tobytes())
    with open(out + '.yaml', 'w') as f:
        f.write(f"image: {os.path.basename(out)}.pgm\n"
                f"resolution: {RES}\n"
                f"origin: [{xmin}, {ymin}, 0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
                # Stamp the sensor origin this map was built with. The drill
                # builds BOTH variants and promotes the winner over the
                # canonical name, at which point nothing recorded which one it
                # was -- and localization defaulted to 0.0. A map built with the
                # 0.1933 m correction, localized against with 0.0, is worse than
                # either done consistently. The map now carries its own answer.
                f"lidar_offset: {LIDAR_FROM_POSE}\n")
    with open(out + '_traj.csv', 'w') as f:
        f.write('t,x,y,yaw\n')
        for row in traj_rows:
            f.write(','.join(f'{v:.4f}' for v in row) + '\n')
    print(f'wrote {out}.pgm/.yaml and traj csv')


if __name__ == '__main__':
    main()
