"""ROS-free particle filter core, shared by the live node and offline tuning.

Keeping this independent of rclpy lets the offline evaluator replay a bag
deterministically (every scan processed, no real-time drops), which is the
only way parameter sweeps mean anything.
"""

import math

import numpy as np
import yaml


DEFAULTS = dict(
    n_particles=800,
    n_beams=60,
    sigma_hit=0.06,
    beam_temp=8.0,
    motion_noise_xy=0.18,
    motion_noise_theta=0.02,
    init_spread=0.6,
    lost_threshold=0.22,
    lost_patience=5,
    max_step=0.6,
    resample_frac=0.8,
)


def load_map(prefix):
    """Return (dist_to_wall_m, meta) for a map_server-format map."""
    from PIL import Image
    from scipy import ndimage
    with open(prefix + '.yaml') as f:
        meta = yaml.safe_load(f)
    img = np.flipud(np.array(Image.open(prefix + '.pgm')))
    occupied = img <= 50
    dist = ndimage.distance_transform_edt(~occupied) * float(meta['resolution'])
    return dist, img, meta


class ParticleFilterCore:

    def __init__(self, dist, img, meta, centerline=None, **params):
        self.p = dict(DEFAULTS)
        self.p.update({k: v for k, v in params.items() if v is not None})
        self.dist = dist
        self.img = img
        self.res = float(meta['resolution'])
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        self.H, self.W = dist.shape
        self.centerline = centerline

        n = self.p['n_particles']
        self.px = np.zeros(n)
        self.py = np.zeros(n)
        self.pth = np.zeros(n)
        self.w = np.ones(n) / n
        self.score = 0.0
        self.lost_count = 0
        self.relocalizations = 0
        self.initialized = False

    # ------------------------------------------------------------ lifecycle

    def init_near(self, x, y, spread=None):
        n = self.p['n_particles']
        spread = self.p['init_spread'] if spread is None else spread
        self.px = np.random.normal(x, spread, n)
        self.py = np.random.normal(y, spread, n)
        self.pth = np.random.normal(0.0, 0.15, n)
        self.w = np.ones(n) / n
        self.initialized = True

    def relocalize(self):
        """Resample along the centerline: checkpoint resets are always
        on-track, so the whole map is not worth searching."""
        n = self.p['n_particles']
        if self.centerline is not None:
            idx = np.random.randint(0, len(self.centerline), n)
            self.px = self.centerline[idx, 0] + np.random.normal(0, 0.15, n)
            self.py = self.centerline[idx, 1] + np.random.normal(0, 0.15, n)
        else:
            free = np.argwhere(self.img >= 250)
            idx = np.random.randint(0, len(free), n)
            self.py = self.oy + free[idx, 0] * self.res
            self.px = self.ox + free[idx, 1] * self.res
        # The IMU-yaw offset survives a teleport, so keep the current estimate.
        base = float(np.average(self.pth, weights=self.w)) if self.initialized else 0.0
        self.pth = np.random.normal(base, 0.10, n)
        self.w = np.ones(n) / n
        self.lost_count = 0
        self.relocalizations += 1
        self.initialized = True

    # -------------------------------------------------------------- filter

    def predict(self, speed, dt, imu_yaw):
        n = self.p['n_particles']
        head = imu_yaw + self.pth
        step = float(np.clip(speed * dt, -self.p['max_step'], self.p['max_step']))
        self.px += step * np.cos(head) + np.random.normal(0, self.p['motion_noise_xy'], n)
        self.py += step * np.sin(head) + np.random.normal(0, self.p['motion_noise_xy'], n)
        self.pth += np.random.normal(0, self.p['motion_noise_theta'], n)

    def update(self, ranges, angle_min, angle_inc, range_min, range_max, imu_yaw):
        r = np.asarray(ranges, dtype=np.float64)
        nb = r.size
        step = max(1, nb // self.p['n_beams'])
        idx = np.arange(0, nb, step)
        rr = r[idx]
        ang = angle_min + idx * angle_inc
        good = np.isfinite(rr) & (rr > range_min) & (rr < range_max * 0.95)
        rr, ang = rr[good], ang[good]
        if rr.size < 8:
            return

        head = (imu_yaw + self.pth)[:, None]
        world_ang = head + ang[None, :]
        ex = self.px[:, None] + rr[None, :] * np.cos(world_ang)
        ey = self.py[:, None] + rr[None, :] * np.sin(world_ang)
        gx = ((ex - self.ox) / self.res).astype(np.int32)
        gy = ((ey - self.oy) / self.res).astype(np.int32)
        np.clip(gx, 0, self.W - 1, out=gx)
        np.clip(gy, 0, self.H - 1, out=gy)
        d = self.dist[gy, gx]

        logw = -0.5 * np.sum((d / self.p['sigma_hit']) ** 2, axis=1) / self.p['beam_temp']
        logw -= logw.max()
        w = np.exp(logw)

        pgx = ((self.px - self.ox) / self.res).astype(np.int32)
        pgy = ((self.py - self.oy) / self.res).astype(np.int32)
        inside = (pgx >= 0) & (pgx < self.W) & (pgy >= 0) & (pgy < self.H)
        w[~inside] = 0.0
        np.clip(pgx, 0, self.W - 1, out=pgx)
        np.clip(pgy, 0, self.H - 1, out=pgy)
        w[inside & (self.dist[pgy, pgx] <= 0.02)] *= 0.01

        s = w.sum()
        if s <= 1e-12:
            self.lost_count += 1
            self.score = 0.0
            if self.lost_count >= self.p['lost_patience']:
                self.relocalize()
            return

        self.w = w / s
        self.score = float(np.exp(np.average(logw, weights=self.w)))
        if self.score < self.p['lost_threshold']:
            self.lost_count += 1
            if self.lost_count >= self.p['lost_patience']:
                self.relocalize()
                return
        else:
            self.lost_count = 0
        self.resample()

    def resample(self):
        n = self.p['n_particles']
        n_eff = 1.0 / np.sum(self.w ** 2)
        if n_eff > self.p['resample_frac'] * n:
            return
        pos = (np.arange(n) + np.random.uniform()) / n
        idx = np.clip(np.searchsorted(np.cumsum(self.w), pos), 0, n - 1)
        self.px, self.py, self.pth = self.px[idx], self.py[idx], self.pth[idx]
        self.w = np.ones(n) / n

    def estimate(self, imu_yaw):
        x = float(np.average(self.px, weights=self.w))
        y = float(np.average(self.py, weights=self.w))
        dth = float(np.average(self.pth, weights=self.w))
        return x, y, imu_yaw + dth
