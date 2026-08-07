"""2D LiDAR odometry: vehicle motion from consecutive scans.

Why not KISS-ICP: it registers 3D point clouds and carries a deskewing and
voxel-map pipeline we do not need. Here the LiDAR is planar and the IMU already
gives absolute yaw to good accuracy, so the only unknown between two scans is a
2D *translation*. Solving just that is a much smaller problem and runs
comfortably inside the 10 Hz budget.

The gain over differentiating a wheel angle: this measures the vehicle's motion
against the world itself, so it does not care about wheel slip, a throttle
calibration, or how the encoder is scaled — which is what the particle filter's
motion model needs on a fast, unfamiliar track.

Geometry: for a static world point, with `d` the vehicle displacement expressed
in the previous vehicle frame and `dyaw` the rotation between scans,

    p_curr = R(-dyaw) (p_prev - d)   =>   d = p_prev - R(dyaw) p_curr

so rotating the current scan by +dyaw reduces the problem to finding the
translation that best aligns it onto the previous scan.
"""

import numpy as np
from scipy.spatial import cKDTree


def scan_to_points(ranges, angle_min, angle_inc, range_min, range_max,
                   stride=4, max_range=8.0):
    """Planar scan -> (N,2) points in the vehicle frame, invalid returns dropped."""
    r = np.asarray(ranges, dtype=np.float64)[::stride]
    a = angle_min + np.arange(0, len(ranges), stride) * angle_inc
    ok = np.isfinite(r) & (r > range_min) & (r < min(range_max * 0.98, max_range))
    r, a = r[ok], a[ok]
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


class ScanOdometry:
    """Translation-only ICP between consecutive scans, with yaw from the IMU."""

    def __init__(self, iters=8, max_pair_dist=0.5, min_points=40,
                 trim_frac=0.75):
        self.iters = iters
        self.max_pair = max_pair_dist
        self.min_points = min_points
        self.trim = trim_frac        # keep the closest fraction of pairs
        self._prev = None
        self._tree = None
        self.ok = False

    def update(self, points, dyaw):
        """Return (dx, dy) travelled in the previous vehicle frame, or None."""
        self.ok = False
        if points is None or len(points) < self.min_points:
            self._prev, self._tree = None, None
            return None
        if self._prev is None:
            self._prev = points
            self._tree = cKDTree(points)
            return None

        c, s = np.cos(dyaw), np.sin(dyaw)
        rot = np.column_stack([c * points[:, 0] - s * points[:, 1],
                               s * points[:, 0] + c * points[:, 1]])

        d = np.zeros(2)
        for _ in range(self.iters):
            dist, idx = self._tree.query(rot + d, k=1)
            keep = dist < self.max_pair
            if keep.sum() < self.min_points:
                break
            # Trim the worst pairs: moving obstacles and newly-revealed
            # geometry are outliers that would bias a plain mean.
            order = np.argsort(dist[keep])
            n_keep = max(self.min_points, int(len(order) * self.trim))
            sel = np.flatnonzero(keep)[order[:n_keep]]
            step = (self._prev[idx[sel]] - (rot[sel] + d)).mean(axis=0)
            d = d + step
            if np.hypot(*step) < 1e-4:
                break
        else:
            pass

        self._prev = points
        self._tree = cKDTree(points)
        # Reject physically impossible jumps (a dropped scan, or divergence)
        if np.hypot(*d) > 2.0:
            return None
        self.ok = True
        return float(d[0]), float(d[1])


class ScanSpeed:
    """Forward speed from scan odometry, lightly smoothed."""

    def __init__(self, alpha=0.5, **kw):
        self.odom = ScanOdometry(**kw)
        self.alpha = alpha
        self.speed = 0.0

    def update(self, points, dyaw, dt):
        d = self.odom.update(points, dyaw)
        if d is None or dt <= 1e-3:
            return self.speed
        v = float(np.hypot(d[0], d[1]) / dt)
        if d[0] < 0:            # travelling backwards
            v = -v
        if abs(v) < 25.0:
            self.speed += self.alpha * (v - self.speed)
        return self.speed
