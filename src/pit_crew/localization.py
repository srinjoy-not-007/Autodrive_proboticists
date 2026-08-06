"""Monte Carlo localization against the pre-built occupancy map.

Design notes specific to this platform:

* The IMU reports absolute orientation, so heading is known up to a single
  constant offset between IMU yaw and map yaw. Particles therefore carry
  (x, y, dtheta) with heading = imu_yaw + dtheta, which collapses most of the
  search space and makes recovery cheap.
* Collisions teleport the car to a hidden checkpoint, which always lies on the
  track. Recovery resamples along the centerline rather than over the whole
  map, so relocalization takes a couple of scans instead of seconds.
* Sensor model is a likelihood field (precomputed distance transform), which
  is far cheaper than ray casting and vectorizes cleanly over particles.

Uses only permitted runtime sensors: LiDAR, IMU, wheel encoders.
"""

import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32

from pit_crew import vehicle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ParticleFilter(Node):

    def __init__(self):
        super().__init__('localization')
        p = self.declare_parameters('', [
            ('map_prefix', '/home/data/maps/track_v1'),
            ('centerline_csv', '/home/data/racelines/track_v1_centerline.csv'),
            ('n_particles', 800),
            ('n_beams', 60),
            ('sigma_hit', 0.16),          # m, likelihood field width
            ('beam_temp', 8.0),           # damping on summed beam log-likelihood
            ('motion_noise_xy', 0.035),   # m per update
            ('motion_noise_theta', 0.02), # rad per update
            ('init_x', 0.0),
            ('init_y', 0.0),
            ('init_spread', 0.6),
            ('lost_threshold', 0.22),     # normalized score below = lost
            ('lost_patience', 5),         # consecutive bad scans before reinit
        ])
        (map_prefix, centerline_csv, self.n_p, self.n_beams, self.sigma_hit,
         self.beam_temp, self.noise_xy, self.noise_th, init_x, init_y,
         self.init_spread, self.lost_thresh, self.lost_patience) = \
            [x.value for x in p]

        self.load_map(map_prefix)
        self.load_centerline(centerline_csv)

        self.imu_yaw = None
        self.speed_est = vehicle.SpeedEstimator(alpha=0.5)
        self.last_t = None
        self.lost_count = 0
        self.score = 0.0
        self.initialized = False
        self.init_xy = (init_x, init_y)

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.create_subscription(JointState, f'{ns}/left_encoder',
                                 lambda m: self.on_enc('L', m), qos)
        self.create_subscription(JointState, f'{ns}/right_encoder',
                                 lambda m: self.on_enc('R', m), qos)
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/pit_crew/pose', 10)
        self.pub_particles = self.create_publisher(PoseArray,
                                                   '/pit_crew/particles', 1)
        self.pub_score = self.create_publisher(Float32, '/pit_crew/loc_score', 10)
        self.get_logger().info(
            f'localization ready: {self.n_p} particles, map {self.grid.shape}')

    # ---------------------------------------------------------------- setup

    def load_map(self, prefix):
        with open(prefix + '.yaml') as f:
            meta = yaml.safe_load(f)
        from PIL import Image
        img = np.flipud(np.array(Image.open(prefix + '.pgm')))
        self.res = float(meta['resolution'])
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        occupied = img <= 50
        from scipy import ndimage
        # Distance (m) from every cell to the nearest wall
        self.dist = ndimage.distance_transform_edt(~occupied) * self.res
        self.grid = img
        self.H, self.W = img.shape

    def load_centerline(self, path):
        if os.path.exists(path):
            c = np.loadtxt(path, delimiter=',')
            self.centerline = c[:, :2]
        else:
            self.centerline = None
            self.get_logger().warn('no centerline: recovery will be map-wide')

    # ------------------------------------------------------------ particles

    def init_particles_near(self, x, y, spread):
        self.px = np.random.normal(x, spread, self.n_p)
        self.py = np.random.normal(y, spread, self.n_p)
        self.pth = np.random.normal(0.0, 0.15, self.n_p)   # imu yaw offset
        self.w = np.ones(self.n_p) / self.n_p
        self.initialized = True

    def init_particles_global(self):
        """Resample along the centerline — checkpoint resets are on-track."""
        if self.centerline is not None:
            idx = np.random.randint(0, len(self.centerline), self.n_p)
            self.px = self.centerline[idx, 0] + np.random.normal(0, 0.15, self.n_p)
            self.py = self.centerline[idx, 1] + np.random.normal(0, 0.15, self.n_p)
        else:
            free = np.argwhere(self.grid >= 250)
            idx = np.random.randint(0, len(free), self.n_p)
            self.py = self.oy + free[idx, 0] * self.res
            self.px = self.ox + free[idx, 1] * self.res
        # Heading offset is preserved across teleports: keep the current best
        # estimate, widened slightly.
        base = float(np.average(self.pth, weights=self.w)) \
            if self.initialized else 0.0
        self.pth = np.random.normal(base, 0.10, self.n_p)
        self.w = np.ones(self.n_p) / self.n_p
        self.initialized = True
        self.lost_count = 0
        self.get_logger().warn('RELOCALIZING (centerline-wide resample)')

    # -------------------------------------------------------------- sensors

    def on_enc(self, wheel, msg: JointState):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])

    def on_imu(self, msg: Imu):
        self.imu_yaw = yaw_from_quat(msg.orientation)

    def on_scan(self, scan: LaserScan):
        if self.imu_yaw is None:
            return
        if not self.initialized:
            self.init_particles_near(*self.init_xy, self.init_spread)

        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
        self.last_t = t

        if self.speed_est.jumped:
            self.speed_est.jumped = False
            self.get_logger().warn('encoder jump: teleport suspected')
            self.init_particles_global()

        self.predict(dt)
        self.update(scan)
        self.publish(scan.header.stamp)

    def predict(self, dt):
        head = self.imu_yaw + self.pth
        # Cap per-update travel: a bad speed sample must not fling the whole
        # particle set off the map (from which there is no recovery).
        step = float(np.clip(self.speed_est.speed * dt, -0.6, 0.6))
        self.px += step * np.cos(head) + np.random.normal(0, self.noise_xy, self.n_p)
        self.py += step * np.sin(head) + np.random.normal(0, self.noise_xy, self.n_p)
        self.pth += np.random.normal(0, self.noise_th, self.n_p)

    def update(self, scan: LaserScan):
        r = np.asarray(scan.ranges, dtype=np.float64)
        n = r.size
        step = max(1, n // self.n_beams)
        idx = np.arange(0, n, step)
        rr = r[idx]
        ang = scan.angle_min + idx * scan.angle_increment
        good = np.isfinite(rr) & (rr > scan.range_min) & (rr < scan.range_max * 0.95)
        rr, ang = rr[good], ang[good]
        if rr.size < 8:
            return

        head = (self.imu_yaw + self.pth)[:, None]          # (N,1)
        world_ang = head + ang[None, :]                     # (N,M)
        ex = self.px[:, None] + rr[None, :] * np.cos(world_ang)
        ey = self.py[:, None] + rr[None, :] * np.sin(world_ang)

        gx = np.clip(((ex - self.ox) / self.res).astype(np.int32), 0, self.W - 1)
        gy = np.clip(((ey - self.oy) / self.res).astype(np.int32), 0, self.H - 1)
        d = self.dist[gy, gx]                               # (N,M) m to wall

        # Sum (not mean) of per-beam log-likelihoods: averaging over beams
        # flattens the distribution until 60 beams discriminate no better than
        # one, and the filter just follows dead reckoning. `beam_temp` damps
        # the sum so a single bad scan cannot collapse the particle set.
        logw = -0.5 * np.sum((d / self.sigma_hit) ** 2, axis=1) / self.beam_temp
        logw -= logw.max()
        w = np.exp(logw)

        # Particles inside a wall or off the map are impossible
        pgx = ((self.px - self.ox) / self.res).astype(np.int32)
        pgy = ((self.py - self.oy) / self.res).astype(np.int32)
        inside = ((pgx >= 0) & (pgx < self.W) & (pgy >= 0) & (pgy < self.H))
        w[~inside] = 0.0
        pgx = np.clip(pgx, 0, self.W - 1)
        pgy = np.clip(pgy, 0, self.H - 1)
        w[inside & (self.dist[pgy, pgx] <= 0.02)] *= 0.01

        s = w.sum()
        if s <= 1e-12:
            # Every particle is impossible — recover here rather than
            # returning, or the filter dead-reckons away forever.
            self.lost_count += 1
            self.score = 0.0
            if self.lost_count >= self.lost_patience:
                self.init_particles_global()
            return
        self.w = w / s
        self.score = float(np.exp(np.average(logw, weights=self.w)))

        if self.score < self.lost_thresh:
            self.lost_count += 1
            if self.lost_count >= self.lost_patience:
                self.init_particles_global()
                return
        else:
            self.lost_count = 0

        self.resample()

    def resample(self):
        """Low-variance resampling once particle diversity drops."""
        n_eff = 1.0 / np.sum(self.w ** 2)
        if n_eff > 0.8 * self.n_p:
            return
        pos = (np.arange(self.n_p) + np.random.uniform()) / self.n_p
        idx = np.searchsorted(np.cumsum(self.w), pos)
        idx = np.clip(idx, 0, self.n_p - 1)
        self.px, self.py, self.pth = self.px[idx], self.py[idx], self.pth[idx]
        self.w = np.ones(self.n_p) / self.n_p

    # -------------------------------------------------------------- outputs

    def estimate(self):
        x = float(np.average(self.px, weights=self.w))
        y = float(np.average(self.py, weights=self.w))
        dth = float(np.average(self.pth, weights=self.w))
        return x, y, self.imu_yaw + dth

    def publish(self, stamp):
        x, y, yaw = self.estimate()
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_pose.publish(msg)
        self.pub_score.publish(Float32(data=self.score))

        if self.pub_particles.get_subscription_count() > 0:
            pa = PoseArray()
            pa.header = msg.header
            for i in range(0, self.n_p, 8):
                q = Pose()
                q.position.x = float(self.px[i])
                q.position.y = float(self.py[i])
                h = self.imu_yaw + self.pth[i]
                q.orientation.z = math.sin(h / 2.0)
                q.orientation.w = math.cos(h / 2.0)
                pa.poses.append(q)
            self.pub_particles.publish(pa)


def main():
    rclpy.init()
    node = ParticleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
