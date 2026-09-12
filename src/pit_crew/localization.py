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
            ('map_prefix', '/home/data/maps/track_v2'),
            ('centerline_csv', '/home/data/racelines/track_v2_centerline.csv'),
            ('n_particles', 800),
            ('n_beams', 60),
            ('sigma_hit', 0.06),          # m, likelihood field width
            ('beam_temp', 8.0),           # damping on summed beam log-likelihood
            ('motion_noise_xy', 0.18),   # m per update AT `rate_nominal`
            ('motion_noise_theta', 0.02), # rad per update
            ('init_x', 0.0),
            ('init_y', 0.0),
            ('init_spread', 0.6),
            ('lost_threshold', 0.22),     # normalized score below = lost
            ('lost_patience', 5),         # bad scans AT `rate_nominal`; the
                                          # count actually used is derived from
                                          # the measured period so the blind
                                          # window is fixed in SECONDS
            ('lost_seconds', 0.0),        # >0 overrides the above, in seconds
            ('max_step_mps', 0.0),        # >0: speed cap for the motion model;
                                          # 0 = derive from `max_step_m`
            ('max_step_m', 0.6),          # m per update AT `rate_nominal`
            # Per-SAMPLE and per-UPDATE constants mean different things at
            # different sensor rates (0.104 s dev host, 0.088 s in the
            # image, 0.097 s at the organisers). Both of these are scaled
            # by the measured scan period so the filter's lag and its
            # diffusion are fixed in SECONDS; identity at `rate_nominal`,
            # which is the period every default here was tuned at. Set
            # `enc_tau` or `rate_nominal` to 0 to get the old behaviour.
            ('rate_nominal', vehicle.RATE_NOMINAL),
            ('enc_tau', vehicle.ENC_TAU),   # s; == alpha 0.35 at 0.104 s
            # Velocity input from the calibrated throttle curve (1/0.040).
            # On slow, small Porto a plain random walk is slightly better
            # (0.093 m vs 0.136 m median) and disabling this was tuned to that
            # track — a mistake. On the larger, faster ICRA circuit the car
            # covers ~0.3 m between 10 Hz scans and the random walk cannot keep
            # up: it diverges (mean 1.48 m, p95 7.9 m, 5 relocalizations) where
            # the velocity input stays locked on (mean 0.236 m, p95 0.54 m, 0).
            # Keep it on: a small cost on slow tracks buys stability on fast ones.
            ('throttle_to_mps', 25.0),
            ('lidar_offset', 0.0),        # vehicle.LIDAR_FROM_POSE to enable
            # Review items (Sep 11), each opt-in so it can be measured alone:
            ('keep_prior_weights', False),  # w_k = w_{k-1} * p(z|x) between resamples
            ('publish_on_reloc', True),     # False: skip the pose after a global reinit
            ('midpoint_heading', False),    # propagate with mean(prev, now) IMU yaw
        ])
        (map_prefix, centerline_csv, self.n_p, self.n_beams, self.sigma_hit,
         self.beam_temp, self.noise_xy, self.noise_th, init_x, init_y,
         self.init_spread, self.lost_thresh, self.lost_patience,
         self.lost_seconds, self.max_step_mps, self.max_step_m,
         self.rate_nominal, self.enc_tau,
         self.throttle_to_mps, self.lidar_offset, self.keep_prior_w,
         self.publish_on_reloc, self.midpoint_heading) = [x.value for x in p]
        self.speed_cmd = 0.0

        self.load_map(map_prefix)
        self.load_centerline(centerline_csv)

        self.imu_yaw = None
        self.imu_yaw_prev = None
        self.imu_received = 0.0
        self._just_relocalized = False
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        self.last_t = None
        self.lost_count = 0
        # Both of these are SAMPLE counts the code compares against, derived
        # from a time so that they mean the same thing on a 0.088 s host and a
        # 0.104 s one. Seeded with the nominal answer for the first scan, when
        # no period has been measured yet.
        self._patience = max(1, int(self.lost_patience))
        # Metres-per-update is a SPEED cap in disguise: 0.6 m is 5.77 m/s at
        # 0.104 s and 6.82 at 0.088, and the car reaches 6.58 m/s. Cap the
        # speed, which is a property of the vehicle, not of the host.
        self._v_cap = self.speed_cap()
        self.score = 0.0
        self.initialized = False
        # Where to seed the particle cloud. The submission entrypoint does NOT
        # pass init_x/init_y -- only run_stanley.sh does -- so the default has
        # to be right on its own. It used to be (0.0, 0.0): on Porto the car
        # spawns at (0.80, 3.16), which is 3.26 m away, more than five times
        # init_spread, so every particle started somewhere the car had never
        # been. Every test masked it because the test harness passed the pose.
        #
        # The start line is the first point of the centerline we have already
        # loaded, which is track-agnostic and correct on a circuit nobody has
        # seen. An explicit parameter still wins if given.
        if (init_x, init_y) == (0.0, 0.0) and self.centerline is not None \
                and len(self.centerline):
            init_x, init_y = (float(self.centerline[0][0]),
                              float(self.centerline[0][1]))
            self.get_logger().info(
                f'no init pose given; seeding at the centerline start '
                f'({init_x:.2f}, {init_y:.2f})')
        self.init_xy = (init_x, init_y)

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.create_subscription(JointState, f'{ns}/left_encoder',
                                 lambda m: self.on_enc('L', m), qos)
        self.create_subscription(JointState, f'{ns}/right_encoder',
                                 lambda m: self.on_enc('R', m), qos)
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.create_subscription(Float32, f'{ns}/throttle', self.on_throttle, qos)
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
        # The map knows which sensor origin it was built with; trust it over the
        # parameter default, because the two MUST agree and only the map can
        # say. Older maps have no such key and keep the parameter.
        if 'lidar_offset' in meta:
            self.lidar_offset = float(meta['lidar_offset'])
            self.get_logger().info(
                f'lidar_offset {self.lidar_offset:.4f} m from map metadata')
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
        self._just_relocalized = True
        self.get_logger().warn('RELOCALIZING (centerline-wide resample)')

    # -------------------------------------------------------------- sensors

    def on_enc(self, wheel, msg: JointState):
        # Wheel encoders ARE usable: `position` is cumulative wheel angle in
        # radians, and wheel_radius * d(position)/dt matches IPS ground truth
        # to within 1% on two different tracks. Preferred over the throttle
        # curve because it measures the vehicle instead of assuming a
        # calibration, so it needs no per-track retuning.
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])

    def on_throttle(self, msg: Float32):
        # Fallback only: used if throttle_to_mps > 0 and encoders are silent.
        self.speed_cmd = max(0.0, float(msg.data)) * self.throttle_to_mps

    def on_imu(self, msg: Imu):
        q = msg.orientation
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            return
        self.imu_yaw = yaw_from_quat(msg.orientation)
        if self.imu_yaw_prev is None:
            self.imu_yaw_prev = self.imu_yaw
        self.imu_received = self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, scan: LaserScan):
        if self.imu_yaw is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.imu_received > 0.35:
            return
        ranges = np.asarray(scan.ranges, dtype=float)
        ranges = ranges[::max(1, len(ranges) // self.n_beams)]
        if (not math.isfinite(scan.angle_min)
                or not math.isfinite(scan.angle_increment)
                or scan.angle_increment <= 0
                or not math.isfinite(scan.range_max)
                or np.count_nonzero(np.isfinite(ranges)
                    & (ranges > scan.range_min)
                    & (ranges < scan.range_max * 0.95)) < 8):
            return
        if not self.initialized:
            self.init_particles_near(*self.init_xy, self.init_spread)

        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        if t <= 0 or now - t > 0.35 or t > now + 0.1:
            return
        if self.last_t is not None and t <= self.last_t:
            return
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
        self.last_t = t
        # The encoders share the scan's WebSocket frame, so the scan period IS
        # the encoder sample period. Everything per-sample follows it; see
        # `repoint`.
        self.repoint(dt)

        self.predict(dt)
        self.update(scan)
        self.imu_yaw_prev = self.imu_yaw
        if self._just_relocalized and not self.publish_on_reloc:
            # The centroid of a track-wide cloud is not a pose; let the
            # controller's stale gate cover one missing update instead.
            self._just_relocalized = False
            return
        self._just_relocalized = False
        self.publish(scan.header.stamp)

    def speed_cap(self):
        """The motion model's cap, as a SPEED.

        `max_step_m` is metres per UPDATE, which is a speed cap in disguise:
        0.6 m is 5.77 m/s at a 0.104 s period and 6.82 m/s at 0.088 s, either
        side of the 6.58 m/s the car actually reaches, so the dev host and the
        submission image silently dead-reckon under different models exactly
        where the car is fastest. Identity at `rate_nominal`.
        """
        if self.max_step_mps > 0.0:
            return float(self.max_step_mps)
        return (float(self.max_step_m)
                / max(self.rate_nominal or vehicle.RATE_NOMINAL, 1e-3))

    def repoint(self, dt):
        """Re-point every per-SAMPLE constant at the measured scan period.

        The encoders share the scan's WebSocket frame, so the scan period IS
        the encoder sample period. Each of these counts samples and means a
        TIME; every formula is the identity at `rate_nominal`.
        """
        if not dt > 0.0:
            return
        if self.enc_tau > 0.0:
            # Low-pass lag stays 0.193 s on a 9.6 Hz host and an 11.4 Hz one.
            self.speed_est.alpha = vehicle.alpha_for_lag(dt, self.enc_tau)
            # ...and the reject-persistence window stays 0.624 s, rather than
            # shrinking to 0.528 s of steering by a stale speed at 0.088 s.
            self.speed_est.reject_max = vehicle.reject_max_for_dt(dt)
        # How many consecutive bad scans are a lost filter. 5 at 0.104 s is
        # 0.52 s of blind driving -- 3.1 m at 6 m/s -- and the same 5 is only
        # 0.44 s at 0.088. Fix the time, derive the count.
        base = (float(self.lost_seconds) if self.lost_seconds > 0.0
                else self.lost_patience
                * (self.rate_nominal or vehicle.RATE_NOMINAL))
        self._patience = max(1, int(round(base / dt)))

    def predict(self, dt):
        yaw = self.imu_yaw
        if self.midpoint_heading and self.imu_yaw_prev is not None:
            # The step covers [t_prev, t_now]; using the end-of-interval yaw
            # biases every step in a corner by half the heading change.
            d = (self.imu_yaw - self.imu_yaw_prev + math.pi) % (2 * math.pi) - math.pi
            yaw = self.imu_yaw_prev + 0.5 * d
        head = yaw + self.pth
        # Cap per-update travel: a bad speed sample must not fling the whole
        # particle set off the map (from which there is no recovery).
        # A checkpoint reset teleports the car, so the motion model has nothing
        # to say about this step: propagating the pre-jump speed would march the
        # whole cloud away from where the car actually now is. Stand still and
        # spread instead, which lets the sensor update pull the particles to the
        # checkpoint rather than fighting a confident wrong prior. The existing
        # lost-score relocalization still covers the case where it does not.
        jumped = self.speed_est.consume_jump()
        # Encoder speed when available, throttle estimate as fallback
        v = self.speed_est.speed if self.speed_est.speed > 1e-3 else self.speed_cmd
        # The cap is on the SPEED (see `_v_cap`): a per-update distance cap
        # silently truncates dead reckoning above 0.6/dt m/s, which is 5.77 m/s
        # on the dev host and 6.82 in the submission image, either side of the
        # 6.58 m/s the car actually reaches.
        step = 0.0 if jumped else float(
            np.clip(v, -self._v_cap, self._v_cap) * dt)
        # Per-update noise is per-TIME noise only at one rate; scale it so the
        # cloud diffuses at the same m/sqrt(s) whatever the host does.
        ns = (vehicle.noise_scale_for_dt(dt, self.rate_nominal)
              if self.rate_nominal > 0.0 else 1.0)
        noise = self.noise_xy * ns * (4.0 if jumped else 1.0)
        if jumped:
            self.get_logger().warn('encoder jump: teleport assumed, spreading particles')
        self.px += step * np.cos(head) + np.random.normal(0, noise, self.n_p)
        self.py += step * np.sin(head) + np.random.normal(0, noise, self.n_p)
        # The heading-offset walk is per-update too, and scaling only the xy
        # one would make the RATIO of translational to rotational diffusion
        # rate-dependent -- worse than either being uniformly wrong.
        self.pth += np.random.normal(0, self.noise_th * ns, self.n_p)

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
        # Rays leave the LiDAR, not the pose point: the sensor sits 0.1933 m
        # ahead of it (measured from the simulator scene, see vehicle.py).
        # DEFAULT OFF, and the reason is in docs/STATUS.md: it is physically
        # correct, it sharpens both maps and it is worth 1.6 s on Porto, but the
        # sharper map changes what the skeletonizer sees as the track topology
        # and ICRA stops producing a drivable centerline at all. Must match
        # whatever built the map.
        sx = self.px[:, None] + self.lidar_offset * np.cos(head)
        sy = self.py[:, None] + self.lidar_offset * np.sin(head)
        ex = sx + rr[None, :] * np.cos(world_ang)
        ey = sy + rr[None, :] * np.sin(world_ang)

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
        if self.keep_prior_w:
            # Proper SIR: between resampling steps the prior weight carries
            # the evidence of earlier scans; discarding it makes every
            # non-resampled update a one-scan estimator.
            w = w * self.w

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
            if self.lost_count >= self._patience:
                self.init_particles_global()
            return
        self.w = w / s
        self.score = float(np.exp(np.average(logw, weights=self.w)))

        if self.score < self.lost_thresh:
            self.lost_count += 1
            if self.lost_count >= self._patience:
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
