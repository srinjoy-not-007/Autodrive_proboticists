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
            # ---- teleport handling (Sep 12), ALL default-off ----------------
            # A collision returns the car to its last numbered checkpoint, at
            # rest. Measured over 329 real collisions in data/logs: the body's
            # true speed goes 2.85 -> 0.85 m/s while the wheel encoders go
            # 3.15 -> 3.02, i.e. the wheels keep spinning and the encoder
            # over-reads by +2.12 m/s (p90 +3.06) for ~0.4 s. `SpeedEstimator`
            # therefore marches the whole cloud forward while the car is
            # standing still. The reset itself is small -- p50 0.65 m, p90
            # 1.06 m against 0.28 m of normal travel per scan -- which is why
            # neither `SpeedEstimator.jumped` nor the lost-score branch ever
            # sees it. See the measurements in docs/STATUS.md.
            ('checkpoints_csv', ''),        # <map>_checkpoints.csv; '' = off
            ('teleport_scan_jump', 0.0),    # m; median |dr| between consecutive
                                            # scans above this = teleport.
                                            # 0 = off. 0.25 measured 0 false
                                            # positives in 3582 scans and
                                            # catches the 12% of resets that
                                            # move the car more than ~1.5 m.
            ('teleport_imu_g', 0.0),        # m/s2; |linear accel| above this is
                                            # an impact. 0 = off. 15.0 measured
                                            # 0 false positives in 2980 clean
                                            # scans (clean max 8.76); its
                                            # DETECTION rate is unmeasured --
                                            # no bag exists with IMU across a
                                            # collision.
            ('teleport_hold_n', 4),         # scans of zero dead-reckoning after
                                            # a detection (the encoder's
                                            # phantom speed lasts ~0.4 s)
            ('teleport_reseed', True),      # reseed the cloud at the nearby
                                            # checkpoints (needs checkpoints_csv)
            # ---- telemetry-gap handling (Sep 12), ALL default-off ----------
            # A stalled simulator (OBS + rosbag on the organisers' box) stops
            # physics AND telemetry together, so the car does NOT move while
            # the filter is blind; a stalled bridge or a delayed socket stops
            # only the telemetry, and the car keeps driving. The filter cannot
            # tell them apart from one inflated `dt`, and the shipped answer --
            # dead-reckon the whole interval at the pre-gap encoder speed, up
            # to `scan_dt_max` x `max_step_mps` = 4.5 m -- is right for one and
            # catastrophic for the other. Measured on the real collision bag,
            # see docs/STATUS.md.
            ('gap_dt', 0.0),                # s; a scan interval longer than
                                            # this is a telemetry gap. 0 = off
                                            # (nothing below is consulted).
            ('gap_step_cap_m', 0.0),        # m; >0 caps the SINGLE motion step
                                            # taken across a gap. 0 = uncapped.
            ('gap_noise_mult', 1.0),        # >1 widens the cloud on the
                                            # post-gap step instead of trusting
                                            # dead reckoning; the scan update
                                            # then pulls the particles in.
            ('lost_patience_min', 1),       # scans; floor under the count
                                            # `repoint` derives from the
                                            # measured period. At dt >= 0.35 s
                                            # that count is 1, so ONE bad-score
                                            # scan after a gap declares the
                                            # filter lost and (with
                                            # publish_on_reloc false) drops the
                                            # pose the controller is waiting
                                            # for.
            ('scan_dt_max', 0.5),           # s; the motion-step clamp on the
                                            # measured scan interval.
            ('enc_gap_max', 0.0),           # s; >0 overrides the encoder's own
                                            # `MAX_ENC_GAP` 0.5 s ceiling. The
                                            # two coincide by default, so a gap
                                            # of 0.5 s or more is dead-reckoned
                                            # at 100% of the STALE pre-gap
                                            # speed. Raise BOTH together.
            # ---- along-track anchoring (Sep 19), ALL default-off -----------
            # The finals track's s 2-16 is an 18 m straight with parallel,
            # featureless walls. The scan likelihood pins the LATERAL offset
            # and the heading there and says almost nothing about how far
            # along the car is, so the along-track posterior is driven by the
            # process noise alone. Measured on four 0.097/0.058 s finals logs
            # (see docs/STATUS.md): the along-track error is a random walk
            # over the straight with per-lap sd 0.17-0.32 m and a range of
            # -1.15 to +0.61 m, released as a systematic +0.28-0.35 m STEP at
            # s ~ 10.5 (p90 0.53, max 1.27 m in one 0.097 s scan) where the
            # first along-track feature comes into view.
            #
            # `along_clamp_mps` bounds how far the cloud's weighted mean may
            # move ALONG its own heading between scans, relative to the
            # distance the motion model was given by the encoders. It is a
            # SPEED so it means the same thing at every sensor period; the
            # per-scan allowance is `along_clamp_mps * dt`. The clamp shifts
            # the whole cloud, not just the published mean, so the filter
            # stays self-consistent. It stands down for a relocalization, a
            # detected teleport and below `along_clamp_min_v`, because those
            # are exactly the cases where a large along-track jump is the
            # correct answer. 0 = off.
            ('along_clamp_mps', 0.0),
            ('along_clamp_min_v', 1.0),     # m/s; below this the clamp is
                                            # inert (standing still, a reset,
                                            # the grid start).
            # Process noise ALONG the heading, separately from the lateral
            # one. On an unobservable straight the along-track spread is pure
            # diffusion: 0.18 m per update over the ~19 scans of this straight
            # is 0.78 m of spread, which is the scale of the excursions above,
            # while the lateral noise is pulled straight back by the walls and
            # is the component that earns its keep. 0 = use `motion_noise_xy`
            # for both, which is the shipped, isotropic behaviour.
            ('motion_noise_along', 0.0),
            ('teleport_cp_radius', 2.5),    # m; only checkpoints this close to
                                            # the current estimate are seeded.
                                            # 0 = all of them, which measured
                                            # WORSE (p90 peak error 2.25 m
                                            # against 0.32 m) because the cloud
                                            # can cross to the far side.
            # ---- launch wheelspin (Sep 19), default off -------------------
            # Measured on all three clean finals logs: from the grid the drive
            # torque spins the wheels, the encoder reads up to 2.2x the true
            # speed for ~2 s (8.08 m/s against a true 6.00), and because the
            # first 18 m of this track is a featureless straight the scan
            # cannot contradict it. The published estimate runs +1.8 m ahead
            # at rest and +5.0 to +5.4 m ahead by t+1.9 s, snapping back only
            # at s ~ 14-15 where the geometry becomes observable again -- 3.1
            # to 3.7 s and 13-14.6 m into the race, at up to 7.5 m/s. It has
            # never cost a lap here (the plan is flat at 8.0 m/s across the
            # whole straight) but it leaves ~3 m between the estimate and the
            # s 16-19 hairpin entry, which it would then command at 8 m/s.
            #
            # `dr_accel_max` slew-limits the speed the MOTION MODEL uses (not
            # the published encoder speed) to a physical acceleration, so a
            # spinning wheel cannot march the cloud forward faster than the
            # car can actually accelerate. It is a rate in m/s^2 and so means
            # the same thing at every sensor period. 0 = off, which is the
            # shipped behaviour.
            ('dr_accel_max', 0.0),
            # A scan stamp this far BEHIND the last one is a clock reset, not
            # an out-of-order frame: without this the `t <= last_t` test in
            # `on_scan` rejects every later scan for the rest of the run and
            # the filter silently stops updating. Inert on any monotone
            # stream, which is every recorded run.
            ('clock_reset_s', 1.0),
            # ---- pose reference geometry (Sep 19), BOTH default-off --------
            # The IPS truth topic is the REAR AXLE (measured: the published
            # pose sits +0.08..+0.10 m ahead of it wherever `s` is observable,
            # at both sensor periods, and the truth point's sideslip through
            # both hairpins is ~0). The scan likelihood pins the LiDAR, which
            # is `vehicle.LIDAR_FROM_REAR_AXLE` = 0.2733 m ahead of that axle,
            # so a filter told the lever arm is 0.1933 m converges on a point
            # 0.0800 m (= `vehicle.IPS_FROM_REAR_AXLE`) ahead of the rear
            # axle. The controller therefore reads the line's curvature and
            # its speed plan ~0.10 m early, everywhere.
            #
            # Two independent levers, because they are NOT the same fix:
            #
            # * `pose_along_offset_m` shifts the PUBLISHED pose backwards
            #   along its own heading by that many metres, leaving the filter
            #   untouched. 0.08 makes the published point the rear axle, which
            #   is the point `mpc3`'s kinematic bicycle (kappa = tan(delta)/L)
            #   is written about. It cannot touch the lateral bias, because a
            #   shift along the heading is along-track by construction.
            # * `lidar_offset_override` sets the lever arm the FILTER casts
            #   from, which changes where the scans are matched and therefore
            #   the whole posterior -- including the curvature-proportional
            #   lateral bias toward the OUTSIDE of a corner (-0.09..-0.105 m
            #   at the s 43 hairpin apex). 0.2733 makes the estimated point
            #   the rear axle directly. It exists because the map YAML's
            #   `lidar_offset` OVERRIDES the `-p lidar_offset` argument (see
            #   `load_map`), so there is otherwise no way to change the lever
            #   arm without rebuilding the map -- and a mesh map's grid does
            #   not depend on it at all (`tools/map_from_mesh.py` uses the
            #   constant only to STAMP the YAML).
            #
            # Both are 0 by default and the shipped path is byte-identical.
            # 0.0 means "off" for the override, not "cast from the pose
            # point": that is what an unstamped map already does.
            ('pose_along_offset_m', 0.0),
            ('lidar_offset_override', 0.0),
        ])
        (map_prefix, centerline_csv, self.n_p, self.n_beams, self.sigma_hit,
         self.beam_temp, self.noise_xy, self.noise_th, init_x, init_y,
         self.init_spread, self.lost_thresh, self.lost_patience,
         self.lost_seconds, self.max_step_mps, self.max_step_m,
         self.rate_nominal, self.enc_tau,
         self.throttle_to_mps, self.lidar_offset, self.keep_prior_w,
         self.publish_on_reloc, self.midpoint_heading,
         checkpoints_csv, self.tp_scan_jump, self.tp_imu_g,
         self.tp_hold_n, self.tp_reseed,
         self.gap_dt, self.gap_step_cap_m, self.gap_noise_mult,
         self.lost_patience_min, self.scan_dt_max, self.enc_gap_max,
         self.along_clamp_mps, self.along_clamp_min_v, self.noise_along,
         self.tp_cp_radius, self.dr_accel_max,
         self.clock_reset_s, self.pose_along_offset,
         self.lidar_offset_override) = [x.value for x in p]
        # A zero here is a divide-by-zero in `update` that takes the node down
        # on the first scan of the race; a mistyped `-p sigma_hit:=0` should
        # cost a warning, not the run. Identity for every positive value.
        if not self.sigma_hit > 0.0:
            self.get_logger().warn('sigma_hit must be > 0; using 0.06')
            self.sigma_hit = 0.06
        if not self.beam_temp > 0.0:
            self.get_logger().warn('beam_temp must be > 0; using 1.0')
            self.beam_temp = 1.0
        # What `predict` last handed the motion model, for `dr_accel_max`.
        self._dr_v = 0.0
        self.speed_cmd = 0.0
        # Teleport handler state. Every one of these is inert while both
        # `teleport_scan_jump` and `teleport_imu_g` are 0.0.
        self.checkpoints = None
        self._prev_ranges = None
        self._tp_hold = 0
        self._tp_events = 0
        self.imu_accel = 0.0
        # Along-track anchor state. Inert while `along_clamp_mps` is 0.0.
        self._prev_mean = None
        self._last_step = 0.0
        self._last_v = 0.0
        self._last_jumped = False
        self._anchor_events = 0
        self._anchor_total = 0.0
        if checkpoints_csv:
            self.load_checkpoints(checkpoints_csv)

        self.load_map(map_prefix)
        # ...and only now can the map's stamped lever arm be overridden: the
        # YAML deliberately wins over the `lidar_offset` PARAMETER (the two
        # must agree or the map is being matched against rays that did not
        # come from where it says), so a deliberate change needs its own knob
        # that runs after the map has spoken. Non-finite or 0 = leave the map
        # alone, which is every shipped run.
        if math.isfinite(self.lidar_offset_override) \
                and self.lidar_offset_override != 0.0:
            self.get_logger().warn(
                f'lidar_offset overridden: {self.lidar_offset:.4f} -> '
                f'{float(self.lidar_offset_override):.4f} m')
            self.lidar_offset = float(self.lidar_offset_override)
        self.load_centerline(centerline_csv)

        self.imu_yaw = None
        self.imu_yaw_prev = None
        self.imu_received = 0.0
        self._just_relocalized = False
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        if self.enc_gap_max > 0.0:
            self.speed_est.max_gap = float(self.enc_gap_max)
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

    def load_checkpoints(self, path):
        """Numbered checkpoints (x, y, yaw) from `<map>_checkpoints.csv`.

        These are read out of the simulator scene OFFLINE by
        `tools/map_from_mesh.py`, so using them at runtime touches no
        restricted topic -- it is map data, like the occupancy grid. The car
        reappears 0.14-0.16 m short of the checkpoint it last passed, heading
        along the line, which makes them an exact prior for where a reset put
        the car. Spawn rows and the two named finish lines are skipped: only
        the numbered ones are reset targets.
        """
        import csv as _csv
        if not os.path.exists(path):
            self.get_logger().warn(f'no checkpoints at {path}; teleport '
                                   f'reseed disabled')
            return
        rows = []
        with open(path) as f:
            for r in _csv.DictReader(f):
                if r.get('kind') == 'checkpoint' and str(r.get('name', '')).isdigit():
                    rows.append((float(r['x']), float(r['y']), float(r['yaw'])))
        if rows:
            self.checkpoints = np.asarray(rows, dtype=float)
            self.get_logger().info(
                f'{len(rows)} numbered checkpoints loaded from {path}')

    def teleport_detected(self, ranges, scan):
        """True when this scan cannot be explained by having driven here.

        Two independent branches, each off by default, because only one of
        them is validatable offline:

        * **scan jump** -- the median beam-wise range change between
          consecutive scans. Measured on the clean 268 s Porto bag
          (data/bags/exp_ay50_d180_181558): p50 0.052 m, p99 0.107, max 0.139
          over 2979 scans, so 0.25 m is far outside anything driving produces.
          Spliced against real checkpoint geometry it fires on 2 of 17
          synthetic resets with 0 false positives in 3582 scans. The 12% is
          not a tuning failure: a p50 reset moves the car 0.65 m where one
          scan period of normal driving already moves it 0.28 m, so most
          resets are genuinely indistinguishable from driving.
        * **IMU impact** -- every reset is preceded by hitting a wall, and the
          clean bag's |linear acceleration| never exceeds 8.76 m/s2 over 2980
          scans (p99.9 8.42). A threshold of 15 is therefore free of false
          positives on the only data that exists. Its detection rate is
          UNMEASURED: no bag has been recorded with IMU through a collision.
          Record one with BAG=1 before trusting this branch.
        """
        if self.tp_imu_g > 0.0 and self.imu_accel > self.tp_imu_g:
            return 'imu'
        if self.tp_scan_jump > 0.0 and self._prev_ranges is not None:
            r0, r1 = self._prev_ranges, ranges
            if r0.shape == r1.shape:
                g = (np.isfinite(r0) & np.isfinite(r1)
                     & (r0 > scan.range_min) & (r0 < scan.range_max * 0.95)
                     & (r1 > scan.range_min) & (r1 < scan.range_max * 0.95))
                if np.count_nonzero(g) >= 50:
                    if float(np.median(np.abs(r1[g] - r0[g]))) > self.tp_scan_jump:
                        return 'scan'
        return None

    def seed_at_checkpoints(self):
        """Resample the cloud at the checkpoints a reset could have used.

        Only checkpoints within `teleport_cp_radius` of the current estimate
        are candidates: the reset moves the car backwards along the track, not
        to the far side of it, and seeding all 21 measured worse in the replay
        harness (p90 peak error 2.25 m against 0.32 m for the gated version).
        Heading comes from the checkpoint's own yaw, which is exactly what the
        car reappears with, so the IMU-yaw offset is re-derived rather than
        carried over.
        """
        if self.checkpoints is None or self.imu_yaw is None:
            return False
        cx, cy = self.checkpoints[:, 0], self.checkpoints[:, 1]
        ex = float(np.average(self.px, weights=self.w))
        ey = float(np.average(self.py, weights=self.w))
        if self.tp_cp_radius > 0.0:
            sel = np.flatnonzero(np.hypot(cx - ex, cy - ey) <= self.tp_cp_radius)
            if sel.size == 0:
                sel = np.array([int(np.argmin(np.hypot(cx - ex, cy - ey)))])
        else:
            sel = np.arange(len(cx))
        j = sel[np.random.randint(0, sel.size, self.n_p)]
        self.px = cx[j] + np.random.normal(0, 0.20, self.n_p)
        self.py = cy[j] + np.random.normal(0, 0.20, self.n_p)
        self.pth = (self.checkpoints[j, 2] - self.imu_yaw
                    + np.random.normal(0, 0.12, self.n_p))
        self.w = np.ones(self.n_p) / self.n_p
        self.lost_count = 0
        return True

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
            # `SpeedEstimator.update` drops a sample whose interval is
            # non-positive WITHOUT advancing its reference, which is right for
            # a duplicate and a permanent wedge for a stamp from the future:
            # one glitched sample ahead of the stream freezes `self.speed` for
            # the rest of the race, and the motion model then dead-reckons at
            # that stale speed forever. Re-seed when the stream goes
            # backwards. Inert while stamps are monotone.
            last = getattr(self.speed_est, '_last', {}).get(wheel)
            if last is not None and t < last[0] - 1e-3:
                self.get_logger().warn(
                    f'{wheel} encoder stamp went back {last[0] - t:.2f} s; '
                    f're-seeding the speed estimator')
                self.speed_est.reset()
            self.speed_est.update(wheel, t, msg.position[0])

    def on_throttle(self, msg: Float32):
        # Fallback only: used if throttle_to_mps > 0 and encoders are silent.
        self.speed_cmd = max(0.0, float(msg.data)) * self.throttle_to_mps

    def on_imu(self, msg: Imu):
        q = msg.orientation
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            return
        if self.tp_imu_g > 0.0:
            a = msg.linear_acceleration
            self.imu_accel = math.hypot(a.x, a.y)
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
            # An out-of-order or duplicated frame is dropped, as ever. A stamp
            # far BEHIND the last one is something else: the clock was reset
            # (a restarted bridge under `use_sim_time`, a replayed bag). The
            # test above would then reject EVERY later scan and the filter
            # would never update again while still holding a pose the
            # controller believes. Re-seed instead. Inert on a monotone
            # stream, which is every run recorded here.
            if self.last_t - t < float(self.clock_reset_s):
                return
            self.get_logger().warn(
                f'scan clock went back {self.last_t - t:.2f} s; re-seeding '
                f'the filter timebase')
            self.last_t = None
            self.speed_est.reset()
        dt = 0.0 if self.last_t is None else min(
            max(t - self.last_t, 0.0), float(self.scan_dt_max))
        self.last_t = t
        # The encoders share the scan's WebSocket frame, so the scan period IS
        # the encoder sample period. Everything per-sample follows it; see
        # `repoint`.
        self.repoint(dt)

        # Teleport handling, ahead of the motion model because what it changes
        # is the motion model's input. Inert unless a detector is enabled.
        if self.tp_scan_jump > 0.0 or self.tp_imu_g > 0.0:
            full = np.asarray(scan.ranges, dtype=float)
            why = self.teleport_detected(full, scan)
            if why:
                self._tp_events += 1
                self._tp_hold = max(1, int(self.tp_hold_n))
                # The encoders hold a body that has stopped: drop the filtered
                # speed AND the stored wheel angles, or the next derivative is
                # taken across the standstill.
                self.speed_est.reset()
                seeded = self.tp_reseed and self.seed_at_checkpoints()
                self.get_logger().warn(
                    f'TELEPORT detected ({why}); hold {self._tp_hold} scans, '
                    f'reseeded={seeded}')
            self._prev_ranges = full

        self.predict(dt)
        self.update(scan)
        # Inert while `along_clamp_mps` is 0.0.
        self.anchor_along(dt)
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
        # ...with a floor in SCANS: at dt >= 0.35 s the derived count is 1, so
        # a single bad-score scan immediately after a gap -- exactly when one
        # is most likely -- declares the filter lost.
        self._patience = max(int(getattr(self, 'lost_patience_min', 1)),
                             int(round(base / dt)))

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
        # A detected teleport does the same thing for several scans: the
        # encoder over-reads by +2.12 m/s (p50, measured on 329 real
        # collisions) for ~0.4 s after the reset, so dead reckoning must sit
        # still for that long rather than march the cloud forward.
        # getattr: the rate-invariance tests lift `predict` onto a synthetic
        # namespace that carries only the fields it is testing.
        if getattr(self, '_tp_hold', 0) > 0:
            self._tp_hold -= 1
            jumped = True
        # Encoder speed when available, throttle estimate as fallback
        v = self.speed_est.speed if self.speed_est.speed > 1e-3 else self.speed_cmd
        # Wheels that are spinning (measured: 2.2x the true speed for the
        # first ~2 s of every run here) drive the cloud forward at a speed the
        # car cannot reach. Hold the MOTION MODEL's speed to a physical
        # acceleration; the published encoder speed is untouched.
        # getattr: the rate-invariance tests lift `predict` onto a synthetic
        # namespace that carries only the fields it is testing.
        a_max = float(getattr(self, 'dr_accel_max', 0.0) or 0.0)
        if a_max > 0.0 and dt > 0.0:
            v = min(v, float(getattr(self, '_dr_v', 0.0)) + a_max * dt)
        self._dr_v = float(v)
        # The cap is on the SPEED (see `_v_cap`): a per-update distance cap
        # silently truncates dead reckoning above 0.6/dt m/s, which is 5.77 m/s
        # on the dev host and 6.82 in the submission image, either side of the
        # 6.58 m/s the car actually reaches.
        step = 0.0 if jumped else float(
            np.clip(v, -self._v_cap, self._v_cap) * dt)
        # A telemetry gap: the interval is far longer than a scan period, so
        # this ONE step carries the whole blind window at the pre-gap encoder
        # speed. If the SIMULATOR stalled, the car did not move at all and the
        # step is pure error; if only the telemetry stalled, it is roughly
        # right. Inert while `gap_dt` is 0.
        gap = getattr(self, 'gap_dt', 0.0) > 0.0 and dt > self.gap_dt
        if gap:
            cap = getattr(self, 'gap_step_cap_m', 0.0)
            if cap > 0.0:
                step = float(np.clip(step, -cap, cap))
        # Per-update noise is per-TIME noise only at one rate; scale it so the
        # cloud diffuses at the same m/sqrt(s) whatever the host does.
        ns = (vehicle.noise_scale_for_dt(dt, self.rate_nominal)
              if self.rate_nominal > 0.0 else 1.0)
        noise = self.noise_xy * ns * (4.0 if jumped else 1.0)
        # ...and a gap is a reason to be LESS certain, not to relocalize: widen
        # the cloud over the blind window and let the scan update pick the pose
        # out of it.
        gm = float(getattr(self, 'gap_noise_mult', 1.0)) if gap else 1.0
        if gm > 1.0:
            noise = max(noise, self.noise_xy * ns * gm)
        if jumped:
            self.get_logger().warn('encoder jump: teleport assumed, spreading particles')
        # What the motion model was told, kept for `anchor_along`. getattr is
        # not needed to WRITE onto the synthetic namespace the rate-invariance
        # tests lift `predict` onto.
        self._last_step = float(step)
        self._last_v = float(v)
        self._last_jumped = bool(jumped)
        # Process noise, isotropic by default. `motion_noise_along` > 0 splits
        # it into an along-heading and a lateral component: on a straight only
        # the lateral one is ever corrected by the scan, so the along one is
        # free diffusion that the filter cannot undo. The default path below
        # is untouched, RNG draws included, so the shipped filter is
        # bit-identical.
        noise_a = float(getattr(self, 'noise_along', 0.0))
        if noise_a > 0.0 and self.noise_xy > 0.0:
            # Carry whatever `noise` picked up from the jump and gap
            # multipliers, so a teleport still spreads the cloud in BOTH
            # directions; only the ratio of the two is `motion_noise_along`.
            na = np.random.normal(0, noise * (noise_a / self.noise_xy),
                                  self.n_p)
            nl = np.random.normal(0, noise, self.n_p)
            c, s = np.cos(head), np.sin(head)
            self.px += step * c + na * c - nl * s
            self.py += step * s + na * s + nl * c
        else:
            self.px += step * np.cos(head) + np.random.normal(0, noise, self.n_p)
            self.py += step * np.sin(head) + np.random.normal(0, noise, self.n_p)
        # The heading-offset walk is per-update too, and scaling only the xy
        # one would make the RATIO of translational to rotational diffusion
        # rate-dependent -- worse than either being uniformly wrong.
        self.pth += np.random.normal(0, self.noise_th * ns * gm, self.n_p)

    def anchor_along(self, dt):
        """Bound the cloud's along-track motion by the encoder's own distance.

        On a long straight with parallel walls the scan update constrains the
        lateral offset and the heading and leaves the along-track coordinate
        free, so the weighted mean walks along the corridor at whatever rate
        the process noise and the resampling happen to allow. Measured on the
        finals logs it moves the published pose by up to 1.27 m in ONE 0.097 s
        scan, where the car itself covers 0.78 m.

        The encoders do measure that direction, and well: on the straight the
        filtered encoder speed sits within -0.10 m/s (p50) of the IPS truth.
        So the along-track advance of the estimate is held to the distance the
        motion model was given, plus or minus `along_clamp_mps * dt`. The
        correction shifts every particle, not just the mean, so the cloud the
        next scan sees is the cloud the controller was told about.

        Returns the metres removed (signed, 0.0 when the clamp is off or
        stood down), so a caller can measure how often it fires.
        """
        if not (float(getattr(self, 'along_clamp_mps', 0.0)) > 0.0) \
                or not dt > 0.0:
            return 0.0
        hx = float(np.average(self.px, weights=self.w))
        hy = float(np.average(self.py, weights=self.w))
        head = self.imu_yaw + float(np.average(self.pth, weights=self.w))
        prev, self._prev_mean = self._prev_mean, (hx, hy)
        # Stand down wherever a big along-track jump is the RIGHT answer: the
        # first scan, a centerline-wide relocalization, a detected teleport or
        # encoder jump, and any speed at which the encoder cannot vouch for a
        # distance.
        if (prev is None or self._just_relocalized or self._last_jumped
                or getattr(self, '_tp_hold', 0) > 0
                or abs(self._last_v) < float(self.along_clamp_min_v)):
            return 0.0
        c, s = math.cos(head), math.sin(head)
        adv = (hx - prev[0]) * c + (hy - prev[1]) * s
        slack = float(self.along_clamp_mps) * dt
        hi, lo = self._last_step + slack, self._last_step - slack
        excess = adv - hi if adv > hi else (adv - lo if adv < lo else 0.0)
        if excess == 0.0:
            return 0.0
        self.px -= excess * c
        self.py -= excess * s
        self._prev_mean = (hx - excess * c, hy - excess * s)
        self._anchor_events += 1
        self._anchor_total += abs(excess)
        return excess

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

        # Particles inside a wall or off the map are impossible.
        # `floor`, not a cast: `astype` truncates TOWARDS ZERO, so a particle
        # up to one cell below the map origin lands on index 0 and reads as
        # INSIDE the map -- the one direction in which "off the map" must not
        # be forgiving, because the weight that should be zero then comes from
        # a real cell. Identity for every particle at or above the origin,
        # which is every particle in a healthy run.
        pgx = np.floor((self.px - self.ox) / self.res).astype(np.int32)
        pgy = np.floor((self.py - self.oy) / self.res).astype(np.int32)
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
        # Move the published point back along the heading, so what the
        # controller receives is the rear axle rather than the IPS point the
        # lever arm implies. Output only: the cloud, the particle topic and
        # everything the next scan sees are untouched, which is the whole
        # point of having this separate from `lidar_offset_override`.
        # getattr: the audit tests lift `publish` onto synthetic namespaces.
        # Non-finite arithmetic falls through to the guard below.
        d = float(getattr(self, 'pose_along_offset', 0.0) or 0.0)
        if d != 0.0:
            x -= d * math.cos(yaw)
            y -= d * math.sin(yaw)
        # A NaN here is worse than a missing pose: the controller's stale gate
        # covers a gap, and nothing covers a pose that is not a number. The
        # weights are normalized on every update, so this cannot fire in a
        # healthy run -- it is the last line before the wire.
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            self.get_logger().warn('non-finite pose; not publishing')
            return
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
