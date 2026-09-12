"""Pose-stamp-to-yaw-rate timing, measured IN THE RACE LOOP.

The open question this exists to answer
---------------------------------------
On the real display (sensor period 0.058 s) the car's turn-in arrives
0.12-0.22 m earlier in arc length than on Xvfb (0.095 s) with identical
commands and speeds -- a 0.03-0.055 s phase shift somewhere between the
published command and the achieved curvature.  Neither the offline harness
nor the open-floor staircase (`sysid_run`) can see it:

  * the harness does not reproduce it at all (STATUS, "Closed-loop timing
    hunt"), so it is a property of the machine, not of the model;
  * `sysid_run` publishes on a 50 Hz timer that is ASYNCHRONOUS to the
    telemetry tick, so its command-to-feedback fit averages over a whole
    random phase and cannot separate `period + c` from `bridge + period/2`
    (STATUS, "Dead-time law settled" settled the *command* law on six race
    logs, but only for the command; nothing measured the pose stamp to the
    achieved YAW RATE, which is the quantity the phase shift is about).

So this node measures the whole chain, on the track, at whatever rate the
simulator happens to be running, with the publish PHASE-LOCKED TO THE TICK
exactly as `mpc3`'s is -- everything is published from inside the pose
callback, nothing is on a timer except the safety watchdog.

The experiment
--------------
Drive the SAFE line (not the shipping min-curvature one -- measured, it never
has 0.6 m of corridor anywhere flat) at a constant probe speed (curvature-limited in the
corners so the car survives them, flat on the straights where it matters),
steering with a plain pure-pursuit law.  At each of N trigger points chosen
offline on the STRAIGHTS, replace the steering command with a RAMP to
``amp_rad`` at ``ramp_radps`` on top of the tracking command, hold for
``pulse_s``, then revert and let the pure-pursuit law re-acquire the line.
The sign alternates so the excursions cancel.  A ramp rather than a step
because the rack slews at 3.2 rad/s and would cover a 0.12 rad step inside
half a telemetry period, leaving nothing but an integer tick count -- see
`pulse_steer`.

Defaults: 0.12 rad at 0.25 rad/s (a 0.48 s edge: 5 samples at 0.104 s, 8 at
0.058 s), held 0.70 s, at 2.5 m/s, three trigger points a lap.  All of that is
bounded by the track: Porto's longest stretch that is both flatter than
kappa 0.15 and wider than 0.60 m of corridor is 4.9 m, i.e. 2.0 s at the probe
speed.

Every telemetry message writes a row, tagged with which stream produced it,
so the offline fit sees each signal at its own arrival instants:

    src=pose  a /pit_crew/pose message (the control instant; this row's
              `t_pub` is the command this very callback published)
    src=fb    a /steering message -- the ACTUATOR angle, in radians
    src=imu   an /imu message -- the gyro, i.e. the achieved yaw rate
    src=ips   an /ips message -- ground-truth position, for an independent
              yaw rate that does not go through the IMU at all
    src=enc   an encoder message

Sub-sample resolution at 10 Hz comes from the ramp, not from counting
samples.  Inside the simulator the actuator feedback is tick-quantised -- the
rack finishes each commanded increment between ticks -- so the
command-to-feedback dead time is only ever an integer number of periods plus
whatever the bridge adds, which is the law STATUS already settled.  The YAW
RATE is different: it is a continuous state the physics integrates at its own
fixed step, so its rising edge lands wherever it really lands.  Least-squares
through the rising yaw samples, extrapolated back to the pre-pulse baseline,
puts the curvature onset between samples.  That is the number the phase shift
is about, and `tools/turnin_fit.py` fits it.

Running it
----------
    VISIBLE=1 ./scripts/turnin_probe.sh          # the real display (~0.058 s)
                ./scripts/turnin_probe.sh        # Xvfb :99 (~0.095-0.104 s)

then

    .venv/bin/python tools/turnin_fit.py data/logs/turnin_<ts>.csv --per-pulse
    .venv/bin/python tools/turnin_fit.py --compare <display.csv> <xvfb.csv>

`NO_LOC=1` runs it straight off IPS with no localization at all -- the control
that says whether localization is itself carrying the rate dependence.

This node is an INSTRUMENT, not a racing node: it reads IPS, which is a
runtime-restricted topic and would not be legal in a race.  It is also not a
safety-rated one -- see `guard()` for the three things that stop it.
"""

import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32

from pit_crew import vehicle
from pit_crew.mpc2_core import RefPath, wrap
from pit_crew.throttle2 import ServoThrottle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pick_triggers(path, pre_m, flat_m, travel_m, kappa_max, clear_min,
                  clear_travel, n_pulses, gap_m):
    """Arc lengths at which a pulse may start: straight, wide, and spaced.

    Two windows, because they ask different questions.

    * The MEASUREMENT window, ``pre_m`` behind through ``flat_m`` ahead, must
      be below ``kappa_max`` of curvature and keep ``clear_min`` of corridor
      on both sides.  Straightness is not cosmetic: on a bend the
      pure-pursuit command is nonzero, so the ramp is no longer a ramp from
      zero and the baseline the fit extrapolates to is contaminated.
    * The TRAVEL window, the same ``pre_m`` behind through ``travel_m`` ahead
      (the pulse plus the whole recovery), only has to keep
      ``clear_travel``.  Porto is 28.5 m long and its longest stretch under
      kappa 0.15 with 0.60 m of corridor is 4.9 m on the safe line and
      **0.3 m on the shipping min-curvature line**, which hugs the inside --
      measured, not assumed.  Demanding a straight long enough for the
      recovery too admits nothing at all.

    Returns at most ``n_pulses``, greedily, widest-clearance first and never
    closer together than ``gap_m``.
    """
    n = path.n
    if n < 8:
        return []
    ds = float(path.spacing)
    back = max(1, int(round(pre_m / max(ds, 1e-3))))
    flat = max(1, int(round(flat_m / max(ds, 1e-3))))
    trav = max(flat, int(round(travel_m / max(ds, 1e-3))))
    clear = np.minimum(path.w_left, path.w_right)
    if not np.isfinite(clear).any():        # no map loaded: gate is off
        clear = np.full(n, max(clear_min, clear_travel))
    score = np.full(n, -np.inf)
    for i in range(n):
        w = (np.arange(i - back, i + flat + 1) % n)
        kmax = float(np.abs(path.kappa[w]).max())
        if kmax > kappa_max or float(clear[w].min()) < clear_min:
            continue
        t = (np.arange(i - back, i + trav + 1) % n)
        if float(clear[t].min()) < clear_travel:
            continue
        score[i] = float(clear[w].min()) - 0.02 * kmax
    out = []
    total = float(path.s[-1] + ds)
    for i in np.argsort(-score):
        if not np.isfinite(score[i]):
            break
        s_i = float(path.s[i])
        if all(min(abs(s_i - s_j), total - abs(s_i - s_j)) >= gap_m
               for s_j in out):
            out.append(s_i)
        if len(out) >= n_pulses:
            break
    return sorted(out)


class TurnInProbe(Node):

    def __init__(self):
        super().__init__('turnin_probe')
        p = self.declare_parameters('', [
            # NOT the shipping min-curvature line: measured, it never has
            # 0.6 m of corridor anywhere flat (0.3 m of admissible straight on
            # the whole lap).  The safe line sits in the middle of the road --
            # 0.52 m minimum, 0.68 m median, 4.9 m of flat-and-wide straight.
            ('raceline_csv', '/home/data/racelines/porto_mesh_safe.csv'),
            ('map_prefix', '/home/data/maps/porto_mesh'),
            ('pose_topic', '/pit_crew/pose'),   # 'ips' to trigger off IPS
            # --- the pulse ---------------------------------------------------
            ('amp_rad', 0.12),          # ramp height, rad at the road wheel
            ('ramp_radps', 0.25),       # 0 = a true step; see `pulse_steer`
            ('pulse_s', 0.70),          # held (0.48 s of ramp, then a plateau)
            ('recover_s', 1.00),        # pure-pursuit re-acquisition after it
            ('n_pulses', 3),            # trigger points per lap
            ('trigger_gap_m', 4.0),     # minimum spacing between them
            ('alternate_sign', True),
            # --- the cruise --------------------------------------------------
            ('v_probe', 2.5),           # m/s held on the straights
            ('ay_cruise', 3.0),         # m/s2; caps the speed in the corners
            ('a_brake', 3.5),           # m/s2 used to plan the corner entries
            ('lookahead_m', 0.70),      # pure pursuit, at v_probe
            ('lookahead_tv', 0.25),     # ...plus this many seconds of travel
            ('warmup_s', 8.0),          # settle before the first pulse
            # --- gates and guards --------------------------------------------
            ('kappa_max', 0.15),        # 1/m; a trigger window must be flatter
            ('clear_min', 0.60),        # m of corridor either side, at the pulse
            ('clear_travel', 0.45),     # ...and through the recovery after it
            ('pre_m', 0.60),            # settled straight required BEFORE the pulse
            ('cte_gate', 0.15),         # m; do not fire further off the line
            ('v_gate', 0.40),           # m/s; nor at the wrong speed
            # The quiet gates have to admit the straight the pulse lives on,
            # not just a stationary car: at v_probe on a stretch at kappa_max
            # the car is legitimately yawing at v*kappa_max (0.38 rad/s) with
            # atan(L*kappa_max) (0.05 rad) of lock on.  Set below that and
            # nothing ever fires; the discriminator against a post-pulse swing
            # (1.4 rad/s, measured) is still wide.
            ('gyro_gate', 0.45),        # rad/s; do not fire while turning hard
            ('gyro_span', 0.25),        # rad/s of SWING over the settle window
            ('steer_gate', 0.10),       # rad of actuator; nor with lock on
            ('settle_s', 0.30),         # s of CONSECUTIVE quiet before a pulse
            ('cte_abort', 0.45),        # m; cut a pulse short beyond this
            ('lidar_stop_m', 0.40),     # m; stop the whole run
            ('max_pulses', 400),        # stop after this many
            ('max_run_s', 900.0),
            ('stale_pose_s', 0.50),     # watchdog: coast if the pose dries up
            ('out_dir', '/home/data/logs'),
        ])
        # Positional unpacking of 28 parameters is a bug waiting to happen --
        # insert one and every later field silently shifts. Bind by name.
        V = {x.name: x.value for x in p}
        rl_csv, map_prefix = V['raceline_csv'], V['map_prefix']
        self.pose_topic = V['pose_topic']
        self.amp, self.ramp = V['amp_rad'], V['ramp_radps']
        self.pulse_s, self.recover_s = V['pulse_s'], V['recover_s']
        self.n_pulses, self.gap_m = V['n_pulses'], V['trigger_gap_m']
        self.alternate = V['alternate_sign']
        self.v_probe, self.ay_cruise = V['v_probe'], V['ay_cruise']
        self.a_brake = V['a_brake']
        self.la_m, self.la_tv = V['lookahead_m'], V['lookahead_tv']
        self.warmup_s = V['warmup_s']
        self.kappa_max, self.clear_min = V['kappa_max'], V['clear_min']
        self.clear_travel, self.pre_m = V['clear_travel'], V['pre_m']
        self.cte_gate, self.v_gate = V['cte_gate'], V['v_gate']
        self.gyro_gate, self.steer_gate = V['gyro_gate'], V['steer_gate']
        self.gyro_span = V['gyro_span']
        self.settle_s = V['settle_s']
        self.cte_abort, self.lidar_stop = V['cte_abort'], V['lidar_stop_m']
        self.max_pulses, self.max_run_s = V['max_pulses'], V['max_run_s']
        self.stale_pose_s = V['stale_pose_s']
        out_dir = V['out_dir']

        self.path = RefPath(rl_csv)
        if map_prefix:
            try:
                from pit_crew import pf_core
                dist, _img, meta = pf_core.load_map(map_prefix)
                self.path.load_corridor(dist, meta)
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().warning(
                    f'no corridor from {map_prefix!r} ({exc}); the clearance '
                    f'gate on the trigger points is OFF')

        self.thr = ServoThrottle()
        self.speed_est = vehicle.SpeedEstimator()
        self.total_s = float(self.path.s[-1] + self.path.spacing)
        v = max(self.v_probe, 0.5)
        self.triggers = pick_triggers(
            self.path, self.pre_m, v * self.pulse_s,
            v * (self.pulse_s + self.recover_s), self.kappa_max,
            self.clear_min, self.clear_travel, int(self.n_pulses), self.gap_m)

        # -- state
        self.pose = None            # (x, y, yaw, stamp)
        self.idx = None
        self.s = 0.0
        self.cte = 0.0
        self.gyro = 0.0
        self.t_imu = 0.0
        self.steer_fb = 0.0
        self.t_fb = 0.0
        self.ips = None
        self.t_ips = 0.0
        self.ips_wz = 0.0
        self._ips_trail = []
        self.lidar_min = float('inf')
        self.cmd_steer = 0.0
        self.cmd_thr = 0.0
        self.pulse_base = 0.0
        self.quiet_run = 0
        self._gyros = []
        self._stamps = []
        self.t_pub = 0.0
        self.phase = 'warmup'       # warmup -> cruise -> pulse -> recover
        self.t_phase = None
        self.t_start = None
        self.pulse_id = 0
        self.sign = 1.0
        self.armed = set()          # trigger s values not yet consumed this lap
        self.last_s = None
        self.done = False
        self.abort = None
        self.fired = []

        os.makedirs(out_dir, exist_ok=True)
        self.path_csv = os.path.join(
            out_dir, 'turnin_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
        self.fh = open(self.path_csv, 'w', buffering=1)
        self.fh.write('src,t_recv,pose_stamp,x,y,yaw,s,cte,pulse_id,phase,'
                      'cmd_steer,t_pub,steer_fb,t_fb,gyro_z,t_imu,'
                      'ips_x,ips_y,ips_wz,t_ips,v_enc,cmd_thr,lidar_min\n')

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        if self.pose_topic == 'ips':
            self.create_subscription(Point, f'{ns}/ips', self.on_ips_pose, qos)
        else:
            self.create_subscription(PoseStamped, self.pose_topic,
                                     self.on_pose, 1)
            self.create_subscription(Point, f'{ns}/ips', self.on_ips, qos)
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.create_subscription(Float32, f'{ns}/steering', self.on_fb, qos)
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'{ns}/{side}_encoder',
                lambda m, w=side: self.on_enc(w, m), qos)
        self.pub_steer = self.create_publisher(
            Float32, f'{ns}/steering_command', 1)
        self.pub_thr = self.create_publisher(
            Float32, f'{ns}/throttle_command', 1)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'turnin_probe: {len(self.triggers)} trigger points at s '
            f'{[round(s, 2) for s in self.triggers]} on a {self.total_s:.1f} m '
            f'line; step {self.amp:+.3f} rad for {self.pulse_s:.2f} s at '
            f'{self.v_probe:.1f} m/s -> {self.path_csv}')
        if not self.triggers:
            self.get_logger().error(
                'no straight is both flatter than kappa_max and wider than '
                'clear_min for the whole pulse window; relax one of them')

    # -- inputs ------------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg: Imu):
        if math.isfinite(msg.angular_velocity.z):
            self.gyro = float(msg.angular_velocity.z)
            self.t_imu = self.now()
        self.row('imu')

    def on_fb(self, msg: Float32):
        if math.isfinite(msg.data):
            self.steer_fb = float(msg.data)
            self.t_fb = self.now()
        self.row('fb')

    def on_enc(self, wheel, msg: JointState):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])
        self.row('enc')

    def on_scan(self, msg: LaserScan):
        good = [r for r in msg.ranges
                if vehicle.LIDAR_RANGE_MIN < r < vehicle.LIDAR_RANGE_MAX]
        self.lidar_min = min(good) if good else float('inf')

    def on_ips(self, msg: Point):
        """Ground truth, logged only -- an independent yaw rate for the fit."""
        t = self.now()
        self.ips = (float(msg.x), float(msg.y))
        self.t_ips = t
        self._ips_trail.append((t, self.ips[0], self.ips[1]))
        while len(self._ips_trail) > 12:
            self._ips_trail.pop(0)
        self.ips_wz = self.ips_yaw_rate()
        self.row('ips')

    def on_ips_pose(self, msg: Point):
        """`pose_topic:=ips` -- run without localization at all.

        The Point carries no stamp, so the "pose stamp" is the receipt time
        and the total delay this measures is missing the transport from the
        simulator.  Use it to check that localization is not itself the
        rate-dependent term, not as the primary number.
        """
        self.on_ips(msg)
        if self.ips is None:
            return
        yaw = self.ips_course()
        if yaw is None:
            return
        self.accept_pose(self.ips[0], self.ips[1], yaw, self.t_ips)

    def ips_yaw_rate(self):
        """d(course)/dt from the IPS trail. Lags; a cross-check, not a truth."""
        tr = self._ips_trail
        if len(tr) < 3:
            return 0.0
        h0 = self.ips_course(upto=-1)
        h1 = self.ips_course(upto=None)
        dt = tr[-1][0] - tr[-2][0]
        if h0 is None or h1 is None or dt <= 1e-3:
            return 0.0
        return float(wrap(h1 - h0) / dt)

    def ips_course(self, upto=None):
        tr = self._ips_trail if upto is None else self._ips_trail[:upto]
        if len(tr) < 2:
            return None
        t1, x1, y1 = tr[-1]
        for t0, x0, y0 in reversed(tr[:-1]):
            if math.hypot(x1 - x0, y1 - y0) > 0.05:
                return math.atan2(y1 - y0, x1 - x0)
        return None

    def on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        yaw = yaw_from_quat(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not all(math.isfinite(q) for q in (x, y, yaw, stamp)):
            return
        now = self.now()
        if stamp <= 0.0 or now - stamp > self.stale_pose_s:
            return
        if self.pose is not None and stamp <= self.pose[3]:
            return
        self.accept_pose(x, y, yaw, stamp)

    # -- the loop ----------------------------------------------------------

    def accept_pose(self, x, y, yaw, stamp):
        """One control instant, phase-locked to the telemetry tick.

        This is the whole point of the node: `mpc3` publishes from inside this
        callback, so the probe does too.  A timer publisher (what `sysid_run`
        uses) lands at a uniformly random phase inside the period and smears
        exactly the 0.03-0.055 s being hunted.
        """
        if self.done:
            return
        t = self.now()
        if self.t_start is None:
            self.t_start = t
            self.t_phase = t
        self.pose = (x, y, yaw, stamp)
        self.idx = self.path.nearest(x, y, self.idx, yaw=yaw)
        self.s = float(self.path.s[self.idx])
        px, py = self.path.xy[self.idx]
        pyaw = float(self.path.yaw[self.idx])
        self.cte = float(-(x - px) * math.sin(pyaw) + (y - py) * math.cos(pyaw))

        why = self.guard()
        if why:
            self.abort = why
            self.finish()
            return

        self._gyros.append(self.gyro)
        self.quiet_run = (self.quiet_run + 1
                          if (abs(self.gyro) <= self.gyro_gate
                              and abs(self.steer_fb) <= self.steer_gate
                              and abs(self.cte) <= self.cte_gate)
                          else 0)
        keep = max(3, int(math.ceil(self.settle_s / max(self.period_est(),
                                                        1e-3))))
        while len(self._gyros) > keep:
            self._gyros.pop(0)
        self._stamps.append(stamp)
        if len(self._stamps) > 24:
            self._stamps.pop(0)
        self.advance(t)
        steer = self.steer_command(t)
        self.cmd_steer = float(steer)
        self.cmd_thr = float(self.thr.command(
            self.speed_target(), self.speed_est.speed,
            now=t, kappa=float(self.path.kappa[self.idx])))
        self.t_pub = self.now()
        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(steer)))
        self.pub_thr.publish(Float32(data=self.cmd_thr))
        self.row('pose')

    def advance(self, t):
        """The pulse state machine. Runs once per accepted pose."""
        elapsed = t - (self.t_phase if self.t_phase is not None else t)
        if self.phase == 'warmup':
            if elapsed >= self.warmup_s:
                self.enter('cruise', t)
        elif self.phase == 'cruise':
            if self.should_fire():
                self.pulse_id += 1
                self.pulse_base = float(self.cmd_steer)
                self.fired.append((self.pulse_id, self.s, self.sign))
                self.get_logger().info(
                    f'pulse {self.pulse_id}: {self.sign * self.amp:+.3f} rad '
                    f'at s {self.s:.2f}, v {self.speed_est.speed:.2f} m/s, '
                    f'cte {self.cte:+.3f} m')
                self.enter('pulse', t)
        elif self.phase == 'pulse':
            if elapsed >= self.pulse_s or abs(self.cte) > self.cte_abort:
                if abs(self.cte) > self.cte_abort:
                    self.get_logger().warning(
                        f'  pulse {self.pulse_id} cut short at cte '
                        f'{self.cte:+.3f} m')
                if self.alternate:
                    self.sign = -self.sign
                self.enter('recover', t)
        elif self.phase == 'recover':
            if elapsed >= self.recover_s:
                if self.pulse_id >= self.max_pulses:
                    self.finish()
                    return
                self.enter('cruise', t)
        self.track_lap()

    def enter(self, phase, t):
        self.phase = phase
        self.t_phase = t
        if phase == 'pulse':
            self.quiet_run = 0

    def period_est(self):
        """The sensor period this run is actually getting, from pose stamps."""
        if len(self._stamps) < 4:
            return float(vehicle.RATE_NOMINAL)
        d = np.diff(np.asarray(self._stamps, float))
        d = d[(d > 1e-3) & (d < 1.0)]
        return float(np.median(d)) if len(d) else float(vehicle.RATE_NOMINAL)

    def track_lap(self):
        """Re-arm every trigger once the car has wrapped past the line."""
        if self.last_s is not None and self.s < self.last_s - 0.5 * self.total_s:
            self.armed = set(self.triggers)
        elif self.last_s is None:
            self.armed = set(self.triggers)
        self.last_s = self.s

    def should_fire(self):
        """Every gate here is a measurement gate as much as a safety one.

        The car must be STRAIGHT and QUIET, not merely near the line.  A pulse
        fired while the previous one was still being unwound has a yaw rate
        already moving in the excitation's direction, and the fit -- which
        finds the edge as a fraction of the plateau above a pre-pulse baseline
        -- then dates the onset early.  Measured against a plant with a known
        delay, ungated pulses scattered the fitted total by +-0.15 s; with the
        yaw, steering and settle gates they land inside one sample.
        """
        if self.pose is None or not self.triggers:
            return False
        if abs(self.cte) > self.cte_gate:
            return False
        v = self.speed_est.speed
        if abs(v - self.v_probe) > self.v_gate:
            return False
        # SUSTAINED quiet, not instantaneous quiet.  The car leaves a pulse
        # yawed and offset, the pure-pursuit recovery swings it back, and the
        # swing crosses zero: sampled once, |gyro| looks small at exactly the
        # moment the car is turning fastest through the middle.  Alternating
        # the sign makes that worse, because the recovery swing has the sign
        # of the NEXT pulse.  Counting consecutive quiet poses cannot be
        # fooled by a zero crossing.
        if self.quiet_run * max(self.period_est(), 1e-3) < self.settle_s:
            return False
        # ...and the SWING, not just the level.  A post-pulse oscillation
        # crosses zero, and at 0.104 s three consecutive samples of a swing
        # through zero all sit inside `gyro_gate` -- measured: 0.385, -0.005,
        # -0.342 rad/s in a row, which passed every magnitude gate and then
        # contaminated the fit by 0.16 s.  The range over the settle window
        # cannot be crossed that way.
        if self._gyros and (max(self._gyros) - min(self._gyros)) > self.gyro_span:
            return False
        if self.lidar_min < max(self.lidar_stop, self.clear_min):
            return False
        for s_t in sorted(self.armed):
            d = (self.s - s_t) % self.total_s
            if d < 0.5 or d > self.total_s - 0.25:   # just crossed it
                self.armed.discard(s_t)
                return True
        return False

    # -- the driver --------------------------------------------------------

    def speed_target(self):
        """Constant on the straights, grip-limited into the corners.

        A flat setpoint everywhere would ask Porto's tightest corner
        (kappa 1.019) for 9 m/s2 at 3 m/s.  The pulses only happen where
        `kappa_max` holds, so the straights the measurement lives on are at
        exactly `v_probe` and the corners are merely survived.
        """
        n = self.path.n
        ahead = int(round(2.5 * max(self.v_probe, 0.5) / max(self.path.spacing, 1e-3)))
        idx = (np.arange(self.idx, self.idx + max(ahead, 2)) % n)
        k = np.abs(self.path.kappa[idx])
        v_k = np.where(k > 1e-3,
                       np.sqrt(self.ay_cruise / np.maximum(k, 1e-3)),
                       self.v_probe)
        v_k = np.minimum(v_k, self.v_probe)
        ds = (np.arange(len(idx))) * self.path.spacing
        return float(max(0.5, np.min(np.sqrt(v_k ** 2 + 2.0 * self.a_brake * ds))))

    def steer_command(self, t=None):
        if self.phase == 'pulse':
            return self.pulse_steer(self.now() if t is None else t)
        return self.pure_pursuit()

    def pulse_steer(self, t):
        """The excitation: a RAMP to `amp_rad`, not a step. Here is why.

        A step would be unmeasurable.  The rack slews at 3.2 rad/s, so it
        covers 0.15 rad in 0.047 s -- less than half a telemetry period -- and
        every `/steering` sample after the dead time is already at the
        plateau.  The feedback then carries an integer number of ticks and
        nothing else, which is precisely the resolution that cannot answer a
        0.03-0.055 s question.

        A ramp at `ramp_radps` (well under the 3.2 rad/s slew, so the rack
        tracks it rather than rate-limiting) puts 0.33 s of rising edge into
        the data: 3 samples at 0.104 s and 6 at 0.058 s on the feedback, and
        the same on the yaw rate, which the physics integrates continuously
        between ticks.  Least-squares through that ramp, extrapolated back to
        the baseline, resolves the onset to a fraction of a sample -- see
        `tools/turnin_fit.py`.

        The excitation is ADDED to `pulse_base`, the pure-pursuit command in
        force when the pulse began, rather than replacing it.  Replacing it
        costs the measurement: the rack then first travels from wherever it
        was tracking back to zero and only afterwards starts the ramp, so on
        the pulses where that residual has the same sign as the excitation the
        edge is late.  Measured in `tests/test_turnin_probe.py` against a plant
        with a known delay, that bias was +0.08 s -- twice the effect being
        hunted -- and it vanishes when the ramp starts from the held value.

        `ramp_radps:=0` restores the plain step, which is still the right
        shape for confirming the integer-tick command law on its own.
        """
        base = self.pulse_base
        if self.ramp <= 0.0:
            d = base + self.sign * self.amp
        else:
            held = t - (self.t_phase if self.t_phase is not None else t)
            d = base + self.sign * min(self.amp, self.ramp * max(held, 0.0))
        return float(max(-vehicle.MAX_STEER, min(vehicle.MAX_STEER, d)))

    def pure_pursuit(self):
        """Plain geometric tracker -- the probe does not need mpc3's line.

        Using `mpc3` here would put the thing under test inside the
        instrument: its dead-time model, its `nd`, its barrier and its rate
        terms all shift the command in time, which is the measurement.  A
        lookahead arc has no memory and no delay model at all.
        """
        if self.pose is None:
            return 0.0
        x, y, yaw, _ = self.pose
        L = max(0.35, self.la_m + self.la_tv * max(self.speed_est.speed, 0.0))
        n = self.path.n
        step = max(1, int(round(L / max(self.path.spacing, 1e-3))))
        j = (self.idx + step) % n
        tx, ty = self.path.xy[j]
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        d = max(math.hypot(tx - x, ty - y), 1e-3)
        delta = math.atan2(2.0 * vehicle.WHEELBASE * math.sin(alpha), d)
        return float(max(-vehicle.MAX_STEER, min(vehicle.MAX_STEER, delta)))

    # -- guards, logging, shutdown ----------------------------------------

    def guard(self):
        if self.lidar_min < self.lidar_stop:
            return f'obstacle at {self.lidar_min:.2f} m'
        if self.t_start is not None and self.now() - self.t_start > self.max_run_s:
            return f'time limit {self.max_run_s:.0f} s'
        return None

    def watchdog(self):
        """No pose for `stale_pose_s`: straighten and coast. Commands latch."""
        if self.done:
            return
        if self.pose is None or self.now() - self.pose[3] <= self.stale_pose_s:
            return
        self.cmd_steer = 0.0
        self.cmd_thr = 0.0
        self.pub_steer.publish(Float32(data=0.0))
        self.pub_thr.publish(Float32(data=0.0))

    def row(self, src):
        if self.fh.closed or self.pose is None:
            return
        x, y, yaw, stamp = self.pose
        ix, iy = self.ips if self.ips else (float('nan'), float('nan'))
        self.fh.write(
            f'{src},{self.now():.6f},{stamp:.6f},{x:.4f},{y:.4f},{yaw:.5f},'
            f'{self.s:.4f},{self.cte:+.4f},{self.pulse_id},{self.phase},'
            f'{self.cmd_steer:+.5f},{self.t_pub:.6f},{self.steer_fb:+.5f},'
            f'{self.t_fb:.6f},{self.gyro:+.5f},{self.t_imu:.6f},'
            f'{ix:.4f},{iy:.4f},{self.ips_wz:+.5f},{self.t_ips:.6f},'
            f'{self.speed_est.speed:.3f},{self.cmd_thr:.4f},'
            f'{self.lidar_min:.3f}\n')

    def finish(self):
        if self.done:
            return
        self.done = True
        for _ in range(3):
            self.pub_steer.publish(Float32(data=0.0))
            self.pub_thr.publish(Float32(data=0.0))
        if self.abort:
            self.get_logger().warning(f'STOPPED: {self.abort}')
        self.get_logger().info(
            f'{self.pulse_id} pulses fired; rows in {self.path_csv}')
        if not self.fh.closed:
            self.fh.close()


def main():
    rclpy.init()
    node = TurnInProbe()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        # Commands latch: leave the car stopped and straight, whatever happened.
        for _ in range(5):
            node.pub_steer.publish(Float32(data=0.0))
            node.pub_thr.publish(Float32(data=0.0))
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
