"""Is a MODERATE negative throttle a proportional brake, or just reverse drive?

The question
------------
`VehicleController.Drive()` applies `brakeTorque = MotorTorque` (428 N.m, all
four wheels) only when the throttle is **exactly 0.0**; that locks the wheels
and buys a held ~4.55 m/s2 plus drag -- the tyre's post-peak longitudinal
asymptote, 0.464 g.  It is the only brake the stack owns, and `throttle2`'s
`brake_mode:='zero'` is what finally made the finals entry legal.

But the longitudinal tyre peaks at ~7.1 m/s2 (mu 0.72) at a small slip ratio,
and a locked wheel is far past that peak.  A NEGATIVE throttle is reverse DRIVE
torque, which on a wheel still rolling forward is a retarding torque the tyre
can transmit *without* locking -- so the hypothesis is that some moderate
negative command decelerates roughly in proportion to the torque, up to the
tyre peak, i.e. **harder than the locked 4.55 m/s2 and with the wheels still
turning**.  If that is true, `brake_mode` has a third setting worth having.

Why the August evidence does not settle it: `brake_gain` 0.3 put the car in
reverse for 52% of a run and took 176 collisions, but that was (a) with the
wheel-encoder speed estimate that was broken until Aug 30, reading down to
-10.8 m/s, and (b) mostly at low speed, where any negative command is simply
reverse.  Neither says what a -0.15 command does to a car doing 7 m/s.  This
node measures that, one command at a time, on ground truth.

Method
------
Per trial, on the open floor: steer straight, accelerate at `u_accel` until the
IPS-differentiated speed reaches `v_target`, then publish one constant test
command `u_brake` and measure what the car does until it is down to `v_stop`.
Ground truth is IPS, not the encoder -- the encoder is exactly the signal that
cannot tell a locked wheel from a stopped car, and that difference is half the
answer.  Both are logged: `slip = 1 - v_enc/v_ips` over the brake window
separates "the tyre is retarding the car" (slip small) from "the wheels are
locked" (slip -> 1).

`u_brakes` is swept as a list, always including **0.0**, which is the locked
four-wheel brake and therefore the reference every other command is judged
against.  The summary prints the fitted deceleration per command next to it.

**Run this on the `-explore` image**, which has no track at all, only `Floor`:

    TAG=2026-iros-explore ./scripts/sim_restart_connect.sh
    colcon build --packages-select pit_crew --symlink-install
    ros2 run pit_crew brake_probe
    ros2 run pit_crew brake_probe --ros-args -p u_brakes:="[0.0,-0.1,-0.2]"

`skidpad.py` learned the hard way what happens when a grip instrument is
pointed at a world with walls, and `sysid_run.py` learned that the LiDAR guard
is useless there (103 rows, every range `inf`), so the guards here are
geometric instead: the run aborts, with the actuators zeroed -- which is itself
the full brake -- if the car gets more than `fence_m` from its spawn point, if
the yaw rate exceeds `yaw_abort` while it is supposed to be going straight (the
drive home is allowed its U-turn, capped by what it asks of the tyres), if the
IPS goes quiet, or if it overspeeds.  Inside that hard fence there is a soft
one: a command too weak to stop the car within `fence_m - fence_margin_m` ends
its own window there and the sweep carries on, because a 1 m/s2 command needs
25 m to come down from 7 m/s and that is the whole floor we trust.

Between trials the car drives back towards the spawn point at `home_speed`
(bearing minus the course angle from the IPS trace, as in `sysid_run`), which
also turns it round, so successive trials ping-pong across the spawn instead of
walking away from it.

This node is an INSTRUMENT, not a racing node: it reads IPS, a runtime-
restricted topic, which is fine here and would not be fine in a race.

Output: one row per IPS tick to `<out_dir>/brake_<timestamp>.csv`, plus a
summary table on stdout.
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

from pit_crew import vehicle

# What the locked four-wheel brake is worth, for the summary's comparison
# column only -- nothing here is tuned to it.  Re-measured on the finals track
# (Sep 19) from the samples published as exactly 0.0: -a = 4.31 + 0.284 v,
# against throttle2's model of 4.60 + 0.273 v.
LOCKED_BRAKE = 4.55         # m/s2, held, excluding drag
# mu 0.72 -- the ceiling the hypothesis asks for
TYRE_LONG_PEAK = 7.1        # m/s2


def fit_decel(samples):
    """Least-squares deceleration over `(t, v)` samples; positive = slowing.

    Returns `(a, r2, n)`.  A straight line is the right model here: the
    hypothesis is about a roughly constant retarding torque, and over a single
    brake window (7 -> 1.5 m/s, well under a second at 7 m/s2) the drag term is
    a ~0.3 m/s2 curvature that a slope fit reports as its mean.  `r2` is what
    says whether the constant-deceleration model held at all -- a wheel that
    locks part way through, or a car that crosses into reverse drive, shows up
    as a poor fit and not as a wrong number.
    """
    pts = [(float(t), float(v)) for t, v in samples
           if math.isfinite(t) and math.isfinite(v)]
    n = len(pts)
    if n < 3:
        return float('nan'), float('nan'), n
    tm = sum(p[0] for p in pts) / n
    vm = sum(p[1] for p in pts) / n
    stt = sum((p[0] - tm) ** 2 for p in pts)
    if stt <= 1e-12:
        return float('nan'), float('nan'), n
    slope = sum((p[0] - tm) * (p[1] - vm) for p in pts) / stt
    svv = sum((p[1] - vm) ** 2 for p in pts)
    if svv <= 1e-12:
        r2 = 1.0
    else:
        res = sum((p[1] - (vm + slope * (p[0] - tm))) ** 2 for p in pts)
        r2 = max(0.0, 1.0 - res / svv)
    return -slope, r2, n


def mean_slip(samples, v_min=1.0):
    """Mean `1 - v_enc/v_ips` over the samples above `v_min` of true speed.

    ~0 while the tyre is rolling and retarding the car, ~1 once the wheels are
    locked and the encoder reads a car that has stopped when it has not.

    Read the SIGN and the magnitude separately.  Both speeds are low-passed
    and with different lags (the encoder's alpha 0.35 is ~0.19 s, `ips_tau` is
    0.08), and on a ramp a first-order lag is a constant offset `a*tau`, so a
    hard stop shows a spurious slip of about `-(0.19 - ips_tau)*a/v` -- -0.19
    at 7 m/s2 and 4 m/s, with nothing sliding at all.  A LOCK is a large
    POSITIVE slip; a small negative one is this artefact.  `a_ratio` below is
    the lag-free version of the same question.
    """
    vals = [1.0 - v_enc / v for _t, v, v_enc in samples if v > v_min]
    return sum(vals) / len(vals) if vals else float('nan')


class BrakeProbe(Node):

    def __init__(self):
        super().__init__('brake_probe')
        p = self.declare_parameters('', [
            # 0.0 is the locked four-wheel brake and is the reference: keep it
            # first so the sweep is anchored before anything drifts.
            ('u_brakes', [0.0, -0.05, -0.10, -0.15, -0.20, -0.30, -0.45]),
            ('repeats', 1),
            ('u_accel', 0.40),         # throttle held while spinning up
            ('v_target', 7.0),         # m/s, entry speed for the brake window
            ('v_min_valid', 4.0),      # below this the trial is marked short
            ('accel_timeout_s', 15.0),
            ('brake_s', 3.0),          # cap on one brake window
            ('v_stop', 2.0),           # end the window here; below ~1 m/s a
                                       # negative command is only ever reverse,
                                       # and this speed is a LAGGED estimate --
                                       # at 7 m/s2 and ~0.15 s of lag the car
                                       # is already 1 m/s slower than it reads,
                                       # so the margin has to be paid here
            ('brake_lag_s', 0.12),     # never fit before this much has passed
            ('fit_drop_mps', 0.20),    # ...nor before the speed has fallen
                                       # this far, which is what actually
                                       # finds the start of the deceleration
            ('warmup_s', 2.0),
            ('home_speed', 2.0),       # m/s held while driving back
            ('home_r_m', 5.0),         # ...until this close to the spawn
            ('home_min_s', 2.5),       # ...and for at least this long, so the
                                       # car really does turn round
            ('home_max_s', 30.0),
            ('straighten_s', 0.6),     # rack back to zero before the next run
            ('home_steer_max', 0.35),  # rad, and further capped by `home_ay`
            ('home_ay', 4.5),          # m/s2 the U-turn may ask the tyres for
            ('fence_m', 25.0),         # hard geofence around the spawn point
            ('fence_margin_m', 5.0),   # end a window this far inside it
            ('yaw_abort', 1.0),        # rad/s, while steering straight
            ('yaw_abort_home', 6.0),   # rad/s, hard cap while turning round
            ('ay_abort', 9.0),         # m/s2, the turn's own limit; the tyre's
                                       # measured extremum is 9.81
            ('v_abort', 12.0),         # m/s, runaway guard
            ('ips_timeout_s', 1.0),    # s without a pose = stop
            ('ips_tau', 0.08),         # s, lag of the speed low-pass
            ('max_run_s', 600.0),
            ('out_dir', '/home/data/logs'),
        ])
        (self.u_brakes, self.repeats, self.u_accel, self.v_target,
         self.v_min_valid, self.accel_timeout_s, self.brake_s, self.v_stop,
         self.brake_lag_s, self.fit_drop_mps, self.warmup_s,
         self.home_speed, self.home_r_m, self.home_min_s, self.home_max_s,
         self.straighten_s,
         self.home_steer_max, self.home_ay, self.fence_m, self.fence_margin_m,
         self.yaw_abort, self.yaw_abort_home, self.ay_abort, self.v_abort,
         self.ips_timeout_s, self.ips_tau, self.max_run_s,
         out_dir) = [x.value for x in p]

        self.plan = [float(u) for _ in range(max(1, int(self.repeats)))
                     for u in self.u_brakes]
        if not self.plan:
            # An empty sweep would index off the end of `plan` on the first
            # tick. Fall back to the one trial that is always worth having.
            self.get_logger().warning(
                'u_brakes is empty; measuring the locked brake alone')
            self.u_brakes = [0.0]
            self.plan = [0.0]
        self.trial = 0

        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        self.gyro = 0.0
        self.imu_ax = 0.0
        self.thr_act = 0.0
        self.xy = None
        self.xy0 = None
        self.t_ips = None
        self.t_start = None
        self.v = 0.0                # IPS speed, signed along `ref_dir`
        self.v_seen = False
        self.ref_dir = None         # direction of travel; frozen while braking
        self.ref_frozen = False
        self.track = []             # recent (t, x, y) for the course angle
        self.u_cmd = 0.0
        self.steer_cmd = 0.0

        self.phase = 'warmup'
        self.t_phase = None
        self.brake_samples = []     # (t, v_ips, v_enc)
        self.v_entry = 0.0
        self.results = []
        self.done = False
        self.abort = None

        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(
            out_dir, 'brake_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
        self.fh = open(self.path, 'w', buffering=1)
        self.fh.write('t,trial,phase,u_brake,u_cmd,thr_act,x,y,v_ips,v_enc,'
                      'wz,imu_ax,dist_home\n')

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.create_subscription(Point, f'{ns}/ips', self.on_ips, qos)
        self.create_subscription(
            Float32, f'{ns}/throttle',
            lambda m: setattr(self, 'thr_act', float(m.data)), qos)
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'{ns}/{side}_encoder',
                lambda m, s=side: self.on_enc(s, m), qos)
        self.pub_steer = self.create_publisher(
            Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(
            Float32, f'{ns}/throttle_command', 1)
        self.create_timer(0.02, self.step)

        self.get_logger().info(
            f'brake_probe: {len(self.plan)} trials '
            f'({len(self.u_brakes)} commands x {int(self.repeats)}), entry '
            f'{self.v_target:.1f} m/s, window down to {self.v_stop:.1f} m/s, '
            f'geofence {self.fence_m:.0f} m -> {self.path}')

    # -- inputs ------------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg: Imu):
        self.gyro = float(msg.angular_velocity.z)
        self.imu_ax = float(msg.linear_acceleration.x)

    def on_enc(self, side, msg: JointState):
        if msg.position:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(side, stamp, msg.position[0])

    def on_ips(self, msg: Point):
        """Ground-truth speed, signed along the direction the car was going.

        Signed, not the magnitude of the displacement, because the whole risk
        of a negative throttle is that the car crosses zero and drives away
        backwards -- and a magnitude reports that as an honest-looking
        acceleration.  The reference direction is refreshed while the car is
        under power and FROZEN when the brake window opens, so a reversal
        inside the window reads as a negative speed.
        """
        t = self.now()
        xy = (float(msg.x), float(msg.y))
        if self.xy0 is None:
            self.xy0 = xy
        if self.xy is not None and self.t_ips is not None:
            dt = t - self.t_ips
            if 1e-3 < dt < 0.5:
                dx, dy = xy[0] - self.xy[0], xy[1] - self.xy[1]
                d = math.hypot(dx, dy)
                if not self.ref_frozen and d > 0.02:
                    self.ref_dir = math.atan2(dy, dx)
                if self.ref_dir is None:
                    v_raw = d / dt
                else:
                    v_raw = (dx * math.cos(self.ref_dir)
                             + dy * math.sin(self.ref_dir)) / dt
                # A 0.058 s IPS step differentiated puts occasional nonsense in
                # (throttle2 measured outliers to 37 m/s on the display log);
                # drop those rather than filter them, and keep the lag of the
                # low-pass fixed in SECONDS so the fitted number does not
                # depend on which machine this ran on.
                if abs(v_raw) < 2.0 * self.v_abort:
                    a = vehicle.alpha_for_lag(dt, self.ips_tau)
                    self.v = (self.v + a * (v_raw - self.v)
                              if self.v_seen else v_raw)
                    self.v_seen = True
        self.xy = xy
        self.t_ips = t
        self.track.append((t, xy[0], xy[1]))
        if len(self.track) > 12:
            self.track.pop(0)
        self.log_row()

    # -- helpers -----------------------------------------------------------

    def dist_home(self):
        if not (self.xy and self.xy0):
            return 0.0
        return math.hypot(self.xy[0] - self.xy0[0], self.xy[1] - self.xy0[1])

    def heading(self):
        """Course angle from the IPS trace; None while barely moving."""
        if len(self.track) < 3:
            return None
        t1, x1, y1 = self.track[-1]
        for t0, x0, y0 in reversed(self.track[:-1]):
            if t1 - t0 >= 0.20 and math.hypot(x1 - x0, y1 - y0) > 0.10:
                return math.atan2(y1 - y0, x1 - x0)
        return None

    def home_steer(self):
        """Steering towards the spawn point, capped by the grip it can spend.

        The cap has to be `atan(L * ay / v^2)` and not a fixed angle -- the
        same law `mpc3` uses as `ay_limit`.  A weak test command leaves the car
        still doing 5-6 m/s when the window ends, and 0.35 rad at 5.6 m/s is
        6.3 rad/s of yaw and ~35 m/s2 of demand: it would spin, and the
        instrument would have caused the only interesting event in its own log.
        """
        h = self.heading()
        if h is None or not (self.xy and self.xy0):
            return 0.0
        bearing = math.atan2(self.xy0[1] - self.xy[1],
                             self.xy0[0] - self.xy[0])
        err = (bearing - h + math.pi) % (2.0 * math.pi) - math.pi
        v = max(abs(self.v), 0.5)
        lim = min(self.home_steer_max,
                  math.atan(vehicle.WHEELBASE * self.home_ay / (v * v)))
        return max(-lim, min(lim, 0.8 * err))

    def u_home(self):
        """Feedforward throttle for the drive home (`vehicle`'s measured
        0.040 per m/s).  Open loop on purpose: nothing about the homing needs
        to be accurate, and a loop closed at 10 Hz surges."""
        return 0.040 * self.home_speed + 0.005

    def publish(self, throttle, steer):
        self.u_cmd = float(throttle)
        self.steer_cmd = float(steer)
        self.pub_throttle.publish(Float32(data=self.u_cmd))
        self.pub_steer.publish(
            Float32(data=vehicle.steer_to_cmd(self.steer_cmd)))

    def log_row(self):
        if self.fh.closed:
            return
        x, y = self.xy if self.xy else (float('nan'), float('nan'))
        u_b = self.plan[self.trial] if self.trial < len(self.plan) else 0.0
        self.fh.write(
            f'{time.time():.3f},{self.trial},{self.phase},{u_b:.4f},'
            f'{self.u_cmd:.4f},{self.thr_act:.4f},{x:.4f},{y:.4f},'
            f'{self.v:.3f},{self.speed_est.speed:.3f},{self.gyro:.4f},'
            f'{self.imu_ax:.3f},{self.dist_home():.2f}\n')

    # -- guards ------------------------------------------------------------

    def guard(self, t):
        """Why the run must stop now, or None.

        The LiDAR is deliberately not in here: in the explore world every ray
        returns `inf` (`sysid_run`, 103 rows, not one finite range), so a
        LiDAR guard there is a guard that can never fire.  These are geometric
        and kinematic instead.
        """
        if self.dist_home() > self.fence_m:
            return (f'geofence: {self.dist_home():.1f} m from the spawn '
                    f'(limit {self.fence_m:.0f} m)')
        # The tight yaw guard belongs to the phases that are supposed to be
        # going straight.  The drive home is a deliberate U-turn and
        # `straighten` is where the rack unwinds from it, so there the limit is
        # on what the turn is ASKING OF THE TYRES -- a fixed yaw-rate ceiling
        # is speed-blind, and the same 0.35 rad of rack is 2.3 rad/s at
        # 2 m/s and 6.3 rad/s at 5.6.
        if self.phase in ('home', 'straighten'):
            ay = abs(self.gyro) * max(abs(self.v), 0.5)
            if ay > self.ay_abort or abs(self.gyro) > self.yaw_abort_home:
                return (f'turn asking {ay:.1f} m/s2 at '
                        f'{self.gyro:+.2f} rad/s in phase {self.phase} '
                        f'(limits {self.ay_abort:.1f} m/s2, '
                        f'{self.yaw_abort_home:.1f} rad/s)')
        elif abs(self.gyro) > self.yaw_abort:
            return (f'yaw rate {self.gyro:+.2f} rad/s in phase '
                    f'{self.phase} (limit {self.yaw_abort:.2f})')
        if abs(self.v) > self.v_abort:
            return f'speed {self.v:+.1f} m/s past the {self.v_abort:.0f} cap'
        if (self.t_ips is not None and self.phase != 'warmup'
                and t - self.t_ips > self.ips_timeout_s):
            return f'no IPS for {t - self.t_ips:.1f} s'
        if self.t_start is not None and t - self.t_start > self.max_run_s:
            return f'max_run_s {self.max_run_s:.0f} s reached'
        return None

    # -- the sequence ------------------------------------------------------

    def enter(self, phase, t):
        self.phase = phase
        self.t_phase = t

    def step(self):
        if self.done:
            return
        t = self.now()
        if self.t_phase is None:
            self.t_phase = t
        if self.t_start is None:
            self.t_start = t
        elapsed = t - self.t_phase

        why = self.guard(t)
        if why:
            self.abort = why
            self.finish()
            return

        u_b = self.plan[self.trial] if self.trial < len(self.plan) else 0.0

        if self.phase == 'warmup':
            # Exactly 0.0 is the four-wheel brake: the car is held still while
            # the first poses arrive and the spawn point is fixed.
            self.publish(0.0, 0.0)
            if elapsed > self.warmup_s and self.xy is not None:
                self.get_logger().info(
                    f'trial 1/{len(self.plan)}: u_brake {u_b:+.3f}')
                self.enter('accel', t)

        elif self.phase == 'accel':
            self.publish(self.u_accel, 0.0)
            if (self.v >= self.v_target or elapsed > self.accel_timeout_s
                    or self.dist_home() > self.soft_fence()):
                self.v_entry = self.v
                self.ref_frozen = True          # sign the brake window by this
                self.brake_samples = []
                self.enter('brake', t)

        elif self.phase == 'brake':
            self.publish(u_b, 0.0)
            self.brake_samples.append((t, self.v, self.speed_est.speed))
            # Stop at `v_stop` and not at zero: below a walking pace a negative
            # command is unambiguously reverse drive, and driving the car
            # backwards is exactly what August already proved it will do.
            # The soft fence is the third exit and it matters: a command worth
            # only ~1 m/s2 takes 25 m to come down from 7 m/s to `v_stop`,
            # which is the whole geofence. Ending the window there keeps a
            # perfectly good partial measurement AND the rest of the sweep,
            # where the hard fence would throw both away.
            clipped = self.dist_home() > self.soft_fence()
            if elapsed > self.brake_s or self.v < self.v_stop or clipped:
                self.record(u_b, elapsed, clipped=clipped)
                self.trial += 1
                self.ref_frozen = False
                if self.trial >= len(self.plan):
                    self.finish()
                    return
                self.get_logger().info(
                    f'trial {self.trial + 1}/{len(self.plan)}: u_brake '
                    f'{self.plan[self.trial]:+.3f}')
                self.enter('home', t)

        elif self.phase == 'home':
            # Entered while the car is still rolling at `v_stop`, so the course
            # angle is observable and it can be turned round under power.  From
            # a standstill the heading is unknown and it would drive off in
            # whatever direction it happened to be facing -- outwards.
            self.publish(self.u_home(), self.home_steer())
            if (self.dist_home() < self.home_r_m
                    and elapsed > self.home_min_s):
                self.enter('straighten', t)
            elif elapsed > self.home_max_s:
                self.get_logger().warning(
                    f'  drive home gave up {self.dist_home():.1f} m out; '
                    f'carrying on -- the geofence is still there')
                self.enter('straighten', t)

        elif self.phase == 'straighten':
            # Unwind the rack before the next measurement, under the loose yaw
            # limit: a 0.35 rad rack takes ~0.11 s to slew out and the yaw it
            # is still producing would otherwise trip the straight-line guard
            # on the first tick of `accel`.
            self.publish(self.u_home(), 0.0)
            if elapsed > self.straighten_s:
                self.enter('accel', t)

    def soft_fence(self):
        """Radius at which a phase gives up gracefully, inside the hard one."""
        return max(self.home_r_m + 1.0, self.fence_m - self.fence_margin_m)

    def record(self, u_b, elapsed, clipped=False):
        """Fit one brake window and keep it.

        The window does not start when the command was PUBLISHED.  The
        simulator consumes one command per telemetry tick, so up to a whole
        period passes before the car is braking at all, and the speed signal
        is itself lagged; a fit that includes that flat head under-reports the
        deceleration by ~10% (measured on the synthetic plant in
        `tests/test_brake_probe.py`: 6.31 against a true 7.1).  Start instead
        at the first sample that has actually slowed by `fit_drop_mps`, and
        never before `brake_lag_s`.
        """
        t0 = self.t_phase + self.brake_lag_s
        moved = [s[0] for s in self.brake_samples
                 if s[1] <= self.v_entry - self.fit_drop_mps]
        if moved:
            t0 = max(t0, moved[0])
        win = [s for s in self.brake_samples if s[0] >= t0]
        a, r2, n = fit_decel([(s[0], s[1]) for s in win])
        a_enc, _, _ = fit_decel([(s[0], s[2]) for s in win])
        slip = mean_slip(win, v_min=max(1.0, self.v_stop * 0.6))
        # The lock discriminator that no filter lag can move: over a ramp both
        # low-passes are offset by a constant, so the two fitted SLOPES agree
        # when the wheels are turning with the car (ratio ~1) and diverge
        # completely when they are locked and the encoder is only measuring
        # its own filter decaying to zero (ratio ~0).
        a_ratio = (a_enc / a if (math.isfinite(a) and math.isfinite(a_enc)
                                 and abs(a) > 0.2) else float('nan'))
        v_end = win[-1][1] if win else float('nan')
        rec = {
            'u_brake': float(u_b),
            'v_entry': float(self.v_entry),
            'v_end': float(v_end),
            'a_ips': a,
            'a_enc': a_enc,
            'r2': r2,
            'n': n,
            'slip': slip,
            'a_ratio': a_ratio,
            'dur': float(elapsed),
            'short': bool(self.v_entry < self.v_min_valid),
            'clipped': bool(clipped),
            'reversed': bool(min([s[1] for s in win], default=0.0) < -0.2),
        }
        self.results.append(rec)
        self.get_logger().info(
            f'  u {u_b:+.3f}: {rec["v_entry"]:.2f} -> {rec["v_end"]:.2f} m/s '
            f'in {elapsed:.2f} s, a {a:5.2f} m/s2 (r2 {r2:.2f}, n {n}), '
            f'encoder a {a_enc:5.2f} (ratio {a_ratio:4.2f}), '
            f'slip {slip:+.2f}'
            + ('  [SHORT ENTRY]' if rec['short'] else '')
            + ('  [CLIPPED BY THE FENCE]' if rec['clipped'] else '')
            + ('  [REVERSED]' if rec['reversed'] else ''))

    # -- shutdown ----------------------------------------------------------

    def finish(self):
        if self.done:
            return
        # Commands latch.  Zero throttle IS the brake here, which is the right
        # thing to leave behind either way.
        self.publish(0.0, 0.0)
        if self.abort:
            self.get_logger().warning(f'ABORTED: {self.abort}')
            if not self.fh.closed:
                self.fh.write(f'# STOPPED: {self.abort}\n')
        self.report()
        if not self.fh.closed:
            self.fh.close()
        self.done = True

    def report(self):
        self.get_logger().info('--- brake_probe summary ---')
        self.get_logger().info(
            '  u_brake   n   v_entry   a_ips   a_enc  ratio    r2   slip'
            '   note')
        by_u = {}
        for r in self.results:
            by_u.setdefault(round(r['u_brake'], 4), []).append(r)
        ref = None
        for u in sorted(by_u, reverse=True):
            rows = by_u[u]
            a = median([r['a_ips'] for r in rows])
            a_e = median([r['a_enc'] for r in rows])
            r2 = median([r['r2'] for r in rows])
            sl = median([r['slip'] for r in rows])
            ar = median([r['a_ratio'] for r in rows])
            ve = median([r['v_entry'] for r in rows])
            note = []
            if u == 0.0:
                ref = a
                note.append('LOCKED BRAKE (reference)')
            if any(r['short'] for r in rows):
                note.append('short entry')
            if any(r['clipped'] for r in rows):
                note.append('window clipped by the fence')
            if any(r['reversed'] for r in rows):
                note.append('REVERSED')
            self.get_logger().info(
                f'  {u:+7.3f} {len(rows):3d} {ve:8.2f} {a:7.2f} {a_e:7.2f} '
                f'{ar:6.2f} {r2:5.2f} {sl:+6.2f}   {", ".join(note)}')
        if ref is not None and math.isfinite(ref):
            best = None
            for u, rows in by_u.items():
                if u >= 0.0:
                    continue
                a = median([r['a_ips'] for r in rows])
                sl = median([r['slip'] for r in rows])
                ar = median([r['a_ratio'] for r in rows])
                if (math.isfinite(a) and not any(r['reversed'] for r in rows)
                        and (best is None or a > best[1])):
                    best = (u, a, sl, ar)
            if best is None:
                self.get_logger().info(
                    '  no usable negative command: every one reversed the car '
                    'or produced no fit. The locked brake stands.')
            else:
                u, a, sl, ar = best
                verdict = ('BEATS' if a > ref + 0.3 else
                           'does not beat')
                self.get_logger().info(
                    f'  best negative command {u:+.3f} gives {a:.2f} m/s2 '
                    f'against the locked {ref:.2f} -- {verdict} it '
                    f'({a - ref:+.2f} m/s2), slip {sl:+.2f}, encoder/IPS '
                    f'slope ratio {ar:.2f}, tyre peak is '
                    f'~{TYRE_LONG_PEAK:.1f}')
                if sl > 0.5 or (math.isfinite(ar) and ar < 0.5):
                    self.get_logger().info(
                        '  ...but the wheels are LOCKED under it, so that is '
                        'not a proportional brake however hard it pulls '
                        '-- and a locked wheel steers nothing.')
        self.get_logger().info(f'rows written to {self.path}')


def median(xs):
    vals = sorted(x for x in xs if isinstance(x, float) and math.isfinite(x))
    if not vals:
        return float('nan')
    n = len(vals)
    return vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])


def main():
    rclpy.init()
    node = BrakeProbe()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.abort = node.abort or 'interrupted'
        node.finish()
    finally:
        # Commands latch: leave the car braked and straight whatever happened.
        for _ in range(5):
            node.pub_throttle.publish(Float32(data=0.0))
            node.pub_steer.publish(Float32(data=0.0))
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
