"""Steering staircase for lateral system identification, on the -explore image.

`tools/lateral_id.py` fits the car's lateral model from ordinary race logs and
then says, honestly, what those logs cannot answer:

  * the largest steady-state steering angle ever logged is 0.31 rad, against a
    0.5236 rad lock, so the far half of the steering range is unmeasured;
  * there are four samples in the whole corpus above 4.5 m/s with more than
    0.15 rad of steering, so `ay_max(v)` above 4.5 m/s is extrapolation;
  * the saturation asymptote is unidentified -- a hard clip fits at 5.9 m/s2
    and a tanh asymptote anywhere from 8 to 15 m/s2 fits equally well.

No further racing fixes that, because the controller never asks for what it
has never been given. This node asks: hold a speed, step the steering up, and
measure what comes back, exactly where the race logs are blank.

**Run it on the `-explore` image.** `skidpad.py` learned this the hard way --
on Porto the car understeers into the walls and the measurement is scrap. The
explore world has no track at all, only `Floor`:

    TAG=2026-iros-explore ./scripts/sim_restart_connect.sh
    colcon build --packages-select pit_crew --symlink-install
    ros2 run pit_crew sysid_run
    ros2 run pit_crew sysid_run --ros-args -p speeds:="[3.0,4.0,5.0]"

This node is an INSTRUMENT, not a racing node. It reads IPS (a runtime-
restricted topic) for the space guard and for ground truth, which is fine here
and would not be fine in a race.

**What the first run taught (2026-09-11, `data/logs/sysid_20260911_212223.csv`,
103 rows).** It measured exactly ONE cell -- 2.0 m/s at 0.05 rad, 15 samples,
ratio 1.003 against the kinematic model -- and then tripped the radius guard
18.2 m from the spawn, 10 s in. The cause was geometry, not grip: the spin-up
held zero steering, so the car drove 9.4 m in a straight line before the
staircase even began, and the first two steering steps are shallow arcs that
carry on outwards. The LiDAR never saw anything at all: every one of the 103
rows reads `inf`, so in the explore world that guard cannot fire and is now off
by default. Three changes came out of it:

  * **spin up on a circle**, radius max(`spin_radius_m`, v^2/`spin_ay`), so the
    car gains speed inside a bounded disc instead of walking away from the only
    ground known to be clear. A closed circle, not a figure of eight -- the
    eight ends up pointing outwards and simulates worse (22 m against 18.6 m).
  * **drive home between cells** when the spawn is more than `recentre_r_m`
    away: steer at the spawn point (bearing minus the course angle taken from
    the IPS trace, no yaw topic and no gyro integration) at `home_speed` until
    inside `home_r_m`, then resume the cell that was about to start. At most
    ONE drive home per cell -- a 2 s settle at 5.5 m/s covers 11 m, more than
    the gap between the two radii, so without that limit the car arrives home,
    settles, is outside again, and ping-pongs forever. Simulated, that hangs.
  * **the radius guard becomes a backstop**, at 40 m rather than 18, because
    the recentre is what keeps the run local now. If it ever fires, something
    is wrong that stopping is the right answer to.

Simulated against the identified model end to end (policy, not code): 61 cells,
~420 s, 32 drives home, never further than 17.7 m from the spawn. Under the
pessimistic hard-clip grip model instead: 434 s and 16.2 m. The previous run
reached 18.2 m with no LiDAR return in any direction, so that much floor is
known to be clear.

Output: one row per telemetry tick to `<out_dir>/sysid_<timestamp>.csv`, with
the commanded cell, the actuator feedback, IPS ground truth and the IMU. Fit it
with the same tool that motivated it:

    .venv/bin/python tools/lateral_id.py --sysid data/logs/sysid_<ts>.csv
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32

from pit_crew import vehicle
from pit_crew.throttle2 import ServoThrottle


class SysIdRun(Node):

    def __init__(self):
        super().__init__('sysid_run')
        p = self.declare_parameters('', [
            ('speeds', [2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5]),
            ('steers', [0.05, 0.10, 0.15, 0.20, 0.25, 0.30,
                        0.35, 0.40, 0.45, float(vehicle.MAX_STEER)]),
            ('settle_s', 2.0),         # per step, before anything is recorded
            ('hold_s', 1.5),           # per step, measured
            ('straighten_s', 0.8),     # steering back to zero between steps
            ('spin_up_s', 5.0),        # per speed, before the first step
            ('max_demand', 20.0),      # m/s2; skip cells asking for more
            ('knee_gain', 0.20),       # m/s2 of ay a step must add to continue
            ('spin_radius_m', 2.0),    # spin up on a circle of at least this
            ('spin_ay', 4.5),          # ...and never asking more than this
            ('recentre_r_m', 8.0),     # go home when a cell ends beyond this
            ('home_r_m', 4.0),         # ...and resume once back inside this
            ('home_speed', 3.0),       # m/s held while driving home
            ('lidar_stop_m', 0.0),     # 0 disables; see the module docstring
            ('max_radius_m', 40.0),    # hard abort, recentre should prevent it
            ('out_dir', '/home/data/logs'),
        ])
        (self.speeds, self.steers, self.settle_s, self.hold_s,
         self.straighten_s, self.spin_up_s, self.max_demand, self.knee_gain,
         self.spin_radius_m, self.spin_ay, self.recentre_r_m, self.home_r_m,
         self.home_speed, self.lidar_stop_m, self.max_radius_m,
         out_dir) = [x.value for x in p]

        self.throttle = ServoThrottle()
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        self.gyro = 0.0
        self.imu_ax = self.imu_ay = 0.0
        self.steer_act = 0.0
        self.thr_act = 0.0
        self.xy = None
        self.xy0 = None
        self.track = []             # recent (t, x, y) for a heading estimate
        self.resume = 'settle'      # phase to enter once we are home again
        self.recentred = False      # at most one drive home per cell
        self.lidar_min = float('inf')

        self.iv = 0                 # index into speeds
        self.istep = 0              # index into steers
        self.sign = 1.0
        self.phase = 'spinup'       # spinup -> settle -> hold -> straighten
        self.t_phase = None
        self.samples = []
        self.last_ay = 0.0
        self.stall = 0
        self.results = []
        self.done = False
        self.abort = None

        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(
            out_dir, 'sysid_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
        self.fh = open(self.path, 'w', buffering=1)
        self.fh.write('t,phase,v_target,delta_target,delta_act,throttle,'
                      'x,y,v_enc,wz,imu_ax,imu_ay,lidar_min\n')

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.create_subscription(Point, f'{ns}/ips', self.on_ips, qos)
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.create_subscription(Float32, f'{ns}/steering',
                                 lambda m: setattr(self, 'steer_act', m.data), qos)
        self.create_subscription(Float32, f'{ns}/throttle',
                                 lambda m: setattr(self, 'thr_act', m.data), qos)
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'{ns}/{side}_encoder',
                lambda m, s=side: self.on_enc(s, m), qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.create_timer(0.02, self.step)

        cells = sum(1 for v in self.speeds for d in self.steers
                    if self.demand(v, d) <= self.max_demand)
        self.get_logger().info(
            f'sysid_run: {len(self.speeds)} speeds x up to {len(self.steers)} '
            f'steering steps, {cells} cells within the demand cap, '
            f'~{cells * (self.settle_s + self.hold_s + self.straighten_s) + len(self.speeds) * self.spin_up_s:.0f} s '
            f'-> {self.path}')

    # -- inputs ------------------------------------------------------------

    def on_imu(self, msg: Imu):
        self.gyro = float(msg.angular_velocity.z)
        self.imu_ax = float(msg.linear_acceleration.x)
        self.imu_ay = float(msg.linear_acceleration.y)
        self.log_row()

    def on_ips(self, msg: Point):
        self.xy = (float(msg.x), float(msg.y))
        if self.xy0 is None:
            self.xy0 = self.xy
        self.track.append((self.now(), self.xy[0], self.xy[1]))
        if len(self.track) > 12:
            self.track.pop(0)

    def on_scan(self, msg: LaserScan):
        good = [r for r in msg.ranges
                if vehicle.LIDAR_RANGE_MIN < r < vehicle.LIDAR_RANGE_MAX]
        self.lidar_min = min(good) if good else float('inf')

    def on_enc(self, side, msg: JointState):
        if msg.position:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(side, stamp, msg.position[0])

    # -- helpers -----------------------------------------------------------

    @staticmethod
    def demand(v, delta):
        """Lateral acceleration the kinematic bicycle asks for at this cell."""
        return v * v * math.tan(abs(delta)) / vehicle.WHEELBASE

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def dist_home(self):
        if not (self.xy and self.xy0):
            return 0.0
        return math.hypot(self.xy[0] - self.xy0[0], self.xy[1] - self.xy0[1])

    def heading(self):
        """Course angle from the IPS trace. No yaw topic is needed and the
        gyro would have to be integrated, which drifts; two poses ~0.3 s apart
        give the direction of travel directly. None while barely moving."""
        if len(self.track) < 4:
            return None
        t1, x1, y1 = self.track[-1]
        for t0, x0, y0 in reversed(self.track[:-1]):
            if t1 - t0 >= 0.25 and math.hypot(x1 - x0, y1 - y0) > 0.15:
                return math.atan2(y1 - y0, x1 - x0)
        return None

    def spin_steer(self):
        """Steering held during spin-up.

        The first run of this node drove 9.4 m in a straight line during
        spin-up, then a 6.5 m-radius arc outwards, and tripped the radius
        guard 18 m from the spawn before it had measured a second cell. A
        straight spin-up is a walk away from the only place we know is clear.
        Spin up on a circle instead: the radius is the larger of
        `spin_radius_m` and whatever keeps the lateral demand at or under
        `spin_ay`, so the car gains speed inside a bounded disc and never
        asks the tyres for anything interesting while doing it.
        """
        v = max(self.speeds[self.iv], 0.5)
        r = max(self.spin_radius_m, v * v / max(self.spin_ay, 0.5))
        # A closed circle, not a figure of eight: the eight ends up pointing
        # away from home and, simulated against the identified model, reached
        # 22 m where the plain circle reaches 18.6 m. A circle of radius r
        # bounds the excursion at 2r, and r is only large where the speed
        # makes it so.
        return self.sign * math.atan(vehicle.WHEELBASE / r)

    def home_steer(self):
        """Proportional steering towards the spawn point."""
        h = self.heading()
        if h is None or not (self.xy and self.xy0):
            return 0.0
        bearing = math.atan2(self.xy0[1] - self.xy[1], self.xy0[0] - self.xy[0])
        err = (bearing - h + math.pi) % (2.0 * math.pi) - math.pi
        return max(-0.35, min(0.35, 0.8 * err))

    def target_steer(self):
        if self.phase == 'spinup':
            return self.spin_steer()
        if self.phase == 'recentre':
            return self.home_steer()
        if self.phase == 'straighten':
            return 0.0
        return self.sign * self.steers[self.istep]

    def log_row(self):
        x, y = self.xy if self.xy else (float('nan'), float('nan'))
        v_t = self.speeds[self.iv] if self.iv < len(self.speeds) else 0.0
        self.fh.write(
            f'{time.time():.3f},{self.phase},{v_t:.2f},'
            f'{self.target_steer():.4f},{self.steer_act:.4f},{self.thr_act:.4f},'
            f'{x:.4f},{y:.4f},{self.speed_est.speed:.3f},{self.gyro:.4f},'
            f'{self.imu_ax:.3f},{self.imu_ay:.3f},{self.lidar_min:.3f}\n')

    def guard(self):
        # The LiDAR guard is OFF by default: in the explore world every ray
        # returns inf (measured -- 103 rows, not one finite range), so it can
        # never fire there and would only mask a real problem. Raise it if this
        # node is ever pointed at a world that has walls.
        if self.lidar_stop_m > 0.0 and self.lidar_min < self.lidar_stop_m:
            return f'obstacle at {self.lidar_min:.2f} m'
        if self.dist_home() > self.max_radius_m:
            return (f'wandered {self.dist_home():.1f} m from the start point '
                    f'even with recentring')
        return None

    # -- the staircase -----------------------------------------------------

    def step(self):
        if self.done:
            return
        t = self.now()
        if self.t_phase is None:
            self.t_phase = t
        elapsed = t - self.t_phase

        why = self.guard()
        if why and not self.done:
            self.abort = why
            self.finish()
            return

        v_t = self.speeds[self.iv]
        if self.phase == 'recentre':
            v_t = min(v_t, self.home_speed)
        self.pub_steer.publish(
            Float32(data=vehicle.steer_to_cmd(self.target_steer())))
        self.pub_throttle.publish(Float32(data=float(
            self.throttle.command(v_t, self.speed_est.speed))))

        if self.phase == 'spinup':
            if elapsed > self.spin_up_s:
                self.enter_or_recentre('settle', t)
        elif self.phase == 'settle':
            if elapsed > self.settle_s:
                # One drive home per cell, and only before the hold starts.
                # Without the limit this ping-pongs forever: a 2 s settle at
                # 5.5 m/s covers 11 m, which is more than the 4 m gap between
                # `recentre_r_m` and `home_r_m`, so the car arrives home,
                # settles, is outside again, and goes home again. Simulated,
                # that hangs the whole staircase. A cell measured 10 m from
                # the spawn is a perfectly good cell.
                if self.dist_home() > self.recentre_r_m and not self.recentred:
                    self.enter_or_recentre('settle', t)
                else:
                    self.samples = []
                    self.enter('hold', t)
        elif self.phase == 'hold':
            self.samples.append((self.speed_est.speed, abs(self.gyro),
                                 abs(self.steer_act)))
            if elapsed > self.hold_s:
                self.record()
                self.enter('straighten', t)
        elif self.phase == 'straighten':
            if elapsed > self.straighten_s:
                self.advance(t)
        elif self.phase == 'recentre':
            if self.dist_home() < self.home_r_m:
                self.get_logger().info(
                    f'  back within {self.dist_home():.1f} m of the spawn; '
                    f'resuming at {self.resume}')
                self.enter(self.resume, t)
            elif elapsed > self.recentre_timeout():
                # Giving up on the drive home is not a reason to throw away
                # the rest of the staircase: the hard radius guard is still
                # there, and a cell measured 10 m from the spawn is just as
                # valid as one measured on top of it.
                self.get_logger().warning(
                    f'  recentre gave up at {self.dist_home():.1f} m after '
                    f'{elapsed:.0f} s; carrying on from here')
                self.enter(self.resume, t)

    def enter(self, phase, t):
        self.phase = phase
        self.t_phase = t

    def recentre_timeout(self):
        """Generous: a circle back from the far edge at the home speed."""
        return 4.0 * self.max_radius_m / max(self.home_speed, 1.0)

    def enter_or_recentre(self, phase, t):
        """Start the next cell, or drive home first if we have drifted.

        Only ever called between cells, never inside a settle or a hold: an
        interrupted hold is a measurement thrown away, and the point of the
        recentre is that it never has to interrupt one. A whole cell is at
        most (settle + hold) x speed of travel, and every cell but the
        shallowest is an arc tight enough to come back on itself, so the drift
        accumulates between cells, which is exactly where this fires.
        """
        if self.dist_home() > self.recentre_r_m and not self.recentred:
            self.recentred = True
            self.resume = phase
            self.get_logger().info(
                f'  {self.dist_home():.1f} m from the spawn: driving home '
                f'before the next cell')
            self.enter('recentre', t)
        else:
            self.enter(phase, t)

    def advance(self, t):
        """Next cell: up the staircase, or on to the next speed."""
        self.recentred = False
        self.sign = -self.sign
        self.istep += 1
        while (self.istep < len(self.steers)
               and self.demand(self.speeds[self.iv],
                               self.steers[self.istep]) > self.max_demand):
            self.get_logger().info(
                f'  skip {self.speeds[self.iv]:.1f} m/s @ '
                f'{self.steers[self.istep]:.3f} rad: asks '
                f'{self.demand(self.speeds[self.iv], self.steers[self.istep]):.1f} m/s2')
            self.istep += 1
        if self.stall >= 2 or self.istep >= len(self.steers):
            if self.stall >= 2:
                self.get_logger().info(
                    f'  {self.speeds[self.iv]:.1f} m/s: two steps without '
                    f'{self.knee_gain:.2f} m/s2 of gain -- that is the limit')
            self.iv += 1
            self.istep = 0
            self.stall = 0
            self.last_ay = 0.0
            if self.iv >= len(self.speeds):
                self.finish()
                return
            self.enter_or_recentre('spinup', t)
        else:
            self.enter_or_recentre('settle', t)

    def record(self):
        if len(self.samples) < 8:
            return
        n = len(self.samples)
        v = sum(s[0] for s in self.samples) / n
        yr = sum(s[1] for s in self.samples) / n
        de = sum(s[2] for s in self.samples) / n
        if v < 0.3:
            return
        ay = yr * v
        kin = v * math.tan(de) / vehicle.WHEELBASE
        ratio = yr / max(kin, 1e-3)
        self.results.append((self.speeds[self.iv], self.steers[self.istep],
                             v, de, yr, ay, ratio))
        gain = ay - self.last_ay
        self.stall = self.stall + 1 if gain < self.knee_gain else 0
        self.last_ay = ay
        self.get_logger().info(
            f'v* {self.speeds[self.iv]:4.1f} delta* {self.steers[self.istep]:5.3f} | '
            f'v {v:4.2f} delta {de:5.3f} yaw {yr:5.2f} (kin {kin:5.2f}, '
            f'ratio {ratio:4.2f})  ay {ay:5.2f} m/s2  d_ay {gain:+5.2f}')

    def finish(self):
        if self.done:
            return
        self.pub_steer.publish(Float32(data=0.0))
        self.pub_throttle.publish(Float32(data=0.0))
        if self.abort:
            self.get_logger().warning(f'ABORTED: {self.abort}')
        self.get_logger().info('--- staircase summary (per speed) ---')
        for v_t in sorted({r[0] for r in self.results}):
            rows = [r for r in self.results if r[0] == v_t]
            peak = max(rows, key=lambda r: r[5])
            fell = [r for r in rows if r[6] < 0.85]
            note = (f'ratio first below 0.85 at delta {fell[0][1]:.3f} '
                    f'(ay {fell[0][5]:.2f})' if fell else
                    'never left the kinematic model in the range tested')
            self.get_logger().info(
                f'  {v_t:4.1f} m/s: {len(rows)} steps, peak ay {peak[5]:5.2f} '
                f'at delta {peak[1]:.3f}; {note}')
        self.get_logger().info(f'rows written to {self.path}')
        self.fh.close()
        self.done = True


def main():
    rclpy.init()
    node = SysIdRun()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish()
    finally:
        # Commands latch: leave the car stopped and straight whatever happened.
        for _ in range(5):
            node.pub_steer.publish(Float32(data=0.0))
            node.pub_throttle.publish(Float32(data=0.0))
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
