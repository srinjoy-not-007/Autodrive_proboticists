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

What it drives, and why those numbers:

  speeds  2.0 ... 5.5 m/s in 0.5 steps -- 2.0-3.5 overlaps the race data (so
          the two measurements can be checked against each other) and 4.0-5.5
          is the blank region. Above 5.5 the arc radius at any interesting
          lateral acceleration exceeds the space guard.
  steers  0.05 ... lock. The race data is dense below 0.25 and empty above
          0.32; the knee is expected between 0.25 and 0.40 if the hard-clip
          estimate is right, and not at all if the tanh estimate is right.
          Either outcome is an answer.
  hold    1.5 s after a 2.0 s settle. The identified lateral pole is 0.05-0.09
          s and the speed servo settles in ~0.3 s, so 2.0 s is ~20 time
          constants; 1.5 s of hold is ~16 samples at the 11 Hz telemetry rate,
          enough for a median that is not one sample's noise.
  sign    alternates every step, so the car weaves instead of spiralling away
          and the run stays inside the space guard.
  skip    cells whose kinematic demand v^2*tan(delta)/L exceeds `max_demand`
          (default 14 m/s2, half again the tyre asset's 9.81 peak) are
          skipped: past that the car is only measuring how it spins.
  stop    the staircase for a speed ends when two consecutive steps fail to
          gain `knee_gain` of lateral acceleration -- that IS the limit, and
          there is nothing above it worth the space.

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
            ('max_demand', 14.0),      # m/s2; skip cells asking for more
            ('knee_gain', 0.20),       # m/s2 of ay a step must add to continue
            ('lidar_stop_m', 2.0),     # abort if anything is this close
            ('max_radius_m', 18.0),    # abort if we wander this far from start
            ('out_dir', '/home/data/logs'),
        ])
        (self.speeds, self.steers, self.settle_s, self.hold_s,
         self.straighten_s, self.spin_up_s, self.max_demand, self.knee_gain,
         self.lidar_stop_m, self.max_radius_m, out_dir) = [x.value for x in p]

        self.throttle = ServoThrottle()
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        self.gyro = 0.0
        self.imu_ax = self.imu_ay = 0.0
        self.steer_act = 0.0
        self.thr_act = 0.0
        self.xy = None
        self.xy0 = None
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

    def target_steer(self):
        if self.phase in ('spinup', 'straighten'):
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
        if self.lidar_min < self.lidar_stop_m:
            return f'obstacle at {self.lidar_min:.2f} m'
        if self.xy and self.xy0:
            r = math.hypot(self.xy[0] - self.xy0[0], self.xy[1] - self.xy0[1])
            if r > self.max_radius_m:
                return f'wandered {r:.1f} m from the start point'
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
        self.pub_steer.publish(
            Float32(data=vehicle.steer_to_cmd(self.target_steer())))
        self.pub_throttle.publish(Float32(data=float(
            self.throttle.command(v_t, self.speed_est.speed))))

        if self.phase == 'spinup':
            if elapsed > self.spin_up_s:
                self.enter('settle', t)
        elif self.phase == 'settle':
            if elapsed > self.settle_s:
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

    def enter(self, phase, t):
        self.phase = phase
        self.t_phase = t

    def advance(self, t):
        """Next cell: up the staircase, or on to the next speed."""
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
            self.enter('spinup', t)
        else:
            self.enter('settle', t)

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
