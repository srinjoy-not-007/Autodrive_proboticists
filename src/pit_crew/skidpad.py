"""Measure the car's real lateral grip, instead of inferring it from crashes.

The velocity profile is capped at ay = 3.0 m/s2. That number came from watching
collisions rise as the cap was raised, which cannot separate three different
causes: the tyres letting go, the tracker failing, and the line running out of
clearance. This measures only the first.

Method (a skidpad): hold a constant steering angle and raise the speed in
steps. Below the friction limit the car follows the kinematic bicycle model, so
the achieved yaw rate matches v*tan(delta)/L. At the limit the front tyres
start to slide and the achieved yaw rate falls away from that prediction while
the steering stays put. The lateral acceleration at which the ratio
achieved/predicted drops below a threshold is the grip limit.

Yaw rate comes from the IMU gyro, so nothing here depends on localization, the
map, or the raceline.

**This needs open space, and neither competition track has any.** Measured on
Porto: the car understeers until its circle grows to ~3.2 m across, then grinds
along the walls — 354 collisions in a 100 s run, which corrupts both the yaw
rate and the speed. Both attempts there (steer 0.30 and full lock) showed
lateral acceleration climbing to 4.96 and 7.74 m/s2 with no plateau, but those
numbers are contaminated and should not be built on. To use this properly,
point it at an empty scene, or rework it to ramp steering at a fixed speed
along a straight instead of holding a circle.

Uncontaminated evidence for the same question, if that is all you need: in the
collision-free 22-lap Porto run the car repeatedly sustained ay ~5 m/s2 against
a 3.0 cap, so the cap is certainly conservative.

Usage:
  ros2 run pit_crew skidpad --ros-args -p steer:=0.35 -p v_max:=6.0
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

from pit_crew import vehicle


class Skidpad(Node):

    def __init__(self):
        super().__init__('skidpad')
        p = self.declare_parameters('', [
            ('steer', 0.35),           # rad, held constant
            ('v_start', 1.0),
            ('v_max', 6.5),
            ('v_step', 0.25),
            ('settle_s', 3.0),         # per speed: time to reach steady state
            ('hold_s', 2.0),           # per speed: time actually measured
            ('throttle_gain', 0.040),
        ])
        (self.steer, self.v_start, self.v_max, self.v_step,
         self.settle_s, self.hold_s, throttle_gain) = [x.value for x in p]

        self.throttle_ctl = vehicle.ThrottleController(ff_gain=throttle_gain)
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        self.gyro_z = 0.0
        self.v_target = self.v_start
        self.t_phase = None
        self.samples = []
        self.results = []

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'{ns}/{side}_encoder',
                lambda m, s=side: self.on_enc(s, m), qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.create_timer(0.02, self.step)
        self.get_logger().info(
            f'skidpad: steer {self.steer:.3f} rad -> kinematic radius '
            f'{vehicle.WHEELBASE / math.tan(self.steer):.2f} m, '
            f'{self.v_start:.1f} to {self.v_max:.1f} m/s')

    def on_imu(self, msg: Imu):
        self.gyro_z = float(msg.angular_velocity.z)

    def on_enc(self, side, msg: JointState):
        if msg.position:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(side, stamp, msg.position[0])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def step(self):
        t = self.now()
        if self.t_phase is None:
            self.t_phase = t
        elapsed = t - self.t_phase

        self.pub_steer.publish(
            Float32(data=vehicle.steer_to_cmd(self.steer)))
        self.pub_throttle.publish(
            Float32(data=float(self.throttle_ctl.command(self.v_target))))

        if elapsed > self.settle_s:
            self.samples.append((self.speed_est.speed, abs(self.gyro_z)))

        if elapsed > self.settle_s + self.hold_s:
            self.record()
            self.t_phase = t
            self.samples = []
            self.v_target += self.v_step
            if self.v_target > self.v_max:
                self.report()
                self.pub_throttle.publish(Float32(data=0.0))
                self.pub_steer.publish(Float32(data=0.0))
                rclpy.shutdown()

    def record(self):
        if len(self.samples) < 10:
            return
        n = len(self.samples)
        v = sum(s[0] for s in self.samples) / n
        yr = sum(s[1] for s in self.samples) / n
        if v < 0.3:
            return
        # What the bicycle model says this steering angle should give
        yr_kin = v * math.tan(abs(self.steer)) / vehicle.WHEELBASE
        ay = yr * v                      # actually achieved
        ay_kin = yr_kin * v              # demanded
        self.results.append((self.v_target, v, yr, yr_kin, ay, ay_kin))
        self.get_logger().info(
            f'v_cmd {self.v_target:4.2f}  v {v:4.2f}  yaw_rate {yr:5.2f} '
            f'(kinematic {yr_kin:5.2f}, ratio {yr / max(yr_kin, 1e-3):4.2f})  '
            f'ay {ay:5.2f} m/s2')

    def report(self):
        self.get_logger().info('--- skidpad result ---')
        limit = None
        for v_cmd, v, yr, yr_kin, ay, ay_kin in self.results:
            ratio = yr / max(yr_kin, 1e-3)
            if ratio < 0.85 and limit is None:
                limit = ay
        peak = max((r[4] for r in self.results), default=0.0)
        self.get_logger().info(
            f'peak achieved lateral accel: {peak:.2f} m/s2')
        if limit is not None:
            self.get_logger().info(
                f'grip limit (yaw rate 15% below kinematic): {limit:.2f} m/s2')
        else:
            self.get_logger().info(
                'no slide detected — grip limit is above the speeds tested')


def main():
    rclpy.init()
    node = Skidpad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
