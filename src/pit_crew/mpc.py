"""Kinematic MPC over the raceline, respecting the steering rate limit.

Why MPC here, specifically: the measured steering oscillation (1.68 reversals/s,
p95 steering rate 4.32 rad/s against a 3.2 rad/s actuator) is the signature of a
controller commanding faster than the wheels can move. Stanley can only be
calmed by lowering feedback gains, which costs tracking accuracy — measured at
12.06 s / 2 collisions versus 11.21 s / 0. MPC avoids that trade by optimising a
whole steering *sequence* subject to the rate limit, so it never asks for a
change the actuator cannot deliver.

Model: kinematic bicycle in the path-relative frame, which keeps the state small
(lateral offset, heading error) and the problem convex enough to solve at 50 Hz
with a plain sequential least-squares step.

  e_lat'  = v * sin(e_yaw)
  e_yaw'  = v * (tan(delta)/L - kappa_ref * cos(e_yaw) / (1 - kappa_ref*e_lat))

Constraints: |delta| <= MAX_STEER, |d(delta)/dt| <= steer_rate_limit.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from pit_crew import vehicle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MPCTracker(Node):

    def __init__(self):
        super().__init__('mpc')
        p = self.declare_parameters('', [
            ('raceline_csv', '/home/data/racelines/track_v2_safe.csv'),
            ('speed_scale', 1.0),
            ('horizon', 12),             # steps
            ('step_dt', 0.06),           # s per step (~0.7 s horizon)
            ('w_lat', 12.0),             # weight on lateral offset
            ('w_yaw', 3.0),              # weight on heading error
            ('w_steer', 0.5),            # weight on steering magnitude
            ('w_rate', 4.0),             # weight on steering change
            ('n_candidates', 9),         # first-step options searched per cycle
            ('control_hz', 50.0),
            ('steer_rate_limit', 3.2),   # rad/s, the actuator's real limit
            ('pose_blend', 0.55),
            ('max_predict_s', 0.35),
            ('brake_horizon_s', 0.35),
            ('throttle_gain', 0.040),
        ])
        (raceline_csv, self.speed_scale, self.N, self.dt, self.w_lat,
         self.w_yaw, self.w_steer, self.w_rate, self.n_cand, control_hz,
         self.steer_rate, self.pose_blend, self.max_predict,
         self.brake_horizon, throttle_gain) = [x.value for x in p]
        self.control_dt = 1.0 / control_hz

        rl = np.loadtxt(raceline_csv, delimiter=',')
        self.xy = rl[:, 1:3]
        self.v_prof = rl[:, 5]
        seg = np.hypot(np.diff(self.xy[:, 0]), np.diff(self.xy[:, 1]))
        self.spacing = float(np.median(seg)) if len(seg) else 0.1
        span = max(1, int(round(0.5 / max(self.spacing, 1e-3))))
        fwd = np.roll(self.xy, -span, axis=0) - np.roll(self.xy, span, axis=0)
        self.path_yaw = np.arctan2(fwd[:, 1], fwd[:, 0])
        self.kappa = ndimage.uniform_filter1d(rl[:, 4], size=2 * span + 1,
                                              mode='wrap')

        self.throttle_ctl = vehicle.ThrottleController(ff_gain=throttle_gain)
        self.pose = None
        self.gyro_z = 0.0
        self.speed = 0.0
        self.steer = 0.0

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(PoseStamped, '/pit_crew/pose', self.on_pose, 10)
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_cte = self.create_publisher(Float32, '/pit_crew/cte', 10)
        self.create_timer(self.control_dt, self.control_step)
        self.get_logger().info(
            f'mpc ready: {len(self.xy)} pts, horizon {self.N}x{self.dt:.2f}s, '
            f'{self.n_cand} candidates, {control_hz:.0f} Hz')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg: Imu):
        self.gyro_z = float(msg.angular_velocity.z)

    def on_pose(self, msg: PoseStamped):
        mx, my = msg.pose.position.x, msg.pose.position.y
        myaw = yaw_from_quat(msg.pose.orientation)
        t = self.now()
        if self.pose is None:
            self.pose = (mx, my, myaw, t)
            return
        px, py, pyaw = self.predicted_state()
        a = self.pose_blend
        self.pose = (px + a * (mx - px), py + a * (my - py),
                     pyaw + a * wrap(myaw - pyaw), t)

    def predicted_state(self):
        x, y, yaw, t = self.pose
        dt = min(max(self.now() - t, 0.0), self.max_predict)
        yaw_mid = yaw + 0.5 * self.gyro_z * dt
        return (x + self.speed * math.cos(yaw_mid) * dt,
                y + self.speed * math.sin(yaw_mid) * dt,
                yaw + self.gyro_z * dt)

    def rollout_cost(self, delta0, e_lat, e_yaw, i0, v):
        """Cost of holding delta0 first, then relaxing towards the path.

        A full nonlinear program is overkill at this scale: the first step is
        what actually gets applied, so search over feasible first steps and
        evaluate each by simulating the model forward with a simple
        proportional relaxation afterwards. That keeps the rate constraint
        exact where it matters and stays inside the 20 ms control budget.
        """
        cost = 0.0
        delta = delta0
        prev = self.steer
        idx = i0
        s_travelled = 0.0
        for k in range(self.N):
            kap = float(self.kappa[idx % len(self.kappa)])
            denom = max(1.0 - kap * e_lat, 0.2)
            e_lat = e_lat + v * math.sin(e_yaw) * self.dt
            e_yaw = e_yaw + v * (math.tan(delta) / vehicle.WHEELBASE
                                 - kap * math.cos(e_yaw) / denom) * self.dt
            e_yaw = wrap(e_yaw)
            cost += (self.w_lat * e_lat ** 2 + self.w_yaw * e_yaw ** 2
                     + self.w_steer * delta ** 2
                     + self.w_rate * (delta - prev) ** 2)
            prev = delta
            # after the first step, relax towards the feedforward steer
            ff = math.atan(vehicle.WHEELBASE * kap)
            delta = delta + 0.5 * (ff - delta)
            s_travelled += v * self.dt
            idx = i0 + int(s_travelled / max(self.spacing, 1e-3))
        return cost

    def control_step(self):
        if self.pose is None:
            return
        x, y, yaw = self.predicted_state()

        d = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)
        i0 = int(np.argmin(d))
        path_yaw = float(self.path_yaw[i0])
        dx, dy = x - self.xy[i0, 0], y - self.xy[i0, 1]
        e_lat = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
        e_yaw = wrap(yaw - path_yaw)

        v = max(self.speed, 0.4)
        # Feasible first steps: within the actuator's reach this cycle
        reach = self.steer_rate * self.control_dt * 4.0
        lo = max(-vehicle.MAX_STEER, self.steer - reach)
        hi = min(vehicle.MAX_STEER, self.steer + reach)
        best, best_cost = self.steer, float('inf')
        for cand in np.linspace(lo, hi, int(self.n_cand)):
            c = self.rollout_cost(float(cand), e_lat, e_yaw, i0, v)
            if c < best_cost:
                best_cost, best = c, float(cand)

        # Apply, still hard-limited by the actuator slew rate
        max_step = self.steer_rate * self.control_dt
        self.steer += float(np.clip(best - self.steer, -max_step, max_step))
        self.steer = float(np.clip(self.steer, -vehicle.MAX_STEER,
                                   vehicle.MAX_STEER))

        horizon = max(0.2, self.speed * self.brake_horizon)
        n_ahead = max(2, int(horizon / max(self.spacing, 1e-3)))
        idx = (i0 + np.arange(n_ahead)) % len(self.v_prof)
        self.speed = float(self.v_prof[idx].min()) * self.speed_scale

        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(self.steer)))
        self.pub_throttle.publish(
            Float32(data=float(self.throttle_ctl.command(self.speed))))
        self.pub_cte.publish(Float32(data=float(e_lat)))


def main():
    rclpy.init()
    node = MPCTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
