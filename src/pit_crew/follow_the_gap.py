"""Reactive Follow-the-Gap (disparity extender) driver.

LiDAR-only baseline: qualifies on any track without a map and serves as the
safety fallback for the raceline stack. Based on the disparity-extender
approach (UNC, F1TENTH 2019) with a safety bubble and clearance-scaled speed.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float32

from pit_crew import vehicle


class FollowTheGap(Node):

    def __init__(self):
        super().__init__('follow_the_gap')

        p = self.declare_parameters('', [
            ('v_max', 4.0),                 # m/s cap while tuning; raise later
            ('v_min', 1.0),                 # m/s floor so the car never stalls
            ('clearance_for_vmax', 6.0),    # m of free space ahead => v_max
            ('steer_slowdown', 0.5),        # fraction of speed shed at full lock
            ('disparity_threshold', 0.4),   # m jump between adjacent beams
            ('extend_margin', 0.20),        # m added to half car width
            ('bubble_radius', 0.35),        # m safety bubble at closest point
            ('fov_limit_deg', 100.0),       # gap search restricted to ±this
            ('range_clip', 8.0),            # m; ignore structure farther out
            ('steer_sign', 1.0),            # flip if sim convention is mirrored
            ('steer_alpha', 0.5),           # steering low-pass (1.0 = no filter)
            ('center_gain', 0.25),          # corridor-centering blend [rad]
        ])
        (self.v_max, self.v_min, self.clear_vmax, self.steer_slow,
         self.disp_thresh, self.extend_margin, self.bubble_r,
         fov_deg, self.range_clip, self.steer_sign,
         self.steer_alpha, self.center_gain) = [x.value for x in p]
        self._steer_filt = 0.0

        self.fov_limit = math.radians(fov_deg)
        self.angles = vehicle.LIDAR_ANGLE_MIN + \
            np.arange(vehicle.LIDAR_N) * vehicle.LIDAR_ANGLE_INC

        self.speed_est = vehicle.SpeedEstimator()
        self.throttle_ctl = vehicle.ThrottleController()

        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.create_subscription(JointState, f'{ns}/left_encoder',
                                 lambda m: self.on_encoder('L', m), qos)
        self.create_subscription(JointState, f'{ns}/right_encoder',
                                 lambda m: self.on_encoder('R', m), qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)

        self.get_logger().info('Follow-the-Gap ready (v_max=%.1f m/s)' % self.v_max)

    def on_encoder(self, wheel, msg: JointState):
        # Encoder speed is unusable on this platform (see vehicle.SpeedEstimator);
        # throttle runs open loop. Kept subscribed for diagnostics only.
        pass

    # ------------------------------------------------------------------ #

    def on_scan(self, scan: LaserScan):
        r = np.asarray(scan.ranges, dtype=np.float64)
        n = r.size
        angles = self.angles if n == vehicle.LIDAR_N else \
            scan.angle_min + np.arange(n) * scan.angle_increment

        # Sanitize: NaN/inf/too-close readings become max range
        r = np.where(np.isfinite(r), r, vehicle.LIDAR_RANGE_MAX)
        r = np.clip(r, 0.0, self.range_clip)
        proc = r.copy()

        # Disparity extension: occlude the far side of each depth jump by the
        # car's half width (+margin) computed at the *near* distance.
        half_w = vehicle.CAR_WIDTH / 2.0 + self.extend_margin
        d = np.diff(proc)
        inc = scan.angle_increment if scan.angle_increment > 0 else vehicle.LIDAR_ANGLE_INC
        for i in np.nonzero(np.abs(d) > self.disp_thresh)[0]:
            near = min(proc[i], proc[i + 1])
            if near < 1e-3:
                continue
            n_beams = int(math.atan2(half_w, near) / inc) + 1
            if d[i] > 0:      # jump away from us going CCW: mask beams after i
                proc[i + 1:i + 1 + n_beams] = np.minimum(
                    proc[i + 1:i + 1 + n_beams], near)
            else:             # jump toward us: mask beams before i+1
                lo = max(0, i + 1 - n_beams)
                proc[lo:i + 1] = np.minimum(proc[lo:i + 1], near)

        # Safety bubble around the closest obstacle in the search FOV
        fov_mask = np.abs(angles) <= self.fov_limit
        idx_fov = np.nonzero(fov_mask)[0]
        closest = idx_fov[np.argmin(proc[idx_fov])]
        if proc[closest] < self.bubble_r * 4:
            n_beams = int(math.atan2(self.bubble_r,
                                     max(proc[closest], 0.1)) / inc) + 1
            lo, hi = max(0, closest - n_beams), min(n, closest + n_beams + 1)
            proc[lo:hi] = 0.0

        # Pick the deepest point of the widest free gap within the FOV
        free = (proc > self.bubble_r) & fov_mask
        best_idx = self._best_gap_point(free, proc)
        if best_idx is None:                       # boxed in: stop
            self._publish(0.0, 0.0)
            return

        steer_angle = float(angles[best_idx])

        # Corridor centering: bias away from the closer side wall
        left = (angles > math.radians(60)) & (angles < math.radians(120))
        right = (angles < math.radians(-60)) & (angles > math.radians(-120))
        d_left = float(np.median(r[left]))
        d_right = float(np.median(r[right]))
        denom = d_left + d_right
        if denom > 1e-3:
            steer_angle += self.center_gain * (d_left - d_right) / denom

        # Low-pass the steering to tame 10 Hz jitter
        self._steer_filt += self.steer_alpha * (steer_angle - self._steer_filt)
        steer_angle = self._steer_filt

        # Speed from straight-ahead clearance, shed speed while steering
        ahead = np.abs(angles) <= math.radians(25.0)
        front_clear = float(np.min(r[ahead]))
        v = self.v_min + (self.v_max - self.v_min) * \
            min(1.0, front_clear / self.clear_vmax)
        v *= 1.0 - self.steer_slow * min(1.0, abs(steer_angle) / vehicle.MAX_STEER)
        v = max(self.v_min, v)

        self._publish(steer_angle, v)

    def _best_gap_point(self, free: np.ndarray, proc: np.ndarray):
        """Deepest beam inside the best free gap, preferring forward gaps.

        Score = width * forward_preference(gap center). Without the forward
        bias the widest gap can be behind the car (after a collision reset),
        which produced U-turns and wrong-way driving.
        """
        padded = np.concatenate(([0], free.astype(np.int8), [0]))
        d = np.diff(padded)
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)      # exclusive
        if starts.size == 0:
            return None
        best_score = -1.0
        best = None
        for lo, hi in zip(starts, ends):
            center_angle = self.angles[(lo + hi) // 2] if hi <= self.angles.size \
                else 0.0
            score = (hi - lo) * (0.35 + 0.65 * math.cos(center_angle))
            if score > best_score:
                best_score = score
                best = lo + int(np.argmax(proc[lo:hi]))
        return best

    def _publish(self, steer_angle: float, v_target: float):
        steer = Float32()
        steer.data = self.steer_sign * vehicle.steer_to_cmd(steer_angle)
        throttle = Float32()
        throttle.data = self.throttle_ctl.command(v_target)
        self.pub_steer.publish(steer)
        self.pub_throttle.publish(throttle)


def main():
    rclpy.init()
    node = FollowTheGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
