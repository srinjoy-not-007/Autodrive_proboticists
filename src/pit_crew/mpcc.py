"""ROS node for the joint speed-and-steering MPC (`mpcc_core.MPCC`).

Everything except the control law is `mpc3.MPC3Tracker`'s: the same
subscriptions, the same pose filter and dead-reckoning, the same rate
adaptivity, watchdog, stale-pose ride-out, stuck escape and publish guard, and
the same parameters.  Only three things differ, and they are the candidate:

* the solver is `MPCC` -- speed is a state and the longitudinal acceleration
  an input of the same Gauss-Newton problem;
* the extra `MPCC_DEFAULTS` parameters are declared, so every knob is settable
  with `-p` exactly as mpc3's are;
* the BRAKE is the solver's decision.  `ServoThrottle`'s own `brake_mode`
  trigger is switched off and the exact 0.0 is published when the optimised
  longitudinal plan asks for more deceleration than `mpcc_brake_a` at the
  instant the command is actually in force.

Run it exactly as mpc3 is run:

    CONTROLLER=mpcc MAP=finals_mesh LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh ...
"""

import rclpy

from .mpc3 import MPC3Tracker
from .mpcc_core import MPCC, MPCC_DEFAULTS


class MPCCTracker(MPC3Tracker):

    NODE_NAME = 'mpcc'
    MPC_CLASS = MPCC
    EXTRA_PARAMS = tuple((k, v) for k, v in MPCC_DEFAULTS.items())

    def __init__(self):
        super().__init__()
        # The solver owns the brake; the throttle law is pure feedforward.
        self._thr_brake_mode = str(self.thr.brake_mode)
        self.thr.brake_mode = 'none'
        self._mpcc_brakes = 0
        self.get_logger().info(
            'mpcc: joint speed+steering MPC, envelope r_break '
            f"{self.P['r_break']:.2f} rad/s / ay_break "
            f"{self.P['ay_break']:.1f} m/s2 x {self.P['grip_use']:.2f}, "
            f"plan brake {self.P['lon_brake_plan']:.1f} m/s2, brake at "
            f"a < -{self.P['mpcc_brake_a']:.1f} m/s2 "
            f'(ServoThrottle brake_mode {self._thr_brake_mode} -> none)')

    def extra_solver_args(self, v):
        return {k: v[k] for k in MPCC_DEFAULTS if k in v}

    def post_throttle(self, th, info, v):
        if info.get('brake'):
            th = 0.0
            self._mpcc_brakes += 1
        self.mpc.note_throttle(th, v)
        return th


def main():
    rclpy.init()
    node = MPCCTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
