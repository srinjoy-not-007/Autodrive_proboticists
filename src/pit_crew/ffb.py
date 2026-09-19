"""ROS node for the feedforward+feedback steering law (`ffb_core.FFBSteer`).

`mpc3`'s node, its solver and its speed plan, unchanged -- but the ANGLE that
is published is the cheap law's, unless holding that angle would take the car
out of the corridor inside the horizon, in which case the solver's command is
published for that frame.  `ffb_k 0` plus `ffb_env_e 0` is not mpc3; set
`ffb_bypass:=true` for that.

    CONTROLLER=ffb MAP=finals_mesh LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh ...
"""

import rclpy

from .ffb_core import FFB_DEFAULTS, FFBSteer, envelope_trip
from .mpc3 import MPC3Tracker


class FFBTracker(MPC3Tracker):

    NODE_NAME = 'ffb'
    EXTRA_PARAMS = tuple((k, v) for k, v in FFB_DEFAULTS.items()) + (
        ('ffb_bypass', False),)

    def __init__(self):
        super().__init__()
        self.ffb = FFBSteer(self.path,
                            **{k: self.P[k] for k in FFB_DEFAULTS})
        self._trips = 0
        self._calls = 0
        self.get_logger().info(
            'ffb: curvature-preview feedforward + pure-pursuit feedback, '
            f"dead {self.P['ffb_dead_s']:.3f} s, k {self.P['ffb_k']:.2f}, "
            f"Ld {self.P['ffb_ld_k']:.2f} v; MPC envelope margin "
            f"{self.P['ffb_env_margin']:.2f} m over "
            f"{int(self.P['ffb_env_steps'])} steps"
            + ('  [BYPASSED: mpc3 drives]' if self.P['ffb_bypass'] else ''))

    def post_steer(self, cmd, info, now, i0, e_lat, e_yaw, v0, cap):
        if self.P['ffb_bypass']:
            return cmd
        period = float(self.rate_ctl.est.period)
        cand, _d = self.ffb.command(e_lat, e_yaw, self.act.delta_at(now), i0,
                                    v0, cap, period)
        self._calls += 1
        trip = abs(float(e_lat)) > float(self.P['ffb_env_e'])
        if not trip:
            trip = envelope_trip(self.path, i0, e_lat, e_yaw, cand, v0,
                                 self.mpc.dt, int(self.P['ffb_env_steps']),
                                 float(self.P['ffb_env_margin']),
                                 yaw_gain=float(self.P['yaw_gain']))
        if trip:
            self._trips += 1
            self.ffb.hold(cmd)
            return cmd
        self.mpc.last_cmd = float(cand)
        return float(cand)


def main():
    rclpy.init()
    node = FFBTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
