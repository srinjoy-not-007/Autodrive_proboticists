"""ROS node that re-reads the MPC's plan between poses (`planreplay`).

`mpc3`'s node and solver, unchanged; the solve still happens once per pose, at
maximum freshness -- that is the difference from `control_mode:=timer`, which
re-solved at 20 Hz from a pose up to one whole period old and lost (+0.25 s/lap
and 4-6 collisions in 33 laps at every period, docs/STATUS.md).  Here only the
READOUT walks: a timer at `replay_hz` republishes the plan's angle for the
current instant, so the command the bridge latches at the next telemetry tick
is the plan's angle for that tick rather than one solved up to a period
earlier.

Read `pit_crew/planreplay.py` before spending a live slot on this: no topic in
this simulator arrives faster than the pose, and the bridge latches one command
per tick, so the whole prize is at most one sensor period of readout freshness.

    CONTROLLER=fastpub MAP=finals_mesh LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh ...
"""

import rclpy

from .mpc3 import MPC3Tracker
from .planreplay import PlanReplay


class FastPubTracker(MPC3Tracker):

    NODE_NAME = 'fastpub'
    EXTRA_PARAMS = (('replay_gain', 1.0),
                    ('replay_max_steps', 0.0),
                    ('replay_rate_cap', 3.2),
                    ('replay_hz', 100.0))

    def __init__(self):
        super().__init__()
        self.replay = PlanReplay(gain=float(self.P['replay_gain']),
                                 max_steps=float(self.P['replay_max_steps']),
                                 cmd_tau=float(self.P['cmd_tau']),
                                 rate_cap=float(self.P['replay_rate_cap']))
        hz = max(float(self.P['replay_hz']), 1.0)
        self.create_timer(1.0 / hz, self._replay_tick)
        self.get_logger().info(
            f'fastpub: plan re-read at {hz:.0f} Hz, gain '
            f"{self.P['replay_gain']:.2f}, rate cap "
            f"{self.P['replay_rate_cap']:.1f} rad/s")

    def post_steer(self, cmd, info, now, i0, e_lat, e_yaw, v0, cap):
        pub = (self.mpc._pub[0] if getattr(self.mpc, '_pub', None) is not None
               else info.get('delta'))
        if pub is not None:
            k, frac = self.mpc._apply
            self.replay.set_plan(pub, now, self.mpc.dt, k + frac, cap, cmd)
        return cmd

    def _replay_tick(self):
        if self.pose is None or not self.replay.enabled:
            return
        now = self.now()
        if now - self.pose[3] > float(self.rate_ctl.values['stale_pose_s']):
            return                       # the watchdog owns a stale pose
        a = self.replay.angle(now)
        if a is not None:
            self.publish(a, getattr(self, 'throttle_last', 0.0))


def main():
    rclpy.init()
    node = FastPubTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
