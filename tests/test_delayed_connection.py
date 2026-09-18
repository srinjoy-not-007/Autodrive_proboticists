"""The organisers press **Connection** by hand, at a time we do not control.

The sequence on their box is: our container starts, the ROS bridge comes up,
localization and the controller come up -- and then nothing at all arrives on
any sensor topic until a human clicks Connection in the simulator's window.
That may be seconds; it may be minutes; the simulator may not even have been
started yet.  Two things have to be true for the whole of that window:

1. neither node exits, logs an error, or commands the actuators, and
2. when data finally does arrive, the stack tracks normally -- the wait must
   not have poisoned any filter, latch or watchdog.

The supervisor in ``submission/autodrive_devkit.sh`` falls back to the
map-free reactive stack after four restarts, and a restart is triggered by a
child *process exiting*.  So a node that merely sits there with no data can
never trigger the fallback -- provided it really does just sit there, which is
what the first class measures against the real node classes, and what the
second asserts about the supervisor loop's only trigger.
"""

import math
import os
import re

import pytest

import test_node_smoke as smoke

WAIT_S = 300.0          # five minutes before anyone touches the simulator
WATCHDOG_DT = 0.05      # finer than the 20 Hz watchdog, so every tick fires


def idle(node, seconds=WAIT_S, dt=WATCHDOG_DT):
    """Advance the clock with NO sensor messages, firing the node's timers."""
    for _ in range(int(seconds / dt)):
        smoke.rclpy_stub.advance(dt)
        node.stub_fire_timers()
    return node


def commands(node):
    return (node.stub_published('/steering_command')
            + node.stub_published('/throttle_command'))


class TestFiveMinutesBeforeConnection:
    def tracker(self):
        from pit_crew.mpc3 import MPC3Tracker
        return smoke.make_node(
            MPC3Tracker, raceline_csv=smoke.MPC3_LINE,
            map_prefix=smoke.MAP_PREFIX, rate_adaptive=True)

    def localization(self):
        from pit_crew.localization import ParticleFilter
        return smoke.make_node(
            ParticleFilter, map_prefix=smoke.MAP_PREFIX,
            centerline_csv=smoke.CENTERLINE, n_particles=200,
            lidar_offset=smoke.vehicle.LIDAR_FROM_POSE)

    def test_the_tracker_commands_nothing_and_logs_nothing(self):
        n = idle(self.tracker())
        assert commands(n) == [], \
            'the tracker actuated the car before any sensor data arrived'
        smoke.assert_no_errors(n)

    def test_the_tracker_watchdog_really_ran(self):
        """Guard against the test passing because no timer exists at all."""
        n = idle(self.tracker(), seconds=5.0)
        fired = sum(t.calls for t in n._stub_timers)
        assert fired >= 90, fired

    def test_localization_publishes_no_pose_and_logs_nothing(self):
        n = idle(self.localization())
        assert n.stub_published('/pit_crew/pose') == []
        smoke.assert_no_errors(n)

    def test_the_tracker_tracks_normally_after_the_wait(self):
        """The point of the whole file: a five-minute wait costs nothing."""
        n = idle(self.tracker())
        line = smoke.Line(smoke.MPC3_LINE)
        smoke.drive_tracker(n, line)
        smoke.assert_commands(n)
        smoke.assert_no_errors(n)

    def test_localization_converges_normally_after_the_wait(self):
        n = idle(self.localization())
        line = smoke.Line(smoke.MPC3_LINE)
        _, _, yaw0, _ = line.at(0.0)
        smoke.drive_perception(n, yaw0)
        poses = n.stub_published('/pit_crew/pose')
        assert len(poses) >= smoke.CYCLES - 2, len(poses)
        for m in poses:
            assert math.isfinite(m.pose.position.x)
            assert math.isfinite(m.pose.position.y)
        smoke.assert_no_errors(n)


class TestTheSupervisorCannotFallBackOnSilence:
    """A source-level assertion, because the loop cannot be run on the host:
    the ONLY thing that advances the restart counter is a child exiting."""

    def loop(self):
        src = open(os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'submission', 'autodrive_devkit.sh')).read()
        return src[src.index('while :; do'):]

    def test_the_only_trigger_is_a_child_exiting(self):
        loop = self.loop()
        assert re.search(r'^\s*wait -n "\$BRIDGE" "\$\{WORKERS\[@\]\}"', loop,
                         re.M), loop
        # No timeout, deadline or data-liveness check may gate the loop: those
        # would turn "the human has not clicked Connection yet" into a restart.
        for forbidden in ('timeout', 'ros2 topic', 'date +%s'):
            assert forbidden not in loop, forbidden
        # SECONDS may appear ONLY to FORGIVE earlier deaths after a child has
        # already exited (Sep 19 audit A1) -- never to trigger a restart.
        allowed = ('if (( SECONDS - LAST_RESTART > RESTART_FORGIVE_S )); '
                   'then RESTARTS=0; fi', 'LAST_RESTART=$SECONDS')
        for line in loop.splitlines():
            code = line.split('#', 1)[0].strip()
            if 'SECONDS' in code:
                assert code in allowed, code
                assert loop.index('wait -n') < loop.index(code)

    def test_the_restart_counter_only_advances_after_a_wait(self):
        loop = self.loop()
        assert loop.index('wait -n') < loop.index('RESTARTS=$((RESTARTS + 1))')

    @pytest.mark.parametrize('node', ['localization', 'mpc3', 'follow_the_gap'])
    def test_no_node_exits_on_missing_data(self, node):
        """`sys.exit` / `rclpy.shutdown` may only appear in `main`'s teardown --
        a node that exited itself on silence WOULD reach the supervisor."""
        path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'src', 'pit_crew', node + '.py')
        src = open(path).read()
        body = src[:src.index('def main(')] if 'def main(' in src else src
        for forbidden in ('sys.exit', 'rclpy.shutdown(', 'raise SystemExit'):
            assert forbidden not in body, (node, forbidden)
