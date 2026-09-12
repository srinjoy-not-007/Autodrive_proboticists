"""Construct the REAL node classes on the host and drive them for 30 cycles.

The gap this closes: rclpy and the ROS message packages are not installed on
the host, so no host test has ever executed a node's `__init__` or a callback.
A refactor renamed `_speed_cap` -> `speed_cap` in `localization.py` and left the
old call in `__init__`; `py_compile` and 200 host tests passed, the node raised
`AttributeError` on the first line of the race and the car never moved.

`tests/rclpy_stub` supplies the minimum middleware: `declare_parameters`
returns objects with `.value`, publishers and subscriptions and timers record,
the logger is a sink that keeps what it was told, and the clock is `time.time()`
plus a test-controlled offset so 3 s of virtual driving costs no real time.

Each node is then fed a realistic 10 Hz sensor stream -- IMU with a quaternion
and a gyro rate, two encoders with increasing cumulative wheel angle, a 1080
beam LaserScan, a pose walking along the racing line, steering feedback -- and
the timers are fired every cycle.  The assertions are deliberately about
liveness, not quality: the node constructs, nothing raises, no error is logged,
and the commands that come out are finite and inside the actuator's range.

Set `PIT_CREW_SRC` to point the import at a different copy of the sources; the
mutation check in the commit message uses it to confirm this test fails when
`self._speed_cap()` is put back.
"""

import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, 'tests')
SRC = os.environ.get('PIT_CREW_SRC') or os.path.join(ROOT, 'src')
DATA = os.environ.get('PIT_CREW_DATA') or os.path.join(ROOT, 'data')

sys.path.insert(0, TESTS)
import rclpy_stub                                              # noqa: E402

rclpy_stub.install()
sys.path.insert(0, SRC)

from pit_crew import vehicle                                   # noqa: E402

MAP_PREFIX = os.path.join(DATA, 'maps', 'porto_mesh')
CENTERLINE = os.path.join(DATA, 'racelines', 'porto_mesh_centerline.csv')
MPC3_LINE = os.path.join(DATA, 'racelines', 'porto_mesh_mc0.63_ay6.75.csv')
MPC_LINE = os.path.join(DATA, 'racelines', 'track_v2_safe.csv')

CYCLES = 30
DT = 0.1                      # the sensors really arrive at ~10 Hz
V_DRIVE = 3.0                 # m/s the synthetic car travels at


# ------------------------------------------------------------------ messages

def stamp_of(t):
    from sensor_msgs.msg import LaserScan                       # any stamped
    s = LaserScan().header.stamp
    s.sec = int(t)
    s.nanosec = int(round((t - int(t)) * 1e9))
    return s


def imu_msg(t, yaw, gyro_z):
    from sensor_msgs.msg import Imu
    m = Imu()
    m.header.stamp = stamp_of(t)
    m.header.frame_id = 'imu'
    m.orientation.x = 0.0
    m.orientation.y = 0.0
    m.orientation.z = math.sin(yaw / 2.0)
    m.orientation.w = math.cos(yaw / 2.0)
    m.angular_velocity.z = float(gyro_z)
    m.linear_acceleration.x = 0.0
    return m


def encoder_msg(t, wheel_angle):
    from sensor_msgs.msg import JointState
    m = JointState()
    m.header.stamp = stamp_of(t)
    m.name = ['wheel']
    m.position = [float(wheel_angle)]
    m.velocity = [0.0]
    return m


def scan_msg(t, rng=3.0):
    from sensor_msgs.msg import LaserScan
    m = LaserScan()
    m.header.stamp = stamp_of(t)
    m.header.frame_id = 'lidar'
    m.angle_min = vehicle.LIDAR_ANGLE_MIN
    m.angle_increment = vehicle.LIDAR_ANGLE_INC
    m.angle_max = vehicle.LIDAR_ANGLE_MIN + \
        vehicle.LIDAR_ANGLE_INC * (vehicle.LIDAR_N - 1)
    m.range_min = 0.05
    m.range_max = 10.0
    m.ranges = [float(rng)] * vehicle.LIDAR_N
    return m


def pose_msg(t, x, y, yaw):
    from geometry_msgs.msg import PoseStamped
    m = PoseStamped()
    m.header.stamp = stamp_of(t)
    m.header.frame_id = 'map'
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.orientation.z = math.sin(yaw / 2.0)
    m.pose.orientation.w = math.cos(yaw / 2.0)
    return m


def float32(v):
    from std_msgs.msg import Float32
    return Float32(data=float(v))


# --------------------------------------------------------------------- line

class Line:
    """A racing line, walked at a constant speed, as a pose source.

    The heading comes from the GEOMETRY, not from the file's `psi_rad`: that
    column is in the TUM convention (0 along +y), so feeding it straight in
    puts every pose 1.57 rad off the tangent the tracker computes, which reads
    as a 3 m cross-track error and is a bug in the test, not the node.
    """

    def __init__(self, path):
        a = np.loadtxt(path, delimiter=',')
        self.s = a[:, 0]
        self.xy = a[:, 1:3]
        self.kappa = a[:, 4]
        fwd = np.roll(self.xy, -2, axis=0) - np.roll(self.xy, 2, axis=0)
        self.psi = np.arctan2(fwd[:, 1], fwd[:, 0])

    def at(self, s):
        s = float(s) % float(self.s[-1])
        i = int(np.searchsorted(self.s, s, side='right')) - 1
        i = max(0, min(i, len(self.s) - 2))
        f = (s - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-9)
        x = self.xy[i, 0] + f * (self.xy[i + 1, 0] - self.xy[i, 0])
        y = self.xy[i, 1] + f * (self.xy[i + 1, 1] - self.xy[i, 1])
        return x, y, float(self.psi[i]), float(self.kappa[i])


# ------------------------------------------------------------------- driving

def make_node(cls, **overrides):
    rclpy_stub.reset_clock()
    rclpy_stub.set_overrides(overrides)
    node = cls()
    rclpy_stub.set_overrides({})
    return node


def drive_tracker(node, line, cycles=CYCLES, dt=DT):
    """Feed one tracker a 10 Hz stream walking `line`, firing timers too."""
    wheel = 0.0
    s = 0.0
    for _ in range(cycles):
        t = rclpy_stub.advance(dt)
        x, y, yaw, kappa = line.at(s)
        node.stub_deliver('/imu', imu_msg(t, yaw, kappa * V_DRIVE))
        wheel += V_DRIVE * dt / vehicle.WHEEL_RADIUS
        node.stub_deliver('/left_encoder', encoder_msg(t, wheel))
        node.stub_deliver('/right_encoder', encoder_msg(t, wheel))
        if any(sub.topic.endswith('/steering') for sub in node._stub_subs):
            node.stub_deliver('/steering', float32(node.steer))
        node.stub_deliver('/pit_crew/pose', pose_msg(t, x, y, yaw))
        node.stub_fire_timers()
        s += V_DRIVE * dt
    return node


def drive_perception(node, yaw0, cycles=CYCLES, dt=DT):
    """Feed a map-free / map-matching node the same stream without a pose."""
    wheel = 0.0
    for _ in range(cycles):
        t = rclpy_stub.advance(dt)
        node.stub_deliver('/imu', imu_msg(t, yaw0, 0.05))
        wheel += V_DRIVE * dt / vehicle.WHEEL_RADIUS
        node.stub_deliver('/left_encoder', encoder_msg(t, wheel))
        node.stub_deliver('/right_encoder', encoder_msg(t, wheel))
        if any(sub.topic.endswith('/throttle') for sub in node._stub_subs):
            node.stub_deliver('/throttle', float32(0.12))
        node.stub_deliver('/lidar', scan_msg(t))
        node.stub_fire_timers()
    return node


# ----------------------------------------------------------------- assertions

def assert_no_errors(node, allow=()):
    bad = [m for m in node._stub_logger.errors()
           if not any(a in m for a in allow)]
    assert bad == [], bad


def assert_commands(node, n_min=1):
    steer = node.stub_published('/steering_command')
    throttle = node.stub_published('/throttle_command')
    assert len(steer) >= n_min and len(throttle) >= n_min, \
        (len(steer), len(throttle))
    for msgs, what in ((steer, 'steering'), (throttle, 'throttle')):
        for m in msgs:
            v = float(m.data)
            assert math.isfinite(v), f'{what} not finite: {v}'
            assert -1.0 <= v <= 1.0, f'{what} out of range: {v}'
    # A tracker that only ever published zero steering is not tracking.
    assert any(abs(float(m.data)) > 1e-6 for m in steer), \
        'steering was zero on every cycle'


# ---------------------------------------------------------------------- tests

class TestLocalization:
    """The node the rename bug killed."""

    def node(self, **over):
        from pit_crew.localization import ParticleFilter
        over.setdefault('map_prefix', MAP_PREFIX)
        over.setdefault('centerline_csv', CENTERLINE)
        over.setdefault('lidar_offset', vehicle.LIDAR_FROM_POSE)
        over.setdefault('n_particles', 200)      # runtime, not behaviour
        return make_node(ParticleFilter, **over)

    def test_it_constructs(self):
        n = self.node()
        assert n.grid.shape == n.dist.shape
        assert n.centerline is not None and len(n.centerline) > 10
        # seeded at the centerline start, not (0, 0) -- and speed_cap ran
        assert n.init_xy == pytest.approx(tuple(n.centerline[0]), abs=1e-6)
        assert n._v_cap > 0.0

    def test_lidar_offset_comes_from_the_map(self):
        n = self.node()
        assert n.lidar_offset == pytest.approx(0.1933, abs=1e-6)

    def test_it_publishes_a_finite_pose_for_a_full_stream(self):
        line = Line(MPC3_LINE)
        _, _, yaw0, _ = line.at(0.0)
        n = drive_perception(self.node(), yaw0)
        poses = n.stub_published('/pit_crew/pose')
        assert len(poses) >= CYCLES - 2, len(poses)
        for m in poses:
            for v in (m.pose.position.x, m.pose.position.y,
                      m.pose.orientation.z, m.pose.orientation.w):
                assert math.isfinite(v), m
            assert m.header.stamp.sec > 0
        scores = n.stub_published('/pit_crew/loc_score')
        assert scores and all(math.isfinite(float(m.data)) for m in scores)
        # A uniform 3 m scan is not a real map match, so RELOCALIZING is fine;
        # nothing should have been logged at error level.
        assert_no_errors(n)


class TestMPC3:
    """The shipping controller."""

    def node(self, **over):
        from pit_crew.mpc3 import MPC3Tracker
        over.setdefault('raceline_csv', MPC3_LINE)
        over.setdefault('map_prefix', MAP_PREFIX)
        return make_node(MPC3Tracker, **over)

    def test_it_constructs_with_a_corridor(self):
        n = self.node()
        assert n.path.has_corridor
        assert n.control_mode == 'per_pose'

    def test_it_tracks_and_publishes(self):
        n = drive_tracker(self.node(), Line(MPC3_LINE))
        assert n._pose_ok >= CYCLES - 2, (n._pose_ok, n._pose_drops)
        assert_commands(n, n_min=CYCLES - 2)
        assert_no_errors(n)
        cte = n.stub_published('/pit_crew/cte')
        assert cte and all(abs(float(m.data)) < 2.0 for m in cte)

    def test_rate_adaptive_and_ilc_together(self):
        n = self.node(rate_adaptive=True, ilc_enable=True)
        assert n.ilc is not None and n.rate_ctl.enabled
        drive_tracker(n, Line(MPC3_LINE))
        assert_commands(n, n_min=CYCLES - 2)
        assert_no_errors(n)

    @pytest.mark.parametrize('flag', ['rate_adaptive', 'ilc_enable'])
    def test_each_flag_alone(self, flag):
        n = self.node(**{flag: True})
        drive_tracker(n, Line(MPC3_LINE))
        assert_commands(n, n_min=CYCLES - 2)
        assert_no_errors(n)

    def test_timer_mode_solves_on_the_clock(self):
        """`control_mode='timer'` is uncommitted work in the tree: the pose
        callback only stores the estimate, so if the tick were not wired up the
        car would never get a command at all."""
        n = self.node(control_mode='timer', control_hz=20.0)
        drive_tracker(n, Line(MPC3_LINE))
        assert_commands(n, n_min=CYCLES - 2)
        assert_no_errors(n)

    def test_the_watchdog_coasts_when_the_pose_stops(self):
        n = drive_tracker(self.node(), Line(MPC3_LINE), cycles=5)
        rclpy_stub.advance(1.0)                  # the pose goes away
        n.stub_fire_timers()
        throttle = n.stub_published('/throttle_command')
        assert float(throttle[-1].data) == 0.0
        assert any('coasting' in m for m in n._stub_logger.errors())


class TestMPC:
    """The earlier tracker, kept for comparison, on its own shipping line."""

    def node(self, **over):
        from pit_crew.mpc import MPCTracker
        over.setdefault('raceline_csv', MPC_LINE)
        return make_node(MPCTracker, **over)

    def test_it_constructs(self):
        n = self.node()
        assert len(n.xy) > 10 and np.isfinite(n.v_prof).all()

    def test_it_tracks_and_publishes(self):
        n = drive_tracker(self.node(), Line(MPC_LINE))
        assert_commands(n, n_min=CYCLES - 2)
        assert_no_errors(n)


class TestFollowTheGap:
    """The map-free fallback: no pose, no line, LiDAR only."""

    def test_it_constructs_and_publishes(self):
        from pit_crew.follow_the_gap import FollowTheGap
        n = drive_perception(make_node(FollowTheGap), yaw0=0.0)
        steer = n.stub_published('/steering_command')
        throttle = n.stub_published('/throttle_command')
        assert len(steer) >= CYCLES and len(throttle) >= CYCLES
        for msgs in (steer, throttle):
            for m in msgs:
                v = float(m.data)
                assert math.isfinite(v) and -1.0 <= v <= 1.0, v
        # an open 3 m corridor all round: it should be asking for speed
        assert max(float(m.data) for m in throttle) > 0.0
        assert_no_errors(n)

    def test_a_frozen_lidar_zeroes_the_throttle(self):
        """The headless-LiDAR trap: a stale scan must not leave a latched
        command running."""
        from pit_crew.follow_the_gap import FollowTheGap
        n = drive_perception(make_node(FollowTheGap), yaw0=0.0, cycles=5)
        rclpy_stub.advance(1.0)
        n.stub_fire_timers()
        assert float(n.stub_published('/throttle_command')[-1].data) == 0.0
