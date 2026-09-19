"""`brake_probe` against two plants that disagree about what a negative
throttle does, which is the whole question the node exists to settle.

Two different things are checked, as in `test_turnin_probe.py`:

1. **The node's logic.**  It sequences accelerate -> brake -> drive home ->
   straighten -> accelerate, publishes exactly the command under test (and
   exactly the float `0.0` for the locked-brake reference), stays inside the
   geofence, never lets the car end up driving backwards, and zeroes the
   actuators on the way out.
2. **The instrument is calibrated.**  The plant's deceleration under a given
   command is KNOWN, so the number the node reports must come back to within a
   fraction of a m/s2 -- on a plant where negative throttle is a proportional
   brake AND on one where it locks the wheels and then reverses.  An instrument
   that could not tell those two apart would answer the question wrongly in the
   most expensive possible direction.

The sim's own behaviour is the plant's skeleton: throttle is a SPEED SETPOINT
(`SERVO_GAIN` m/s per unit) driven by an inner loop, the exact float 0.0 is the
four-wheel brake, and negative throttle is a reverse setpoint.
"""

import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, 'tests')
SRC = os.environ.get('PIT_CREW_SRC') or os.path.join(ROOT, 'src')

sys.path.insert(0, TESTS)
import rclpy_stub                                              # noqa: E402

rclpy_stub.install()
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from pit_crew import vehicle                                   # noqa: E402
from pit_crew import brake_probe                               # noqa: E402

SUB = 0.002             # s, plant integration step
# the node's own default list, spelled out so this test fails if it changes
SHIPPING_SWEEP = [0.0, -0.05, -0.10, -0.15, -0.20, -0.30, -0.45]
SERVO_GAIN = 24.6       # m/s of setpoint per unit throttle (measured)


# ------------------------------------------------------------------ plant

class Floor:
    """A car on an empty floor, with a switchable negative-throttle model.

    `neg='prop'`   -- the hypothesis: a negative command retards the car by
                      `prop_gain * |u|`, capped at the longitudinal tyre peak,
                      with the wheels still rolling.
    `neg='lock'`   -- the pessimistic reading of August: any negative command
                      locks the wheels at `lock_a`, and once the car is slow
                      enough it drives away backwards towards the reverse
                      setpoint.
    """

    def __init__(self, neg='prop', prop_gain=20.0, lock_a=4.55,
                 tyre_peak=brake_probe.TYRE_LONG_PEAK, brake_a=4.55,
                 K=18.0, a_accel=5.0, a_auth=4.5, drag=0.0,
                 slew=vehicle.STEER_RATE):
        self.t = 0.0
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.wheel = 0.0                 # cumulative wheel angle, rad
        self.rack = 0.0
        self.u = 0.0
        self.steer_cmd = 0.0
        self.a = 0.0
        self.neg = neg
        self.prop_gain = float(prop_gain)
        self.lock_a = float(lock_a)
        self.tyre_peak = float(tyre_peak)
        self.brake_a = float(brake_a)
        self.K, self.a_accel, self.a_auth = K, a_accel, a_auth
        self.drag = float(drag)
        self.slew = float(slew)
        self.v_min = 0.0                 # most negative speed ever reached
        self.locked_now = False

    # what the current command does, per sub-step
    def _accel(self):
        u, v = self.u, self.v
        self.locked_now = False
        if u == 0.0:
            self.locked_now = abs(v) > 1e-6
            if abs(v) < 1e-6:
                return 0.0
            return (-self.brake_a if v > 0 else self.brake_a) - self.drag * v
        if u > 0.0:
            a = self.K * (SERVO_GAIN * u - v)
            return max(-self.a_auth, min(self.a_accel, a)) - self.drag * v
        # negative
        if self.neg == 'prop':
            if v > 0.2:
                return -min(self.tyre_peak, self.prop_gain * abs(u))
            a = self.K * (SERVO_GAIN * u - v)
            return max(-self.a_auth, min(self.a_accel, a))
        # 'lock'
        if v > 0.3:
            self.locked_now = True
            return -self.lock_a
        a = self.K * (SERVO_GAIN * u - v)
        return max(-self.a_auth, min(self.a_accel, a))

    def advance(self, dt):
        n = max(1, int(round(dt / SUB)))
        h = dt / n
        for _ in range(n):
            target = self.steer_cmd
            d = target - self.rack
            self.rack += max(-self.slew * h, min(self.slew * h, d))
            self.a = self._accel()
            self.v += self.a * h
            if not self.locked_now:
                self.wheel += self.v / vehicle.WHEEL_RADIUS * h
            self.yaw += self.v * math.tan(self.rack) / vehicle.WHEELBASE * h
            self.x += self.v * math.cos(self.yaw) * h
            self.y += self.v * math.sin(self.yaw) * h
            self.v_min = min(self.v_min, self.v)
            self.t += h

    @property
    def wz(self):
        return self.v * math.tan(self.rack) / vehicle.WHEELBASE

    def dist(self):
        return math.hypot(self.x, self.y)


# ----------------------------------------------------------------- driving

def stamp_of(t):
    from sensor_msgs.msg import JointState
    s = JointState().header.stamp
    s.sec = int(t)
    s.nanosec = int(round((t - int(t)) * 1e9))
    return s


def make_node(tmp_path, **over):
    ov = dict(out_dir=str(tmp_path), u_brakes=[0.0, -0.10, -0.25],
              warmup_s=0.5, v_target=7.0, brake_s=3.0,
              home_r_m=5.0, home_min_s=2.0, max_run_s=400.0)
    ov.update(over)
    rclpy_stub.set_overrides(ov)
    try:
        return brake_probe.BrakeProbe()
    finally:
        rclpy_stub.set_overrides({})


def run(node, plant, dt=0.104, seconds=200.0, timer_dt=0.02, gyro_of=None):
    """One telemetry tick every `dt`, the node's own timer every `timer_dt`.

    The simulator consumes one command per telemetry tick, so the plant only
    latches what the node published when a tick comes round.
    """
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float32

    rclpy_stub.reset_clock()
    t = 0.0
    next_tick = 0.0
    steps = int(seconds / timer_dt)
    for _ in range(steps):
        plant.advance(timer_dt)
        rclpy_stub.advance(timer_dt)
        t += timer_dt
        if t >= next_tick:
            next_tick += dt
            now = rclpy_stub.now_s()

            imu = Imu()
            imu.header.stamp = stamp_of(now)
            imu.angular_velocity.z = float(
                gyro_of(t, node, plant) if gyro_of else plant.wz)
            imu.linear_acceleration.x = float(plant.a)
            node.stub_deliver('/imu', imu)

            for side in ('left_encoder', 'right_encoder'):
                js = JointState()
                js.header.stamp = stamp_of(now)
                js.position = [float(plant.wheel)]
                node.stub_deliver('/' + side, js)

            node.stub_deliver('/throttle', Float32(data=float(plant.u)))
            node.stub_deliver('/ips', Point(x=float(plant.x),
                                            y=float(plant.y), z=0.0))

        node.stub_fire_timers()
        if node.done:
            break
        if t >= next_tick - timer_dt:      # latch on the tick boundary
            thr = node.stub_published('/throttle_command')
            ste = node.stub_published('/steering_command')
            if thr:
                plant.u = float(thr[-1].data)
            if ste:
                plant.steer_cmd = float(ste[-1].data) * vehicle.MAX_STEER
    node.finish()
    return node.results


def by_command(results):
    out = {}
    for r in results:
        out.setdefault(round(r['u_brake'], 4), []).append(r)
    return out


# ------------------------------------------------------------------- tests

def test_fit_decel_recovers_a_known_slope():
    samples = [(0.1 * k, 7.0 - 5.0 * 0.1 * k) for k in range(20)]
    a, r2, n = fit = brake_probe.fit_decel(samples)
    assert n == 20 and abs(a - 5.0) < 1e-9 and r2 > 0.999
    assert math.isnan(brake_probe.fit_decel([(0.0, 1.0)])[0])
    # a flat window is not a deceleration of anything
    assert brake_probe.fit_decel([(0.1 * k, 3.0) for k in range(5)])[0] == 0.0
    assert fit[0] > 0


def test_mean_slip_separates_rolling_from_locked():
    rolling = [(0.0, 6.0, 5.94), (0.1, 5.0, 4.95)]
    locked = [(0.0, 6.0, 0.0), (0.1, 5.0, 0.0)]
    assert abs(brake_probe.mean_slip(rolling)) < 0.02
    assert brake_probe.mean_slip(locked) > 0.99
    # samples below the speed floor are not evidence either way
    assert math.isnan(brake_probe.mean_slip([(0.0, 0.5, 0.0)], v_min=1.0))


@pytest.mark.parametrize('dt', [0.058, 0.104])
def test_proportional_plant_is_measured_correctly(tmp_path, dt):
    """The calibration, hypothesis-true case: reported a == plant a."""
    node = make_node(tmp_path / f'p{int(dt * 1000)}',
                     u_brakes=[0.0, -0.10, -0.25])
    plant = Floor(neg='prop', prop_gain=20.0, brake_a=4.55)
    res = run(node, plant, dt=dt)

    assert not node.get_logger().errors()
    assert node.abort is None, node.abort
    got = by_command(res)
    assert set(got) == {0.0, -0.10, -0.25}, got.keys()

    expect = {0.0: 4.55, -0.10: 2.0, -0.25: 5.0}
    for u, want in expect.items():
        r = got[u][0]
        assert r['n'] >= 5, (u, r)
        assert abs(r['a_ips'] - want) < 0.5, (
            f'u {u}: reported {r["a_ips"]:.2f} m/s2 against the plant\'s '
            f'{want:.2f}')
        assert r['r2'] > 0.9, (u, r['r2'])
        assert r['v_entry'] > 6.5 and not r['short']
        assert not r['reversed']
        # the wheels are rolling under every negative command on this plant,
        # and locked under the exact 0.0
        if u == 0.0:
            # not 1.0: the encoder estimate is low-passed (alpha 0.35, one
            # sensor period of lag), so the first samples of a lock still
            # carry the speed the wheels had a tick ago
            assert r['slip'] > 0.7, r['slip']
            assert r['a_ratio'] < 0.4, r['a_ratio']
        else:
            # small and NEGATIVE: the two low-passes have different lags, so a
            # hard stop reads as slightly negative slip with nothing sliding
            assert -0.25 < r['slip'] < 0.1, (u, r['slip'])
            # the lag-free discriminator: both fits see the same ramp
            assert abs(r['a_ratio'] - 1.0) < 0.2, (u, r['a_ratio'])

    # the verdict the run exists to produce
    msgs = [m for _lvl, m in node.get_logger().records]
    assert any('BEATS' in m for m in msgs), msgs[-6:]


def test_locking_plant_is_not_mistaken_for_a_brake(tmp_path):
    """Hypothesis-false: no gain over the locked brake, and slip says why."""
    node = make_node(tmp_path, u_brakes=[0.0, -0.10, -0.25])
    plant = Floor(neg='lock', lock_a=4.55, brake_a=4.55)
    res = run(node, plant)

    assert not node.get_logger().errors()
    got = by_command(res)
    for u in (-0.10, -0.25):
        r = got[u][0]
        assert abs(r['a_ips'] - 4.55) < 0.5, (u, r['a_ips'])
        assert r['slip'] > 0.7, (u, r['slip'])
        assert r['a_ratio'] < 0.4, (u, r['a_ratio'])
        # the encoder is measuring its own filter decaying towards a wheel
        # that has stopped turning, not the car: it reads a completely
        # different deceleration from the truth.  This is why the fit is on
        # IPS and the encoder is only ever used for the slip.
        assert r['a_enc'] < r['a_ips'] - 1.0, (u, r['a_enc'], r['a_ips'])
    msgs = [m for _lvl, m in node.get_logger().records]
    assert any('does not beat' in m for m in msgs), msgs[-6:]
    # and the car is never allowed to end up driving backwards
    assert plant.v_min > -0.5, plant.v_min


def test_the_car_stays_local_and_the_sequence_runs(tmp_path):
    node = make_node(tmp_path, u_brakes=[0.0, -0.15], repeats=2)
    plant = Floor(neg='prop')
    res = run(node, plant)

    assert node.done and node.abort is None
    assert len(res) == 4, [r['u_brake'] for r in res]
    assert plant.dist() < node.fence_m

    rows = [l.split(',') for l in
            open(node.path).read().strip().splitlines()[1:]]
    phases = {r[2] for r in rows}
    assert {'warmup', 'accel', 'brake', 'home', 'straighten'} <= phases, phases
    # the command under test is published verbatim during its own window, and
    # the locked-brake reference is the exact float 0.0
    for want in ('0.0000', '-0.1500'):
        assert any(r[2] == 'brake' and r[4] == want for r in rows), want
    assert all(abs(float(r[12])) <= node.fence_m for r in rows)


def test_geofence_aborts_and_leaves_the_car_braked(tmp_path):
    node = make_node(tmp_path, fence_m=6.0, u_brakes=[-0.10])
    plant = Floor(neg='prop')
    run(node, plant, seconds=60.0)

    assert node.done and node.abort and 'geofence' in node.abort
    thr = node.stub_published('/throttle_command')[-1]
    assert thr.data == 0.0 and isinstance(thr.data, float)   # the real brake
    assert abs(node.stub_published('/steering_command')[-1].data) < 1e-12
    assert '# STOPPED' in open(node.path).read()


def test_a_spin_aborts_the_run(tmp_path):
    """|yaw rate| over `yaw_abort` while going straight stops everything."""
    def gyro_of(t, node, plant):
        spin = node.phase in ('accel', 'brake') and t > 3.0
        return 2.0 if spin else plant.wz

    node = make_node(tmp_path, u_brakes=[0.0, -0.10])
    plant = Floor(neg='prop')
    run(node, plant, seconds=60.0, gyro_of=gyro_of)

    assert node.done and node.abort and 'yaw rate' in node.abort
    assert node.stub_published('/throttle_command')[-1].data == 0.0


def test_the_u_turn_does_not_trip_the_straight_line_guard(tmp_path):
    """The drive home yaws at ~2.3 rad/s by design; that is not a spin."""
    node = make_node(tmp_path, u_brakes=[0.0, -0.10])
    plant = Floor(neg='prop')
    run(node, plant)
    assert node.abort is None, node.abort
    assert len(node.results) == 2


def test_an_ips_dropout_stops_the_run(tmp_path):
    node = make_node(tmp_path, u_brakes=[0.0, -0.10], ips_timeout_s=0.5)
    plant = Floor(neg='prop')

    # deliver nothing at all after 4 s: the pose stream dies mid-run
    from geometry_msgs.msg import Point
    real = node.on_ips
    state = {'t': 0.0}

    def gated(msg):
        if state['t'] < 4.0:
            real(msg)
    node.on_ips = gated
    for s in node._stub_subs:
        if s.topic.endswith('/ips'):
            s.callback = gated

    def gyro_of(t, _n, p):
        state['t'] = t
        return p.wz

    run(node, plant, seconds=30.0, gyro_of=gyro_of)
    assert node.done and node.abort and 'IPS' in node.abort
    assert isinstance(Point(), Point)


@pytest.mark.parametrize('dt', [0.058, 0.104])
def test_the_whole_default_sweep_is_accurate_and_stays_local(tmp_path, dt):
    """The instrument, end to end, on the command list it ships with.

    The plant retards by `20 * |u|` up to the tyre peak, so every cell has a
    known answer.  This is the test that caught two real defects: a command
    worth ~1 m/s2 takes 25 m to bring the car down from 7 m/s -- the whole
    geofence -- and the fit used to start when the command was PUBLISHED,
    which includes the tick the simulator had not consumed it in yet and
    under-reported 7.1 m/s2 as 6.3.
    """
    node = make_node(tmp_path / f's{int(dt * 1000)}',
                     u_brakes=SHIPPING_SWEEP)
    plant = Floor(neg='prop', prop_gain=20.0, brake_a=4.55)
    res = run(node, plant, dt=dt, seconds=400.0)

    assert node.abort is None, node.abort
    assert not node.get_logger().errors()
    assert len(res) == 7, [r['u_brake'] for r in res]
    got = by_command(res)
    for u, rows in got.items():
        r = rows[0]
        want = (4.55 if u == 0.0
                else min(brake_probe.TYRE_LONG_PEAK, 20.0 * abs(u)))
        assert abs(r['a_ips'] - want) < 0.25, (
            f'u {u}: reported {r["a_ips"]:.2f} against {want:.2f}')
    # the weakest command cannot outrun the floor, and says so
    assert got[-0.05][0]['clipped']
    assert plant.dist() < node.fence_m
    # and nothing in the sweep ends up driving backwards
    assert plant.v_min > -0.5, plant.v_min


def test_the_default_sweep_is_the_list_under_test(tmp_path):
    """`SHIPPING_SWEEP` above is what the node really ships with, and the
    locked-brake reference is in it -- without that column nothing else in the
    table means anything."""
    rclpy_stub.set_overrides({'out_dir': str(tmp_path)})
    try:
        node = brake_probe.BrakeProbe()
    finally:
        rclpy_stub.set_overrides({})
    assert list(node.u_brakes) == SHIPPING_SWEEP
    assert node.u_brakes[0] == 0.0, 'the locked-brake reference must be first'
    node.fh.close()
