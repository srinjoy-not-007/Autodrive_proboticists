"""Localization audit (Sep 19): four defensive fixes and one opt-in knob.

Everything here is either default-off or provably inert on a monotone,
finite input stream.  The first class is the proof: the real node is driven
over a deterministic replay -- scans ray-cast against the very map it
localizes in, so the sensor model is self-consistent -- and the published
pose sequence is compared, sample for sample, against the SAME replay driven
by the version of `localization.py` at the commit before this change.

What the audit measured on the finals logs, which is why these fixes exist:

* the published pose sits a constant **+0.08 m (mean) / +0.10 m (median)
  ahead** of the IPS truth wherever `s` is observable, on all three clean
  finals logs and at both sensor periods -- the size of
  `vehicle.IPS_FROM_REAR_AXLE`, and the truth point's sideslip through both
  hairpins is ~0, which is what a REAR AXLE reference gives;
* from the grid the wheels spin: the encoder reads up to 2.2x truth for ~2 s
  (8.08 m/s against 6.00), and on this track's featureless 18 m straight the
  scan cannot contradict it, so the estimate runs **+5.0 to +5.4 m ahead**
  before snapping back at s ~ 14-15, 3.1-3.7 s and 13-14.6 m into the race.
  `dr_accel_max` is the (default-off) lever against that;
* two telemetry gaps of 0.79 s and 1.16 s occur inside a clean 100-lap Xvfb
  run (`race_20260918_194320.csv`); the filter rode both out, peak error
  0.38 m, recovered in two scans.
"""

import importlib.util
import math
import os
import subprocess
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

from rclpy_stub import Imu, JointState, LaserScan, Time        # noqa: E402

MAP_PREFIX = os.path.join(DATA, 'maps', 'porto_mesh')
CENTERLINE = os.path.join(DATA, 'racelines', 'porto_mesh_centerline.csv')

# The commit this audit started from.  The byte-identical proof compares
# against `localization.py` as it stood there.
BASE_REF = 'a7dad98'

N_BEAM = 180                      # a coarse scan: the filter subsamples anyway
A_MIN = -math.radians(135.0)
A_INC = math.radians(270.0) / (N_BEAM - 1)
R_MIN, R_MAX = 0.06, 10.0
WHEEL_R = 0.059


def node(cls=None, **over):
    if cls is None:
        from pit_crew.localization import ParticleFilter as cls
    over.setdefault('map_prefix', MAP_PREFIX)
    over.setdefault('centerline_csv', CENTERLINE)
    over.setdefault('n_particles', 200)
    rclpy_stub.reset_clock()
    rclpy_stub.set_overrides(over)
    n = cls()
    rclpy_stub.set_overrides({})
    return n


class Caster:
    """Range image against the occupancy grid the filter itself loads."""

    def __init__(self, prefix):
        import yaml
        from PIL import Image
        with open(prefix + '.yaml') as f:
            meta = yaml.safe_load(f)
        img = np.flipud(np.array(Image.open(prefix + '.pgm')))
        self.res = float(meta['resolution'])
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        self.occ = img <= 50
        self.H, self.W = img.shape
        self.t = np.arange(int(R_MAX / (self.res * 0.5)) + 1) * self.res * 0.5
        self.ang = A_MIN + np.arange(N_BEAM) * A_INC

    def cast(self, x, y, yaw):
        a = yaw + self.ang
        px = x + np.cos(a)[:, None] * self.t[None, :]
        py = y + np.sin(a)[:, None] * self.t[None, :]
        gx = np.floor((px - self.ox) / self.res).astype(np.int64)
        gy = np.floor((py - self.oy) / self.res).astype(np.int64)
        bad = (gx < 0) | (gx >= self.W) | (gy < 0) | (gy >= self.H)
        np.clip(gx, 0, self.W - 1, out=gx)
        np.clip(gy, 0, self.H - 1, out=gy)
        hit = self.occ[gy, gx] | bad
        hit[:, self.t < R_MIN] = False
        first = np.argmax(hit, axis=1)
        r = np.where(hit[np.arange(N_BEAM), first], self.t[first], np.inf)
        return r.tolist()


def trajectory(n=90, dt=0.097):
    """A lap-shaped run along the porto centerline, at its own spacing."""
    c = np.loadtxt(CENTERLINE, delimiter=',')[:, :2]
    step = max(1, len(c) // (n + 2))
    pts = c[::step][:n + 1]
    yaw = np.unwrap(np.arctan2(np.gradient(pts[:, 1]),
                               np.gradient(pts[:, 0])))
    v = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])) / dt
    v = np.concatenate([[v[0]], v])
    t = 1_000_000.0 + np.arange(len(pts)) * dt
    return t, pts[:, 0], pts[:, 1], yaw, v


def replay(cls=None, seed=7, **over):
    """Drive the node over the deterministic stream; return published poses."""
    caster = Caster(MAP_PREFIX)
    t, xs, ys, yaws, vs = trajectory()
    n = node(cls, **over)
    np.random.seed(seed)
    enc = {'L': 0.0, 'R': 0.0}
    out = []
    for k in range(len(t)):
        rclpy_stub.reset_clock()
        rclpy_stub.advance(float(t[k]) - rclpy_stub.now_s() + 0.001)
        stamp = Time(sec=int(t[k]), nanosec=int((t[k] % 1) * 1e9))
        if k:
            for w in ('L', 'R'):
                enc[w] += vs[k] * (t[k] - t[k - 1]) / WHEEL_R
        for w in ('L', 'R'):
            m = JointState()
            m.header.stamp = stamp
            m.position = [enc[w]]
            n.on_enc(w, m)
        im = Imu()
        im.orientation.z = math.sin(yaws[k] / 2.0)
        im.orientation.w = math.cos(yaws[k] / 2.0)
        n.on_imu(im)
        sc = LaserScan()
        sc.header.stamp = stamp
        sc.angle_min = A_MIN
        sc.angle_increment = A_INC
        sc.range_min = R_MIN
        sc.range_max = R_MAX
        sc.ranges = caster.cast(xs[k] + 0.1933 * math.cos(yaws[k]),
                                ys[k] + 0.1933 * math.sin(yaws[k]), yaws[k])
        n.on_scan(sc)
        out.append([(p.pose.position.x, p.pose.position.y,
                     p.pose.orientation.z) for p in n.pub_pose.msgs[-1:]])
    return out, n, (t, xs, ys, yaws, vs)


def _base_module():
    """`localization.py` as of BASE_REF, imported under its own name."""
    try:
        src = subprocess.run(
            ['git', '-C', ROOT, 'show', f'{BASE_REF}:src/pit_crew/localization.py'],
            capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):          # pragma: no cover
        pytest.skip('git unavailable')
    if src.returncode != 0:                                # pragma: no cover
        pytest.skip(f'{BASE_REF} not in this checkout')
    path = os.path.join(os.path.dirname(__file__), '_loc_base_tmp.py')
    with open(path, 'w') as f:
        f.write(src.stdout)
    try:
        spec = importlib.util.spec_from_file_location('_loc_base_tmp', path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        os.remove(path)


class TestShippedPathIsUnchanged:
    """The shipping configuration must produce the same poses as before."""

    def test_replay_matches_the_pre_audit_node_sample_for_sample(self):
        base = _base_module()
        ship = dict(max_step_mps=9.0, publish_on_reloc=False)
        old, _, _ = replay(base.ParticleFilter, **ship)
        new, _, _ = replay(None, **ship)
        assert len(old) == len(new) and len(old) > 50
        for k, (a, b) in enumerate(zip(old, new)):
            assert a == b, f'pose stream diverged at scan {k}'

    def test_the_replay_actually_tracks(self):
        """A regression that would pass trivially if the filter were broken."""
        out, n, (t, xs, ys, yaws, vs) = replay(max_step_mps=9.0,
                                               publish_on_reloc=False)
        err = [math.hypot(p[0][0] - xs[k], p[0][1] - ys[k])
               for k, p in enumerate(out) if p]
        assert len(err) > 50
        assert np.median(err[20:]) < 0.35, np.median(err[20:])

    def test_new_parameters_default_to_inert_values(self):
        n = node()
        assert n.dr_accel_max == 0.0
        assert n.clock_reset_s == 1.0


class TestClockGoingBackwards:
    """A reset clock used to reject every later scan, forever."""

    def _scan(self, n, t):
        sc = LaserScan()
        sc.header.stamp = Time(sec=int(t), nanosec=int((t % 1) * 1e9))
        sc.angle_min = A_MIN
        sc.angle_increment = A_INC
        sc.range_min = R_MIN
        sc.range_max = R_MAX
        sc.ranges = [2.0] * N_BEAM
        rclpy_stub.reset_clock()
        rclpy_stub.advance(t - rclpy_stub.now_s() + 0.001)
        im = Imu()
        n.on_imu(im)
        n.on_scan(sc)

    def test_a_small_step_back_is_still_dropped(self):
        n = node()
        self._scan(n, 1_000_000.0)
        before = len(n.pub_pose.msgs)
        self._scan(n, 1_000_000.0 - 0.05)
        assert len(n.pub_pose.msgs) == before
        assert n.last_t == 1_000_000.0

    def test_a_clock_reset_re_seeds_instead_of_wedging(self):
        n = node()
        self._scan(n, 1_000_000.0)
        before = len(n.pub_pose.msgs)
        self._scan(n, 500.0)               # the clock restarted
        self._scan(n, 500.1)
        assert len(n.pub_pose.msgs) > before
        assert n.last_t == pytest.approx(500.1)

    def test_the_threshold_is_a_parameter(self):
        n = node(clock_reset_s=100.0)
        self._scan(n, 1_000_000.0)
        self._scan(n, 999_999.0)           # 1 s back, under the threshold
        assert n.last_t == 1_000_000.0


class TestEncoderStampFromTheFuture:
    """One sample ahead of the stream used to freeze the speed forever."""

    def _enc(self, n, wheel, t, pos):
        m = JointState()
        m.header.stamp = Time(sec=int(t), nanosec=int((t % 1) * 1e9))
        m.position = [pos]
        n.on_enc(wheel, m)

    def test_speed_recovers_after_a_stamp_from_the_future(self):
        n = node()
        t0, p = 1_000_000.0, 0.0
        for k in range(6):                       # steady 2 m/s
            p += 2.0 * 0.1 / WHEEL_R
            self._enc(n, 'L', t0 + 0.1 * k, p)
            self._enc(n, 'R', t0 + 0.1 * k, p)
        assert n.speed_est.speed > 0.5
        self._enc(n, 'L', t0 + 60.0, p)          # one glitched stamp
        self._enc(n, 'R', t0 + 60.0, p)
        moved = False
        for k in range(6, 20):
            p += 2.0 * 0.1 / WHEEL_R
            self._enc(n, 'L', t0 + 0.1 * k, p)
            self._enc(n, 'R', t0 + 0.1 * k, p)
            moved = moved or n.speed_est.speed > 0.5
        assert moved, 'speed estimate never recovered'
        assert n.speed_est.speed == pytest.approx(2.0, abs=0.6)


class TestLaunchSlipCap:
    """`dr_accel_max` holds the MOTION MODEL to a physical acceleration."""

    def _one_step(self, v_enc, dt=0.1, **over):
        over.setdefault('max_step_mps', 9.0)
        n = node(**over)
        n.init_particles_near(0.0, 0.0, 0.0)
        n.px[:] = 0.0
        n.py[:] = 0.0
        n.pth[:] = 0.0
        n.imu_yaw = 0.0
        n.noise_xy = 0.0
        n.noise_th = 0.0
        n.speed_est.speed = v_enc
        n.predict(dt)
        return float(np.mean(n.px))

    def test_off_by_default_the_whole_encoder_speed_is_used(self):
        assert self._one_step(8.0) == pytest.approx(0.8, abs=1e-9)

    def test_a_spinning_wheel_cannot_outrun_the_cap(self):
        assert self._one_step(8.0, dr_accel_max=4.0) == pytest.approx(0.04,
                                                                      abs=1e-9)

    def test_braking_is_never_slew_limited(self):
        n = node(dr_accel_max=4.0, max_step_mps=9.0)
        n.init_particles_near(0.0, 0.0, 0.0)
        n.px[:] = 0.0
        n.py[:] = 0.0
        n.pth[:] = 0.0
        n.imu_yaw = 0.0
        n.noise_xy = 0.0
        n.noise_th = 0.0
        n._dr_v = 6.0
        n.speed_est.speed = 1.0
        n.predict(0.1)
        assert float(np.mean(n.px)) == pytest.approx(0.1, abs=1e-9)


class TestDefensiveOutputs:

    def test_a_particle_below_the_map_origin_is_outside_not_index_zero(self):
        n = node()
        n.init_particles_near(0.0, 0.0, 0.0)
        n.px[:] = n.ox - 0.5 * n.res         # half a cell below the origin
        n.py[:] = n.oy - 0.5 * n.res
        n.imu_yaw = 0.0
        sc = LaserScan()
        sc.angle_min = A_MIN
        sc.angle_increment = A_INC
        sc.range_min = R_MIN
        sc.range_max = R_MAX
        sc.ranges = [2.0] * N_BEAM
        n._patience = 10_000                 # do not relocalize during the test
        n.update(sc)
        assert n.score == 0.0 and n.lost_count == 1

    def test_a_non_finite_pose_is_never_published(self):
        n = node()
        n.init_particles_near(0.0, 0.0, 0.0)
        n.imu_yaw = 0.0
        n.px[0] = float('nan')
        before = len(n.pub_pose.msgs)
        n.publish(Time())
        assert len(n.pub_pose.msgs) == before

    def test_a_zero_likelihood_width_does_not_kill_the_node(self):
        n = node(sigma_hit=0.0, beam_temp=0.0)
        assert n.sigma_hit > 0.0 and n.beam_temp > 0.0
