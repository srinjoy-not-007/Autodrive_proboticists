"""Every constant that counts SAMPLES must mean the same TIME on every host.

The simulator's sensor period is a property of the machine, not of this stack:
0.104 s on the dev host (the period every default below was tuned at), 0.088 s
inside the submission image, ~0.097 s on the organisers'.  Each formula here is
required to be **the identity at 0.104 s** -- that is what makes the fix safe to
ship without re-validating the tune -- and to hold its quantity fixed in seconds
or metres per second elsewhere.

None of these need ROS.  Where the code under test lives in a node method, the
method is lifted out of the shipped source with ``ast`` (the pattern
``test_mpc3_startup`` established) so the test exercises the real code rather
than a copy of it.
"""

import ast
import math
import os
import sys
import types

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from pit_crew import pf_core, vehicle                            # noqa: E402
from pit_crew.mpc3_core import (ILC, RateControl, _max_circular_gap,  # noqa: E402
                                RATE_NOMINAL)
from pit_crew.throttle2 import CMD_PERIOD, ServoThrottle         # noqa: E402
from pit_crew.vehicle import SpeedEstimator                      # noqa: E402

NOMINAL = 0.104          # the period every default here was validated at
IMAGE = 0.088            # measured inside the submission image
ORGANISERS = 0.097
PERIODS = (IMAGE, ORGANISERS, NOMINAL)

LOC_PY = os.path.join(ROOT, 'src', 'pit_crew', 'localization.py')
FTG_PY = os.path.join(ROOT, 'src', 'pit_crew', 'follow_the_gap.py')


def lift(path, cls_name, *names):
    """Compile the named methods of `cls_name` out of `path`."""
    tree = ast.parse(open(path).read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == cls_name)
    want = [n for n in cls.body
            if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(want) == len(names), [n.name for n in want]
    ns = {'math': math, 'np': np, 'vehicle': vehicle}
    exec(compile(ast.Module(body=want, type_ignores=[]), path, 'exec'), ns)
    return {n: ns[n] for n in names}


def declared_defaults(path, cls_name):
    """The `declare_parameters` defaults of a node, as a dict.

    Only literals and simple `vehicle.X` / arithmetic expressions, which is all
    the parameters these tests care about use.
    """
    tree = ast.parse(open(path).read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == cls_name)
    out = {}
    for node in ast.walk(cls):
        if not (isinstance(node, ast.Call)
                and getattr(node.func, 'attr', '') == 'declare_parameters'):
            continue
        for item in node.args[1].elts:
            key = ast.literal_eval(item.elts[0])
            try:
                out[key] = eval(compile(ast.Expression(item.elts[1]),
                                        path, 'eval'), {'vehicle': vehicle})
            except Exception:                              # noqa: BLE001
                out[key] = None
    return out


# --------------------------------------------------------------------------
# vehicle.reject_max_for_dt  -- the encoder's reject-persistence window
# --------------------------------------------------------------------------
class TestRejectWindow:
    def test_identity_at_the_validated_period(self):
        assert vehicle.reject_max_for_dt(NOMINAL) == vehicle.MAX_CONSEC_REJECT

    def test_the_hold_time_is_fixed_in_seconds(self):
        """6 samples is 0.624 s at 0.104 and 0.528 s at 0.088 -- 0.1 s more of
        steering and braking by a stale speed on the dev host than in the image.
        Derived from the time, every host is within half a period of it."""
        for p in PERIODS:
            held = vehicle.reject_max_for_dt(p) * p
            assert abs(held - vehicle.REJECT_HOLD_S) <= 0.5 * p

    def test_the_default_constant_and_the_hold_time_agree(self):
        assert abs(vehicle.MAX_CONSEC_REJECT * NOMINAL
                   - vehicle.REJECT_HOLD_S) < 1e-9

    def test_floored_so_an_isolated_glitch_is_always_rejected(self):
        assert vehicle.reject_max_for_dt(0.40) >= 2

    def test_degenerate_periods_fall_back_to_the_validated_count(self):
        for bad in (0.0, -0.1, float('nan')):
            assert vehicle.reject_max_for_dt(bad) == vehicle.MAX_CONSEC_REJECT

    def test_the_estimator_honours_the_re_pointed_count(self):
        """A sustained out-of-bounds reading is believed (clamped) after
        `reject_max` rejections, not after the hard-coded 6."""
        est = SpeedEstimator(alpha=1.0)
        est.reject_max = 3
        t, pos = 0.0, 0.0
        est.update('L', t, pos)
        for i in range(1, 4):                 # -20 m/s: outside REVERSE_MAX
            pos += -20.0 * 0.09 / vehicle.WHEEL_RADIUS
            est.update('L', t + i * 0.09, pos)
        assert est.speed == pytest.approx(-vehicle.REVERSE_MAX)


# --------------------------------------------------------------------------
# localization: the motion model's cap is a SPEED
# --------------------------------------------------------------------------
def loc_node(**over):
    P = declared_defaults(LOC_PY, 'ParticleFilter')
    node = types.SimpleNamespace(
        max_step_mps=P['max_step_mps'], max_step_m=P['max_step_m'],
        rate_nominal=P['rate_nominal'], enc_tau=P['enc_tau'],
        lost_patience=P['lost_patience'], lost_seconds=P['lost_seconds'],
        noise_xy=P['motion_noise_xy'], noise_th=P['motion_noise_theta'],
        midpoint_heading=False, imu_yaw=0.0, imu_yaw_prev=0.0,
        speed_cmd=0.0, speed_est=SpeedEstimator(alpha=0.35),
        n_p=64, _patience=P['lost_patience'],
        get_logger=lambda: types.SimpleNamespace(warn=lambda *a: None),
    )
    node.px = np.zeros(node.n_p)
    node.py = np.zeros(node.n_p)
    node.pth = np.zeros(node.n_p)
    for k, v in over.items():
        setattr(node, k, v)
    return node


class TestLocalizationSpeedCap:
    def test_the_default_cap_is_the_old_0_6_m_at_the_validated_period(self):
        f = lift(LOC_PY, 'ParticleFilter', 'speed_cap')
        n = loc_node()
        assert f['speed_cap'](n) * NOMINAL == pytest.approx(0.6)
        assert f['speed_cap'](n) == pytest.approx(5.769, abs=1e-3)

    def test_the_cap_no_longer_depends_on_the_host(self):
        """0.6 m per update is 5.77 m/s at 0.104 s and 6.82 at 0.088 -- either
        side of the 6.58 m/s the car actually reaches, so the two hosts used
        different motion models exactly where the car is fastest."""
        f = lift(LOC_PY, 'ParticleFilter', 'speed_cap', 'predict')
        n = loc_node(noise_xy=0.0, noise_th=0.0)
        n._v_cap = f['speed_cap'](n)
        steps = []
        for p in PERIODS:
            n.px[:] = 0.0
            n.speed_est.speed = 8.0           # above the cap at every period
            f['predict'](n, p)
            steps.append(n.px[0] / p)         # the capped SPEED
        assert steps == pytest.approx([n._v_cap] * 3)

    def test_identity_against_the_old_distance_clip_at_the_validated_period(self):
        f = lift(LOC_PY, 'ParticleFilter', 'speed_cap', 'predict')
        n = loc_node(noise_xy=0.0, noise_th=0.0)
        n._v_cap = f['speed_cap'](n)
        for v in (0.0, 2.0, 5.0, 5.769, 6.58, 8.0, -4.0):
            n.px[:] = 0.0
            n.speed_est.speed = v
            n.speed_cmd = 0.0
            f['predict'](n, NOMINAL)
            old = float(np.clip(v * NOMINAL, -0.6, 0.6)) if v > 1e-3 else 0.0
            assert n.px[0] == pytest.approx(old, abs=1e-12)

    def test_an_explicit_speed_cap_overrides_the_derived_one(self):
        f = lift(LOC_PY, 'ParticleFilter', 'speed_cap')
        assert f['speed_cap'](loc_node(max_step_mps=4.0)) == 4.0

    def test_the_offline_core_caps_the_same_speed(self):
        """`pf_core` is the offline mirror; if the two caps disagree the sweeps
        are measuring a filter that does not race."""
        cap = (pf_core.DEFAULTS['max_step']
               / pf_core.DEFAULTS['rate_nominal'])
        assert cap == pytest.approx(5.769, abs=1e-3)
        pf = pf_core.ParticleFilterCore.__new__(
            pf_core.ParticleFilterCore)
        pf.p = dict(pf_core.DEFAULTS, n_particles=8, motion_noise_xy=0.0,
                    motion_noise_theta=0.0)
        for p in PERIODS:
            pf.px = np.zeros(8)
            pf.py = np.zeros(8)
            pf.pth = np.zeros(8)
            pf.predict(8.0, p, 0.0)
            assert pf.px[0] / p == pytest.approx(cap)
        # ...and the identity at the validated period.
        pf.px = np.zeros(8)
        pf.py = np.zeros(8)
        pf.pth = np.zeros(8)
        pf.predict(4.0, NOMINAL, 0.0)
        assert pf.px[0] == pytest.approx(4.0 * NOMINAL)


class TestLocalizationDiffusion:
    """Both random walks are per-UPDATE; scaling only one makes the RATIO of
    translational to rotational diffusion rate-dependent."""

    def _sigmas(self, dt, monkeypatch):
        f = lift(LOC_PY, 'ParticleFilter', 'speed_cap', 'predict')
        n = loc_node()
        n._v_cap = f['speed_cap'](n)
        seen = []
        real = np.random.normal
        monkeypatch.setattr(np.random, 'normal',
                            lambda mu, sd, size: (seen.append(sd),
                                                  real(mu, 0.0, size))[1])
        f['predict'](n, dt)
        return seen                                # [xy, xy, theta]

    def test_identity_at_the_validated_period(self, monkeypatch):
        xy0, xy1, th = self._sigmas(NOMINAL, monkeypatch)
        assert xy0 == pytest.approx(0.18)
        assert xy1 == pytest.approx(0.18)
        assert th == pytest.approx(0.02)

    def test_both_walks_diffuse_at_a_fixed_rate_per_second(self, monkeypatch):
        for p in PERIODS:
            xy, _, th = self._sigmas(p, monkeypatch)
            assert xy / math.sqrt(p) == pytest.approx(0.18 / math.sqrt(NOMINAL))
            assert th / math.sqrt(p) == pytest.approx(0.02 / math.sqrt(NOMINAL))

    def test_the_ratio_of_the_two_walks_is_the_same_on_every_host(self,
                                                                 monkeypatch):
        ratios = []
        for p in PERIODS:
            xy, _, th = self._sigmas(p, monkeypatch)
            ratios.append(xy / th)
        assert ratios == pytest.approx([ratios[-1]] * 3)


class TestLocalizationRepoint:
    def test_identity_at_the_validated_period(self):
        f = lift(LOC_PY, 'ParticleFilter', 'repoint')
        n = loc_node()
        f['repoint'](n, NOMINAL)
        assert n._patience == 5
        assert n.speed_est.alpha == pytest.approx(0.35, abs=5e-4)
        assert n.speed_est.reject_max == vehicle.MAX_CONSEC_REJECT

    def test_the_blind_window_is_fixed_in_seconds(self):
        """5 scans is 0.52 s of dead reckoning at 0.104 s and 0.44 s at 0.088 --
        0.5 m of blind driving at 6 m/s, on the recovery path."""
        f = lift(LOC_PY, 'ParticleFilter', 'repoint')
        for p in PERIODS:
            n = loc_node()
            f['repoint'](n, p)
            assert abs(n._patience * p - 5 * NOMINAL) <= 0.5 * p
            assert n.speed_est.reject_max * p == pytest.approx(
                vehicle.REJECT_HOLD_S, abs=0.5 * p)

    def test_lost_seconds_overrides_the_scan_count(self):
        f = lift(LOC_PY, 'ParticleFilter', 'repoint')
        n = loc_node(lost_seconds=1.04)
        f['repoint'](n, NOMINAL)
        assert n._patience == 10

    def test_patience_never_drops_below_one_scan(self):
        f = lift(LOC_PY, 'ParticleFilter', 'repoint')
        n = loc_node()
        f['repoint'](n, 0.5)
        assert n._patience >= 1

    def test_a_zero_period_changes_nothing(self):
        f = lift(LOC_PY, 'ParticleFilter', 'repoint')
        n = loc_node()
        before = (n._patience, n.speed_est.alpha, n.speed_est.reject_max)
        f['repoint'](n, 0.0)
        assert (n._patience, n.speed_est.alpha, n.speed_est.reject_max) == before


# --------------------------------------------------------------------------
# throttle2.CMD_PERIOD  -- `hold` IS the sensor period
# --------------------------------------------------------------------------
class TestCommandHold:
    def test_the_default_hold_is_the_nominal_sensor_period(self):
        assert CMD_PERIOD == RATE_NOMINAL == vehicle.RATE_NOMINAL
        assert ServoThrottle().hold == RATE_NOMINAL

    def test_the_shipping_configuration_does_not_read_hold_at_all(self):
        """mpc3 passes `lead_max=0.0`, which kills the only term `hold` feeds,
        so this change cannot move the shipping tracker at any period."""
        out = []
        for hold in (0.088, 0.09, 0.104, 0.5):
            th = ServoThrottle(lead_max=0.0, hold=hold, predict=False)
            out.append(th.command(5.0, 4.0, a_target=6.5, now=0.0))
        assert out == pytest.approx([out[0]] * 4)

    def test_rate_control_re_points_the_hold_at_the_measured_period(self):
        thr = ServoThrottle()
        est = SpeedEstimator(alpha=0.35)
        rc = _rate_control(thr=thr, speed_est=est, enabled=True)
        _feed(rc, IMAGE)
        assert thr.hold == pytest.approx(IMAGE, abs=1e-6)

    def test_rate_control_leaves_it_alone_at_the_validated_period(self):
        thr = ServoThrottle()
        rc = _rate_control(thr=thr, enabled=True)
        _feed(rc, NOMINAL)
        assert thr.hold == pytest.approx(NOMINAL, abs=1e-6)


# --------------------------------------------------------------------------
# mpc3_core.RateControl  -- the encoder's reject window follows too
# --------------------------------------------------------------------------
def _rate_control(thr=None, speed_est=None, enabled=True):
    mpc = types.SimpleNamespace(set_rate=lambda **kw: None)
    act = types.SimpleNamespace(dead_time=0.135)
    return RateControl(mpc, act, enabled=enabled, speed_est=speed_est, thr=thr,
                       nominal=NOMINAL)


def _feed(rc, period, n=30):
    t = 1000.0
    for _ in range(n):
        rc.update(t)
        t += period
    return rc


class TestRateControlSpeedEstimator:
    def test_the_reject_window_follows_the_measured_period(self):
        est = SpeedEstimator(alpha=0.35)
        _feed(_rate_control(speed_est=est, enabled=True), IMAGE)
        assert est.reject_max == vehicle.reject_max_for_dt(IMAGE)
        assert est.reject_max * IMAGE == pytest.approx(vehicle.REJECT_HOLD_S,
                                                       abs=0.5 * IMAGE)

    def test_identity_at_the_validated_period(self):
        est = SpeedEstimator(alpha=0.35)
        _feed(_rate_control(speed_est=est, enabled=True), NOMINAL)
        assert est.reject_max == vehicle.MAX_CONSEC_REJECT
        assert est.alpha == pytest.approx(0.35, abs=5e-4)

    def test_disabled_rate_control_touches_nothing(self):
        """`rate_adaptive` is opt-in and stays that way."""
        est = SpeedEstimator(alpha=0.35)
        thr = ServoThrottle(hold=0.104)
        _feed(_rate_control(thr=thr, speed_est=est, enabled=False), IMAGE)
        assert est.reject_max == vehicle.MAX_CONSEC_REJECT
        assert est.alpha == 0.35
        assert thr.hold == 0.104


# --------------------------------------------------------------------------
# ILC coverage  -- density against the samples, wholeness against the arc
# --------------------------------------------------------------------------
def _ilc(**kw):
    return ILC(28.6, bin_m=0.25, v_enable=False, **kw)


def _drive(il, samples, e=0.05, frac=1.0):
    """`samples` poses spread evenly over `frac` of a 28.6 m lap."""
    for i in range(samples):
        il.observe(28.6 * frac * i / samples, e, 0.5)
    return il


class TestILCCoverage:
    def test_a_whole_lap_is_learned_from_at_every_period(self):
        """~77 poses a lap at 0.104 s against ~91 at 0.088, over 114 bins: the
        old `min_cover * nb` gate sat just under the slow host's ceiling and
        could never bind on the fast one."""
        for samples in (77, 85, 91):
            assert _drive(_ilc(), samples).finish_lap() is not None

    def test_a_sparser_lap_than_the_dev_host_ever_sees_still_counts(self):
        assert _drive(_ilc(), 60).finish_lap() is not None

    def test_a_fragment_is_still_not_a_lap(self):
        """The defence that matters: a 20%-covered fragment carrying a constant
        error drove all 114 bins to the clamp. It is dense in its own bins, so
        only the unseen ARC can reject it."""
        assert _drive(_ilc(), 23, e=0.30, frac=0.20).finish_lap() is None
        assert _drive(_ilc(), 60, e=0.30, frac=0.60).finish_lap() is None

    def test_an_empty_lap_is_not_a_lap(self):
        assert _ilc().finish_lap() is None

    def test_max_circular_gap(self):
        assert _max_circular_gap([True] * 5) == 0
        assert _max_circular_gap([False] * 5) == 5
        assert _max_circular_gap([True, False, False, True, False]) == 2
        # wraps: the run spans the end and the start
        assert _max_circular_gap([False, True, False, False, False]) == 4


# --------------------------------------------------------------------------
# follow_the_gap: the steering low pass is a lag in SECONDS
# --------------------------------------------------------------------------
class TestFollowTheGapSteerFilter:
    def test_the_default_tau_is_the_identity_at_the_validated_period(self):
        P = declared_defaults(FTG_PY, 'FollowTheGap')
        assert vehicle.alpha_for_lag(NOMINAL, P['steer_tau']) == pytest.approx(
            P['steer_alpha'])

    def test_the_lag_no_longer_depends_on_the_host(self):
        """alpha 0.5 per scan is a 0.104 s lag at 0.104 s and 0.088 s at
        0.088 -- the fallback is 15% less damped in the image than in any run
        it was judged on."""
        P = declared_defaults(FTG_PY, 'FollowTheGap')
        for p in PERIODS:
            a = vehicle.alpha_for_lag(p, P['steer_tau'])
            assert p * (1.0 - a) / a == pytest.approx(P['steer_tau'])

    def test_the_node_derives_alpha_from_the_scan_period(self):
        """Guards the wiring, not the formula: `on_scan` is too entangled with
        numpy scan geometry to lift, so assert the source uses the period."""
        src = open(FTG_PY).read()
        assert 'vehicle.alpha_for_lag(stamp - prev_scan, self.steer_tau)' in src
        assert 'prev_scan = self._valid_scan_at' in src

    def test_a_gap_in_the_scans_weights_the_newer_sample_more(self):
        P = declared_defaults(FTG_PY, 'FollowTheGap')
        a1 = vehicle.alpha_for_lag(NOMINAL, P['steer_tau'])
        a2 = vehicle.alpha_for_lag(2 * NOMINAL, P['steer_tau'])
        assert a2 > a1


# --------------------------------------------------------------------------
# the publish index: `apply_ahead_s` must survive a sub-`step_dt` period
# --------------------------------------------------------------------------
class TestApplyFloorSteps:
    """`MPC3.solve` publishes the plan angle at `nd + apply_ahead_s/step_dt`.

    That second term used to be written `max(step_dt, apply_ahead_s)/step_dt`,
    a floor of exactly one horizon step.  `rate_adaptive` scales
    `apply_ahead_s` to 0.87 of the measured period, so below a 0.069 s sensor
    period the floor discarded the adaptation entirely -- and made the index an
    exact integer, which publishes a raw plan knot instead of an interpolation
    and lets successive commands step by a whole plan step (3.31 rad/s at a
    0.058 s period, above the rack's 3.2 rad/s).  Measured in
    `tools/sim_tracker.py`: 24.9% of the commands through s 12-17 slew past the
    rack limit at frame 0.058 and 0.0% at every frame >= 0.065; the same
    signature is in the live 0.058 s log and absent from the 0.094 s one.
    """

    @staticmethod
    def _f(cfg, nd):
        """The shipped expression, read out of `mpc3_core.MPC3.solve`."""
        return nd + max(float(cfg['apply_floor_steps']),
                        float(cfg['apply_ahead_s']) / float(cfg['step_dt']))

    def test_the_default_floor_reproduces_the_old_expression(self):
        from pit_crew.mpc3_core import DEFAULTS
        dt = DEFAULTS['step_dt']
        assert DEFAULTS['apply_floor_steps'] == 1.0
        for apply_s in (0.020, 0.045, 0.060, 0.0904, 0.150):
            cfg = dict(DEFAULTS, apply_ahead_s=apply_s)
            assert self._f(cfg, 3) == 3 + max(dt, apply_s) / dt

    def test_the_shipping_tune_is_untouched(self):
        from pit_crew.mpc3_core import DEFAULTS
        assert self._f(DEFAULTS, 3) == pytest.approx(4.5)

    def test_the_old_floor_silently_ate_the_adaptation_below_0_069_s(self):
        from pit_crew.mpc3_core import DEFAULTS, RateAdapter
        ad = RateAdapter(apply_ahead_s=DEFAULTS['apply_ahead_s'])
        dt = DEFAULTS['step_dt']
        for p in PERIODS:                       # 0.088, 0.097, 0.104
            asked = ad.values(p)['apply_ahead_s']
            assert asked > dt, 'the floor must not bind at a validated period'
        asked = ad.values(0.058)['apply_ahead_s']
        assert asked == pytest.approx(0.0502, abs=1e-3)
        assert asked < dt                       # ...but it does at 0.058 s
        old = dict(DEFAULTS, apply_ahead_s=asked, apply_floor_steps=1.0)
        new = dict(old, apply_floor_steps=0.0)
        assert self._f(old, 3) == 4.0           # an exact plan knot
        assert self._f(new, 3) == pytest.approx(3.837, abs=1e-3)

    def test_the_rate_clamp_keeps_the_index_off_nd_with_no_floor(self):
        """`apply_floor_steps=0` is only safe because `RateControl` refuses to
        believe a period below 0.5 x nominal, so the adapted lead can never
        reach zero and publish the already-committed angle."""
        from pit_crew.mpc3_core import DEFAULTS, RateAdapter
        ad = RateAdapter(apply_ahead_s=DEFAULTS['apply_ahead_s'])
        floor_period = 0.5 * RATE_NOMINAL
        asked = ad.values(floor_period)['apply_ahead_s']
        assert asked / DEFAULTS['step_dt'] > 0.7
