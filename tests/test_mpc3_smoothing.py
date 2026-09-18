"""The opt-in command-smoothing knobs of mpc3 (`w_dcmd`, `cmd_tau`,
`cmd_notch`, `e_dead`).

They exist for one measured failure: at a fast sensor period the published
steering alternates by exactly ``steer_rate * step_dt`` from frame to frame --
a dead-time relay limit cycle, reproduced in ``tools/sim_tracker.py`` with the
pose noise switched off.  The contract these tests defend is that every one of
them is EXACTLY the identity at its 0.0 default, because the racing tune is
0.0, and that each does the one thing it claims when switched on.
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew import vehicle                                    # noqa: E402
from pit_crew.mpc3_core import (DEFAULTS, MPC3, RATE_NOMINAL,     # noqa: E402
                                Actuator, RateControl, RefPath)

from test_mpc3_core import straight_path as straight_csv        # noqa: E402

I0 = 300          # mid-line: far from the wrap seam of a synthetic straight


def straight_path(tmp_path):
    return RefPath(straight_csv(tmp_path))


def drive(path, frames=40, offset=0.05, **cfg):
    """Solve repeatedly from a fixed small lateral error; return the commands.

    The state is held constant on purpose: any change in the published command
    is then the solver's own doing, which is exactly the flutter under test.
    """
    m = MPC3(path, **cfg)
    out = []
    for _ in range(frames):
        cmd, _info = m.solve(offset, 0.0, 0.0, I0, 4.0)
        out.append(cmd)
    return np.array(out)


def test_defaults_are_zero():
    for k in ('w_dcmd', 'cmd_tau', 'cmd_notch', 'e_dead'):
        assert DEFAULTS[k] == 0.0


def test_knobs_off_are_the_identity(tmp_path):
    path = straight_path(tmp_path)
    a = drive(path)
    b = drive(path, w_dcmd=0.0, cmd_tau=0.0, cmd_notch=0.0, e_dead=0.0)
    assert np.array_equal(a, b)


def test_w_dcmd_pulls_the_published_angle_towards_the_last_one(tmp_path):
    """The term is a real cost, not an output filter: with the previous
    command far from where the plan wants to be, the solve lands between the
    two, and harder the larger the weight."""
    path = straight_path(tmp_path)
    out = []
    for w in (0.0, 20.0, 200.0):
        m = MPC3(path, w_dcmd=w)
        m.solve(0.12, 0.0, 0.0, I0, 4.0)
        m.last_cmd = 0.30              # a command nowhere near the plan
        cmd, _ = m.solve(0.12, 0.0, 0.0, I0, 4.0)
        out.append(cmd)
    assert out[0] < out[1] < out[2] < 0.30
    # ...and with the weight off, poking the same field changes nothing: two
    # identical solvers, one of them poked, publish the same angle.
    a, b = MPC3(path), MPC3(path)
    a.solve(0.12, 0.0, 0.0, I0, 4.0)
    b.solve(0.12, 0.0, 0.0, I0, 4.0)
    b.last_cmd = 0.30
    assert (a.solve(0.12, 0.0, 0.0, I0, 4.0)[0]
            == pytest.approx(b.solve(0.12, 0.0, 0.0, I0, 4.0)[0], abs=1e-12))


def test_cmd_tau_is_a_first_order_lag_at_the_measured_period(tmp_path):
    """The filter is a lag in SECONDS: alpha must follow the sensor period."""
    path = straight_path(tmp_path)
    m = MPC3(path, cmd_tau=0.10)
    first, _ = m.solve(0.05, 0.0, 0.0, I0, 4.0)
    assert m.last_cmd == pytest.approx(first)     # nothing to blend against
    prev = m.last_cmd
    m.period = 0.104
    cmd, info = m.solve(0.05, 0.0, 0.0, I0, 4.0)
    alpha = 1.0 - math.exp(-0.104 / 0.10)
    assert cmd == pytest.approx(prev + alpha * (info['cmd_raw'] - prev))
    # ...and half the period is half the step, near enough
    m2 = MPC3(path, cmd_tau=0.10)
    m2.solve(0.05, 0.0, 0.0, I0, 4.0)
    prev2 = m2.last_cmd
    m2.period = 0.052
    cmd2, info2 = m2.solve(0.05, 0.0, 0.0, I0, 4.0)
    a2 = 1.0 - math.exp(-0.052 / 0.10)
    assert cmd2 == pytest.approx(prev2 + a2 * (info2['cmd_raw'] - prev2))
    assert a2 < alpha


def test_cmd_notch_is_a_two_tap_average(tmp_path):
    path = straight_path(tmp_path)
    m = MPC3(path, cmd_notch=0.5)
    m.solve(0.05, 0.0, 0.0, I0, 4.0)
    prev = m.last_cmd
    cmd, info = m.solve(0.05, 0.0, 0.0, I0, 4.0)
    assert cmd == pytest.approx(0.5 * info['cmd_raw'] + 0.5 * prev)


def test_e_dead_is_silent_inside_the_band_and_offset_outside(tmp_path):
    """Inside the deadband the lateral cost has no gradient at all, so a
    straight line with a sub-band offset is answered with no steering."""
    path = straight_path(tmp_path)
    small = drive(path, frames=6, offset=0.02, e_dead=0.05)
    assert np.allclose(small, 0.0, atol=1e-3)
    big = drive(path, frames=6, offset=0.20, e_dead=0.05)
    plain = drive(path, frames=6, offset=0.20)
    # a real excursion is still corrected, in the same direction, just less
    assert np.sign(big[0]) == np.sign(plain[0]) != 0
    assert abs(big[0]) < abs(plain[0])


def test_rate_control_records_the_period_even_when_disabled(tmp_path):
    """`cmd_tau` needs the measured period whether or not rate adaptivity is
    on, so `RateControl.update` records it unconditionally."""
    path = straight_path(tmp_path)
    m = MPC3(path)
    rc = RateControl(m, Actuator(), enabled=False)
    assert m.period == pytest.approx(RATE_NOMINAL)
    t = 0.0
    for _ in range(30):
        t += 0.058
        rc.update(t)
    assert m.period == pytest.approx(0.058, abs=1e-6)
    assert m.cfg['apply_ahead_s'] == pytest.approx(DEFAULTS['apply_ahead_s'])


# --------------------------------------------------------------------------
# straight-only weave conditioning (`weave_notch_*`, `straight_*`)
# --------------------------------------------------------------------------
#
# The knobs exist for the 2-3 Hz lateral limit cycle the 18 m finals straight
# carries.  The contract is the same as above: identity at the defaults, and
# each does exactly what it claims when switched on.


def test_weave_defaults_are_inert():
    assert DEFAULTS['weave_notch_f'] == 0.0
    assert DEFAULTS['weave_notch_k'] == 0.0
    assert DEFAULTS['straight_hf_gain'] == 1.0
    assert DEFAULTS['straight_w_lat_scale'] == 1.0
    assert DEFAULTS['straight_w_yaw_scale'] == 1.0
    assert DEFAULTS['straight_lead_scale'] == 1.0


def test_weave_knobs_off_are_the_identity(tmp_path):
    path = straight_path(tmp_path)
    a = drive(path)
    b = drive(path, weave_notch_f=0.0, weave_notch_k=0.0,
              straight_hf_gain=1.0, straight_w_lat_scale=1.0,
              straight_w_yaw_scale=1.0, straight_lead_scale=1.0)
    assert np.array_equal(a, b)


def test_straight_gate_is_one_on_a_straight_and_zero_at_a_corner(tmp_path):
    """The ramp, which every one of these knobs is scaled by."""
    path = straight_path(tmp_path)
    m = MPC3(path, weave_notch_f=2.2)
    assert m._straight_gate(I0, 0.0) == pytest.approx(1.0, abs=1e-6)
    # off line by the release distance -> shut, whatever the geometry
    assert m._straight_gate(I0, DEFAULTS['straight_e']) == 0.0
    # a corner inside the window -> shut
    k = path.kappa.copy()
    k[I0:I0 + 20] = 0.9
    path.kappa = k
    assert m._straight_gate(I0, 0.0) == 0.0


def test_the_notch_rejects_its_own_centre_frequency(tmp_path):
    """A pure sine at f0 through the filter, gate wide open, comes out ~0."""
    path = straight_path(tmp_path)
    m = MPC3(path, weave_notch_f=2.2, weave_notch_q=0.7)
    m.period = 0.058
    t = np.arange(400) * 0.058
    drive_in = 0.05 * np.sin(2 * math.pi * 2.2 * t)
    out = np.array([m._weave_filter(float(x), 1.0) for x in drive_in])
    assert np.abs(out[200:]).max() < 0.1 * np.abs(drive_in[200:]).max()
    # ...and a constant passes untouched (unity gain, no lag, at DC)
    m2 = MPC3(path, weave_notch_f=2.2, weave_notch_q=0.7)
    m2.period = 0.058
    dc = [m2._weave_filter(0.03, 1.0) for _ in range(200)]
    assert dc[-1] == pytest.approx(0.03, abs=1e-9)


def test_the_notch_state_is_dropped_when_the_gate_shuts(tmp_path):
    """No transient survives a release -- the teleport-recovery contract."""
    path = straight_path(tmp_path)
    m = MPC3(path, weave_notch_f=2.2)
    m.period = 0.058
    for x in (0.05, -0.05, 0.05, -0.05):
        m._weave_filter(x, 1.0)
    assert m._wv is not None
    assert m._weave_filter(0.2, 0.0) == 0.2
    assert m._wv is None


def test_weave_notch_k_scales_the_centre_with_the_period(tmp_path):
    """`weave_notch_k` is Hz*s: f0 = k / (period + dead_tick_c)."""
    path = straight_path(tmp_path)
    for period, want in ((0.058, 0.227 / 0.103), (0.094, 0.227 / 0.139)):
        m = MPC3(path, weave_notch_k=0.227, weave_notch_q=0.7)
        m.period = period
        t = np.arange(600) * period
        for f, small in ((want, True), (want * 3.0, False)):
            mm = MPC3(path, weave_notch_k=0.227, weave_notch_q=0.7)
            mm.period = period
            sig = 0.05 * np.sin(2 * math.pi * f * t)
            out = np.array([mm._weave_filter(float(x), 1.0) for x in sig])
            rej = np.abs(out[300:]).max() / np.abs(sig[300:]).max()
            assert (rej < 0.15) if small else (rej > 0.5)
