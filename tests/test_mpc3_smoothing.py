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
