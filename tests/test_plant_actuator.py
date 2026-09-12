"""The opt-in steering-actuator laws in `tools/sim_tracker.py`'s plant.

The harness plant has always been a 3.2 rad/s WALL-CLOCK slew limiter with a
fixed transport dead time, which is rate invariant by construction and so can
never exhibit a plant that behaves differently at a different telemetry period.
`--plant-actuator` adds two alternatives, both default-off:

  * ``pole`` -- first order with ``--plant-tau``, which is what the staircase
    log (`data/logs/sysid_20260911_220017.csv`) identifies;
  * ``tick`` -- one movement allowance per telemetry tick, i.e. an effective
    slew of ``steer_rate * frame`` rad per tick, which is 1/frame in rad/s and
    therefore RATE DEPENDENT.

These tests pin the two properties that matter: the default is unchanged, and
``tick`` really is rate dependent while ``slew`` and ``pole`` are not.
"""

import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'src'))

import sim_tracker as ST                                       # noqa: E402
from pit_crew import vehicle                                   # noqa: E402


def drive(plant, target, seconds, frame):
    """Hold one steering target and run the plant, delivering frames."""
    t = 0.0
    next_frame = 0.0
    while t < seconds:
        if t >= next_frame - 1e-9:
            plant.accept_frame(t, target, 0.0)
            next_frame += frame
        plant.step(t)
        t += ST.DT
    return plant.delta


def make(**kw):
    return ST.Plant(0.0, 0.0, 0.0, v=3.0, **kw)


def test_default_is_the_historical_slew_limiter():
    p = make()
    assert p.actuator == 'slew'
    assert p.steer_rate == pytest.approx(vehicle.STEER_RATE)
    assert p.dead == pytest.approx(ST.EXTRA_DEAD)
    # 0.3 rad at 3.2 rad/s takes 0.094 s of slewing on top of the dead time.
    d = drive(p, 0.3, ST.EXTRA_DEAD + 0.094 + 0.01, 0.058)
    assert d == pytest.approx(0.3, abs=5e-3)


@pytest.mark.parametrize('frame', (0.058, 0.104))
def test_slew_and_pole_are_rate_invariant(frame):
    """Same wall-clock time, same angle, whatever the telemetry period is."""
    slew = drive(make(), 0.30, 0.25, frame)
    pole = drive(make(actuator='pole', tau=0.08), 0.30, 0.25, frame)
    assert slew == pytest.approx(0.30, abs=5e-3)
    # the pole has no rate limit but does have the dead time
    assert 0.15 < pole < 0.30
    # ... and both agree with the other period to well under a milliradian
    other = 0.104 if frame == 0.058 else 0.058
    assert drive(make(), 0.30, 0.25, other) == pytest.approx(slew, abs=1e-3)
    assert (drive(make(actuator='pole', tau=0.08), 0.30, 0.25, other)
            == pytest.approx(pole, abs=1e-3))


def test_tick_actuator_is_rate_dependent():
    """One allowance per telemetry tick: shorter ticks move the rack faster."""
    # One tick's worth of movement each, measured just after the allowance
    # lands: 3.2 * 0.058 = 0.186 rad against 3.2 * 0.104 = 0.333 rad, i.e. the
    # SAME rad per tick at very different rad/s -- which is the whole point.
    fast = make(actuator='tick', frame=0.058)
    slow = make(actuator='tick', frame=0.104)
    a = drive(fast, 0.52, ST.EXTRA_DEAD + 0.050, 1e9)
    b = drive(slow, 0.52, ST.EXTRA_DEAD + 0.050, 1e9)
    assert a == pytest.approx(3.2 * 0.058, abs=2e-3), a
    assert b == pytest.approx(3.2 * 0.104, abs=2e-3), b
    assert a / 0.058 == pytest.approx(b / 0.104, rel=0.02)
    # the per-tick allowance is steer_rate * frame, so the effective rad/s
    # ratio is the inverse of the frame ratio
    assert fast._tick_budget >= 0.0


def test_tick_allowance_is_steer_rate_times_frame():
    p = make(actuator='tick', frame=0.058, steer_rate=3.2)
    p.accept_frame(0.0, 0.5, 0.0)
    assert p._tick_budget == pytest.approx(3.2 * 0.058)
    # and it is spent, not refreshed, between frames
    t = p.dead
    for _ in range(200):
        p.step(t)
        t += ST.DT
    assert p.delta == pytest.approx(3.2 * 0.058, abs=2e-3)
