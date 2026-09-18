"""The opt-in real brake: throttle == 0.0 is a four-wheel brake, not a setpoint.

The simulator's ``VehicleController.Drive()`` computes
``DriveTorque = throttle * MotorTorque`` and, when that is exactly zero,
applies ``brakeTorque = MotorTorque`` on all four wheels.  Negative throttle is
REVERSE DRIVE under our prefab's brake type, so the exact float ``0.0`` is the
only brake the stack owns -- and ``throttle2.ServoThrottle`` has never emitted
it, because ``v_floor`` holds a walking-pace setpoint at all times.

Three properties are pinned here:

  * the defaults are unchanged (``brake_mode`` 'none' never emits 0.0, and the
    harness plant with ``brake_at_zero`` 0.0 is the old servo);
  * 'zero' emits an EXACT float 0.0 when the setpoint asks for deceleration,
    and never below ``v_floor``;
  * 'pulse' emits 0.0 on a fraction of ticks that tracks the requested
    deceleration.
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'src'))

import sim_tracker as ST                                       # noqa: E402
from pit_crew.throttle2 import ServoThrottle                   # noqa: E402


def ticks(thr, v_target, v_meas, n=40, hold=0.09):
    """`n` commands at a fixed setpoint and a fixed measured speed."""
    return [thr.command(v_target, v_meas, 0.0, now=k * hold) for k in range(n)]


def test_default_never_publishes_zero():
    thr = ServoThrottle(hold=0.09)
    us = ticks(thr, 2.0, 7.0)
    assert min(us) > 0.0
    assert thr.brake_mode == 'none'


def test_zero_mode_brakes_exactly_zero():
    thr = ServoThrottle(hold=0.09, brake_mode='zero')
    us = ticks(thr, 2.0, 7.0)
    assert us[0] == 0.0 and isinstance(us[0], float)
    assert all(u == 0.0 for u in us)


def test_zero_mode_does_not_brake_when_accelerating():
    thr = ServoThrottle(hold=0.09, brake_mode='zero')
    us = ticks(thr, 7.0, 2.0)
    assert min(us) > 0.0


def test_zero_mode_respects_the_crawl_floor():
    """Below `v_floor` the car is parking; a full brake there stops it dead."""
    thr = ServoThrottle(hold=0.09, brake_mode='zero', v_floor=1.0)
    us = ticks(thr, 0.2, 0.6)
    assert min(us) > 0.0


def test_pulse_duty_tracks_the_requested_deceleration():
    """A small ask brakes on some ticks; a big ask brakes on all of them."""
    small = ServoThrottle(hold=0.09, brake_mode='pulse', brake_a_full=4.6,
                          predict=False)
    big = ServoThrottle(hold=0.09, brake_mode='pulse', brake_a_full=4.6,
                        predict=False)
    n = 60
    f_small = sum(1 for u in ticks(small, 6.6, 7.0, n) if u == 0.0) / n
    f_big = sum(1 for u in ticks(big, 3.0, 7.0, n) if u == 0.0) / n
    assert 0.0 < f_small < 1.0
    assert f_big > f_small
    assert f_big > 0.9


def test_plant_brake_at_zero_default_off():
    p = ST.Plant(0.0, 0.0, 0.0, v=6.0, **ST.PLANT_REAL)
    p.accept_frame(0.0, 0.0, 0.0)
    for k in range(200):
        p.step(k * ST.DT + 1.0)
    servo_only = p.v
    q = ST.Plant(0.0, 0.0, 0.0, v=6.0, brake_at_zero=7.0, **ST.PLANT_REAL)
    q.accept_frame(0.0, 0.0, 0.0)
    for k in range(200):
        q.step(k * ST.DT + 1.0)
    # The servo's own a_brake clip is 4.0 m/s2 and it eases off as it arrives;
    # the real brake is a constant 7.0 and must end up slower.
    assert q.v < servo_only - 0.2


def test_plant_brake_only_on_exact_zero():
    """A hair above zero is drive torque, not a brake -- as in the C# source."""
    p = ST.Plant(0.0, 0.0, 0.0, v=6.0, brake_at_zero=7.0, **ST.PLANT_REAL)
    p.accept_frame(0.0, 0.0, 1e-6)
    for k in range(50):
        p.step(k * ST.DT + 1.0)
    q = ST.Plant(0.0, 0.0, 0.0, v=6.0, brake_at_zero=7.0, **ST.PLANT_REAL)
    q.accept_frame(0.0, 0.0, 0.0)
    for k in range(50):
        q.step(k * ST.DT + 1.0)
    assert q.v < p.v
