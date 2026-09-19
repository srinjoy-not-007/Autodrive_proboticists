"""Opt-in closed-loop PI throttle (`ServoThrottle`, `pi_*` params).

The feedforward law tracks the plan but never spends the servo's authority to
CLOSE an error, so the car sits ~0.5 m/s under plan accelerating out of the slow
corners.  The PI mode adds a proportional+integral correction on
(setpoint - encoder speed) as EXTRA SPEED SETPOINT, in the same shape MonacoF1's
public stack uses (PI on speed error, integrator clamped to a traction-slip
bound, on top of the 0.0408/(m/s) feedforward).

The properties pinned here are the ones that keep it out of the `throttle_kp`
bang-bang trap (75 collisions, car reversing) and keep the shipping defaults
untouched:

  * defaults (both gains 0) are byte-identical to the feedforward law;
  * the output is NEVER negative -- no reverse command, at any error;
  * the exact-float 0.0 brake is preserved (the PI is upstream of it);
  * the integral term is bounded by the traction-slip clamp;
  * a step in the setpoint does not slam the loop (prefilter) and the closed
    loop does not limit-cycle between full and zero throttle.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'src', 'pit_crew'))

from throttle2 import ServoThrottle                              # noqa: E402

# The Sep 19 shipping servo config, so the tests exercise the real construction.
SHIP = dict(K=18.0, drag=0.273, tau=0.135, a_auth=4.5, a_accel=5.0,
            k_p=0.5, head_max=0.3, head_kappa=0.4, lead_max=0.0, ay_limit=9.0,
            v_floor=1.0, u_max=0.60, servo_gain=24.6, brake_mode='zero',
            brake_a_min=1.5, brake_dv=0.3, brake_a_full=4.6)

PI = dict(pi_kp=3.0, pi_ki=5.0, pi_prefilter_tau=0.15, pi_slip_clamp=0.15)


def seq(thr, pairs, kappa=0.0, hold=0.09):
    """`pairs` is [(v_target, v_meas), ...]; returns the commands."""
    return [thr.command(vt, vm, 0.0, now=k * hold, kappa=kappa)
            for k, (vt, vm) in enumerate(pairs)]


# ---- defaults byte-identical --------------------------------------------------

def test_default_is_the_feedforward_law():
    """Both gains 0 -> the PI block never runs; commands are unchanged."""
    a = ServoThrottle(**SHIP)
    b = ServoThrottle(pi_kp=0.0, pi_ki=0.0, pi_prefilter_tau=0.0,
                      pi_slip_clamp=0.0, **SHIP)
    pairs = [(2.0 + (k % 5), 1.5 + (k % 7) * 0.4) for k in range(200)]
    assert seq(a, pairs, kappa=0.3) == seq(b, pairs, kappa=0.3)


def test_pi_off_touches_no_state():
    thr = ServoThrottle(**SHIP)               # gains default to 0
    assert thr._pi_on is False
    seq(thr, [(6.0, 3.0)] * 20)
    assert thr._pi_int == 0.0 and thr._pi_sp is None


# ---- never reverse ------------------------------------------------------------

def test_never_negative_even_on_a_deep_brake_request():
    thr = ServoThrottle(**dict(SHIP, brake_mode='none'), **PI)
    # target far below measured speed for a long time: the error is strongly
    # negative and drives the integral to its lower clamp.
    us = seq(thr, [(1.0, 8.0)] * 200, kappa=0.0)
    assert min(us) >= 0.0


def test_never_negative_across_a_wide_sweep():
    thr = ServoThrottle(**SHIP, **PI)         # brake_mode='zero'
    pairs = []
    for k in range(400):
        pairs.append((0.5 + (k % 9), 0.3 + ((k * 3) % 11) * 0.8))
    us = seq(thr, pairs, kappa=(0.6 if 0 else 0.0))
    assert min(us) >= 0.0


# ---- exact-0.0 brake preserved ------------------------------------------------

def test_exact_zero_brake_still_fires_with_pi_on():
    thr = ServoThrottle(**SHIP, **PI)         # brake_mode='zero'
    us = seq(thr, [(2.0, 7.0)] * 40)
    assert us[0] == 0.0 and isinstance(us[0], float)
    assert any(u == 0.0 for u in us)
    # the brake decision is on the speed error, not on the PI term
    assert all(u == 0.0 for u in us)


def test_pi_does_not_brake_when_accelerating():
    thr = ServoThrottle(**SHIP, **PI)
    us = seq(thr, [(7.0, 2.0)] * 40)
    assert min(us) > 0.0


# ---- integrator bounded -------------------------------------------------------

def test_integral_is_clamped_to_the_slip_bound():
    v_meas = 4.0
    thr = ServoThrottle(**dict(SHIP, brake_mode='none'), **PI)
    seq(thr, [(6.0, v_meas)] * 300)           # a persistent under-speed
    assert abs(thr._pi_int) <= PI['pi_slip_clamp'] * v_meas + 1e-9
    # and it actually reaches the bound (the clamp binds, it is not slack)
    assert abs(thr._pi_int) > 0.5 * PI['pi_slip_clamp'] * v_meas


def test_no_integral_windup_recovers_after_overspeed():
    thr = ServoThrottle(**dict(SHIP, brake_mode='none'), **PI)
    seq(thr, [(2.0, 8.0)] * 200)              # integral driven negative
    lo = thr._pi_int
    seq(thr, [(8.0, 8.0)] * 200)              # now ask to accelerate
    assert thr._pi_int > lo                   # it unwinds, no permanent lock


# ---- prefilter: a step does not slam the loop --------------------------------

def test_prefilter_lags_a_step():
    thr = ServoThrottle(**dict(SHIP, brake_mode='none'), **PI)
    thr.command(3.0, 3.0, 0.0, now=0.0)       # settle the filter at 3
    thr.command(8.0, 3.0, 0.0, now=0.09)      # step the target up
    # the setpoint the loop tracks has not jumped the whole way to 8
    assert 3.0 < thr._pi_sp < 8.0


def test_prefilter_off_uses_the_raw_setpoint():
    thr = ServoThrottle(**dict(SHIP, brake_mode='none'),
                        pi_kp=3.0, pi_ki=0.0, pi_prefilter_tau=0.0,
                        pi_slip_clamp=0.15)
    thr.command(8.0, 3.0, 0.0, now=0.0)
    # no prefilter state is kept when tau is 0; the loop tracks v_target itself
    assert thr._pi_sp is None


# ---- closed loop does not limit-cycle (no bang-bang) -------------------------

def _servo_plant(thr, v_target, n=260, hold=0.09, K=18.0, g=24.6,
                 a_accel=5.0, a_auth=4.5, drag=0.273, tau=0.135):
    """Close the loop around the same servo model ServoThrottle inverts,
    WITH the dead time it compensates for: a command issued now takes effect
    `tau` later.  Leaving the dead time out would make the law's dead-time
    prediction wrong and manufacture an oscillation the real plant does not
    have."""
    v = 0.0
    us, vs, hist = [], [], []
    for k in range(n):
        t = k * hold
        u = thr.command(v_target, v, 0.0, now=t, kappa=0.0)
        hist.append((t, u))
        steps = max(1, int(hold / 0.001))
        h = hold / steps
        for s in range(steps):
            tt = t + s * h
            ue = 0.0
            for ta, ua in hist:                # last command that has landed
                if ta <= tt - tau:
                    ue = ua
            a = K * (g * ue - v)
            a = max(-a_auth, min(a_accel, a)) - drag * v
            v = max(0.0, v + a * h)
        us.append(u)
        vs.append(v)
    return us, vs


def test_step_response_does_not_bang_bang():
    thr = ServoThrottle(**dict(SHIP, brake_mode='none'), **PI)
    us, vs = _servo_plant(thr, 6.0)
    tail_u, tail_v = us[-60:], vs[-60:]
    # settled near the target, no overshoot into a limit cycle
    assert 6.0 - 0.5 <= min(tail_v) and max(tail_v) <= 6.0 + 0.5
    # a bang-bang loop visits BOTH full throttle and (near) zero in steady
    # state -- the `throttle_kp` pathology.  This one must not.
    assert not (max(tail_u) > 0.95 * thr.u_max
                and min(tail_u) < 0.05 * thr.u_max)
    # and the steady command is not a square wave
    tick_swing = max(abs(tail_u[i + 1] - tail_u[i])
                     for i in range(len(tail_u) - 1))
    assert tick_swing < 0.15


def test_pi_closes_the_steady_state_deficit():
    """The feedforward law settles UNDER target; PI closes most of it, on
    average (compared over the settled tail, not a single sample)."""
    ff = ServoThrottle(**dict(SHIP, brake_mode='none'))          # gains 0
    cl = ServoThrottle(**dict(SHIP, brake_mode='none'), **PI)
    _, v_ff = _servo_plant(ff, 6.0)
    _, v_cl = _servo_plant(cl, 6.0)
    mean_ff = sum(v_ff[-60:]) / 60.0
    mean_cl = sum(v_cl[-60:]) / 60.0
    assert mean_cl > mean_ff                  # closed loop closes the gap
    assert mean_cl >= 6.0 - 0.2               # and lands close to the target
