"""servo_head_kappa: the over-command headroom fades out with curvature; 0 = off."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src', 'pit_crew'))
from throttle2 import ServoThrottle


def cmd(head_kappa, kappa, k_p=0.5):
    s = ServoThrottle(k_p=k_p, head_max=0.3, head_kappa=head_kappa)
    out = None
    for i in range(30):                       # car stuck 1 m/s under its target
        out = s.command(4.0, 3.0, 0.0, now=0.1 * i, kappa=kappa)
    return out


def test_off_by_default_is_identical_at_any_curvature():
    assert cmd(0.0, 0.0) == cmd(0.0, 0.8)


def test_straight_keeps_the_full_headroom():
    assert cmd(0.5, 0.0) == cmd(0.0, 0.0)


def test_headroom_is_gone_at_and_above_the_gate():
    no_servo = cmd(0.0, 0.6, k_p=0.0)
    assert cmd(0.5, 0.5) == no_servo
    assert cmd(0.5, 0.9) == no_servo


def test_headroom_fades_monotonically():
    a, b, c = cmd(0.5, 0.1), cmd(0.5, 0.25), cmd(0.5, 0.4)
    assert a >= b >= c >= cmd(0.0, 0.0, k_p=0.0)
    assert a > c


def test_head_scale_zero_removes_the_over_command():
    s0 = ServoThrottle(k_p=0.5, head_max=0.3); s1 = ServoThrottle(k_p=0.5, head_max=0.3); s2 = ServoThrottle(k_p=0.0, head_max=0.3)
    a = b = c = None
    for i in range(30):
        a = s0.command(4.0, 3.0, 0.0, now=0.1 * i, kappa=0.0)
        b = s1.command(4.0, 3.0, 0.0, now=0.1 * i, kappa=0.0, head_scale=0.0)
        c = s2.command(4.0, 3.0, 0.0, now=0.1 * i, kappa=0.0)
    assert b == c and a > b
