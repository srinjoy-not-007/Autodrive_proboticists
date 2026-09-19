"""Delay-compensated feedforward + feedback steering, with the MPC as envelope.

The hypothesis this tests (from the overhaul brief): the straight-line swivel
and the hot first-hairpin entry are artefacts of re-solving a 10-step QP from a
pose that is one sensor period old, and a law whose ANTICIPATION is explicit --
a curvature preview sized to the rack's own 3.2 rad/s rate limit -- plus a
low-gain feedback on a dead-time-projected error would track the same line with
less phase lag and no limit cycle.

Two pieces:

* `FFBSteer.command` -- `delta = delta_ff(preview) + k_fb * delta_pp(projected
  error)`.  The preview distance is not a constant: the rack needs
  `|delta_want - delta_now| / STEER_RATE` seconds to travel, so the preview is
  solved for in two fixed-point sweeps, which is what "sized to the rate limit"
  means.  The feedback is the same pure-pursuit form the shipping
  `straight_pp_k` blend uses on the straights, applied everywhere, with the
  lateral error projected over the measured transport delay `dead + tau`.
* `envelope_trip` -- roll the CANDIDATE angle forward on the kinematic bicycle
  and report whether the car leaves the corridor inside the horizon.  When it
  does, the caller publishes the MPC's command for that frame instead.  That is
  the "MPC as a safety envelope" arrangement: the cheap law drives, the solver
  vetoes.

Pure numpy; no state except the caller's.
"""

import math

import numpy as np

from . import vehicle
from .mpc3_core import wrap

FFB_DEFAULTS = dict(
    # anticipation: the command bites `ffb_dead_s` from now (rate-adapted by
    # the caller, which hands in the measured value) and then has to be THERE,
    # so the preview is `v * (dead + rack travel time)`.
    ffb_dead_s=0.175,           # cmd_dead_time 0.135 + steer_tau 0.04
    ffb_preview_extra_s=0.0,    # bias on top of the solved preview
    ffb_preview_max_m=3.0,
    ffb_rate_frac=1.0,          # fraction of STEER_RATE the preview assumes
    ffb_kappa_span_m=0.30,      # curvature averaged over this much line
    # feedback
    ffb_k=1.0,                  # gain on the pure-pursuit correction
    ffb_ld_k=0.55,              # lookahead = ld_k * v
    ffb_ld_min=0.80,
    ffb_ld_max=5.0,
    ffb_yaw_k=1.0,              # weight on the heading part of the correction
    ffb_proj_tk=0.5,            # projection time = proj_t0 + proj_tk * period
    ffb_proj_t0=0.056,
    # output
    ffb_cmd_tau=0.06,           # first-order output filter, s (0 = off)
    # envelope
    ffb_env_margin=0.02,        # extra corridor the veto defends, m
    ffb_env_steps=12,           # horizon steps checked
    ffb_env_e=0.45,             # |e_lat| beyond which the MPC always drives
)


class FFBSteer:
    """The feedforward+feedback steering law.  One call per sensor frame."""

    def __init__(self, path, **cfg):
        self.path = path
        self.cfg = dict(FFB_DEFAULTS)
        self.cfg.update({k: v for k, v in cfg.items()
                         if k in FFB_DEFAULTS and v is not None})
        self.last = None
        self.trips = 0
        self.calls = 0

    # ------------------------------------------------------------------
    def _kappa_at(self, i0, d_m):
        """Line curvature `d_m` metres ahead, averaged over `ffb_kappa_span_m`."""
        P = self.path
        sp = max(P.spacing, 1e-3)
        j0 = int(round(max(0.0, d_m) / sp))
        n = max(1, int(round(float(self.cfg['ffb_kappa_span_m']) / sp)))
        j = (int(i0) + j0 + np.arange(n)) % P.n
        return float(np.mean(P.kappa[j]))

    def feedforward(self, i0, v, delta0):
        """Line-matching angle at the point the rack can actually be there."""
        c = self.cfg
        rate = max(float(c['ffb_rate_frac']) * vehicle.STEER_RATE, 1e-3)
        t_lead = float(c['ffb_dead_s']) + float(c['ffb_preview_extra_s'])
        d = min(v * t_lead, float(c['ffb_preview_max_m']))
        ang = 0.0
        for _ in range(2):
            ang = math.atan(vehicle.WHEELBASE * self._kappa_at(i0, d))
            t_travel = abs(ang - delta0) / rate
            d = min(v * (t_lead + t_travel), float(c['ffb_preview_max_m']))
        return ang, d

    def feedback(self, e_lat, e_yaw, v, period):
        """Pure-pursuit correction on the dead-time-projected error."""
        c = self.cfg
        vv = max(float(v), 0.5)
        Td = float(c['ffb_proj_t0']) + float(c['ffb_proj_tk']) * float(period)
        e_p = float(e_lat) + vv * Td * math.sin(float(e_yaw))
        psi_p = float(c['ffb_yaw_k']) * float(e_yaw)
        Ld = min(max(float(c['ffb_ld_k']) * vv, float(c['ffb_ld_min'])),
                 float(c['ffb_ld_max']))
        alpha = -psi_p - math.atan2(e_p, Ld)
        return math.atan(2.0 * vehicle.WHEELBASE * math.sin(alpha) / Ld)

    def command(self, e_lat, e_yaw, delta0, i0, v, cap, period):
        c = self.cfg
        ff, prev_m = self.feedforward(i0, max(float(v), 0.5), float(delta0))
        fb = self.feedback(e_lat, e_yaw, v, period)
        cmd = ff + float(c['ffb_k']) * fb
        cmd = float(np.clip(cmd, -cap, cap))
        tau = float(c['ffb_cmd_tau'])
        if tau > 0.0 and self.last is not None:
            al = 1.0 - math.exp(-max(float(period), 1e-3) / tau)
            cmd = self.last + al * (cmd - self.last)
            cmd = float(np.clip(cmd, -cap, cap))
        self.last = cmd
        return cmd, dict(ff=ff, fb=fb, preview_m=prev_m)

    def hold(self, cmd):
        """Tell the filter what was actually published (after a veto)."""
        self.last = float(cmd)


def envelope_trip(path, i0, e_lat, e_yaw, delta, v, dt, steps, margin,
                  yaw_gain=0.956):
    """True if holding `delta` takes the car outside the corridor.

    The same path-relative bicycle rollout the MPC uses, with the steering
    HELD -- which is the honest question for a law that publishes one angle per
    frame: if the next frame never came, would this angle put the car in a
    wall?
    """
    if not path.has_corridor:
        return False
    hw = 0.5 * vehicle.CAR_WIDTH + margin
    sp = max(path.spacing, 1e-3)
    e, psi = float(e_lat), float(wrap(e_yaw))
    kc = yaw_gain * math.tan(float(delta)) / vehicle.WHEELBASE
    s_acc = 0.0
    ds = max(float(v) * dt, 0.05)
    for _ in range(int(steps)):
        j = (int(i0) + int(s_acc / sp)) % path.n
        kr = float(path.kappa[j])
        den = max(1.0 - kr * e, 0.2)
        e = e + math.sin(psi) * ds
        psi = wrap(psi + (kc - kr * math.cos(psi) / den) * ds)
        s_acc += ds
        j = (int(i0) + int(s_acc / sp)) % path.n
        if e > float(path.w_left[j]) - hw or -e > float(path.w_right[j]) - hw:
            return True
    return False
