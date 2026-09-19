#!/usr/bin/env python3
"""Episode rollouts of the REAL mpc3 plus a residual policy, on the surrogate.

This is the training environment for `tools/rl_train.py`.  It is deliberately
NOT a gym `step()` API: the policy is consulted from inside the controller, at
exactly the point `mpc3.MPC3Tracker.control` consults it, through the same
`pit_crew.residual_policy` interface.  An episode therefore runs end to end and
hands back the (obs, action, logprob, reward) trajectory, which is all PPO
needs and removes any chance of the training-time law drifting from the
deployed one.

The plant is `tools/sim_tracker.py --plant-fit`, i.e. the chain identified from
the live logs (docs/FINALS_TUNING.md section 4c) -- with one deliberate
difference.  That surrogate is measurably OPTIMISTIC at the limit: it
reproduces the p95 of cross-track error and not the rare wide excursion that
hits the wall.  So training uses a PESSIMISTIC variant on top of the domain
randomisation:

* the lateral delivery ratio is cut a further `grip_pess` (0.10-0.15) above
  asked ay 7 whenever the car is accelerating, which is the state every live
  wide-exit failure is in;
* the wall is inflated by `wall_inflate` for the CONTACT test, so a policy
  that learns to graze a wall in the surrogate is already clear of it in
  reality.  **0.05 m, not 0.10**: measured, the shipping line's own minimum
  footprint clearance on a zero-residual pessimistic rollout is 0.079 m, so
  inflating by 0.10 makes contact unavoidable no matter what the policy does
  and the only reachable policy is "go slower" -- which is exactly what 0.10
  trained (near-wall contacts fell, lap time did not move).  An infeasible
  penalty is not conservatism, it is a broken objective;
* the sensor period, the pose noise and bias, and the rack lag are randomised
  per episode over the measured spread and beyond.

Offline only.  HOST RULE: at most 6 single-threaded workers on cores 22-27.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src'))

import sim_tracker as ST                                        # noqa: E402
from pit_crew import pf_core, residual_policy as RP             # noqa: E402
from pit_crew.mpc2_core import RefPath, wrap                    # noqa: E402

# The shipping controller: the policy is a residual ON THIS, never a
# replacement.  Same list as tools/surrogate_race.py SHIPPING.
SHIPPING = dict(rate_adaptive=1, w_dcmd=50.0, cmd_tau=0.10, steer_tau=0.04,
                barrier_margin=0.40, thr_brake_lead_s=0.05, brake_mode='zero',
                brake_a_min=1.5, brake_dv=0.3, brake_a_full=4.6,
                pub_gate_corr=0.05, servo_k_p=0.5, servo_head_max=0.3,
                servo_head_kappa=0.4)

# The three zones the live evidence says are capped by something other than the
# controller (steering lock at the last hairpin, the organisers' sensor period
# at the other two -- docs/FINALS_TUNING.md section 5 and docs/STATUS.md).  The
# policy gets no authority to add speed inside them.
FINALS_ZONES = ((17.5, 21.5), (36.0, 40.0), (42.0, 2.0))   # last one wraps


class _Hook:
    """What `MPC3Sim` sees: an object with `.act()` and the preview spec.

    It records every observation and the action that was taken for it, so the
    trainer gets the trajectory without the controller knowing anything about
    training.  The bounds and the no-speed-up gate are the SHIPPING ones from
    `pit_crew.residual_policy`, applied here exactly as `ResidualPolicy.act`
    applies them.
    """

    def __init__(self, act_fn, steer_max=RP.STEER_MAX, v_lo=RP.V_LO,
                 v_hi=RP.V_HI, kappa_gate=RP.KAPPA_GATE, zones=FINALS_ZONES,
                 track_len=0.0):
        self.act_fn = act_fn
        self.steer_max = float(steer_max)
        self.v_lo, self.v_hi = float(v_lo), float(v_hi)
        self.kappa_gate = float(kappa_gate)
        self.zones = np.asarray(zones, float).reshape(-1, 2)
        self.track_len = float(track_len)
        self.preview_kappa_m = RP.PREVIEW_KAPPA_M
        self.preview_wall_m = RP.PREVIEW_WALL_M
        self.obs, self.raw, self.extra = [], [], []

    # `ResidualPolicy.no_speedup`, verbatim in behaviour.
    def no_speedup(self, s, kappa):
        return RP.ResidualPolicy.no_speedup(self, s, kappa)

    def act(self, obs, s=None, kappa=None):
        y, extra = self.act_fn(obs)
        d_steer = float(np.clip(y[0], -1.0, 1.0)) * self.steer_max
        half = 0.5 * (self.v_hi - self.v_lo)
        v_scale = 0.5 * (self.v_hi + self.v_lo) + float(
            np.clip(y[1], -1.0, 1.0)) * half
        if self.no_speedup(s, kappa) and v_scale > 1.0:
            v_scale = 1.0
        self.obs.append(np.asarray(obs, float))
        self.raw.append(np.asarray(y, float))
        self.extra.append(extra)
        return d_steer, v_scale


class Track:
    """Map, line and corridor, loaded once and shared by every episode."""

    def __init__(self, line, map_prefix):
        self.dist, _img, self.meta = pf_core.load_map(map_prefix)
        self.line = line
        self.path = RefPath(line)
        self.path.load_corridor(self.dist, self.meta)
        self.total = float(self.path.s[-1] + self.path.spacing)


def sample_domain(rng, frame=None):
    """One episode's randomised plant.  Measured spread, then some."""
    return dict(
        frame=float(frame if frame is not None else rng.uniform(0.055, 0.110)),
        pose_scale=float(rng.uniform(1.0, 1.5)),
        rack=float(rng.uniform(0.7, 1.3)),
        grip_pess=float(rng.uniform(0.10, 0.15)),
        grip_b=float(ST.GRIP_B * rng.uniform(0.9, 1.15)),
        # The breakaway point is the least certain constant in the plant: the
        # racing logs fit 8.8 and the skidpad says 9.9.  Randomise across both.
        grip_cliff=float(rng.uniform(8.8, 9.9)),
        grip_post=float(rng.uniform(0.48, 0.72)),
        seed=int(rng.integers(0, 2 ** 31 - 1)),
    )


def rollout(track, act_fn, dom, laps=3, wall_inflate=0.05,
            clear_target=0.30, w_progress=1.0, w_time=4.0, w_clear=6.0,
            collision_penalty=60.0, residual=True, max_s=None):
    """Drive `laps` laps and return the trajectory and the summary.

    Reward per controller step:

        progress_m  -  w_time * dt  -  w_clear * max(0, 0.30 - clearance)
                    -  collision_penalty * contact

    The first two terms together are a MINIMUM-TIME objective, and the pair is
    load-bearing.  An episode is a fixed number of LAPS, so the total progress
    is a constant (one lap length per lap) whatever the policy does; "progress
    per second" summed over such an episode is therefore also a constant and
    rewards nothing at all.  Subtracting `w_time * dt` makes the return
    `laps * L - w_time * T`, i.e. minimise the time.  `w_time` 4.0 is roughly
    the lap's mean speed, so a step is positive exactly when the car is beating
    it.

    Nothing rewards steering smoothness or punishes the residual for being
    non-zero: the bounds in `pit_crew.residual_policy` are the regularisation.
    """
    path = RefPath(track.line)
    path.load_corridor(track.dist, track.meta)
    hook = _Hook(act_fn, track_len=track.total) if residual else None
    kw = dict(SHIPPING)
    tr = ST.MPC3Sim(path, **kw)
    if hook is not None:
        tr.residual = hook
    # The rack's dead time is the one plant constant the live logs pin to
    # 0.056 s; randomise around it rather than trusting it.
    act_cfg = dict(actuator='slew', steer_rate=None, tau=0.08,
                   dead=ST.FIT_DEAD * dom['rack'], tick_frac=1.0,
                   lat_law='fit', grip_a=ST.GRIP_A, grip_b=dom['grip_b'],
                   grip_c=dom['grip_pess'] / 2.0, grip_ay0=7.0,
                   grip_cliff=dom.get('grip_cliff', ST.GRIP_CLIFF),
                   grip_cliff_w=ST.GRIP_CLIFF_W,
                   grip_post=dom.get('grip_post', ST.GRIP_POST),
                   lat_tau=ST.LAT_TAU)
    out = ST.run(
        tr, path, laps=laps, dist=track.dist, meta=track.meta,
        seed=dom['seed'], frame=dom['frame'],
        pose_noise=ST.FIT_POSE_NOISE * dom['pose_scale'],
        pose_noise_kappa=ST.FIT_POSE_NOISE_K * dom['pose_scale'],
        pose_bias_inside=ST.FIT_POSE_BIAS_INSIDE * dom['pose_scale'],
        pose_bias_kappa=ST.FIT_POSE_BIAS_KAPPA,
        pose_lon_bias=ST.FIT_POSE_LON_BIAS * dom['pose_scale'],
        pose_latency=ST.FIT_POSE_LATENCY, yaw_noise=ST.FIT_YAW_NOISE,
        frame_jitter=ST.FIT_FRAME_JITTER, enc_alpha=ST.FIT_ENC_ALPHA,
        enc_slip=ST.FIT_ENC_SLIP, log_enc_alpha=ST.LOG_ENC_ALPHA,
        servo=dict(ST.PLANT_REAL, brake_at_zero=ST.FIT_BRAKE_AT_ZERO),
        actuator=act_cfg, verbose=False)

    rec = out['rec']
    n = len(hook.obs) if hook is not None else 0
    res = dict(laps=out['laps'], collisions=out['collisions'],
               coll_s=out['coll_s'], lap_times=out['lap_times'],
               n=n, rec=rec)
    if hook is None or n == 0:
        return res

    # Map every controller call onto the record, which is sampled at 100 Hz.
    # `rec` columns: t,x,y,yaw,v,delta,s,e,clear,lap,thr_plant,thr_cmd,enc,log
    t_rec = rec[:, 0]
    s_rec = rec[:, 6]
    clear_raw = rec[:, 8]
    # The wall is inflated for the CONTACT test only.  Inflating it for the
    # proximity penalty as well would put the target at 0.40 m of real
    # clearance, which this line does not have anywhere (0.377 m effective),
    # so the penalty would be a constant and carry no gradient.
    clear = clear_raw - wall_inflate
    # Controller calls happen one per delivered pose; their count and the
    # record's length differ, so bin the record by call index in time.
    dt_call = dom['frame']
    t0 = t_rec[0]
    idx = np.clip(((t_rec - t0) / dt_call).astype(int), 0, n - 1)
    prog = np.zeros(n)
    minclear = np.full(n, 10.0)
    minraw = np.full(n, 10.0)
    hit = np.zeros(n, bool)
    ds = np.diff(s_rec, prepend=s_rec[0])
    ds[ds < -0.5 * track.total] += track.total
    ds[ds > 0.5 * track.total] -= track.total
    np.add.at(prog, idx, ds)
    np.minimum.at(minclear, idx, clear)
    np.minimum.at(minraw, idx, clear_raw)
    # A contact is a record sample whose (inflated) clearance went negative.
    np.logical_or.at(hit, idx, clear <= 0.0)

    r = w_progress * prog - w_time * dt_call             # minimum time
    seen = minraw < 10.0                                 # calls with a record
    r -= w_clear * np.where(seen, np.maximum(0.0, clear_target - minraw), 0.0)
    r -= collision_penalty * hit.astype(float)
    res.update(obs=np.asarray(hook.obs), raw=np.asarray(hook.raw),
               extra=hook.extra, reward=r, hit=hit,
               min_clear=float(np.min(clear_raw)) if len(clear_raw)
               else float('nan'),
               min_clear_inflated=float(np.min(clear)) if len(clear)
               else float('nan'))
    return res
