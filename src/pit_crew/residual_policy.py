"""A bounded RESIDUAL policy on top of mpc3 — pure numpy, opt-in, default off.

What this is
------------
A small MLP whose weights live in an ``.npz``.  It reads the state mpc3
already has and returns two numbers:

* ``d_steer`` -- an ADDITIVE steering delta, hard-bounded to ``steer_max``
  radians (default 0.03).  Added to the angle the solver produced, before the
  publish guard.
* ``v_scale`` -- a MULTIPLICATIVE scale on the speed setpoint, hard-bounded to
  ``[v_lo, v_hi]`` (default 0.95-1.05).

It cannot replace the controller and it cannot be a large correction.  With no
``residual_policy`` parameter the whole file is dead: `mpc3` does not import a
policy, does not build an observation, and publishes byte-identical commands.

Why the bounds are asymmetric in a corner
-----------------------------------------
Every live failure on this track is the car running WIDE ON A CORNER EXIT
UNDER ACCELERATION (docs/FINALS_TUNING.md sections 3 and 5), and the surrogate
this policy is trained on is measurably OPTIMISTIC exactly there: it
reproduces the p95 of cross-track error and not the rare wide excursion that
hits the wall (section 4c).  So the policy is given no authority to add speed
where that failure lives:

* inside the track's declared slow zones (``zones``, arc-length spans), and
* wherever ``|kappa|`` under the car exceeds ``kappa_gate`` (default 0.3),

``v_scale`` is clamped to at most 1.0.  It may still slow the car down
anywhere.  Steering authority is symmetric but small.

The ``.npz``
------------
Required arrays: ``W0, b0, W1, b1, ... Wn, bn`` (dense layers, tanh between,
tanh on the output), ``obs_mean``, ``obs_std``.  Optional: ``steer_max``,
``v_lo``, ``v_hi``, ``kappa_gate``, ``zones`` (N x 2 of s_lo, s_hi),
``preview_kappa_m``, ``preview_wall_m``, ``track_len``.  Anything absent takes
the default below, so an export that only carries weights still runs.

Offline training and export live in ``tools/rl_train.py``; the observation is
built by ``observation()`` here so the node and the trainer cannot drift.
"""

import math
import os

import numpy as np

# Defaults, deliberately conservative.  An .npz may narrow them, never widen
# them: `_clip_bound` takes the tighter of the two.
STEER_MAX = 0.03            # rad
V_LO, V_HI = 0.95, 1.05
KAPPA_GATE = 0.3            # 1/m under the car above which no speed-up
PREVIEW_KAPPA_M = (0.0, 0.5, 1.0, 1.5, 2.5, 4.0)
PREVIEW_WALL_M = (0.0, 1.0, 2.0, 3.5)
N_OBS = 4 + len(PREVIEW_KAPPA_M) + 2 * len(PREVIEW_WALL_M)


def observation(path, i0, e_lat, e_yaw, v, period,
                preview_kappa_m=PREVIEW_KAPPA_M,
                preview_wall_m=PREVIEW_WALL_M):
    """The policy's input vector, from state mpc3 already holds.

    ``path`` is a ``RefPath``; wall distances need ``path.load_corridor`` to
    have run (``path.has_corridor``), and are reported FROM THE CAR, i.e. the
    reference half-width minus the car's own offset.  Without a corridor they
    are reported as the 2.0 m probe ceiling, which is what `load_corridor`
    itself saturates at.
    """
    sp = max(float(path.spacing), 1e-3)
    n = int(path.n)
    obs = [float(e_lat), float(e_yaw), float(v), float(period)]
    for d in preview_kappa_m:
        j = (int(i0) + int(round(d / sp))) % n
        obs.append(float(path.kappa[j]))
    has = bool(getattr(path, 'has_corridor', False))
    for d in preview_wall_m:
        j = (int(i0) + int(round(d / sp))) % n
        if has:
            obs.append(float(path.w_left[j]) - float(e_lat))
            obs.append(float(path.w_right[j]) + float(e_lat))
        else:
            obs.append(2.0)
            obs.append(2.0)
    return np.asarray(obs, dtype=np.float64)


class ResidualPolicy:
    """Pure-numpy inference.  No torch, no scipy -- the image has numpy only."""

    def __init__(self, path_npz):
        d = np.load(path_npz)
        self.layers = []
        k = 0
        while f'W{k}' in d:
            self.layers.append((np.asarray(d[f'W{k}'], dtype=np.float64),
                                np.asarray(d[f'b{k}'], dtype=np.float64)))
            k += 1
        if not self.layers:
            raise ValueError(f'{path_npz}: no W0/b0 in the archive')
        self.obs_mean = np.asarray(d['obs_mean'], dtype=np.float64)
        self.obs_std = np.maximum(np.asarray(d['obs_std'], dtype=np.float64),
                                  1e-6)
        if self.layers[0][0].shape[0] != len(self.obs_mean):
            raise ValueError(f'{path_npz}: W0 expects '
                             f'{self.layers[0][0].shape[0]} inputs but '
                             f'obs_mean has {len(self.obs_mean)}')
        if self.layers[-1][1].shape[0] != 2:
            raise ValueError(f'{path_npz}: the last layer must emit 2 outputs')

        def take(name, default):
            return float(d[name]) if name in d else float(default)

        # Never wider than the module defaults.
        self.steer_max = min(take('steer_max', STEER_MAX), STEER_MAX)
        self.v_lo = max(take('v_lo', V_LO), V_LO)
        self.v_hi = min(take('v_hi', V_HI), V_HI)
        self.kappa_gate = take('kappa_gate', KAPPA_GATE)
        self.zones = (np.asarray(d['zones'], dtype=np.float64).reshape(-1, 2)
                      if 'zones' in d else np.zeros((0, 2)))
        self.track_len = take('track_len', 0.0)
        self.preview_kappa_m = (tuple(np.asarray(d['preview_kappa_m']).ravel())
                                if 'preview_kappa_m' in d else PREVIEW_KAPPA_M)
        self.preview_wall_m = (tuple(np.asarray(d['preview_wall_m']).ravel())
                               if 'preview_wall_m' in d else PREVIEW_WALL_M)
        self.n_obs = len(self.obs_mean)
        self.calls = 0
        self.clipped = 0

    # -- the net ---------------------------------------------------------
    def _forward(self, obs):
        z = (np.asarray(obs, dtype=np.float64) - self.obs_mean) / self.obs_std
        for i, (W, b) in enumerate(self.layers):
            z = z @ W + b
            z = np.tanh(z)                   # tanh throughout, output included
        return z

    # -- the law ---------------------------------------------------------
    def act(self, obs, s=None, kappa=None):
        """``(d_steer_rad, v_scale)``, already bounded and zone-gated.

        A non-finite output is not a reason to hold a stale correction: the
        residual's identity is (0.0, 1.0), so that is what a bad net gets.
        """
        self.calls += 1
        try:
            y = self._forward(obs)
        except Exception:
            return 0.0, 1.0
        if not np.all(np.isfinite(y)):
            return 0.0, 1.0
        d_steer = float(np.clip(y[0], -1.0, 1.0)) * self.steer_max
        half = 0.5 * (self.v_hi - self.v_lo)
        v_scale = 0.5 * (self.v_hi + self.v_lo) + float(
            np.clip(y[1], -1.0, 1.0)) * half
        if self.no_speedup(s, kappa) and v_scale > 1.0:
            v_scale = 1.0
            self.clipped += 1
        return d_steer, v_scale

    def no_speedup(self, s, kappa):
        """True where the policy may only slow the car down."""
        if kappa is not None and abs(float(kappa)) > self.kappa_gate:
            return True
        if s is None or not len(self.zones):
            return False
        ss = float(s)
        if self.track_len > 0.0:
            ss = ss % self.track_len
        for lo, hi in self.zones:
            if lo <= hi:
                if lo <= ss <= hi:
                    return True
            elif ss >= lo or ss <= hi:       # a zone that wraps the start line
                return True
        return False


def load(path_npz):
    """``ResidualPolicy`` or ``None`` for an empty/absent path.

    Opt-in means opt-in: an empty parameter is not an error, and a MISSING file
    is.  Silently driving without a policy that was asked for would be the
    worst of both.
    """
    p = str(path_npz or '').strip()
    if not p:
        return None
    if not os.path.exists(p):
        raise FileNotFoundError(f'residual_policy: no such file: {p}')
    return ResidualPolicy(p)
