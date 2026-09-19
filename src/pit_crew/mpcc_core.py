"""Joint speed-and-steering MPC ("MPCC") in the path-relative frame.

`mpc3` optimises the STEERING against a speed that is planned, not decided:
`MPC3._speed_plan` builds a ceiling from the fixed offline profile, cuts it by a
heuristic on the lateral error, and the throttle law then decides on its own --
with two hand-tuned thresholds (`brake_a_min`, `brake_dv`) and a lead in seconds
(`thr_brake_lead_s`) -- when to publish the exact 0.0 that is the simulator's
only brake.  Three measured consequences of that split, all in
docs/FINALS_TUNING.md:

* the offline profile plans **brake 4.6 m/s2** while the car's real
  throttle-0.0 deceleration is **4.99 (v 2-3) ... 5.74 (v 5-7) m/s2** and 6.39
  from 7.1 m/s (`data/logs/brake_20260919_132335.csv`) -- so every braking zone
  is entered earlier than it has to be;
* the brake TRIGGER is a threshold on a lead that scales with the sensor
  period, so the same config arrives +0.25 m/s hot at the first hairpin and
  0.2-0.3 m/s cold into s 37 at the organisers' period;
* the speed plan does not know what the steering plan is about to ask for, so a
  corner EXIT tracks the profile's exit ACCELERATION while the car is still
  turning, and runs wide -- the failure mode behind every ay-9 rung that was
  disqualified.

This module makes the speed a STATE and the longitudinal acceleration an INPUT
of the same Gauss-Newton problem the steering is solved in, with

* the **measured envelope** (`sysid_run` in the `-explore` image, refitted by
  `tools/sysid_breakaway.py`) as a pair of SOFT hinges: an asked-yaw-rate limit
  `v |kappa| <= 2.71 rad/s` -- which is what breakaway on this tyre actually
  is -- and an asked-ay ceiling `v^2 |kappa| <= 10.5 m/s2` that only binds
  above ~5 m/s.  Soft, so the solver may pay for lock with speed or for speed
  with lock, and a short excursion (measured: 0.19-0.29 s past 2.71 still
  delivers 0.95 of what is asked) costs something finite rather than being
  forbidden;
* the **real brake** as the lower bound of the longitudinal input
  (`4.15 + drag*v`, the fitted throttle-0.0 law), realised as the discrete
  "publish exactly 0.0" decision the simulator actually has;
* the **fixed offline profile kept as the reference and the terminal set**: the
  horizon is only ~1.2 s, so without a terminal constraint the solver would
  happily carry speed into a corner it cannot see.  The ceiling the horizon
  tracks is built from the envelope and then swept backwards at the REAL brake
  from a tail seeded by the offline profile, which is exactly the freedom the
  fixed profile cannot express.

The solver is Gauss-Newton with a VECTORISED finite-difference Jacobian: the
whole perturbation batch is rolled out in one pass over the horizon, so a
34-variable Jacobian costs ~20 numpy calls on length-35 arrays.  Pure numpy --
the competition image has numpy 1.22 and no scipy and no CasADi (checked).

Everything after the solve -- the readout lead, `w_dcmd`, `cmd_tau`, the publish
gate, the straight-mode pure-pursuit blend, the parameter schedule -- is
`MPC3`'s, unchanged and inherited, so an A/B against mpc3 isolates the core.
"""

import math

import numpy as np

from . import vehicle
from .mpc3_core import MPC3, wrap

# --- the measured envelope -------------------------------------------------
# Refitted from the `-explore` sysid staircase (`tools/sysid_breakaway.py`):
# breakaway on this tyre is a YAW-RATE limit, not a constant-ay circle.  With
# the asked yaw rate `r = v * tan(delta) / L`:
#
#   * attached up to r = 2.63 rad/s; steady-state break from 2.79 (~1 s holds);
#   * on track, excursions above 2.71 lasting 0.19-0.29 s still deliver 0.95
#     of what is asked -- so the limit is a SOFT one with a short-excursion
#     allowance, and the right way to encode it is a cost, not a switch (the
#     surrogate agent measured that a hard 2.71 threshold inside the PLANT made
#     the harness worse: a threshold amplifies small residuals);
#   * a second ceiling, `ay = v^2 |kappa| <= ~10.5 m/s2`, binds only above
#     ~5 m/s;
#   * past breakaway the car holds only `2.30 * sqrt(v)` m/s2 whatever the
#     steering, which is what makes exceeding it expensive rather than merely
#     inaccurate.
#
# Both appear below as one-sided quadratic hinges with their own weights, so
# the solver may buy lock with speed or speed with lock, and a brief excursion
# costs something finite instead of being forbidden.
R_BREAK = 2.71                 # rad/s of asked yaw rate
AY_BREAK = 10.5                # m/s2 of asked lateral acceleration

MPCC_DEFAULTS = dict(
    # --- longitudinal plant model (the harness's identified plant) ----------
    # `dv/dt = clip(K*(g*u - v), -a_auth, a_accel) - drag*v` for u > 0, and
    # `-(brake_a0 + drag*v)` at the exact 0.0 (tools/sim_tracker.py FIT_*).
    lon_K=18.0,
    lon_g=24.6,                 # m/s of servo setpoint per unit throttle
    lon_u_max=0.60,
    lon_a_accel=5.5,            # servo accelerating authority, m/s2
    lon_drag=0.273,             # 1/s
    lon_brake_a0=4.15,          # throttle == 0.0, constant part, m/s2
    lon_v_min=0.35,
    lon_v_max=9.0,
    # --- measured envelope (soft) -------------------------------------------
    r_break=R_BREAK,            # rad/s, asked yaw rate at breakaway
    ay_break=AY_BREAK,          # m/s2, the second ceiling
    grip_use=0.95,              # fraction of both the plan may spend
    w_break=250.0,              # hinge on asked yaw rate beyond r_break
    w_ay=40.0,                  # hinge on asked ay beyond ay_break
    # --- the speed objective -----------------------------------------------
    w_v=6.0,                    # track the online ceiling
    w_v_over=150.0,             # ... asymmetric: exceeding it is much worse
    w_v_term=400.0,             # terminal set: v[N] <= the profile's reach
    w_arate=0.25,               # smoothness of the longitudinal input
    w_ajerk=0.10,
    # --- the ceiling --------------------------------------------------------
    # The backward pass inside the horizon uses the REAL brake, which is the
    # whole point; `lon_brake_plan` is how much of it the PLAN may rely on.
    lon_brake_plan=5.2,         # m/s2, between the profile's 4.6 and the
                                # measured 4.99-5.74
    v_ceiling_gain=1.00,        # scale on the envelope ceiling (1.0 = envelope)
    # Never plan more than this x the offline profile speed.  1.00 is
    # deliberate: the profile carries geometry the 1.2 s horizon cannot see --
    # the hand-held safe zones at s 17.5-21.5, 36-40 and 42-2.1 that every live
    # ay-9 rung was disqualified for removing.  MPCC's freedom is therefore
    # WHEN to brake and how to modulate, not how fast to go; at 1.25 it laps
    # 0.6 s quicker in the surrogate and collides at s 36.5-36.8, which is
    # exactly the corner the live record says is marginal.
    v_prof_cap=1.00,
    # --- solver -------------------------------------------------------------
    fd_u=0.02,                  # rad/s, steering-rate perturbation
    fd_a=0.05,                  # m/s2, acceleration perturbation
    mpcc_iters=2,
    mpcc_outer=2,
    # --- output -------------------------------------------------------------
    # How much deceleration the plan has to ask for before the exact-0.0 brake
    # is published instead of a drive throttle.  Unlike `brake_a_min` this is
    # read off the PLAN's own acceleration at the instant the command is in
    # force, not off a threshold applied to a lead in seconds.
    mpcc_brake_a=1.2,
    # The simulator's throttle is a SPEED SERVO that saturates as soon as the
    # setpoint is ~0.3 m/s off and eases as it arrives, so handing it the
    # acceleration-feasible trajectory under-drives it (`MPC3._speed_plan`
    # records the measurement: 1.25 m/s under the profile on the straight).
    # The setpoint is therefore the plan's speed plus enough reserve to keep
    # the servo saturated, and never more than the ceiling -- so a plan that
    # deliberately backs off from the ceiling by more than the reserve (the
    # joint trade against the corridor) is still seen by the throttle.
    mpcc_servo_reserve=1.0,
    # ... and faded to zero as the curvature UNDER THE CAR rises to this, for
    # the same measured reason `servo_head_kappa` exists in `ServoThrottle`:
    # running wide tracks EXIT ACCELERATION, not apex speed (finals, Sep 19:
    # the ungated servo took 6 collisions in 100 Xvfb laps, all 0.3 m wide on a
    # corner exit).  Measured here too: without it, 3 collisions per 200
    # surrogate laps at 0.058 and 2 at 0.098, at the last hairpin's exit
    # (s 0.9-3.2) and the tightest corner (s 24-25).
    # DEFAULT OFF, and measured to be a trap: unlike `servo_head_kappa`, which
    # fades only the EXTRA head on top of the setpoint, this fades the reserve
    # that keeps the servo saturated at all, so at 0.4 the car sits under plan
    # through every corner -- 14.40 s/lap against 11.06 over 48 laps. The
    # curvature cap below is the version of this idea that works.
    mpcc_res_kappa=0.0,
    # Hard cap on the setpoint by the MEASURED envelope at the curvature the
    # car is at RIGHT NOW, not at the lead point.  On a hairpin exit the line's
    # curvature falls away within one lead distance while the car is still
    # turning, so a setpoint read ahead is a licence to accelerate mid-corner.
    mpcc_cap_now=1,
    # DEFAULT OFF and CLOSED.  The idea was to cap by the worst curvature over
    # this much line BEHIND the car as well as under it, on the theory that at
    # the last hairpin's exit the line has straightened but the car has not.
    # It is a third trap of the same family as `mpcc_res_kappa`: every value
    # over-caps the whole lap.  96 laps per cell against the 10.94 / 11.11 the
    # default gives -- 0.8 m: 11.330 (0.058) and 11.520 with a collision
    # (0.098); 1.5 m: 11.760 and 12.070; 2.5 m: 12.550 with a collision and
    # 12.860.  Do not re-open without a per-zone version.
    mpcc_cap_back_m=0.0,
    # Combined slip on the ACCELERATING side of the longitudinal input (0 = off,
    # 1 = the full friction ellipse against `grip_use * ay_break`).  See
    # `_lon_bounds`.
    mpcc_ellipse=1.0,
)


def envelope_speed(kappa, r_break=R_BREAK, ay_break=AY_BREAK, use=1.0,
                   v_max=9.0):
    """Fastest speed at which `kappa` is still inside the measured envelope.

    `v <= r_break / |kappa|` (yaw rate) and `v <= sqrt(ay_break / |kappa|)`
    (lateral acceleration).  At a hairpin's 1.6 1/m the yaw-rate limit gives
    1.7 m/s and the ay one 2.5, so the yaw rate is what caps the hairpins; on a
    0.3 1/m sweep it is 9.0 against 5.9, so there the ay ceiling binds -- which
    is exactly what the staircase says.
    """
    k = np.maximum(np.abs(kappa), 1e-3)
    return np.minimum(v_max, np.minimum(use * r_break / k,
                                        np.sqrt(use * ay_break / k)))


class MPCC(MPC3):
    """`MPC3` with the speed promoted from a plan to a decision variable.

    Inherits the reference path, the corridor bounds, the rate-adaptivity
    interface (`period`, `set_rate`, `steer_tau_eff`), the parameter schedule
    and every output stage.  Only the core solve is replaced.
    """

    def __init__(self, path, **cfg):
        base = dict(MPCC_DEFAULTS)
        base.update({k: v for k, v in cfg.items() if v is not None})
        super().__init__(path, **base)
        nf = max(1, self.N - self.nd)
        self.a = np.zeros(nf)         # warm start, longitudinal input
        self.a_hold = 0.0             # acceleration the in-flight throttle gives
        self.v_traj = None            # last optimised speed trajectory
        self.a_traj = None
        self.v_tgt = None             # the ceiling it was judged against
        self.last_brake = False

    # ------------------------------------------------------------------
    def _lon_bounds(self, v, ay=None):
        c = self.cfg
        a_hi = np.minimum(c['lon_a_accel'],
                          c['lon_K'] * (c['lon_g'] * c['lon_u_max'] - v))
        a_hi = a_hi - c['lon_drag'] * v
        # COMBINED SLIP on the accelerating side only.  Every live failure on
        # this track is the car running wide on a corner EXIT under
        # acceleration, and `docs/FINALS_TUNING` §5.2 says it tracks exit
        # ACCELERATION rather than apex speed.  A tyre that is already spending
        # `ay` of its envelope on turning has `sqrt(1 - (ay/ay_max)^2)` of the
        # longitudinal axis left, so the plan is not allowed to ask for more.
        # Deliberately one-sided: the braking side is left alone, because
        # entering a corner slower is never the failure here.
        el = float(c.get('mpcc_ellipse', 0.0) or 0.0)
        if el > 0.0 and ay is not None:
            f = np.sqrt(np.maximum(
                0.0, 1.0 - (ay / (float(c['grip_use'])
                                  * float(c['ay_break']))) ** 2))
            a_hi = a_hi * (1.0 - el + el * f)
        a_lo = -(c['lon_brake_a0'] + c['lon_drag'] * v)
        return a_lo, np.maximum(a_hi, a_lo + 1e-3)

    def _ceiling(self, idx, ds, k_excess=None):
        """The speed the horizon is allowed to hold, step by step.

        Envelope first (grip map against the REFERENCE curvature), then the
        offline profile as a bounded cap, then a backward sweep at the real
        brake from a tail seeded by the profile -- the terminal set.
        """
        c = self.cfg
        P = self.path
        N = self.N
        k_ref = np.abs(P.kappa[idx]).astype(float)
        if k_excess is not None:
            # A car that is running wide has to turn HARDER than the line does,
            # and must pay for it in speed -- the same bounded-excess rule
            # `MPC3._speed_plan` uses, which is what stops a corner EXIT being
            # planned at the reference's curvature while the car is still
            # correcting.  Bounded by `k_excess_max` so one solver transient
            # cannot collapse the plan.
            k_ref = k_ref + np.clip(k_excess - k_ref, 0.0,
                                    float(c['k_excess_max']))
        v_prof = P.v_prof[idx].astype(float)
        vsc = self._sched_vscale(P.s[idx])
        if vsc is not None:
            v_prof = v_prof * vsc
        v = envelope_speed(k_ref, float(c['r_break']), float(c['ay_break']),
                           float(c['grip_use']), float(c['lon_v_max']))
        v = np.minimum(v * float(c['v_ceiling_gain']),
                       v_prof * float(c['v_prof_cap']))
        # Terminal set: beyond the horizon only the offline profile is known,
        # so the last step must still be able to meet it.
        tail = idx[N]
        n_tail = max(2, int(6.0 / max(P.spacing, 1e-3)))
        j = (tail + np.arange(n_tail)) % P.n
        v_tail = P.v_prof[j].astype(float)
        vst = self._sched_vscale(P.s[j])
        if vst is not None:
            v_tail = v_tail * vst
        ab = float(c['lon_brake_plan'])
        v_term = float(np.sqrt(v_tail ** 2
                               + 2.0 * ab * np.arange(n_tail) * P.spacing).min())
        v[N] = min(v[N], v_term)
        for k in range(N - 1, -1, -1):
            v[k] = min(v[k], math.sqrt(v[k + 1] ** 2 + 2.0 * ab * ds[k]))
        return np.maximum(v, float(c['lon_v_min'])), v_term

    # ------------------------------------------------------------------
    def _look(self, i0, S):
        """Reference curvature and corridor at arc length `S` ahead of `i0`.

        LINEARLY interpolated, not nearest-cell: `S` is a function of the
        decision variables, and a step function of it has no usable
        finite-difference gradient (a 0.05 m/s2 perturbation moves the arc by
        millimetres, i.e. by less than one 0.05 m cell, so a nearest lookup
        would report exactly zero sensitivity almost everywhere and jump by a
        whole cell occasionally).
        """
        P = self.path
        f = float(i0) + S / max(P.spacing, 1e-3)
        j0 = np.floor(f).astype(np.int64)
        fr = f - j0
        j0 %= P.n
        j1 = (j0 + 1) % P.n
        kap = P.kappa[j0] * (1.0 - fr) + P.kappa[j1] * fr
        if not P.has_corridor:
            inf = np.full(kap.shape, np.inf)
            return kap, -inf, inf
        hw = 0.5 * vehicle.CAR_WIDTH + self.cfg['barrier_margin']
        wl = P.w_left[j0] * (1.0 - fr) + P.w_left[j1] * fr
        wr = P.w_right[j0] * (1.0 - fr) + P.w_right[j1] * fr
        return kap, -(wr - hw), wl - hw

    def _roll(self, U, A, z0, i0, cap, committed):
        """Batched rollout.  `U`, `A` are (nf, B); returns (N+1, B) states.

        Unlike `MPC3._rollout`, the arc step is `v * dt` with `v` a DECISION
        variable and the reference is looked up at the arc length the plan
        actually reaches.  That is the whole point of the candidate: with a
        frozen arc grid the corridor barrier has no gradient with respect to
        the speed at all, so the solver cannot answer "I am about to run out of
        road" with "then go slower" -- which is what `mpc3` has to approximate
        with the `e_free`/`k_e` heuristic cut.
        """
        c = self.cfg
        N, nd = self.N, self.nd
        B = U.shape[1]
        g, knee, ayd = c['yaw_gain'], c['sat_knee'], c['ay_deliver']
        L = vehicle.WHEELBASE
        dt = self.dt
        E = np.empty((N + 1, B))
        PSI = np.empty((N + 1, B))
        D = np.empty((N + 1, B))
        V = np.empty((N + 1, B))
        E[0] = z0[0]
        PSI[0] = z0[1]
        D[0] = z0[2]
        V[0] = z0[3]
        tau_a = self.steer_tau_eff
        tau_c = float(c['cmd_tau']) if float(c['model_cmd_tau']) else 0.0
        lag_on = tau_a > 0.0 or tau_c > 0.0
        Ea = math.exp(-dt / tau_a) if tau_a > 0.0 else 0.0
        Ec = math.exp(-dt / tau_c) if tau_c > 0.0 else 0.0
        DC = np.empty((N + 1, B)) if lag_on else None
        FL = np.empty((N + 1, B)) if lag_on else None
        KAP = np.empty((N, B))
        LO = np.empty((N, B))
        HI = np.empty((N, B))
        S = np.zeros(B)
        min_ds = float(c['min_step_m'])
        # One table lookup per step, not two: the lookup at the END of step k
        # supplies both that step's corridor and step k+1's curvature.
        kap_k = self._look(i0, S)[0]
        for k in range(N):
            if lag_on and k == nd:
                h_raw = (self.last_cmd_raw if self.last_cmd_raw is not None
                         else (self.last_cmd if self.last_cmd is not None
                               else float(D[nd].mean())))
                h_f = self.last_cmd if self.last_cmd is not None else h_raw
                DC[:nd + 1] = h_raw
                FL[:nd + 1] = h_f
            ek, pk, dk, vk = E[k], PSI[k], D[k], V[k]
            den = np.maximum(1.0 - kap_k * ek, 0.2)
            sp, cp = np.sin(pk), np.cos(pk)
            # Delivered curvature, the same saturating law MPC3 uses, batched.
            k_lin = np.tan(dk) / L
            k_max = ayd / np.maximum(vk * vk, 0.25)
            a_abs = np.abs(k_lin)
            kn = knee * k_max
            if np.any(a_abs > kn):
                span = np.maximum(k_max - kn, 1e-6)
                f = np.where(a_abs > kn,
                             kn + span * (1.0 - np.exp(-(a_abs - kn) / span)),
                             a_abs)
                kach = g * np.copysign(f, k_lin)
            else:
                kach = g * k_lin
            if k < nd:
                a_look = self.a_hold
            else:
                ay_k = vk * vk * np.abs(k_lin)
                lo_a, hi_a = self._lon_bounds(vk, ay_k)
                a_look = np.clip(A[k - nd], lo_a, hi_a)
            v_next = np.clip(vk + a_look * dt, c['lon_v_min'], c['lon_v_max'])
            ds_k = np.maximum(0.5 * (vk + v_next) * dt, min_ds)
            E[k + 1] = ek + sp * ds_k
            PSI[k + 1] = wrap(pk + (kach - kap_k * cp / den) * ds_k)
            KAP[k] = kap_k
            S = S + ds_k
            kap_k, LO[k], HI[k] = self._look(i0, S)
            V[k + 1] = v_next
            if k < nd:
                D[k + 1] = committed[k] if committed is not None else dk
            elif lag_on:
                dcn = np.clip(DC[k] + U[k - nd] * dt, -cap, cap)
                DC[k + 1] = dcn
                sl = (dcn - DC[k]) / dt
                fn = (dcn - sl * tau_c + (FL[k] - DC[k] + sl * tau_c) * Ec
                      if tau_c > 0.0 else dcn)
                FL[k + 1] = fn
                if tau_a > 0.0:
                    slf = (fn - FL[k]) / dt
                    an = fn - slf * tau_a + (dk - FL[k] + slf * tau_a) * Ea
                else:
                    an = fn
                D[k + 1] = np.clip(an, -cap, cap)
            else:
                D[k + 1] = np.clip(dk + U[k - nd] * dt, -cap, cap)
        self.k_cmd = np.abs(np.tan(D[:N, 0])) / L
        return E, PSI, D, V, (DC if lag_on else D), KAP, LO, HI

    # ------------------------------------------------------------------
    def _resid(self, U, A, E, PSI, D, V, lo, hi, v_tgt, v_term, pub):
        """Weighted residual stack, shape (n_res, B)."""
        c = self.cfg
        N, nd = self.N, self.nd
        nf = N - nd
        L = vehicle.WHEELBASE
        wl_s, wy_s = self._wscale
        sw_lat = np.full(N, math.sqrt(c['w_lat'] * wl_s))
        sw_yaw = np.full(N, math.sqrt(c['w_yaw'] * wy_s))
        sw_lat[-1] = math.sqrt((c['w_lat'] + c['w_lat_T']) * wl_s)
        sw_yaw[-1] = math.sqrt((c['w_yaw'] + c['w_yaw_T']) * wy_s)
        sw_bar = math.sqrt(c['w_barrier'])
        e1 = E[1:]
        v1 = V[1:]
        d1 = D[1:]
        e_dead = float(c['e_dead'])
        ee = e1
        if e_dead > 0.0:
            ee = np.where(e1 > e_dead, e1 - e_dead,
                          np.where(e1 < -e_dead, e1 + e_dead, 0.0))
        rows = [sw_lat[:, None] * ee,
                sw_yaw[:, None] * PSI[1:],
                math.sqrt(c['w_steer']) * d1,
                sw_bar * np.maximum(0.0, e1 - hi),
                sw_bar * np.maximum(0.0, lo - e1)]
        # --- longitudinal -------------------------------------------------
        tgt = v_tgt[1:, None]
        rows.append(math.sqrt(c['w_v']) * (v1 - tgt))
        rows.append(math.sqrt(c['w_v_over']) * np.maximum(0.0, v1 - tgt))
        # --- the measured envelope, which is what couples the two ----------
        # SOFT hinges, deliberately: the staircase says a 0.2-0.3 s excursion
        # past 2.71 rad/s still delivers 0.95 of what is asked, so a hard
        # threshold here would refuse manoeuvres the car can actually make.
        use = float(c['grip_use'])
        k_ask = np.abs(np.tan(d1)) / L
        r_ask = v1 * k_ask
        rows.append(math.sqrt(c['w_break'])
                    * np.maximum(0.0, r_ask - use * float(c['r_break'])))
        ay = v1 * r_ask
        rows.append(math.sqrt(c['w_ay'])
                    * np.maximum(0.0, ay - use * float(c['ay_break'])) / 10.0)
        # --- input regularisation ------------------------------------------
        dt = self.dt
        rows.append(math.sqrt(c['w_rate']) * U * dt)
        du = U - np.concatenate([U[:1], U[:-1]], axis=0)
        rows.append(math.sqrt(c['w_jerk']) * du * dt)
        rows.append(math.sqrt(c['w_arate']) * A * dt)
        da = A - np.concatenate([A[:1], A[:-1]], axis=0)
        rows.append(math.sqrt(c['w_ajerk']) * da * dt)
        # --- terminal set ---------------------------------------------------
        rows.append((math.sqrt(c['w_v_term'])
                     * np.maximum(0.0, V[N] - v_term))[None, :])
        # --- move suppression across frames (the shipping `w_dcmd`) ---------
        wdc = float(c['w_dcmd'])
        if wdc > 0.0 and self.last_cmd is not None:
            ka, fr = self._apply
            da_ = (pub[N] if ka >= N
                   else pub[ka] + fr * (pub[ka + 1] - pub[ka]))
            rows.append((math.sqrt(wdc) * (da_ - self.last_cmd))[None, :])
        _ = nf
        return np.concatenate(rows, axis=0)

    # ------------------------------------------------------------------
    def _solve(self, e_lat, e_yaw, delta0, i0, v0, cap=None, committed=None):
        c = self.cfg
        N, nd = self.N, self.nd
        nf = N - nd
        if not all(math.isfinite(float(q))
                   for q in (e_lat, e_yaw, delta0, v0)):
            raise ValueError(f'non-finite state: e_lat={e_lat} e_yaw={e_yaw} '
                             f'delta0={delta0} v0={v0}')
        if committed is not None and not np.isfinite(
                np.asarray(committed, dtype=float)).all():
            raise ValueError(f'non-finite committed steering: {committed}')
        cap = vehicle.MAX_STEER if cap is None else min(cap, vehicle.MAX_STEER)
        rate = float(c['steer_rate'])
        z0 = (float(e_lat), float(wrap(e_yaw)),
              float(np.clip(delta0, -vehicle.MAX_STEER, vehicle.MAX_STEER)),
              max(float(v0), float(c['lon_v_min'])))
        if committed is not None:
            committed = np.clip(np.asarray(committed, dtype=float)[:nd],
                                -vehicle.MAX_STEER, vehicle.MAX_STEER)

        # warm start, re-sliced if `nd` moved
        u = np.clip(np.roll(self.u, -1)[:nf], -rate, rate)
        if nf > 1:
            u[-1] = u[-2]
        a = np.asarray(self.a, float)
        a = (np.roll(a, -1)[:nf] if len(a) >= nf
             else np.concatenate([a, np.zeros(nf - len(a))]))
        if nf > 1:
            a[-1] = a[-2]

        # the straight gate and the readout index: MPC3's, verbatim
        gate = 0.0
        wls = float(c.get('straight_w_lat_scale', 1.0) or 1.0)
        wys = float(c.get('straight_w_yaw_scale', 1.0) or 1.0)
        if (float(c.get('weave_notch_f', 0.0) or 0.0) > 0.0
                or float(c.get('weave_notch_k', 0.0) or 0.0) > 0.0
                or float(c.get('straight_hf_gain', 1.0) or 1.0) != 1.0
                or float(c.get('straight_lead_scale', 1.0) or 1.0) != 1.0
                or wls != 1.0 or wys != 1.0):
            gate = self._straight_gate(i0, e_lat)
        self.last_gate = gate
        self._wscale = (1.0 + gate * (wls - 1.0), 1.0 + gate * (wys - 1.0))
        _f = nd + max(float(c['apply_floor_steps']),
                      float(c['apply_ahead_s']) / self.dt)
        ls = float(c.get('straight_lead_scale', 1.0) or 1.0)
        if gate > 0.0 and ls != 1.0:
            _f = max(nd + (1.0 + gate * (ls - 1.0)) * (_f - nd), float(nd))
        _k = min(int(_f), N)
        self._apply = (_k, _f - _k)

        fdu, fda = float(c['fd_u']), float(c['fd_a'])
        nv = 2 * nf
        cost = float('inf')
        E = PSI = D = V = PUB = KAP = LO = HI = None
        idx = ds = None
        v_tgt = None
        v_term = 0.0
        for _ in range(max(1, int(c['mpcc_outer']))):
            # The arc steps and the reference come from the CURRENT speed
            # trajectory; that is the alternation MPC3 already runs, and it is
            # what keeps the lateral rollout on a frozen grid inside one solve.
            self.v_plan = np.concatenate([[z0[3]], np.full(N, z0[3])]) \
                if self.v_traj is None else self.v_traj
            idx, ds = self._indices(i0, v0)
            kx = None
            if self.k_cmd is not None and len(self.k_cmd) == N:
                kx = np.concatenate([np.abs(self.k_cmd), [abs(self.k_cmd[-1])]])
                kx[:nd] = 0.0      # committed steering is history, not a demand
            v_tgt, v_term = self._ceiling(idx, ds, kx)

            for _it in range(max(1, int(c['mpcc_iters']))):
                # one nominal + 2*nf perturbed rollouts, in a single pass
                U = np.repeat(u[:, None], nv + 1, axis=1)
                A = np.repeat(a[:, None], nv + 1, axis=1)
                for i in range(nf):
                    U[i, 1 + i] += fdu
                    A[i, 1 + nf + i] += fda
                E, PSI, D, V, PUB, KAP, LO, HI = self._roll(
                    U, A, z0, i0, cap, committed)
                R = self._resid(U, A, E, PSI, D, V, LO, HI, v_tgt, v_term, PUB)
                r0 = R[:, 0]
                J = np.empty((R.shape[0], nv))
                J[:, :nf] = (R[:, 1:1 + nf] - r0[:, None]) / fdu
                J[:, nf:] = (R[:, 1 + nf:] - r0[:, None]) / fda
                cost = float(r0 @ r0)
                H = J.T @ J
                gr = J.T @ r0
                dg = np.diag(H).copy()
                dg[dg < 1e-9] = 1e-9
                lam = float(c['lam0'])
                ok = False
                step = np.zeros(nv)
                for _try in range(6):
                    try:
                        step = np.linalg.solve(H + lam * np.diag(dg), -gr)
                    except np.linalg.LinAlgError:
                        lam *= 10.0
                        continue
                    ut = np.clip(u + step[:nf], -rate, rate)
                    at = a + step[nf:]
                    E2, P2, D2, V2, PB2, _K2, LO2, HI2 = self._roll(
                        ut[:, None], at[:, None], z0, i0, cap, committed)
                    r2 = self._resid(ut[:, None], at[:, None], E2, P2, D2, V2,
                                     LO2, HI2, v_tgt, v_term, PB2)[:, 0]
                    c2 = float(r2 @ r2)
                    if c2 < cost:
                        u, a, cost, ok = ut, at, c2, True
                        break
                    lam *= 6.0
                if not ok or float(np.max(np.abs(step))) < 1e-3:
                    break
            E, PSI, D, V, PUB, KAP, LO, HI = self._roll(
                u[:, None], a[:, None], z0, i0, cap, committed)
            self.e_pred = E[:, 0]
            self.v_traj = V[:, 0].copy()
            self.a_traj = np.concatenate([np.full(nd, self.a_hold), a])

        self.u = np.concatenate([u, np.full(nd, u[-1] if nf else 0.0)])
        self.a = a
        self.last_cost = cost
        self.last_iters = 1
        self.v_plan = self.v_traj
        self.v_ceiling = v_tgt
        self.v_ceiling_idx = idx
        self.v_tgt = v_tgt

        pub = PUB[:, 0]
        k, frac = self._apply
        cmd = pub[N] if k >= N else pub[k] + frac * (pub[k + 1] - pub[k])
        cmd = float(np.clip(cmd, -cap, cap))
        cmd = self._output_stages(cmd, cap, i0, e_lat, e_yaw, v0, gate)

        # --- what the throttle is asked for --------------------------------
        # Read the plan at the instant the command is actually in force.  This
        # is the ONE place the fixed `thr_brake_lead_s` heuristic is replaced:
        # the brake decision comes from the plan's own acceleration there.
        lead_s = float(c['thr_lead_s'])
        fk = min(lead_s / self.dt, N - 1e-6)
        ki = int(fk)
        v_plan_at = float(self.v_traj[ki]
                          + (fk - ki) * (self.v_traj[ki + 1] - self.v_traj[ki]))
        v_ceil_at = float(v_tgt[ki] + (fk - ki) * (v_tgt[ki + 1] - v_tgt[ki]))
        ja = min(max(ki, nd), N - 1)
        a_cmd = float(self.a_traj[ja])
        # ONE-SIDED.  The reserve exists to keep the speed servo saturated
        # while the plan is accelerating; added on a braking approach it is a
        # command to go FASTER than the plan wants, and the simulator has no
        # proportional brake to take it back.  Measured: two-sided, this
        # collides 4-6 times per 24 laps at s 24 (the tightest corner) at the
        # organisers' period.
        res = float(c['mpcc_servo_reserve']) * max(0.0, min(1.0, a_cmd / 0.5))
        bk = float(c.get('mpcc_cap_back_m', 0.0) or 0.0)
        if bk > 0.0:
            nb = max(1, int(round(bk / max(self.path.spacing, 1e-3))))
            jb = (int(i0) - np.arange(nb + 1)) % self.path.n
            k_now = float(np.abs(self.path.kappa[jb]).max())
        else:
            k_now = abs(float(self.path.kappa[i0]))
        rk = float(c.get('mpcc_res_kappa', 0.0) or 0.0)
        if rk > 0.0:
            res *= max(0.0, 1.0 - k_now / rk)
        v_cmd = min(v_ceil_at, v_plan_at + res)
        if int(c.get('mpcc_cap_now', 0) or 0):
            v_cmd = min(v_cmd, float(envelope_speed(
                k_now, float(c['r_break']), float(c['ay_break']),
                float(c['grip_use']), float(c['lon_v_max']))))
        brake = bool(a_cmd < -float(c['mpcc_brake_a']))
        self.last_brake = brake
        info = dict(cost=cost, iters=1, delta=D[:, 0], e=E[:, 0],
                    psi=PSI[:, 0], cmd_raw=self.last_cmd_raw,
                    v=self.v_traj, v_ceiling=v_tgt, v_cmd=v_cmd,
                    v_plan_at=v_plan_at, v_ceil_at=v_ceil_at,
                    a_cmd=a_cmd, brake=brake, v_term=v_term,
                    lo=LO[:, 0], hi=HI[:, 0], kap=KAP[:, 0], idx=idx, ds=ds,
                    u=u.copy())
        return cmd, info

    # ------------------------------------------------------------------
    def _output_stages(self, cmd, cap, i0, e_lat, e_yaw, v0, gate):
        """`MPC3._solve`'s publish chain, so the A/B isolates the core.

        Straight-mode pure pursuit, the no-anticipation publish gate,
        `cmd_notch` / `cmd_tau`, and the straight-gated weave filter -- in the
        same order and with the same parameters.
        """
        c = self.cfg
        P = self.path
        spk = float(c.get('straight_pp_k', 0.0) or 0.0)
        ppm = float(c.get('straight_pp_period_max', 0.0) or 0.0)
        spk_g = (min(1.0, max(0.0, (ppm - float(self.period)) / 0.010))
                 if (spk > 0.0 and ppm > 0.0) else 1.0)
        if spk > 0.0 and spk_g > 0.0:
            gpp = self._straight_gate(i0, e_lat) * spk_g
            if gpp > 0.0:
                vv = max(float(v0), 0.5)
                Ld = min(max(spk * vv, float(c['straight_pp_ld_min'])),
                         float(c['straight_pp_ld_max']))
                Td = (float(c['straight_pp_td'])
                      + float(c.get('straight_pp_tk', 0.0) or 0.0)
                      * max(float(self.period), 1e-3))
                e_p = float(e_lat) + vv * Td * math.sin(float(e_yaw))
                alpha = -float(e_yaw) - math.atan2(e_p, Ld)
                jm = (int(i0)
                      + int(round(0.5 * Ld / max(P.spacing, 1e-3)))) % P.n
                pp = (math.atan(2.0 * vehicle.WHEELBASE * math.sin(alpha) / Ld)
                      + math.atan(vehicle.WHEELBASE * float(P.kappa[jm])))
                cmd = float(np.clip((1.0 - gpp) * cmd + gpp * pp, -cap, cap))
        gc = float(c.get('pub_gate_corr', 0.0) or 0.0)
        if gc > 0.0:
            look = max(1, int(round(float(c['pub_gate_d'])
                                    / max(P.spacing, 1e-3))))
            j = (int(i0) + np.arange(look + 1)) % P.n
            k_ahead = float(np.abs(P.kappa[j]).max())
            if (k_ahead < float(c['pub_gate_kappa'])
                    and abs(float(e_lat)) <= float(c['pub_gate_e'])):
                lim = abs(math.atan(vehicle.WHEELBASE * k_ahead)) + gc
                cmd = float(np.clip(cmd, -lim, lim))
        self.last_cmd_raw = cmd
        if self.last_cmd is not None:
            b = float(c['cmd_notch'])
            if b > 0.0:
                cmd = (1.0 - b) * cmd + b * self.last_cmd
            tau = float(c['cmd_tau'])
            if tau > 0.0:
                al = 1.0 - math.exp(-max(float(self.period), 1e-3) / tau)
                cmd = self.last_cmd + al * (cmd - self.last_cmd)
            cmd = float(np.clip(cmd, -cap, cap))
        if (float(c.get('weave_notch_f', 0.0) or 0.0) > 0.0
                or float(c.get('weave_notch_k', 0.0) or 0.0) > 0.0
                or float(c.get('straight_hf_gain', 1.0) or 1.0) != 1.0):
            cmd = float(np.clip(self._weave_filter(cmd, gate), -cap, cap))
        self.last_cmd = cmd
        return cmd

    # ------------------------------------------------------------------
    def note_throttle(self, u_cmd, v_meas):
        """Tell the solver what the throttle it just published will do.

        The first `nd` horizon steps are in flight longitudinally exactly as
        the steering is, so the rollout has to hold the acceleration the
        published command produces rather than pretend the plan starts now.
        """
        c = self.cfg
        v = max(float(v_meas), 0.1)
        if float(u_cmd) == 0.0:
            self.a_hold = -(c['lon_brake_a0'] + c['lon_drag'] * v)
        else:
            a = c['lon_K'] * (c['lon_g'] * float(u_cmd) - v)
            lo, hi = self._lon_bounds(np.array([v]))
            self.a_hold = float(np.clip(a - c['lon_drag'] * v, lo[0], hi[0]))
