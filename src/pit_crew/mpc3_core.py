"""Pure-python core of the mpc3 tracker: a speed-aware Gauss-Newton horizon.

ROS-free on purpose, exactly like ``mpc2_core``: ``tools/sim_tracker.py`` drives
these same objects, so what is measured offline is the law that runs live.

What mpc3 keeps from mpc2, because it was measured to work
---------------------------------------------------------
``RefPath``   (raceline, smoothed curvature, corridor half-widths marched along
              the path normal from ``pf_core.load_map``'s distance field),
``Actuator``  (command history + ``/autodrive/roboracer_1/steering`` feedback,
              0.135 s dead time, 3.2 rad/s slew, interpolated across the samples
              the feedback covers), ``PoseFilter`` and ``wrap`` are imported
              from ``mpc2_core`` unchanged.  They are not re-derived here, and
              mpc2_core is not edited.

What is new, and why
--------------------
1. **Speed is part of the state.**  Every tracker before this one drove a fixed
   offline velocity profile: the steering solver could see that it was 0.25 m
   wide at a hairpin exit and could do nothing about the fact that it was also
   going too fast to rejoin.  That is the mechanism behind every s 13.6 / s 27.7
   collision in ``docs/STATUS.md``.  Here the horizon carries ``v`` and the
   speed the throttle is asked for comes out of the same plan the steering does:

   * the profile speed is the ceiling, never the floor;
   * it is cut by the *curvature the plan is actually asking for* --
     ``v <= sqrt(ay_budget / |kappa_cmd|)`` -- so running wide (which needs more
     curvature to rejoin) automatically buys less speed;
   * it is cut again in proportion to the predicted lateral error itself;
   * and the result is made feasible with a backward brake pass and a forward
     acceleration pass at the measured servo authority, so the plan is a speed
     trajectory the car can really follow rather than a wish.

2. **The lateral model is the measured one.**  Yaw rate is
   ``0.956 * v * tan(delta) / L`` (measured this session at every steering
   angle), and asked curvature above ``ay_deliver / v^2`` is not delivered --
   a smooth knee consistent with the skidpad, where the car holds ~0.9 of asked
   curvature to 9.9 m/s2 and then lets go.  This is a *saturation*, not the
   static understeer table CLAUDE.md forbids in a rollout: below the knee it is
   the identity, so it can never tell the controller to wind on more lock in the
   linear region.  Above the knee it makes extra lock worthless, which is the
   only thing that stops the solver asking for it.

3. **The dead time is handled in time, not by moving the pose.**  The first
   ``round(delay/dt)`` steps of the horizon are *committed*: their steering is
   whatever the already-published commands will produce, replayed through
   ``Actuator``.  The decision variables start where the new command can first
   bite, and the reference they are judged against is the reference at that
   TIME along the path.  Nothing is shifted by hand.

4. **No cut term.**  mpc2 needed ``cut_offset_m`` to match the legacy tracker's
   lap time because the legacy tracker cut corners by 0.139 m and the safe
   centreline did not describe that shorter path.  Tuning it is tuning a
   geometry knob inside a tracker, and the offline optimum (0.27) drove to the
   clearance floor and collided live.  mpc3 tracks the line it is given,
   honestly, and the geometry is the *line's* job -- run it on a min-curvature
   line if a shorter path is wanted.
"""

import math

import numpy as np

try:
    from pit_crew import vehicle
    from pit_crew.mpc2_core import Actuator, PoseFilter, RefPath, wrap
except ImportError:                                  # running from src/pit_crew
    import vehicle
    from mpc2_core import Actuator, PoseFilter, RefPath, wrap

__all__ = ['Actuator', 'PoseFilter', 'RefPath', 'wrap', 'MPC3', 'DEFAULTS']


DEFAULTS = dict(
    # horizon ---------------------------------------------------------------
    horizon=20,
    step_dt=0.060,            # 1.20 s of preview
    min_step_m=0.10,          # never preview less than this per step
    # actuator --------------------------------------------------------------
    steer_rate=vehicle.STEER_RATE,
    delay_s=0.180,            # 0.135 dead time + 0.045 estimation, measured
    apply_ahead_s=0.090,      # which plan point the actuator is handed
    # measured lateral model ------------------------------------------------
    yaw_gain=0.956,           # measured yaw rate / (v tan(delta)/L)
    ay_deliver=9.5,           # asked curvature above this/v^2 is not delivered
    sat_knee=0.80,            # fraction of it where the knee starts
    # speed plan ------------------------------------------------------------
    ay_budget=7.5,            # grip the SPEED plan may spend on curvature
    k_excess_max=0.15,        # 1/m of CORRECTION curvature the speed cut may see
    a_brake=4.0,              # m/s2 the plan may ask the servo to shed
    a_accel=5.0,              # ...and to add (measured servo authority)
    # `e_free` MUST sit above the tracker's own normal error or the emergency
    # brake becomes a permanent tax.  At 0.05 it sat BELOW the live median
    # (0.072 m, p95 0.179), so the cut was active essentially all the time:
    # measured by replaying the 100-lap live log through this solver, it alone
    # held the speed ceiling 0.188 m/s under the profile for the whole lap --
    # every other term contributed 0.000.  0.18 is just above the measured live
    # p95, so it is silent in normal running, and `k_e` is doubled so that it
    # still bites hard when the car is genuinely off line.
    e_free=0.18,              # lateral error tolerated before slowing, m
    k_e=2.0,                  # speed cut per metre of error beyond e_free
    v_mod_min=0.55,           # never cut the profile speed below this fraction
    thr_lead_s=0.225,         # plan speed at the time the command is in force
    # cost ------------------------------------------------------------------
    w_lat=24.0,
    w_yaw=3.0,
    w_steer=0.30,
    w_rate=0.60,              # on the applied steering change per step
    w_jerk=0.05,
    w_lat_T=40.0,
    w_yaw_T=8.0,
    w_barrier=400.0,
    barrier_margin=0.24,      # corridor the barrier defends, beyond the body.
                              # Raised with the speed: a faster car spends more
                              # of the corridor, and 0.16 put the offline worst
                              # footprint clearance at 0.100 m on seed 3.
    # solver ----------------------------------------------------------------
    iters=6,
    outer=2,                  # speed-plan / steering-plan fixed point passes
    lam0=1e-3,
)


def _sat_curvature(delta, k_max, knee, gain):
    """Delivered curvature and its derivative w.r.t. the steering angle.

    Identity (times ``gain``) until ``knee * k_max``, then an exponential
    approach to ``k_max``.  Smooth, monotone, and with a derivative that decays
    to zero -- which is what makes extra lock worthless to the optimiser instead
    of merely expensive.
    """
    k = math.tan(delta) / vehicle.WHEELBASE
    dk = (1.0 + math.tan(delta) ** 2) / vehicle.WHEELBASE
    a = abs(k)
    kn = knee * k_max
    if k_max <= 0.0 or a <= kn:
        return gain * k, gain * dk
    span = max(k_max - kn, 1e-6)
    e = math.exp(-(a - kn) / span)
    f = kn + span * (1.0 - e)
    return gain * math.copysign(f, k), gain * dk * e


class MPC3:
    """Gauss-Newton over a steering-rate sequence, with a coupled speed plan.

    Decision variable ``u`` is the steering RATE at each *free* step, boxed at
    the actuator's 3.2 rad/s.  State is ``(e_lat, e_yaw, delta)`` in the
    path-relative frame plus the speed ``v``, which is planned rather than
    optimised:

        e_lat'  = sin(e_yaw)                                    (per metre)
        e_yaw'  = g*sat(tan(delta)/L) - k_ref cos(e_yaw)/(1 - k_ref e_lat)
        delta'  = u dt                       |u| <= 3.2, |delta| <= cap

    The speed enters through the arc length of each step and through the grip
    saturation, and is held fixed inside one Gauss-Newton solve so the Jacobian
    stays the cheap 3-state one.  ``outer`` passes alternate
    speed-plan -> steering-plan, which is a fixed point that converges in two
    (measured: the third pass moves the first command by < 1e-3 rad).
    """

    def __init__(self, path: RefPath, **cfg):
        self.path = path
        p = dict(DEFAULTS)
        p.update({k: v for k, v in cfg.items() if v is not None})
        self.cfg = p
        self.N = int(p['horizon'])
        self.dt = float(p['step_dt'])
        self.nd = max(0, min(self.N - 2, int(round(p['delay_s'] / self.dt))))
        self.u = np.zeros(self.N)
        self.v_plan = None            # last speed plan, warm start for ds
        self.v_ceiling = None         # the ceiling the throttle is handed
        self.e_pred = np.zeros(self.N + 1)
        self.k_cmd = np.zeros(self.N)
        self.last_cost = float('nan')
        self.last_iters = 0

    # ..................................................................
    def _indices(self, i0, v):
        """Reference indices and arc lengths, from the warm speed plan."""
        P = self.path
        N = self.N
        if self.v_plan is None or len(self.v_plan) != N + 1:
            self.v_plan = np.full(N + 1, max(v, 0.5))
        ds = np.maximum(self.v_plan[:N] * self.dt, self.cfg['min_step_m'])
        s_acc = np.concatenate([[0.0], np.cumsum(ds)])
        idx = (i0 + (s_acc / max(P.spacing, 1e-3)).astype(int)) % P.n
        return idx, ds

    # ..................................................................
    def _speed_plan(self, idx, ds, v0):
        """A feasible speed trajectory over the horizon.

        Ceilings, in order: the offline profile; the profile cut by the
        predicted lateral error; and the grip budget at the curvature the
        current plan is ASKING for (not the reference's curvature -- that is the
        whole point: a car running wide has to turn harder than the line does,
        and must pay for it in speed).  Then a backward pass at ``a_brake`` so
        every ceiling ahead is reachable, and a forward pass at ``a_accel`` and
        ``a_brake`` so the trajectory starts from the speed the car HAS.
        """
        c = self.cfg
        P = self.path
        N = self.N
        v_lim = P.v_prof[idx].astype(float).copy()

        e = np.abs(self.e_pred[:N + 1])
        mod = np.clip(1.0 - c['k_e'] * np.maximum(0.0, e - c['e_free']),
                      c['v_mod_min'], 1.0)
        v_lim *= mod

        # The grip ceiling.  It is built from the REFERENCE curvature plus a
        # BOUNDED allowance for the correction the plan is adding on top -- not
        # from the plan's raw commanded curvature.
        #
        # Measured (Sept 12, live mc0.60 log replayed through this solver): the
        # raw version loses the whole fast straight.  The steering-rate plan may
        # swing the angle by up to 0.19 rad in one step and swing it back, and
        # such a 60 ms transient is invisible in the trajectory but sets
        # `sqrt(ay_budget/0.52) = 3.8 m/s` at that step -- which the backward
        # brake pass then propagates all the way back to the car.  At s 3.5 the
        # profile asked 6.60 m/s and the plan commanded 3.85, on a straight with
        # 0.034 m of lateral error.  The deficit is self-sustaining: a slower
        # car has shorter steps, so the horizon stays inside the corner-entry
        # region and the blip never leaves it.
        #
        # Only the EXCESS over the reference is a real statement about grip the
        # profile has not already paid for, and it is clipped so that no single
        # solver transient can cost more than `k_excess_max`.  The committed
        # steps are excluded outright: their steering is history (it may be
        # unwinding a previous correction) and is not a demand at all.
        k_ref = np.abs(P.kappa[idx])
        k_ex = np.zeros(N + 1)
        nd = self.nd
        if N > nd:
            k_ex[nd:N] = np.clip(np.abs(self.k_cmd[nd:N]) - k_ref[nd:N],
                                 0.0, c['k_excess_max'])
        kd = k_ref + k_ex
        with np.errstate(divide='ignore'):
            v_grip = np.where(kd > 1e-3, np.sqrt(c['ay_budget'] / np.maximum(kd, 1e-9)),
                              np.inf)
        v_lim = np.minimum(v_lim, v_grip)

        # Beyond the horizon: the profile still has to be met, and the horizon
        # is only ~1.2 s.  Seed the backward pass with the fastest speed that
        # can still make every planned speed over the next few metres.
        tail = idx[N]
        n_tail = max(2, int(4.0 / max(P.spacing, 1e-3)))
        j = (tail + np.arange(n_tail)) % P.n
        d_ahead = np.arange(n_tail) * P.spacing
        v_lim[N] = min(v_lim[N], float(np.sqrt(
            P.v_prof[j] ** 2 + 2.0 * c['a_brake'] * d_ahead).min()))

        for k in range(N - 1, -1, -1):
            v_lim[k] = min(v_lim[k], math.sqrt(v_lim[k + 1] ** 2
                                               + 2.0 * c['a_brake'] * ds[k]))
        # Two trajectories come out of here and they are NOT the same thing.
        #
        # `v_lim` is the CEILING: the fastest the car may go at each step and
        # still make every planned speed ahead.  That is what the throttle
        # should be handed.  The simulator's throttle is a speed servo with
        # +5.0 / -4.5 m/s2 of authority that saturates as soon as the setpoint
        # is ~0.3 m/s off, so it closes the gap by itself and eases off on
        # arrival; handing it an acceleration-feasible setpoint instead just
        # rate-limits the command for no gain.  Measured on the live mc0.60
        # log: commanding the feasible value left the setpoint 1.25 m/s under
        # the profile on the straight even after the grip-cut fix.
        #
        # `v` is the FEASIBLE trajectory, integrated from the speed the car
        # actually has.  The horizon's arc lengths and the delivered-curvature
        # saturation must use this one, because that is where the car will be.
        v = np.empty(N + 1)
        v[0] = max(float(v0), 0.3)
        for k in range(N):
            hi = math.sqrt(v[k] ** 2 + 2.0 * c['a_accel'] * ds[k])
            lo = math.sqrt(max(v[k] ** 2 - 2.0 * c['a_brake'] * ds[k], 0.04))
            v[k + 1] = max(lo, min(v_lim[k + 1], hi))
        self.v_ceiling = v_lim
        return v

    # ..................................................................
    def _bounds(self, idx):
        """One-sided corridor hinges, left and right, at each horizon step."""
        P = self.path
        hw = 0.5 * vehicle.CAR_WIDTH + self.cfg['barrier_margin']
        if not P.has_corridor:
            return (np.full(self.N, -np.inf), np.full(self.N, np.inf))
        hi = P.w_left[idx[1:]] - hw
        lo = -(P.w_right[idx[1:]] - hw)
        return lo, hi


    # ..................................................................
    def _rollout(self, u, z0, ds, kap, v, cap, committed, jac=False):
        """Path-relative rollout over the horizon.

        ``u`` holds the FREE steering rates only.  Over the first ``nd`` steps
        the steering angle is ``committed`` -- whatever the commands already in
        flight will produce -- because the dead time makes it data, not a
        decision.  Forcing it here, rather than shifting the pose forward and
        pretending the horizon starts later, keeps the reference the free steps
        are judged against correct in TIME and keeps the delay in the structure
        instead of in the cost.
        """
        N, nd = self.N, self.nd
        nf = N - nd
        c = self.cfg
        g, knee, ayd = c['yaw_gain'], c['sat_knee'], c['ay_deliver']
        e = np.empty(N + 1)
        psi = np.empty(N + 1)
        d = np.empty(N + 1)
        e[0], psi[0], d[0] = z0
        S = np.zeros((3, nf)) if jac else None
        Sout = np.empty((N + 1, 3, nf)) if jac else None
        if jac:
            Sout[0] = 0.0
        kc = np.empty(N)
        for k in range(N):
            ek, pk, dk = e[k], psi[k], d[k]
            kr = kap[k]
            dsk = ds[k]
            den = 1.0 - kr * ek
            if den < 0.2:
                den = 0.2
                dden_de = 0.0
            else:
                dden_de = -kr
            sp, cp = math.sin(pk), math.cos(pk)
            k_max = ayd / max(v[k] * v[k], 0.25)
            kach, dkach = _sat_curvature(dk, k_max, knee, g)
            kc[k] = math.tan(dk) / vehicle.WHEELBASE
            e[k + 1] = ek + sp * dsk
            psi[k + 1] = wrap(pk + (kach - kr * cp / den) * dsk)
            raw = None
            if k < nd:
                d[k + 1] = committed[k] if committed is not None else dk
            else:
                raw = dk + u[k - nd] * self.dt
                d[k + 1] = min(cap, max(-cap, raw))
            if jac:
                a10 = -(-kr * cp) * dden_de / (den * den) * dsk
                a11 = 1.0 + (kr * sp / den) * dsk
                a12 = dkach * dsk
                Sn = np.empty((3, nf))
                Sn[0] = S[0] + cp * dsk * S[1]
                Sn[1] = a10 * S[0] + a11 * S[1] + a12 * S[2]
                if k < nd:
                    Sn[2] = 0.0            # committed: independent of u
                elif raw != d[k + 1]:
                    Sn[2] = 0.0            # against the steering cap
                else:
                    Sn[2] = S[2]
                    Sn[2, k - nd] += self.dt
                S = Sn
                Sout[k + 1] = S
        self.k_cmd = kc
        return e, psi, d, Sout

    # ..................................................................
    def _residuals(self, u, e, psi, d, lo, hi, jac=None):
        """Weighted residual stack (and its Jacobian rows).

        e_lat at every step, again at the terminal, heading error, a small
        penalty on steering magnitude, one-sided hinges on each corridor wall,
        and rate/jerk on the decision variable.  No cut term, no inside/outside
        asymmetry: the geometry belongs to the line.
        """
        c = self.cfg
        N, nd = self.N, self.nd
        nf = N - nd
        r = []
        J = [] if jac is not None else None
        sw_yaw = math.sqrt(c['w_yaw'])
        sw_steer = math.sqrt(c['w_steer'])
        sw_rate = math.sqrt(c['w_rate'])
        sw_jerk = math.sqrt(c['w_jerk'])
        sw_bar = math.sqrt(c['w_barrier'])
        sw_lat = math.sqrt(c['w_lat'])
        sw_latT = math.sqrt(c['w_lat'] + c['w_lat_T'])
        sw_yawT = math.sqrt(c['w_yaw'] + c['w_yaw_T'])
        zero = np.zeros(nf)
        for k in range(N):
            term = (k == N - 1)
            sw = sw_latT if term else sw_lat
            swy = sw_yawT if term else sw_yaw
            r.append(sw * e[k + 1])
            r.append(swy * psi[k + 1])
            r.append(sw_steer * d[k + 1])
            hu = e[k + 1] - hi[k]
            hd = lo[k] - e[k + 1]
            r.append(sw_bar * (hu if hu > 0 else 0.0))
            r.append(sw_bar * (hd if hd > 0 else 0.0))
            if jac is not None:
                Se, Sp, Sd = jac[k + 1][0], jac[k + 1][1], jac[k + 1][2]
                J.append(sw * Se)
                J.append(swy * Sp)
                J.append(sw_steer * Sd)
                J.append(sw_bar * Se if hu > 0 else zero)
                J.append(-sw_bar * Se if hd > 0 else zero)
        for i in range(nf):
            r.append(sw_rate * u[i] * self.dt)
            du = u[i] - (u[i - 1] if i else u[0])
            r.append(sw_jerk * du * self.dt)
            if jac is not None:
                ek = np.zeros(nf)
                ek[i] = 1.0
                dk = np.zeros(nf)
                dk[i] = 1.0
                if i:
                    dk[i - 1] = -1.0
                J.append(sw_rate * self.dt * ek)
                J.append(sw_jerk * self.dt * dk)
        r = np.asarray(r)
        return (r, np.asarray(J)) if jac is not None else (r, None)

    # ..................................................................
    def solve(self, e_lat, e_yaw, delta0, i0, v0, cap=None, committed=None):
        """One control cycle.  Call once per new pose.

        ``delta0`` is the actuator angle at the pose stamp, ``committed`` the
        steering angle the in-flight commands produce at each of the first
        ``nd`` horizon steps (both from ``Actuator``).  Returns
        ``(command_rad, info)``; ``info['v_cmd']`` is the speed the throttle
        should be asked for, which is the *plan's*, not the profile's.
        """
        c = self.cfg
        N, nd = self.N, self.nd
        nf = N - nd
        cap = vehicle.MAX_STEER if cap is None else min(cap, vehicle.MAX_STEER)
        rate = float(c['steer_rate'])
        z0 = (float(e_lat), float(wrap(e_yaw)),
              float(np.clip(delta0, -vehicle.MAX_STEER, vehicle.MAX_STEER)))
        if committed is not None:
            committed = np.clip(np.asarray(committed, dtype=float)[:nd],
                                -vehicle.MAX_STEER, vehicle.MAX_STEER)

        u = np.clip(np.roll(self.u, -1)[:nf], -rate, rate)
        if nf > 1:
            u[-1] = u[-2]

        cost = float('nan')
        it = 0
        e = psi = d = None
        idx = ds = kap = lo = hi = None
        for _ in range(max(1, int(c['outer']))):
            # speed plan -> arc lengths -> reference, then the steering solve
            idx, ds = self._indices(i0, v0)
            self.v_plan = self._speed_plan(idx, ds, v0)
            idx, ds = self._indices(i0, v0)
            kap = self.path.kappa[idx[:N]]
            lo, hi = self._bounds(idx)

            e, psi, d, S = self._rollout(u, z0, ds, kap, self.v_plan, cap,
                                         committed, jac=True)
            r, J = self._residuals(u, e, psi, d, lo, hi, jac=S)
            cost = float(r @ r)
            lam = float(c['lam0'])
            step = np.zeros(nf)
            for it in range(int(c['iters'])):
                H = J.T @ J
                gr = J.T @ r
                # Damp the DIAGONAL of H, not a trace-scaled identity: the
                # barrier makes the trace explode exactly when a violation
                # needs correcting (mpc2 learned this the hard way).
                dg = np.diag(H).copy()
                dg[dg < 1e-9] = 1e-9
                for _ in range(6):
                    try:
                        step = np.linalg.solve(H + lam * np.diag(dg), -gr)
                    except np.linalg.LinAlgError:
                        lam *= 10.0
                        continue
                    u_try = np.clip(u + step, -rate, rate)
                    e2, psi2, d2, _ = self._rollout(u_try, z0, ds, kap,
                                                    self.v_plan, cap, committed)
                    r2, _ = self._residuals(u_try, e2, psi2, d2, lo, hi)
                    c2 = float(r2 @ r2)
                    if c2 < cost:
                        u, cost = u_try, c2
                        lam = max(lam * 0.4, 1e-6)
                        break
                    lam *= 6.0
                else:
                    break
                e, psi, d, S = self._rollout(u, z0, ds, kap, self.v_plan, cap,
                                             committed, jac=True)
                r, J = self._residuals(u, e, psi, d, lo, hi, jac=S)
                if float(np.max(np.abs(step))) < 1e-3:
                    break
            e, psi, d, _ = self._rollout(u, z0, ds, kap, self.v_plan, cap,
                                         committed)
            self.e_pred = e

        self.u = np.concatenate([u, np.full(nd, u[-1] if nf else 0.0)])
        self.last_cost = cost
        self.last_iters = it + 1

        # What to publish: the plan's steering angle `apply_ahead_s` after the
        # new command can first bite.  The plan is feasible by construction
        # (|u| <= the slew rate), so that is an angle the actuator can reach and
        # hold for the frame.  Handing it d[nd] -- the angle it is already
        # predicted to hold -- would make the command never change.
        f = nd + max(self.dt, float(c['apply_ahead_s'])) / self.dt
        k = int(f)
        cmd = d[N] if k >= N else d[k] + (f - k) * (d[k + 1] - d[k])
        cmd = float(np.clip(cmd, -cap, cap))

        vp = self.v_ceiling if self.v_ceiling is not None else self.v_plan
        fk = min(float(c['thr_lead_s']) / self.dt, N - 1e-6)
        ki = int(fk)
        v_cmd = float(vp[ki] + (fk - ki) * (vp[ki + 1] - vp[ki]))
        info = dict(cost=cost, iters=self.last_iters, delta=d, e=e, psi=psi,
                    v=self.v_plan, v_ceiling=vp, v_cmd=v_cmd, lo=lo, hi=hi,
                    kap=kap, idx=idx, ds=ds, u=u.copy())
        return cmd, info
