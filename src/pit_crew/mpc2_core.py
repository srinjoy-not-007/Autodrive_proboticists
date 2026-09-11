"""Pure-python core of the mpc2 tracker: reference path, actuator model, solver.

Kept free of rclpy on purpose so the identical control law can be driven by the
offline closed-loop simulator in ``tools/sim_tracker.py``. Every behavioural
change belongs here, not in the node, so that what is measured offline is what
runs live.

Three pieces:

``RefPath``     the raceline plus everything derived from it once -- heading,
                smoothed curvature, and (when a map is available) the distance
                from each reference point to the wall on the left and on the
                right, which is what lets the optimiser see the inside wall of
                a hairpin instead of only its own lateral error.

``Actuator``    the command pipeline as measured: a command published now is
                not seen by the simulator for ``dead_time`` seconds (median
                0.135 s, structural -- the devkit bridge is request/response at
                the 90 ms frame rate), after which the steering slews towards
                it at 3.2 rad/s.  Anchored on the ``/autodrive/roboracer_1/
                steering`` feedback, which is the real actuator angle in
                radians and was previously unused; mpc.py instead assumed the
                last published *command* was the current angle, which is wrong
                by up to a full slew step whenever the steering is moving.

``MPC2``        finite-horizon Gauss-Newton over a steering-RATE sequence.  The
                slew limit is the box on the decision variable and the dead
                time is handled by starting the horizon at the predicted state
                when the new command bites, so both are structure rather than
                penalties.
"""

import math

import numpy as np

try:
    from pit_crew import vehicle
except ImportError:                                  # running from src/pit_crew
    import vehicle


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --------------------------------------------------------------------------
# reference path
# --------------------------------------------------------------------------

class RefPath:
    """Raceline plus the corridor it lives in."""

    def __init__(self, csv_path, kappa_span_m=0.5, speed_scale=1.0):
        rl = np.loadtxt(csv_path, delimiter=',')
        if (rl.ndim != 2 or rl.shape[0] < 3 or rl.shape[1] < 6
                or not np.isfinite(rl).all() or np.any(rl[:, 5] < 0)):
            raise ValueError('raceline needs >=3 finite rows and nonnegative speeds')
        self.s = rl[:, 0]
        self.xy = np.ascontiguousarray(rl[:, 1:3])
        self.v_prof = rl[:, 5] * speed_scale
        self.a_prof = rl[:, 6] if rl.shape[1] > 6 else np.zeros(len(rl))
        seg = np.hypot(np.diff(self.xy[:, 0]), np.diff(self.xy[:, 1]))
        self.spacing = float(np.median(seg)) if len(seg) else 0.1
        span = max(1, int(round(kappa_span_m / max(self.spacing, 1e-3))))
        fwd = np.roll(self.xy, -span, axis=0) - np.roll(self.xy, span, axis=0)
        self.yaw = np.arctan2(fwd[:, 1], fwd[:, 0])
        k = rl[:, 4]
        w = 2 * span + 1
        pad = np.concatenate([k[-span:], k, k[:span]])
        self.kappa = np.convolve(pad, np.ones(w) / w, mode='valid')
        self.n = len(self.xy)
        # Corridor: +inf until a map is loaded, i.e. barrier disabled.
        self.w_left = np.full(self.n, np.inf)
        self.w_right = np.full(self.n, np.inf)
        self.has_corridor = False

    # ..................................................................
    def load_corridor(self, dist, meta, max_probe_m=2.0, step_m=0.02):
        """Wall distance along the path normal, left and right, per point.

        ``dist`` is the Euclidean distance transform of the free space (metres)
        that ``pf_core.load_map`` already builds for localization; re-using it
        costs nothing.  Marching along the normal rather than reading the
        distance field at the reference point matters: the field gives the
        nearest wall in *any* direction, which on a hairpin is the inside one
        even when the car is running wide, so it cannot tell a cut from a
        drift.
        """
        res = float(meta['resolution'])
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
        H, W = dist.shape
        nx, ny = -np.sin(self.yaw), np.cos(self.yaw)     # unit left normal
        steps = int(max_probe_m / step_m)
        out = []
        for sign in (1.0, -1.0):
            reach = np.full(self.n, max_probe_m)
            live = np.ones(self.n, bool)
            for j in range(1, steps + 1):
                t = j * step_m
                px = self.xy[:, 0] + sign * t * nx
                py = self.xy[:, 1] + sign * t * ny
                cx = np.clip(((px - ox) / res).astype(int), 0, W - 1)
                cy = np.clip(((py - oy) / res).astype(int), 0, H - 1)
                free = dist[cy, cx] > 0.5 * res
                newly = live & ~free
                reach[newly] = t - step_m
                live &= free
            out.append(reach)
        self.w_left, self.w_right = out
        self.has_corridor = True

    # ..................................................................
    def nearest(self, x, y, last=None, window=90, back=3, lost_m=1.0,
                yaw=None, yaw_tol=1.2):
        """Index of the reference point the car is at, heading-gated.

        A plain nearest-point search cannot be trusted on a hairpin: the two
        legs of Porto's s 10-16 complex pass within a metre of each other, so a
        car that has run half a metre wide is genuinely closer to the returning
        leg than to its own, and both a global argmin and a window that allows
        a metre of slip-back will happily latch onto it.  The reference then
        points the other way and the tracker drives in a circle -- reproduced
        offline: index 156 -> 153 -> 149 while the car went forwards, then
        +-1.0 m of lateral error for two seconds.

        Gate on direction of travel: a candidate whose path heading disagrees
        with the car's by more than ``yaw_tol`` is not where the car is, however
        close it is.  The window then only has to stop the index running away,
        so it can be tight.
        """
        n = self.n
        if last is None:
            idx = np.arange(n)
        else:
            idx = (np.arange(last - back, last - back + window) % n)
        d = np.hypot(self.xy[idx, 0] - x, self.xy[idx, 1] - y)
        if yaw is not None:
            bad = np.abs((yaw - self.yaw[idx] + np.pi) % (2 * np.pi) - np.pi) > yaw_tol
            if not bad.all():
                d = d + 100.0 * bad
        j = int(np.argmin(d))
        if last is not None and float(d[j]) > lost_m:
            return self.nearest(x, y, None, yaw=yaw, yaw_tol=yaw_tol)
        return int(idx[j])


# --------------------------------------------------------------------------
# actuator + dead time
# --------------------------------------------------------------------------

class Actuator:
    """Replays published commands through the measured dead time and slew.

    The state that matters is the *actual* steering angle, which the simulator
    publishes on ``/autodrive/roboracer_1/steering`` in radians.  Record every
    sample with the time it describes; between samples, and into the future,
    advance the last one towards whatever command is acting then, at the
    3.2 rad/s slew, with a command published now not acting for ``dead_time``.

    Why a history and not a single anchor: the pose is stamped with the scan
    time and arrives after the particle filter has run, so by the time a
    control cycle starts the newest steering sample is **newer than the pose**.
    The first version held the newest sample as though it were the angle at the
    pose stamp and then integrated the delay window forward from there, which
    double-counts every slew between the two -- up to 0.29 rad per frame at the
    rate limit, i.e. exactly at a corner entry.  The model then believed the car
    had already turned in, predicted it further round the corner than it was,
    and asked for less lock; live, mpc2 ran 0.24-0.34 m WIDE at both hairpins
    and hit the outside wall six times at s 27.7.  Interpolating the recorded
    samples for any time they cover removes that whole class of error.
    """

    def __init__(self, dead_time=0.135, rate=vehicle.STEER_RATE, sub_dt=0.005,
                 keep_s=1.0):
        self.dead_time = float(dead_time)
        self.rate = float(rate)
        self.sub_dt = float(sub_dt)
        self.keep_s = float(keep_s)
        self.hist = [(-1e9, 0.0)]          # (t_published, command_rad)
        self.fb = []                       # (t_measured, actual_angle_rad)
        self.fallback = 0.0                # when no feedback has arrived yet

    # .................................................................. commands
    def record(self, t, cmd):
        self.hist.append((float(t), float(cmd)))
        keep = t - self.dead_time - self.keep_s
        while len(self.hist) > 2 and self.hist[1][0] < keep:
            self.hist.pop(0)
        if not self.fb:
            self.fallback = float(cmd)

    def _target_at(self, t):
        """The command the actuator is chasing at time ``t``."""
        te = t - self.dead_time
        h = self.hist
        if te <= h[0][0]:
            return h[0][1]
        lo, hi = 0, len(h) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if h[mid][0] <= te:
                lo = mid
            else:
                hi = mid - 1
        return h[lo][1]

    # .................................................................. feedback
    def snap(self, t, delta):
        """Record a measured actuator angle, stamped with the time it describes."""
        if not (math.isfinite(delta) and math.isfinite(t)):
            return
        if self.fb and t <= self.fb[-1][0]:
            return
        self.fb.append((float(t), float(delta)))
        while len(self.fb) > 2 and self.fb[0][0] < t - self.keep_s:
            self.fb.pop(0)

    def _measured_at(self, t):
        """Interpolate the recorded feedback. Only valid inside its span."""
        fb = self.fb
        if t <= fb[0][0]:
            return fb[0][1]
        lo, hi = 0, len(fb) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if fb[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        if lo >= len(fb) - 1:
            return fb[-1][1]
        (ta, da), (tb, db) = fb[lo], fb[lo + 1]
        w = (t - ta) / max(tb - ta, 1e-9)
        return da + w * (db - da)

    def advance(self, delta, t_from, t_to):
        """Slew ``delta`` from ``t_from`` to ``t_to`` under the in-flight commands."""
        if t_to <= t_from:
            return delta
        n = max(1, int(math.ceil((t_to - t_from) / self.sub_dt)))
        dt = (t_to - t_from) / n
        step = self.rate * dt
        for k in range(n):
            tgt = self._target_at(t_from + (k + 0.5) * dt)
            delta += max(-step, min(step, tgt - delta))
        return delta

    def delta_at(self, t):
        """Actuator angle at ``t``: measured where we have it, modelled after."""
        if not self.fb:
            return self.fallback
        if t <= self.fb[-1][0]:
            return self._measured_at(t)
        return self.advance(self.fb[-1][1], self.fb[-1][0], t)

    # .................................................................. rollout
    def predict(self, x, y, yaw, t0, t1, v, kappa_of=None):
        """Bicycle + actuator forward from ``t0`` to ``t1``.

        Returns ``(x, y, yaw, delta)`` at ``t1``.  ``t0`` is the pose stamp and
        ``t1`` is when the command being chosen now will bite.  Inside the span
        the feedback covers, the steering angle is the measured one; beyond it,
        the slew model takes over.
        """
        kappa_of = kappa_of or (lambda d: math.tan(d) / vehicle.WHEELBASE)
        delta = self.delta_at(t0)
        if t1 <= t0:
            return x, y, yaw, delta
        t_fb = self.fb[-1][0] if self.fb else -1e9
        n = max(1, int(math.ceil((t1 - t0) / self.sub_dt)))
        dt = (t1 - t0) / n
        step = self.rate * dt
        for k in range(n):
            tm = t0 + (k + 0.5) * dt
            if tm <= t_fb:
                delta = self._measured_at(tm)
            else:
                tgt = self._target_at(tm)
                delta += max(-step, min(step, tgt - delta))
            yr = v * kappa_of(delta)
            ym = yaw + 0.5 * yr * dt
            x += v * math.cos(ym) * dt
            y += v * math.sin(ym) * dt
            yaw = wrap(yaw + yr * dt)
        return x, y, yaw, delta


class PoseFilter:
    """Complementary filter on the localized pose, propagated by the actuator.

    mpc blends the new pose with a gyro dead-reckoning of the old one at 0.55.
    The blend is worth keeping -- localization lateral error is 0.04-0.17 m
    depending on the map and an unfiltered tracker answers every sample of it --
    but the propagation should use the model that knows what the steering is
    doing, which is the same ``Actuator`` the delay compensation uses.  A large
    innovation is taken whole: that is a correction, not jitter.
    """

    def __init__(self, act: Actuator, blend=0.7, gate_m=0.5):
        self.act = act
        self.blend = float(blend)
        self.gate = float(gate_m)
        self.state = None            # (x, y, yaw, t)

    def update(self, t, x, y, yaw, v):
        if self.state is None or self.blend >= 1.0:
            self.state = (x, y, yaw, t)
            return self.state
        px, py, pyaw, _d = self.act.predict(*self.state[:3], self.state[3], t, v)
        a = self.blend
        if self.gate > 0.0 and math.hypot(x - px, y - py) > self.gate:
            a = 1.0
        self.state = (px + a * (x - px), py + a * (y - py),
                      pyaw + a * wrap(yaw - pyaw), t)
        return self.state


class CurvatureBias:
    """Offset-free correction: what the car turns minus what the model says.

    The kinematic bicycle is optimistic.  The front tyre returns about 0.92 of
    the curvature ``tan(delta)/L`` asks for even well inside its envelope, so an
    MPC that plans with the bare model plans a convergence that never happens:
    measured offline, the tracker sat 0.15-0.18 m outside the reference for a
    whole corner while its own plan showed the error decaying to zero, and it
    re-planned the same fiction every 90 ms.  Weights cannot fix that -- w_lat
    from 14 to 120 moved the offset by 0.01 m -- because the model, not the
    cost, is what is wrong.

    Both signals needed are legal runtime inputs and were already subscribed:
    the IMU's yaw rate and (new in mpc2) the steering feedback.  The estimate is
    a plain low pass of ``gyro_z / v - tan(delta_meas)/L``, clamped, held
    constant across the horizon.

    This is NOT the static understeer curve that CLAUDE.md forbids in the
    rollout.  That was an open-loop table that told the controller the tyres
    deliver less than they do at large angles, so it wound on more lock and
    saturated.  This is a measured closed-loop residual: when the car IS turning
    as asked it is zero, and the ``ay_limit`` cap still bounds what may be
    demanded.
    """

    def __init__(self, gain=0.8, limit=0.10, v_min=1.0, slew_tol=0.06):
        self.gain = float(gain)
        self.limit = float(limit)
        self.v_min = float(v_min)
        self.slew_tol = float(slew_tol)
        self.value = 0.0
        self._prev = None

    def update(self, gyro_z, v, delta_meas):
        """One sample. Only quasi-steady ones are believed.

        Two gates, both learned the hard way.  **Slew**: gyro and steering
        arrive on the same 90 ms frame, but while the actuator is slewing at
        3.2 rad/s the angle moves 0.29 rad between frames, so any small
        misalignment between the two signals turns into a large fictitious
        bias exactly at a corner entry.  Only update when the steering is
        nearly still.  **Clamp**: the limit is 0.10 1/m, not 0.30.  A big
        adverse bias tells the controller the car is not turning, it answers
        with more lock, that lock slides, and the estimator sees even less
        turning -- the runaway CLAUDE.md documents for the open-loop understeer
        curve, reachable here too.  0.10 1/m is 0.03 rad of steering, enough to
        cancel the model's optimism and too small to wind the car into a slide.
        """
        if self.gain <= 0.0 or v < self.v_min:
            return self.value
        if not (math.isfinite(gyro_z) and math.isfinite(delta_meas)):
            return self.value
        prev, self._prev = self._prev, delta_meas
        if prev is None or abs(delta_meas - prev) > self.slew_tol:
            return self.value
        raw = gyro_z / v - math.tan(delta_meas) / vehicle.WHEELBASE
        self.value += self.gain * (raw - self.value)
        self.value = max(-self.limit, min(self.limit, self.value))
        return self.value


# --------------------------------------------------------------------------
# the optimiser
# --------------------------------------------------------------------------

DEFAULTS = dict(
    horizon=24,
    step_dt=0.055,
    min_step_m=0.10,
    steer_rate=vehicle.STEER_RATE,
    w_lat=14.0,
    w_yaw=3.0,
    w_steer=0.4,
    w_rate=0.25,          # on the applied steering CHANGE per step
    w_jerk=0.06,          # on the change in steering rate
    w_cut=2.0,            # extra lateral weight inside of the TARGET line
    cut_offset_m=0.0,     # how far inside the reference to aim in a bend
    cut_kappa_ref=0.6,    # curvature at which the full offset is used
    w_lat_T=40.0,
    w_yaw_T=8.0,
    w_barrier=400.0,
    barrier_margin=0.14,
    iters=8,
    lam0=1e-3,
    ay_limit=0.0,
    apply_ahead_s=0.09,
    ff_restart=0.0,       # >0: also start from the curvature feedforward when
                          # it costs less than this fraction of the warm start
)


class MPC2:
    """Gauss-Newton MPC over a steering-rate sequence.

    Decision variable ``u`` is the steering RATE at each step, boxed at the
    actuator's real 3.2 rad/s.  State is ``(e_lat, e_yaw, delta)`` in the
    path-relative frame:

        e_lat'  = v sin(e_yaw)
        e_yaw'  = v (tan(delta)/L - k_ref cos(e_yaw) / (1 - k_ref e_lat))
        delta'  = u                                    |u| <= 3.2, |delta| <= cap

    Why not mpc.py's grid: there the only real decision was the first step and
    the rest of the horizon was a fixed proportional relaxation towards the
    curvature feedforward.  That tail always aims the car back at the reference
    by the shortest route, which through a hairpin is the inside of it, and no
    weight on e_lat can argue with a tail that is not a decision variable.
    Here every step is a decision, e_lat is paid for at every step and again at
    the terminal, and the corridor barrier makes the inside wall expensive.
    """

    def __init__(self, path: RefPath, **cfg):
        self.path = path
        p = dict(DEFAULTS)
        p.update({k: v for k, v in cfg.items() if v is not None})
        self.cfg = p
        self.N = int(p['horizon'])
        self.dt = float(p['step_dt'])
        self.u = np.zeros(self.N)
        self.last_cost = float('nan')
        self.last_iters = 0

    # ..................................................................
    def _reference(self, i0, v0, brake_v=None):
        """Arc progression, curvature and corridor bounds along the horizon.

        Speed is taken from the profile as the car moves along it (the car
        brakes into corners, so a constant-speed preview over-reaches at entry
        and under-reaches at exit), and the arc per step never falls below
        ``min_step_m`` -- at 0.5 m/s a fixed time horizon previews less than the
        radius of a hairpin and the controller cannot see the corner at all.
        """
        P = self.path
        N, dt = self.N, self.dt
        idx = np.empty(N + 1, dtype=int)
        ds = np.empty(N)
        s_acc = 0.0
        i = i0
        v = v0
        for k in range(N):
            idx[k] = i
            vk = v if k == 0 else min(v0 + 6.5 * (k * dt), float(P.v_prof[i]))
            ds[k] = max(vk * dt, self.cfg['min_step_m'])
            s_acc += ds[k]
            i = (i0 + int(s_acc / max(P.spacing, 1e-3))) % P.n
        idx[N] = i
        kap = P.kappa[idx[:N] % P.n]
        hw = 0.5 * vehicle.CAR_WIDTH + self.cfg['barrier_margin']
        if P.has_corridor:
            hi = P.w_left[idx[1:] % P.n] - hw
            lo = -(P.w_right[idx[1:] % P.n] - hw)
        else:
            hi = np.full(N, np.inf)
            lo = np.full(N, -np.inf)

        # Where to aim, which is not always the reference.
        #
        # Measured on the mesh map, 32 laps each: mpc.py sits 0.139 m INSIDE
        # the line through every corner and covers 28.96 m per lap; mpc2, which
        # tracks the line, covers 30.13 m and is 0.46 s a lap slower.  The whole
        # difference is the cut.  On a safe centreline the cut is not an error,
        # it is a shorter path the line does not describe -- what made it
        # dangerous before was that it was uncontrolled and stacked on top of a
        # 0.15 m localization bias, so it arrived at the wall.
        #
        # So offer it deliberately and bounded: aim `cut_offset_m` inside in
        # proportion to curvature, and clip that target into the corridor the
        # barrier defends, so the cut can never be spent on clearance the car
        # does not have.  0.0 keeps mpc2 on the reference exactly.
        cut = float(self.cfg['cut_offset_m'])
        if cut <= 0.0:
            return idx, ds, kap, lo, hi, np.zeros(N)
        kref = max(float(self.cfg['cut_kappa_ref']), 1e-3)
        tgt = cut * np.sign(kap) * np.minimum(1.0, np.abs(kap) / kref)
        tgt = np.clip(tgt, lo, hi)
        return idx, ds, kap, lo, hi, tgt

    # ..................................................................
    def _rollout(self, u, z0, ds, kap, cap, jac=False, bias=0.0):
        N = self.N
        e = np.empty(N + 1)
        psi = np.empty(N + 1)
        d = np.empty(N + 1)
        e[0], psi[0], d[0] = z0
        S = np.zeros((3, N)) if jac else None
        Sout = np.empty((N + 1, 3, N)) if jac else None
        if jac:
            Sout[0] = 0.0
        L = vehicle.WHEELBASE
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
            tn = math.tan(dk)
            e[k + 1] = ek + sp * dsk
            psi[k + 1] = wrap(pk + (tn / L + bias - kr * cp / den) * dsk)
            raw = dk + u[k] * self.dt
            d[k + 1] = min(cap, max(-cap, raw))
            if jac:
                # A = d f / d z
                a10 = -(-kr * cp) * dden_de / (den * den) * dsk
                a11 = 1.0 + (kr * sp / den) * dsk
                a12 = (1.0 + tn * tn) / L * dsk
                Sn = np.empty((3, N))
                Sn[0] = S[0] + cp * dsk * S[1]
                Sn[1] = a10 * S[0] + a11 * S[1] + a12 * S[2]
                if raw != d[k + 1]:
                    Sn[2] = 0.0                        # against the steering cap
                else:
                    Sn[2] = S[2]
                    Sn[2, k] += self.dt
                S = Sn
                Sout[k + 1] = S
        return e, psi, d, Sout

    # ..................................................................
    def _residuals(self, u, e, psi, d, kap, lo, hi, tgt, jac=None):
        """Stack the weighted residuals (and their Jacobian rows)."""
        c = self.cfg
        N = self.N
        r = []
        J = [] if jac is not None else None
        sw_yaw = math.sqrt(c['w_yaw'])
        sw_steer = math.sqrt(c['w_steer'])
        sw_rate = math.sqrt(c['w_rate'])
        sw_jerk = math.sqrt(c['w_jerk'])
        sw_bar = math.sqrt(c['w_barrier'])
        for k in range(N):
            # Corner cutting is e_lat on the inside of the bend, i.e. the same
            # sign as the curvature (e_lat > 0 is left of travel, kappa > 0
            # turns left).  Charge it more than a drift to the outside: the
            # outside of a corner is where the corridor is, the inside is where
            # the wall the car actually hits is.
            de = e[k + 1] - tgt[k]
            wl = c['w_lat'] * (1.0 + (c['w_cut'] if de * kap[k] > 0 else 0.0))
            if k == N - 1:
                wl += c['w_lat_T']
            sw = math.sqrt(wl)
            r.append(sw * de)
            swy = math.sqrt(c['w_yaw'] + (c['w_yaw_T'] if k == N - 1 else 0.0))
            r.append(swy * psi[k + 1])
            r.append(sw_steer * d[k + 1])
            r.append(sw_rate * u[k] * self.dt)
            du = u[k] - (u[k - 1] if k else u[0])
            r.append(sw_jerk * du * self.dt)
            hu = e[k + 1] - hi[k]
            hd = lo[k] - e[k + 1]
            r.append(sw_bar * (hu if hu > 0 else 0.0))
            r.append(sw_bar * (hd if hd > 0 else 0.0))
            if jac is not None:
                Se = jac[k + 1][0]
                Sp = jac[k + 1][1]
                Sd = jac[k + 1][2]
                ek = np.zeros(N)
                ek[k] = 1.0
                dk = np.zeros(N)
                dk[k] = 1.0
                if k:
                    dk[k - 1] = -1.0
                J.append(sw * Se)
                J.append(swy * Sp)
                J.append(sw_steer * Sd)
                J.append(sw_rate * self.dt * ek)
                J.append(sw_jerk * self.dt * dk)
                J.append(sw_bar * Se if hu > 0 else np.zeros(N))
                J.append(-sw_bar * Se if hd > 0 else np.zeros(N))
        r = np.asarray(r)
        return (r, np.asarray(J)) if jac is not None else (r, None)

    # ..................................................................
    def solve(self, e_lat, e_yaw, delta0, i0, v0, cap=None, bias=0.0):
        """Return (applied_command_rad, info). Call once per new pose."""
        c = self.cfg
        N = self.N
        if cap is None:
            cap = vehicle.MAX_STEER
        cap = min(cap, vehicle.MAX_STEER)
        idx, ds, kap, lo, hi, tgt = self._reference(i0, v0)
        z0 = (float(e_lat), float(wrap(e_yaw)), float(np.clip(delta0, -cap, cap)))

        # Warm start: last solution shifted one step. The horizon moved on by
        # exactly one control period, so everything but the tail is still the
        # answer to the same question and the solver starts near optimal --
        # usually.  When the situation has changed (a corner exit, a
        # localization correction) the shifted sequence can be a bad basin that
        # a damped Gauss-Newton will not climb out of inside its iteration
        # budget: measured offline, it left full lock applied for six steps
        # after the heading error had already been cancelled, and planned
        # itself 1.0 m off the line.  So also evaluate the steering-rate
        # sequence that simply follows the curvature feedforward and start from
        # whichever costs less.  One extra rollout, ~0.1 ms.
        rate = c['steer_rate']
        u_warm = np.clip(np.roll(self.u, -1), -rate, rate)
        if N > 1:
            u_warm[-1] = u_warm[-2]
        ff = np.arctan(vehicle.WHEELBASE * kap)
        u_ff = np.clip(np.diff(np.concatenate([[z0[2]], ff])) / self.dt,
                       -rate, rate)
        starts = [u_warm] if c['ff_restart'] <= 0 else [u_warm, u_ff]
        u = u_warm
        cost = float('inf')
        for cand in starts:
            e2, psi2, d2, _ = self._rollout(cand, z0, ds, kap, cap, jac=False, bias=bias)
            r2, _ = self._residuals(cand, e2, psi2, d2, kap, lo, hi, tgt)
            c2 = float(r2 @ r2)
            if c2 < cost * float(c['ff_restart'] or 1.0):
                u, cost = cand.copy(), c2

        e, psi, d, S = self._rollout(u, z0, ds, kap, cap, jac=True, bias=bias)
        r, J = self._residuals(u, e, psi, d, kap, lo, hi, tgt, jac=S)
        cost = float(r @ r)
        lam = c['lam0']
        it = 0
        for it in range(int(c['iters'])):
            H = J.T @ J
            g = J.T @ r
            # Levenberg-Marquardt damping on the DIAGONAL of H, not on the
            # identity scaled by its trace: the barrier makes the trace explode
            # exactly when a violation needs correcting, and an absolute
            # damping term derived from it froze the solver at the one moment
            # it had work to do.
            dg = np.diag(H).copy()
            dg[dg < 1e-9] = 1e-9
            for _ in range(6):
                try:
                    step = np.linalg.solve(H + lam * np.diag(dg), -g)
                except np.linalg.LinAlgError:
                    lam *= 10.0
                    continue
                u_try = np.clip(u + step, -rate, rate)
                e2, psi2, d2, _ = self._rollout(u_try, z0, ds, kap, cap, jac=False,
                                               bias=bias)
                r2, _ = self._residuals(u_try, e2, psi2, d2, kap, lo, hi, tgt)
                c2 = float(r2 @ r2)
                if c2 < cost:
                    u, cost = u_try, c2
                    lam = max(lam * 0.4, 1e-6)
                    break
                lam *= 6.0
            else:
                break
            e, psi, d, S = self._rollout(u, z0, ds, kap, cap, jac=True, bias=bias)
            r, J = self._residuals(u, e, psi, d, kap, lo, hi, tgt, jac=S)
            if float(np.max(np.abs(step))) < 1e-3:
                break
        self.u = u
        self.last_cost = cost
        self.last_iters = it + 1

        # What to publish. The plan's delta trajectory is feasible by
        # construction (|u| <= the slew rate), so handing the actuator the
        # angle the plan wants ``apply_ahead_s`` from now is a target it can
        # reach and hold for the rest of the frame.
        e, psi, d, _ = self._rollout(u, z0, ds, kap, cap, jac=False, bias=bias)
        # Never below one step.  apply_ahead_s = 0 hands the actuator d[0],
        # which IS the angle it is already predicted to hold, so the command
        # never changes and the car drives straight off the track -- measured
        # offline, 320 m of cross-track error.
        ta = max(self.dt, float(c['apply_ahead_s']))
        f = ta / self.dt
        k = int(f)
        if k >= N:
            cmd = d[N]
        else:
            cmd = d[k] + (f - k) * (d[k + 1] - d[k])
        cmd = float(np.clip(cmd, -cap, cap))
        info = dict(cost=cost, iters=self.last_iters, delta=d, e=e, psi=psi,
                    u=u.copy(), lo=lo, hi=hi, kap=kap, tgt=tgt)
        return cmd, info
