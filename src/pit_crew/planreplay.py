"""Re-read the MPC's own plan between poses instead of holding its command.

The brief's candidate C was "a 50-100 Hz inner loop".  Two facts have to be
stated before any of it, because they bound what such a loop can possibly buy:

1. **No topic arrives faster than the pose.**  Measured on both shipping
   100-lap logs (`race_20260919_044701`, `_050630`): the IPS row period and the
   distinct `/pit_crew/pose` stamp period are the same number to four decimals
   (0.0580 / 0.0940 s).  Every sensor shares one telemetry WebSocket, so an
   inner loop has no new information to integrate -- the IMU does not arrive at
   100 Hz.  Whatever it does, it does open loop.
2. **The bridge latches ONE command per telemetry tick**, so extra publishes
   are discarded; only the last one before the tick is applied.  This is
   already encoded in `tools/sim_tracker.py`'s `MPC3Sim.pickup_time`.

That is also why `control_mode:=timer` lost (`docs/STATUS.md`: "+0.25 s/lap and
4-6 collisions in 33 laps at *every* period").  Timer mode re-SOLVES at 20 Hz
from a pose that is up to one whole period old, with `rate_adaptive` switched
off and `apply_ahead_s` re-pointed at the tick -- three changes at once, two of
which make the information the solver works from strictly worse.

What is left, and what this module does, is the part that is NOT worse: the
solve stays on the pose, at maximum freshness, and only the READOUT moves.  The
plan `d[k]` is an angle the actuator can hold at time `t_now + k*dt`; mpc3
reads it once, at solve time, at index `nd + apply_ahead_s/dt`, and then
publishes that same number until the next pose.  By the time the bridge latches
it, up to a whole period has passed and the plan wanted a different angle.
`PlanReplay` walks the same plan forward at the wall clock and republishes, so
the number latched at the tick is the plan's angle for THAT tick.

It is bounded by construction: it never leaves the trajectory the MPC solved
and vetted against the corridor, it re-applies the same `cmd_tau` filter in
seconds, and the index it reads is clipped to the horizon.
"""

import math

import numpy as np

from . import vehicle


class PlanReplay:
    """Holds the last plan and returns the angle it wants at any later time."""

    def __init__(self, gain=1.0, max_steps=0.0, cmd_tau=0.0, rate_cap=3.2):
        # `gain` scales how far the readout is allowed to walk (1.0 = the full
        # wall-clock advance, 0.0 = mpc3's hold, which is the identity).
        self.gain = float(gain)
        # Optional ceiling on the advance, in horizon steps.
        self.max_steps = float(max_steps)
        self.cmd_tau = float(cmd_tau)
        # The replayed angle may never be further from the angle the solver
        # itself published than the rack could have slewed in the meantime.
        # That keeps the publish gate's and the straight blend's effect on the
        # solved command -- neither of which the replay re-evaluates -- from
        # being thrown away between poses.
        self.rate_cap = float(rate_cap)
        self.cmd0 = 0.0
        self.plan = None          # the published-angle plan, length N+1
        self.t0 = 0.0             # the time plan[0] refers to
        self.dt = 0.06
        self.k0 = 0.0             # the index mpc3 itself published
        self.cap = vehicle.MAX_STEER
        self.last = None
        self.t_last = None
        self.pubs = 0

    def set_plan(self, plan, t0, dt, k0, cap, cmd):
        self.plan = np.asarray(plan, float)
        self.t0 = float(t0)
        self.dt = float(dt)
        self.k0 = float(k0)
        self.cap = float(cap)
        self.last = float(cmd)
        self.cmd0 = float(cmd)
        self.t_last = None

    @property
    def enabled(self):
        return self.plan is not None and self.gain > 0.0

    def angle(self, t):
        """The plan's angle for wall-clock `t`, or None if there is no plan."""
        if not self.enabled:
            return None
        adv = self.gain * max(0.0, (float(t) - self.t0)) / self.dt
        if self.max_steps > 0.0:
            adv = min(adv, self.max_steps)
        f = min(self.k0 + adv, len(self.plan) - 1.0 - 1e-9)
        k = int(f)
        raw = self.plan[k] + (f - k) * (self.plan[k + 1] - self.plan[k])
        raw = float(np.clip(raw, -self.cap, self.cap))
        if self.rate_cap > 0.0:
            d = self.rate_cap * max(0.0, float(t) - self.t0)
            raw = float(np.clip(raw, self.cmd0 - d, self.cmd0 + d))
        if self.cmd_tau > 0.0 and self.last is not None:
            dt = (0.01 if self.t_last is None
                  else max(1e-4, float(t) - self.t_last))
            al = 1.0 - math.exp(-dt / self.cmd_tau)
            raw = self.last + al * (raw - self.last)
            raw = float(np.clip(raw, -self.cap, self.cap))
        self.t_last = float(t)
        self.last = raw
        self.pubs += 1
        return raw
