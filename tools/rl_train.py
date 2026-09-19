#!/usr/bin/env python3
"""PPO for the bounded residual policy, in numpy, on the log-fitted surrogate.

No torch: the competition image has numpy only, and a net small enough to run
there (18 -> 32 -> 32 -> 2, tanh) is small enough to differentiate by hand.
Training and inference therefore use the SAME arrays, so the export cannot
disagree with what was trained.

    tools/rl_train.py --iters 40 --episodes 12 --laps 4 --out data/rl/res_v1
    tools/rl_train.py --resume data/rl/res_v1.npz --iters 20
    tools/rl_train.py --eval data/rl/res_v1.npz          # surrogate A/B

Rollouts come from `tools/rl_env.py`, which drives the real `mpc3` control law
with the policy plugged in where `mpc3.MPC3Tracker.control` plugs one in.  The
plant is the pessimistic, domain-randomised variant of the identified
surrogate; see that module and docs/FINALS_TUNING.md section 4c for why
pessimism is not optional here.

HOST RULE: at most 6 single-threaded workers on cores 22-27, nice 19.
"""

import argparse
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src'))

import rl_env as E                                              # noqa: E402
from pit_crew import residual_policy as RP                      # noqa: E402

HID = (32, 32)


# ---------------------------------------------------------------------------
# a tanh MLP with hand-written gradients
# ---------------------------------------------------------------------------

def init_mlp(sizes, rng, out_scale=0.01):
    """Orthogonal-ish init; the OUTPUT layer starts tiny so the policy begins
    as the identity residual (zero steer delta, unit speed scale)."""
    Ws, bs = [], []
    for i in range(len(sizes) - 1):
        n_in, n_out = sizes[i], sizes[i + 1]
        scale = (out_scale if i == len(sizes) - 2
                 else math.sqrt(2.0 / n_in))
        Ws.append(rng.normal(0.0, scale, size=(n_in, n_out)))
        bs.append(np.zeros(n_out))
    return Ws, bs


def mlp_forward(Ws, bs, x, tanh_out):
    """Returns (output, cache).  `x` is (N, n_in)."""
    acts = [x]
    z = x
    n = len(Ws)
    for i in range(n):
        z = z @ Ws[i] + bs[i]
        if i < n - 1 or tanh_out:
            z = np.tanh(z)
        acts.append(z)
    return z, acts


def mlp_backward(Ws, acts, dy, tanh_out):
    """Gradients for (Ws, bs) given d(loss)/d(output).  `acts[i]` is layer i's
    INPUT (post-activation of the previous layer)."""
    n = len(Ws)
    gW = [None] * n
    gb = [None] * n
    d = dy
    for i in range(n - 1, -1, -1):
        out = acts[i + 1]
        if i < n - 1 or tanh_out:
            d = d * (1.0 - out * out)          # tanh'
        gW[i] = acts[i].T @ d
        gb[i] = d.sum(0)
        d = d @ Ws[i].T
    return gW, gb


class Adam:
    def __init__(self, params, lr=3e-4, b1=0.9, b2=0.999, eps=1e-8):
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def step(self, params, grads):
        self.t += 1
        for i, (p, g) in enumerate(zip(params, grads)):
            self.m[i] = self.b1 * self.m[i] + (1 - self.b1) * g
            self.v[i] = self.b2 * self.v[i] + (1 - self.b2) * g * g
            mh = self.m[i] / (1 - self.b1 ** self.t)
            vh = self.v[i] / (1 - self.b2 ** self.t)
            p -= self.lr * mh / (np.sqrt(vh) + self.eps)


# ---------------------------------------------------------------------------
# the agent
# ---------------------------------------------------------------------------

class Agent:

    def __init__(self, n_obs, rng, hid=HID, log_std=-1.6):
        self.n_obs = n_obs
        self.pW, self.pb = init_mlp((n_obs,) + tuple(hid) + (2,), rng)
        self.vW, self.vb = init_mlp((n_obs,) + tuple(hid) + (1,), rng,
                                    out_scale=0.1)
        self.log_std = np.full(2, float(log_std))
        self.obs_mean = np.zeros(n_obs)
        self.obs_std = np.ones(n_obs)
        self._cnt = 0
        self._m2 = np.zeros(n_obs)

    # -- obs normalisation, Welford, frozen into the export ---------------
    def observe(self, obs):
        for x in np.asarray(obs, float):
            self._cnt += 1
            d = x - self.obs_mean
            self.obs_mean += d / self._cnt
            self._m2 += d * (x - self.obs_mean)
        if self._cnt > 1:
            self.obs_std = np.sqrt(np.maximum(self._m2 / (self._cnt - 1),
                                              1e-8))

    def norm(self, obs):
        return (np.asarray(obs, float) - self.obs_mean) / np.maximum(
            self.obs_std, 1e-6)

    def mean(self, obs):
        y, _ = mlp_forward(self.pW, self.pb, self.norm(obs), True)
        return y

    def value(self, obs):
        y, _ = mlp_forward(self.vW, self.vb, self.norm(obs), False)
        return y[:, 0]

    def sample(self, obs1, rng):
        """One state.  Returns (action, logprob, value)."""
        o = np.atleast_2d(obs1)
        mu = self.mean(o)[0]
        std = np.exp(self.log_std)
        a = mu + std * rng.normal(size=2)
        lp = float(np.sum(-0.5 * ((a - mu) / std) ** 2 - self.log_std
                          - 0.5 * math.log(2 * math.pi)))
        return a, lp, float(self.value(o)[0])

    # -- flat parameter vector, for shipping to workers -------------------
    def get_flat(self):
        return np.concatenate([p.ravel() for p in
                               self.pW + self.pb + [self.log_std]])

    def set_flat(self, v):
        i = 0
        for p in self.pW + self.pb + [self.log_std]:
            n = p.size
            p[...] = v[i:i + n].reshape(p.shape)
            i += n

    def state(self):
        d = {f'W{k}': W for k, W in enumerate(self.pW)}
        d.update({f'b{k}': b for k, b in enumerate(self.pb)})
        d.update({f'vW{k}': W for k, W in enumerate(self.vW)})
        d.update({f'vb{k}': b for k, b in enumerate(self.vb)})
        d.update(log_std=self.log_std, obs_mean=self.obs_mean,
                 obs_std=self.obs_std, _cnt=np.array(self._cnt),
                 _m2=self._m2)
        return d

    @classmethod
    def from_state(cls, d):
        n_obs = int(np.asarray(d['obs_mean']).size)
        a = cls(n_obs, np.random.default_rng(0))
        a.pW = [np.asarray(d[f'W{k}']) for k in range(len(a.pW))]
        a.pb = [np.asarray(d[f'b{k}']) for k in range(len(a.pb))]
        if 'vW0' in d:
            a.vW = [np.asarray(d[f'vW{k}']) for k in range(len(a.vW))]
            a.vb = [np.asarray(d[f'vb{k}']) for k in range(len(a.vb))]
        a.log_std = np.asarray(d['log_std']).astype(float)
        a.obs_mean = np.asarray(d['obs_mean']).astype(float)
        a.obs_std = np.asarray(d['obs_std']).astype(float)
        a._cnt = int(np.asarray(d['_cnt'])) if '_cnt' in d else 0
        a._m2 = np.asarray(d['_m2']).astype(float) if '_m2' in d \
            else np.zeros(n_obs)
        return a


# ---------------------------------------------------------------------------
# rollout workers
# ---------------------------------------------------------------------------

_W = {}


def _worker_init(line, map_prefix):
    _W['track'] = E.Track(line, map_prefix)


def _worker(job):
    state, seed, laps, frame, deterministic, inflate, ct, wc = job
    track = _W['track']
    ag = Agent.from_state(state)
    rng = np.random.default_rng(seed)
    acts, lps, vals = [], [], []

    def act_fn(obs):
        if deterministic:
            a = ag.mean(np.atleast_2d(obs))[0]
            lp, v = 0.0, 0.0
        else:
            a, lp, v = ag.sample(obs, rng)
        acts.append(a)
        lps.append(lp)
        vals.append(v)
        return a, None

    dom = E.sample_domain(rng, frame=frame)
    r = E.rollout(track, act_fn, dom, laps=laps, wall_inflate=inflate,
                  clear_target=ct, w_clear=wc)
    if r.get('n', 0) == 0:
        return None
    n = len(acts)
    return dict(obs=r['obs'][:n], act=np.asarray(acts),
                logp=np.asarray(lps), val=np.asarray(vals),
                rew=r['reward'][:n], laps=r['laps'],
                collisions=int(r['hit'][:n].sum()),
                sim_collisions=r['collisions'],
                lap_med=float(np.median(r['lap_times']))
                if r['lap_times'] else float('nan'),
                min_clear=r['min_clear'], frame=dom['frame'])


def collect(pool, agent, seeds, laps, frames, deterministic=False,
            inflate=0.05, clear_target=0.30, w_clear=6.0):
    """COMMON RANDOM NUMBERS.

    The domain randomisation (period 0.055-0.110, grip, pose bias and noise,
    rack lag, breakaway point) moves the return by +-200 between iterations
    while a 0.05 s/lap policy improvement is worth about 0.8, so with fresh
    domains every iteration the learning signal is invisible.  The same
    (seed, frame) pairs are therefore reused for a block of iterations, and
    only rotated between blocks: the batch-to-batch difference is then the
    policy, which is the thing being differentiated.
    """
    state = agent.state()
    jobs = [(state, s, laps, f, deterministic, inflate, clear_target, w_clear)
            for s, f in zip(seeds, frames)]
    return [r for r in pool.map(_worker, jobs) if r is not None]


# ---------------------------------------------------------------------------
# PPO
# ---------------------------------------------------------------------------

def gae(rew, val, gamma, lam):
    n = len(rew)
    adv = np.zeros(n)
    last = 0.0
    for t in range(n - 1, -1, -1):
        nxt = val[t + 1] if t + 1 < n else 0.0
        delta = rew[t] + gamma * nxt - val[t]
        last = delta + gamma * lam * last
        adv[t] = last
    return adv, adv + val


def update(agent, batch, opt_p, opt_v, clip=0.2, epochs=6, mb=1024,
           ent_coef=0.002, rng=None):
    obs = batch['obs']
    act = batch['act']
    old = batch['logp']
    adv = batch['adv']
    ret = batch['ret']
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    n = len(obs)
    on = agent.norm(obs)
    stats = dict(kl=0.0, clipfrac=0.0, vloss=0.0, ploss=0.0)
    for _ in range(epochs):
        order = rng.permutation(n)
        for k in range(0, n, mb):
            sel = order[k:k + mb]
            x, a, lo, ad, rt = on[sel], act[sel], old[sel], adv[sel], ret[sel]
            mu, cache = mlp_forward(agent.pW, agent.pb, x, True)
            std = np.exp(agent.log_std)
            lp = np.sum(-0.5 * ((a - mu) / std) ** 2 - agent.log_std
                        - 0.5 * math.log(2 * math.pi), axis=1)
            ratio = np.exp(np.clip(lp - lo, -20.0, 20.0))
            clipped = np.clip(ratio, 1 - clip, 1 + clip)
            use_unclipped = (ratio * ad) <= (clipped * ad)
            # d(-min(r*A, clip(r)*A))/d(lp) = -r*A where the unclipped arm wins
            dlp = np.where(use_unclipped, -ratio * ad, 0.0) / len(sel)
            dmu = dlp[:, None] * (a - mu) / (std * std)
            gW, gb = mlp_backward(agent.pW, cache, dmu, True)
            # entropy bonus on log_std (raises exploration), plus its own term
            dlogstd = np.sum(dlp[:, None] * (((a - mu) / std) ** 2 - 1.0), 0)
            dlogstd -= ent_coef
            opt_p.step(agent.pW + agent.pb + [agent.log_std],
                       gW + gb + [dlogstd])
            agent.log_std[...] = np.clip(agent.log_std, -4.0, -1.2)

            v, vcache = mlp_forward(agent.vW, agent.vb, x, False)
            dv = 2.0 * (v[:, 0] - rt)[:, None] / len(sel)
            gvW, gvb = mlp_backward(agent.vW, vcache, dv, False)
            opt_v.step(agent.vW + agent.vb, gvW + gvb)
            stats['vloss'] = float(np.mean((v[:, 0] - rt) ** 2))
            stats['ploss'] = float(-np.mean(np.minimum(ratio * ad,
                                                       clipped * ad)))
            stats['kl'] = float(np.mean(lo - lp))
            stats['clipfrac'] = float(np.mean(np.abs(ratio - 1.0) > clip))
    return stats


# ---------------------------------------------------------------------------
# export
# ---------------------------------------------------------------------------

def export(agent, path, track_len, zones=E.FINALS_ZONES):
    d = {f'W{k}': W for k, W in enumerate(agent.pW)}
    d.update({f'b{k}': b for k, b in enumerate(agent.pb)})
    d.update(obs_mean=agent.obs_mean, obs_std=agent.obs_std,
             steer_max=np.float64(RP.STEER_MAX),
             v_lo=np.float64(RP.V_LO), v_hi=np.float64(RP.V_HI),
             kappa_gate=np.float64(RP.KAPPA_GATE),
             zones=np.asarray(zones, float),
             track_len=np.float64(track_len),
             preview_kappa_m=np.asarray(RP.PREVIEW_KAPPA_M, float),
             preview_wall_m=np.asarray(RP.PREVIEW_WALL_M, float))
    np.savez(path, **d)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--line',
                    default='data/racelines/finals_mesh_exitm_m10t6_h3_race.csv')
    ap.add_argument('--map', default='data/maps/finals_mesh')
    ap.add_argument('--iters', type=int, default=30)
    ap.add_argument('--episodes', type=int, default=12)
    ap.add_argument('--laps', type=int, default=4)
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--gamma', type=float, default=0.995)
    ap.add_argument('--lam', type=float, default=0.95)
    ap.add_argument('--block', type=int, default=10,
                    help='iterations per fixed domain block (common random '
                         'numbers; see collect())')
    ap.add_argument('--ent-coef', type=float, default=0.0)
    ap.add_argument('--clear-target', type=float, default=0.30,
                    help='footprint clearance below which the proximity '
                         'penalty starts, m')
    ap.add_argument('--w-clear', type=float, default=6.0,
                    help='weight of that penalty, per metre short')
    ap.add_argument('--wall-inflate', type=float, default=0.05,
                    help='metres the wall is inflated by for the CONTACT test '
                         '(see rl_env: 0.10 makes contact unavoidable on this '
                         'line and trains "go slower")')
    ap.add_argument('--log-std', type=float, default=-2.3,
                    help='initial action std, log; -2.3 = 0.10 of the bound')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default='data/rl/residual')
    ap.add_argument('--resume', default='')
    ap.add_argument('--eval', default='')
    ap.add_argument('--eval-episodes', type=int, default=24)
    a = ap.parse_args()
    line = a.line if os.path.isabs(a.line) else os.path.join(ROOT, a.line)
    mp = a.map if os.path.isabs(a.map) else os.path.join(ROOT, a.map)
    os.makedirs(os.path.dirname(os.path.join(ROOT, a.out)) or '.',
                exist_ok=True)
    rng = np.random.default_rng(a.seed)
    track = E.Track(line, mp)

    if a.eval:
        pol = RP.ResidualPolicy(a.eval)
        print('eval only: see tools/surrogate_race.py -r residual_policy=...')
        print(pol.n_obs, 'inputs,', len(pol.layers), 'layers')
        return

    if a.resume:
        agent = Agent.from_state(np.load(a.resume))
    else:
        agent = Agent(RP.N_OBS, rng, log_std=a.log_std)
    opt_p = Adam(agent.pW + agent.pb + [agent.log_std], lr=a.lr)
    opt_v = Adam(agent.vW + agent.vb, lr=a.lr * 3)

    FRAMES = [0.055, 0.065, 0.080, 0.094, 0.104, 0.110]
    seeds = frames = None
    with ProcessPoolExecutor(max_workers=a.workers, initializer=_worker_init,
                             initargs=(line, mp)) as pool:
        for it in range(a.iters):
            t0 = time.perf_counter()
            if it % a.block == 0:
                seeds = [int(rng.integers(0, 2 ** 31 - 1))
                         for _ in range(a.episodes)]
                frames = [FRAMES[i % len(FRAMES)] for i in range(a.episodes)]
            eps = collect(pool, agent, seeds, a.laps, frames,
                          inflate=a.wall_inflate,
                          clear_target=a.clear_target, w_clear=a.w_clear)
            if not eps:
                print('no episodes came back'); break
            advs, rets = [], []
            for e in eps:
                ad, rt = gae(e['rew'], e['val'], a.gamma, a.lam)
                advs.append(ad); rets.append(rt)
            batch = dict(
                obs=np.concatenate([e['obs'] for e in eps]),
                act=np.concatenate([e['act'] for e in eps]),
                logp=np.concatenate([e['logp'] for e in eps]),
                adv=np.concatenate(advs), ret=np.concatenate(rets))
            st = update(agent, batch, opt_p, opt_v, rng=rng,
                        ent_coef=a.ent_coef)
            # Only AFTER the update: the workers' log-probs were computed with
            # the normalisation the state carried, and moving it under them
            # would corrupt the importance ratio.
            for e in eps:
                agent.observe(e['obs'])
            ret_mean = float(np.mean([e['rew'].sum() for e in eps]))
            lap = float(np.nanmedian([e['lap_med'] for e in eps]))
            coll = int(np.sum([e['collisions'] for e in eps]))
            simcoll = int(np.sum([e['sim_collisions'] for e in eps]))
            print('it %3d  ep %2d  return %8.1f  lap_med %6.3f  near-wall %3d '
                  ' sim-coll %2d  std %.3f  kl %+.4f  %.0f s'
                  % (it, len(eps), ret_mean, lap, coll, simcoll,
                     float(np.exp(agent.log_std).mean()), st['kl'],
                     time.perf_counter() - t0), flush=True)
            np.savez(os.path.join(ROOT, a.out) + '_ckpt.npz', **agent.state())
            export(agent, os.path.join(ROOT, a.out) + '.npz', track.total)
    print('wrote', os.path.join(ROOT, a.out) + '.npz')


if __name__ == '__main__':
    main()
