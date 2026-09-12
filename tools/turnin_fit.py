#!/usr/bin/env python3
"""Fit the pose-stamp-to-yaw-rate delay from a `turnin_probe` log.

    .venv/bin/python tools/turnin_fit.py data/logs/turnin_<ts>.csv
    .venv/bin/python tools/turnin_fit.py --compare <display.csv> <xvfb.csv>

Per steering pulse it reports four numbers, all in seconds:

  ``dead_cmd_fb``   publish instant -> the rack starts moving
                    (`/steering` is the ACTUATOR angle, not the command).
                    STATUS's dead-time law, quoted from the publish instant,
                    is ``period + 0.045..0.052``; this column has the
                    zero-order hold taken off, so the law predicts
                    ``0.045..0.052`` here and the line printed next to it
                    shows the uncorrected number too.  TICK-QUANTISED: inside the
                    simulator the rack finishes each commanded increment
                    between ticks, so this can only ever be an integer number
                    of periods plus the bridge's constant.  Read it as a
                    confirmation of that law, not as a sub-sample number.
  ``lag_fb_yaw``    rack starts moving -> the yaw rate starts rising.
                    Plant, not transport: it should NOT scale with the period.
  ``slew``          rad/s of the observed feedback ramp.  With the default
                    ramp excitation that is ``ramp_radps`` (the rack is
                    tracking, not rate-limiting); with ``ramp_radps:=0`` it is
                    the 3.2 rad/s slew itself.
  ``total``         the pose stamp that triggered the pulse -> the yaw rate
                    rising, i.e. the full loop the phase shift lives in.
                    Measured as (yaw onset - COMMAND onset) + pose age, both
                    onsets from the same estimator, so the zero-order hold's
                    one-period offset cancels instead of masquerading as a
                    rate dependence.
  ``cmd_zoh``       the correction applied (one period).  ``total_raw`` is the
                    number before it and ``total_ticks`` the same thing in
                    periods -- inside the simulator everything is
                    tick-synchronous, so an honest single-rate answer is an
                    integer number of ticks plus a sub-tick constant, and only
                    two rates can separate the two.

``total`` is the headline and the only one that is genuinely sub-sample.  The
yaw rate is a continuous state the physics integrates at its own fixed step,
so a least-squares line through the rising yaw samples, extrapolated back to
the pre-pulse baseline, puts the onset between samples.  That needs a rising
edge several samples long, which is why `turnin_probe` RAMPS the command
instead of stepping it (a 0.15 rad step is over inside half a period).  The
residual error is the ramp fit's, roughly ``sigma / slope``; the standard
error printed per column is over pulses, and with 3 pulses a lap and a few
minutes of driving it is comfortably under the 0.03 s in question.

`--compare` is the actual question: two logs at two sensor periods, same
commands, same speeds.  If `total` differs by the ~0.03-0.055 s the arc-length
cross-correlation implied, it is here; if `dead_cmd_fb` accounts for all of it
the cause is the command path (the bridge's next-tick emit), and if
`lag_fb_yaw` does, it is the vehicle/physics step.
"""

import argparse
import csv
import sys

import numpy as np

STREAM = {'pose', 'fb', 'imu', 'ips', 'enc'}


# ----------------------------------------------------------------- loading

def load(path):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            if r.get('src') not in STREAM:
                continue
            try:
                rows.append({k: (r[k] if k in ('src', 'phase') else float(r[k]))
                             for k in r})
            except (TypeError, ValueError):
                continue
    rows.sort(key=lambda r: r['t_recv'])
    return rows


def period_of(rows):
    """The sensor period this log ran at, from the accepted pose stamps."""
    ts = np.array([r['pose_stamp'] for r in rows if r['src'] == 'pose'])
    ts = np.unique(ts)
    d = np.diff(ts)
    d = d[(d > 1e-3) & (d < 1.0)]
    return float(np.median(d)) if len(d) else float('nan')


def series(rows, src, tcol, vcol):
    """Unique (t, v) samples of one signal at its own arrival instants."""
    out, seen = [], set()
    for r in rows:
        if r['src'] != src:
            continue
        t = r[tcol]
        if t <= 0.0 or t in seen:
            continue
        seen.add(t)
        out.append((t, r[vcol]))
    out.sort()
    return np.array([p[0] for p in out]), np.array([p[1] for p in out])


# -------------------------------------------------------------------- fit

def ramp_onset(t, y, t0, base, sign, window=1.0, lo=0.15, hi=0.80,
               min_amp=0.01):
    """When did `y` leave `base`, to better than one sample?

    Select the rising edge by FRACTION OF THE PLATEAU, never by an absolute
    threshold.  The car is tracking a line while it waits, so neither the
    steering feedback nor the yaw rate is quiet before the pulse: an absolute
    dead band either latches onto pre-pulse drift (and reports an onset that
    is metres too early) or, made large enough not to, eats the bottom of the
    ramp.  Taking the samples between ``lo`` and ``hi`` of the plateau keeps
    only samples that are unambiguously on the edge.

    Times come in as unix seconds (~1.8e9) with 0.058 s spacing -- a relative
    spacing of 3e-11 -- so the fit is done relative to ``t0`` and only the
    answer is put back; fitting in absolute time is rank-deficient and returns
    a zero slope.

    Returns (onset, slope, n_edge), or (nan, nan, 0) with no usable edge.
    """
    m = (t >= t0 - 1e-9) & (t <= t0 + window)
    tt, yy = t[m] - t0, (y[m] - base) * sign
    if len(tt) < 3:
        return float('nan'), float('nan')
    peak = float(np.max(yy))
    if not np.isfinite(peak) or peak < min_amp:
        return float('nan'), float('nan')
    k = int(np.argmax(yy >= hi * peak))          # first sample at the top
    band = np.flatnonzero((yy[:k + 1] >= lo * peak) & (yy[:k + 1] <= hi * peak))
    if len(band) < 2:
        # One sample on the edge: draw the secant from the last sample still
        # below `lo`.  Biased late by under half a sample and never silent --
        # `n_edge` in the per-pulse table says when this happened.
        if not len(band) or band[0] == 0:
            return float('nan'), float('nan')
        band = np.array([band[0] - 1, band[0]])
    A = np.vstack([tt[band], np.ones(len(band))]).T
    slope, c = np.linalg.lstsq(A, yy[band], rcond=None)[0]
    if not np.isfinite(slope) or slope <= 1e-6:
        return float('nan'), float('nan')
    # `yy` is already sign-corrected, so the slope is the MAGNITUDE of the
    # ramp -- alternating the pulse sign must not cancel it in the median.
    return float(t0 - c / slope), float(slope), len(band)


def _onset(*a, **kw):
    r = ramp_onset(*a, **kw)
    return r if len(r) == 3 else (r[0], r[1], 0)


def pulses(rows, period=None, tail_s=0.30):
    """One record per pulse: the trigger instant and the three onsets.

    The zero-order hold is corrected for, once, here.  The simulator consumes
    one command per telemetry tick and samples the plant on the same tick, so
    the plant integrates the HELD ramp, not the ramp that was asked for; a
    linear fit of a tick-sampled response to a tick-held ramp comes out one
    whole period late.  One period is 0.046 s more at 0.104 s than at
    0.058 s -- the same size and sign as the phase shift under investigation --
    so leaving it in would manufacture the answer.  ``total_raw`` keeps the
    uncorrected number, and the correction's own uncertainty (the alignment
    between the command apply and the telemetry emit, at most half a period)
    is why the RATE COMPARISON, where the correction is a known function of
    the period, is the robust output and a single absolute number is not.
    """
    if period is None:
        period = period_of(rows)
    zoh = period if np.isfinite(period) else 0.0
    t_fb, v_fb = series(rows, 'fb', 't_fb', 'steer_fb')
    t_im, v_im = series(rows, 'imu', 't_imu', 'gyro_z')
    t_ip, v_ip = series(rows, 'ips', 't_ips', 'ips_wz')
    out = []
    ids = sorted({int(r['pulse_id']) for r in rows
                  if r['src'] == 'pose' and r['phase'] == 'pulse'
                  and int(r['pulse_id']) > 0})
    for pid in ids:
        p = [r for r in rows
             if r['src'] == 'pose' and int(r['pulse_id']) == pid
             and r['phase'] == 'pulse']
        if not p:
            continue
        first = p[0]           # the pose callback that started the pulse
        t_pub, stamp = first['t_pub'], first['pose_stamp']
        # With the default RAMP excitation the first published value is still
        # zero -- that is the point, it dates the command's own onset -- so
        # the sign and the height come from the largest command in the pulse,
        # not from the first one.
        top = max(p, key=lambda r: abs(r['cmd_steer']))
        sign = 1.0 if top['cmd_steer'] >= 0 else -1.0
        amp = abs(top['cmd_steer'])
        v = float(np.median([r['v_enc'] for r in p])) or float('nan')
        # Baselines from the quiet 0.3 s before the step.
        def base_of(t, y):
            # The value the signal was HELD at when the ramp started -- the
            # probe adds its excitation to the tracking command rather than
            # replacing it, so this really is a level, not a drift to average.
            m = (t < t_pub) & (t > t_pub - 0.25)
            return float(np.median(y[m])) if m.any() else 0.0
        b_fb, b_im, b_ip = (base_of(t_fb, v_fb), base_of(t_im, v_im),
                            base_of(t_ip, v_ip))
        # What the plant delays are measured FROM: not `t_pub`, but `t_pub`
        # plus the zero-order hold (see the function docstring).
        o_cm = t_pub + zoh
        # Analyse only the pulse itself plus a short tail.  A fixed window
        # would run past the revert, where the response swings hard the other
        # way; that negative excursion is bigger than the plateau and makes
        # the plateau -- and therefore the band the edge is picked from --
        # meaningless.  This is what broke the 0.104 s case: at a 0.18 s loop
        # delay the ramp only just finishes before the revert.
        window = (p[-1]['t_pub'] - t_pub) + tail_s
        # Quality gate, the offline twin of the node's own settle gate: a
        # pulse whose yaw rate was still moving before it started cannot be
        # dated, because the fit's baseline is not a baseline.  Keep it in the
        # per-pulse table, leave it out of the medians.
        pre = (t_im < t_pub) & (t_im > t_pub - 0.25)
        pre_span = float(np.ptp(v_im[pre])) if pre.sum() >= 2 else float('inf')
        o_fb, slew, n_fb = _onset(t_fb, v_fb, t_pub, b_fb, sign, window)
        o_im, k_im, n_im = _onset(t_im, v_im, t_pub, b_im, sign, window)
        o_ip, _, _ = _onset(t_ip, v_ip, t_pub, b_ip, sign, window)
        out.append(dict(
            pulse=pid, s=first['s'], v=v, amp=amp, sign=sign,
            t_pose=stamp, t_pub=t_pub,
            pub_lat=t_pub - first['t_recv'],
            pose_age=t_pub - stamp,
            cmd_zoh=zoh,
            total_raw=(o_im - t_pub) + (t_pub - stamp),
            total_ticks=((o_im - t_pub) / period
                         if np.isfinite(period) and period > 0 else float('nan')),
            dead_cmd_fb=o_fb - o_cm,
            lag_fb_yaw=o_im - o_fb,
            total=(o_im - o_cm) + (t_pub - stamp),
            total_ips=(o_ip - o_cm) + (t_pub - stamp),
            slew=slew, yaw_slope=k_im, n_fb=n_fb, n_im=n_im,
            pre_span=pre_span, quiet=float(pre_span < 0.25)))
    return out


def summarise(name, all_recs, period):
    recs = [r for r in all_recs if r['quiet']] or all_recs

    def col(k):
        a = np.array([r[k] for r in recs], float)
        return a[np.isfinite(a)]
    print(f'\n== {name}')
    print(f'   pulses {len(recs)} of {len(all_recs)} quiet enough to date   '
          f'sensor period {period:.4f} s   '
          f'speed {np.nanmedian(col("v")):.2f} m/s   '
          f'amp {np.nanmedian(col("amp")):.3f} rad')
    for k, unit in (('pose_age', 's'), ('cmd_zoh', 's'), ('dead_cmd_fb', 's'),
                    ('lag_fb_yaw', 's'), ('total', 's'), ('total_raw', 's'),
                    ('total_ticks', 'periods'), ('total_ips', 's'),
                    ('slew', 'rad/s')):
        a = col(k)
        if not len(a):
            print(f'   {k:<12} --')
            continue
        se = a.std(ddof=1) / np.sqrt(len(a)) if len(a) > 1 else float('nan')
        extra = ''
        if k == 'dead_cmd_fb' and np.isfinite(period):
            # STATUS's law is quoted from the publish instant, before the
            # zero-order-hold correction: report both so they are comparable.
            extra = (f'   (t_pub->rack {np.median(a) + period:.4f} = period '
                     f'{period:.3f} + {np.median(a):+.4f})')
        print(f'   {k:<12} median {np.median(a):+.4f} {unit}  '
              f'mean {a.mean():+.4f}  sd {a.std(ddof=1) if len(a) > 1 else 0:.4f}'
              f'  se {se:.4f}  n {len(a)}{extra}')
    return {k: float(np.median(col(k))) if len(col(k)) else float('nan')
            for k in ('dead_cmd_fb', 'lag_fb_yaw', 'total', 'total_ips')}


def analyse(path, quiet=False):
    rows = load(path)
    period = period_of(rows)
    recs = pulses(rows, period)
    if not recs:
        print(f'{path}: no complete pulse found', file=sys.stderr)
        return period, recs, {}
    med = summarise(path, recs, period) if not quiet else {}
    return period, recs, med


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--compare', action='store_true',
                    help='two logs at two sensor periods: print the deltas')
    ap.add_argument('--per-pulse', action='store_true')
    a = ap.parse_args()

    results = []
    for path in a.logs:
        period, recs, med = analyse(path)
        results.append((path, period, recs, med))
        if a.per_pulse:
            print('   pulse      s     v   dead_cmd_fb  lag_fb_yaw     total  '
                  'slew  n_fb n_im')
            for r in recs:
                print(f'   {r["pulse"]:5d} {r["s"]:6.2f} {r["v"]:5.2f}  '
                      f'{r["dead_cmd_fb"]:+10.4f}  {r["lag_fb_yaw"]:+10.4f}  '
                      f'{r["total"]:+8.4f}  {r["slew"]:5.2f}  {r["n_fb"]:4d} '
                      f'{r["n_im"]:4d}')

    if a.compare or len(results) > 1:
        print('\n== rate comparison (later log minus the first)')
        p0, m0 = results[0][1], results[0][3]
        for path, period, _recs, med in results[1:]:
            print(f'   {path}   period {period:.4f} vs {p0:.4f} '
                  f'(d {period - p0:+.4f} s)')
            for k in ('dead_cmd_fb', 'lag_fb_yaw', 'total'):
                d = med.get(k, float('nan')) - m0.get(k, float('nan'))
                share = d / (period - p0) if abs(period - p0) > 1e-6 else float('nan')
                print(f'     d {k:<12} {d:+.4f} s   '
                      f'({share:+.2f} x the period change)')
            print('   A `total` that moves with the period is transport; one '
                  'that does not is the plant.')


if __name__ == '__main__':
    main()
