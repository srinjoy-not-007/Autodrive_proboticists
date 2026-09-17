# Finals tuning — SRL 2026 IROS Track (`2026-iros-compete`)

Submission due **Sep 19 AoE**. Every checkpoint has a gate; a knob that does not
pass its gate is reverted, not kept "because it is faster". Numbers marked
*model* are offline estimates from `tools/reprofile.py` or the relative lap
model — they are not driven results.

## Starting point (Sep 17)

- Line `finals_mesh_race.csv`: 44.82 m, |kappa| max 1.821 (R 0.55 m, just under
  the 0.561 m lock), 0.377 m effective clearance, ideal **11.02 s**.
- Corridor half-width **0.57–0.70 m** — much narrower than Porto.
- Time budget of the ideal lap: straights (kappa < 0.2) are 63.5 % of the
  distance and 49 % of the time; tight corners (kappa > 0.8) are 5.8 % of the
  distance but 12 % of the time. 14 % of the distance sits on the 8.00 m/s cap.

## Target, honestly

8.0 s is ~27 % off the base ideal. On this geometry (*model*, ax/brake 7.1,
vmax 12):

| line flatness | ay 6.75 | ay 8.0 | ay 9.0 |
|---|---|---|---|
| as generated | 9.76 | 9.26 | 8.94 |
| kappa x0.85 | 9.28 | 8.82 | 8.53 |
| kappa x0.70 | 8.76 | 8.35 | 8.09 |
| kappa x0.60 | 8.39 | **8.01** | 7.76 |

8.0 s needs a line ~40 % flatter **and** planned ay ~8. On Porto the car was
measured already at the tyre limit (driven p99 9.6 m/s2) and every ay rung above
6.75 was slower; the corridor here is too narrow to flatten a line 40 %. So the
evidence-backed target is **~9.5–10 s**, and 8 s should be treated as a stretch
that needs something we have not yet found.

## Checkpoints

- [ ] **0. Baseline** — race line, shipping mpc3 args + pick
      (`barrier_margin 0.40`, `thr_brake_lead_s 0.05`). 32 laps Xvfb, then 32
      on the real display. Record race time, collisions, dt_p50, cross-track
      p95, speed deficit vs plan, achieved ay p95/p99 from the driven path,
      and where every collision happened.
      *Gate:* 0 collisions on both. If not: `--min-clearance 0.25`, then
      `safe`, before any tuning.

      **Xvfb run 1 (Sep 17, `race_20260917_134454.csv`) — gate FAILED.**
      32 laps, **3 collisions**, median 11.862 s, best 11.390 s, OFFICIAL
      118.094 s (first 10 scored laps clean). dt_p50 **0.083 s** — faster than
      the usual 0.09–0.10 Xvfb band. Cross-track p50 0.095 / p95 **0.336 m**
      (Porto: 0.21), car 0.26 m/s under plan on average, driven median 7.6 %
      over the 11.02 s ideal (Porto: 0.6 %).
      **All three collisions are at one corner, s 18.97–19.67 m, xy ≈
      (0.7, −15.6).** Curvature there ramps 0.28 → 1.27 → 1.60 in 0.4 m of
      arclength; two of the three hits arrived OVER plan speed (2.17 and
      1.83 m/s against 1.74), with cte 0.31–0.35 m. Planned body clearance at
      that corner is 0.35–0.58 m, so the line is not the problem — the car
      runs wide on a sharp entry. Same shape as Porto's s ≈ 16 corner, and the
      lever that made Porto's line clean was halving the entry ramp with
      `--dkappa-max` (→ `cr_dk0.8`). Try that before a lower clearance floor,
      which would not address a tracking error.

- [ ] **1. Unblock speeds above 9 m/s** — prerequisite for checkpoint 2.
      Two caps sit at 9.0 today: `SPEED_MAX` in `vehicle.py` (the encoder
      estimator rejects faster readings as glitches — and a reject-only
      estimator is exactly what latched the brake into reverse on Aug 30) and
      `max_step_mps 9.0` in the shipped localization args. Also unmeasured:
      the car's real top speed and whether the 0.040/m/s throttle map stays
      linear above 7 m/s (we have never commanded more than 0.28 throttle).
      Measure in the `-explore` world (open floor), then raise both caps.
      *Gate:* encoder speed within 5 % of IPS at 10 m/s; no reverse samples.

- [ ] **2. Longitudinal envelope** (biggest lever on this track — unlike
      Porto, it has straights). Re-profile the SAME line with `reprofile.py`:
      vmax 8 → 10 → 12, brake 4 → 5.5 → 7.0, ax 6.5 → 7.1. *Model*:
      10.98 → 10.50 s ideal all together. Negative throttle is reverse drive:
      watch the reverse-sample fraction on every brake rung.
      *Gate:* 0 collisions / 32 laps, reverse samples 0 %, and the driven lap
      improves by more than the run-to-run spread (~1–2 s per race on Xvfb).

- [ ] **3. Grip confirmation** — one rung only: ay 6.75 with `ay_budget`
      matched (*model* −0.35 s). Porto says this will not pay; confirm on this
      geometry rather than assume, then stop.
      *Gate:* driven achieved-ay p95 rises AND cte p95 does not.

- [ ] **4. Line** — the only lever that reaches below ~10 s.
      - The half-width floor binds (w_veh 0.36–0.66 all give the same line):
        try the `--min-halfwidth-ladder` rungs 0.50 and 0.45.
      - `--dkappa-max` refinement on the tight corner (s at kappa 1.82).
      - Do NOT use `open_corners --method minimax`: it flattens the peak by
        oscillating the line and the jerk limiter kills it (Porto: 8.02 →
        11.10 s ideal).
      *Gate:* effective clearance ≥ 0.28 m, checkpoint margin ≥ 0.35 m, gate
      occupancy ≤ 1, then driven 0/32.

- [ ] **5. Controller, per track** — `barrier_margin` 0.34 vs 0.40 (feasible
      margin is 0.385, so 0.40 is marginal here); `thr_brake_lead_s` 0.05 vs 0;
      `steer_tau` 0.02 vs 0.04 on the display.
      *Gate:* display 0/32.

- [ ] **6. Validation** — Gate C: 100 laps Xvfb, 0 collisions. Gate D: 32 laps
      on the real display, 0 collisions. Both on the final config, nothing
      changed afterwards.

- [ ] **7. Image and submission**
      - `docker build --build-arg FINALS_SLUG=finals_mesh`, and
        `PIT_CREW_MAP` default → `finals_mesh` in `autodrive_devkit.sh`
        (`tests/test_submission_assets.py -k FinalsSlug` must pass).
      - Bake the chosen controller args into the entrypoint.
      - `test_submission_image.sh` on Xvfb and `VISIBLE=1`, 0 collisions.
      - Push `bongracers/roboracer:iros2026-finals` (new tag — never overwrite
        the qualification tag), update the Docker Hub overview, submit the
        Phase 2 form.

**Fallback at every step:** the base race line with the pick args, as long as
checkpoint 0 passed. Freeze it as an image before starting checkpoint 2.
