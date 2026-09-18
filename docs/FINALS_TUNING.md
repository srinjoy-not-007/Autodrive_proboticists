# Finals tuning — SRL 2026 IROS Track (`2026-iros-compete`)

Submission due **Sep 19 AoE = Sep 20, 17:30 IST** (about 48 h from the Sep 18
evening baseline). Gates are therefore the full ones: 100 Xvfb laps + 32
real-display laps per promoted change, ay rungs climbed one at a time. Every checkpoint has a gate; a knob that does not
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

- [x] **0. Baseline** -- PASSED Sep 18 with the brake: display (0.058 s) 32 laps
      / 0 collisions / **114.04 s** (`race_20260918_162222.csv`, throttle exactly
      0 on 29% of samples through the s 19 entry, cut +0.19 p50 / +0.25 worst).
      Frozen into the entrypoint (finals_mesh default map, brake_mode zero +
      pick args). Xvfb regression: finals 32/0/119.5 s (`race_20260918_163645`);
      Porto with the same finals args 32/0/75.1 s (4 s slower than Porto's own
      tune -- expected, the qualifier is done). **Image `bongracers/roboracer:
      iros2026-finals` built from this tree and raced the organisers' way on
      Xvfb: 12 laps / 0 collisions / 119.57 s, both nodes up, extra-shell
      check clean -- the FALLBACK SUBMISSION, ready to push.** Improvement
      A/Bs on the display from here (brake_a_min 2.5, brake lead 0.08, then
      the ay / vmax rungs as the agents rank them). — race line, shipping mpc3 args + pick
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
      **CORRECTED — the first attribution was wrong.** xy (0.72, −15.57) is
      *checkpoint 7*: the row where `collision_count` steps is the respawn,
      not the impact. With the c-2 rule (`tools/racelog.py`) every collision
      is at **s 18.07–18.57, the entry to the left-hand corner at s ≈ 19**,
      with the car 0.33–0.53 m **inside** (left of) the line.

      **Run 2, `finals_mesh_dk08` (`--dkappa-max 0.8`, `race_20260917_135943`)
      — FAILED, 4 collisions**, OFFICIAL 119.07 s + 20 s. Same entry, same
      side. The refinement lowered the corner's peak kappa 1.60 → 1.40 but
      `|dkappa/ds|` stayed 4.77 against a requested 0.8 (25 passes gave the
      identical line): the 0.53 m half-width floor leaves no room to move.

      **Mechanism, measured on every lap (not just the crashes):** the car's
      true signed offset grows +0.04 m (s 16.5) → +0.28 (s 18.0–18.5) →
      **+0.39 m median, +0.48 p95 (s 18.5–19.0)**, then swings −0.25 m wide
      on the exit. The line's planned body clearance on that approach is
      only 0.20–0.30 m, so the collisions are the tail of a cut that happens
      every lap. It is the tracker, not the line and not localization:
      localization lateral error is p95 **0.069 m** (the large "pose error"
      first reported on the straights was along-track — logger pose age 82 ms
      at 8 m/s — computed with the TUM heading convention, 0 = +y, mistaken
      for 0 = +x). Same shape as the documented CDC-TF hairpin cut (~0.4 m
      inside at the s 39 entry on every lap).

      Levers for a corner-entry cut, in the order to try: `barrier_margin`
      is already 0.40, above this line's 0.385 feasible margin, so it is
      spent; next are mpc3's entry-timing knobs (`apply_lead_steps`, see the
      Sep 12 "entry cut is the publish lead" note; `nd_delay_s`; `steer_tau`
      0.02), then a line that sits further from the inside wall on the
      approach (clearance on s 17.5–18.5, not the lap minimum).

      **Sep 18, this host (RTX 5090).** Xvfb baseline, pick args:
      `race_20260918_154740.csv`, 32 laps, **0 collisions**, OFFICIAL
      117.13 s, dt_p50 0.094 s -- but the cut is unchanged: most-inside at
      s 17.5-19.5 per lap p50 **+0.444 m**, worst +0.496. The clean run is
      the tail sitting just under the wall at this period. **Real display
      (0.058 s): `race_20260918_155658.csv`, 11 laps, 10 collisions**, all
      at s 18.1-18.5, +0.35..+0.49 m inside, at 1.0-1.5 m/s true speed
      (crawling, and still steered into the inside wall). On the
      organisers' setup this config is a DQ; the s 19 entry is the single
      blocking problem. Live A/Bs in flight: the safe line on the display,
      the race line at barrier 0.34.

      **Display A/Bs (Sep 18): the line and the barrier are NOT the
      variable.** Safe line: 10 collisions / 14 laps, s 18.0-18.4, +0.37..
      +0.56 inside at 1.1-1.5 m/s. Race line at barrier 0.34: 10 / 13,
      same corner. Binned by s, the published steering is already +0.08..
      +0.14 rad over s 16.5-18.0 where the line's kappa is 0.03-0.11 (the
      1.60 hairpin starts at s 19.0): the turn-in is applied 1-2.5 m early
      on a curvature STEP, at BOTH periods (cte +0.35 display / +0.26 Xvfb
      at s 18.0). **The finding that changes the game: the sim brakes only
      when throttle == 0.0 exactly (VehicleController.cs) and our stack
      never publishes 0.0 (0.4% of samples).** With the brake modelled,
      the harness reproduces the display DQ for the first time (4-5
      collisions / 20 laps at s 18.3-18.5) and `brake_mode:=zero` gives
      0/20 with the entry offset +0.49 -> +0.25 (lap +0.7 s). Live A/B on
      the display running (brake, then apply_lead_steps 1.0). Also
      measured: the finals lines' curvature reverses 3.5-6.3 times per
      10 m on the straights (Porto 2.4) -- the user's "rapid rights and
      lefts on a straight road"; smoother candidates being generated.

      **Steering-side knobs ruled out (harness, 144-lap cells):**
      apply_lead_steps, nd_delay_s, steer_tau 0.02, barrier 0.30/0.34/0.44
      all leave the cut at +0.45 and collide at 0.058; apply_ahead_s 0 is
      catastrophic; removing the barrier entirely helps a little (12 -> 2
      collisions). The car is NOT arriving hot in the truth signal (under
      plan by 0.25-0.5 m/s into the entry); the command leads the geometry
      by 4x two metres before the corner. The line has 0.44-0.56 m of body
      clearance there -- the cut consumes all of it. `thr_brake_lead_s` is
      monotone and decisive: 0.05 -> 13 collisions at 0.058, 0.08 -> 0,
      0.12 -> 0 with the cut +0.30 (cost ~+0.5-0.8 s/lap at 0.058).
      **Live brake run (display, brake_mode zero): throttle exactly 0 on
      29% of samples through the entry, most-inside +0.19 p50 / +0.25
      worst (was +0.35..+0.49 at impact), 15 laps clean so far.**

      **Straight-noise correction (Sep 18):** the "3.5 sign changes / 10 m"
      figure was rounding dust of a geometrically dead-straight section
      (|kappa| <= 0.006 over s 2-15; 42% of masked samples under 0.02). The
      driven steering on the true straight is a 0.033 rad limit cycle plus
      the early turn-in bias from s 16 (+0.096 rad where the line asks
      0.03); a flatter line (Porto's 2.4/10 m) drives 1.2 s/lap SLOWER with
      MORE steering, and the safe line is worse still. Keep the race line;
      the swivel the user sees is the s 19 entry, i.e. the same problem
      the brake addresses. tools/smooth_straights.py exists (opt-in
      post-pass), not recommended.

      **From the simulator source (Sep 18 research, VehicleController.cs /
      TireFriction.cs):** BrakeTorque = MotorTorque (428 N.m), constant,
      applied only at throttle == 0.0 exactly; it locks the wheel in one
      physics step, so the HELD brake is the longitudinal asymptote 4.55
      m/s2 plus drag -- **plan brake <= 5.5, never 7.0**; brake_a_full 4.6
      is right. Drive torque is linear in throttle to 1.0 (11 m/s ~ 0.44)
      but constant torque means acceleration decays with speed (VAUL's
      planner uses ax_max_machines 5.3 tapering). Steering slews at 3.2
      rad/s in physics time, no lag, independent of graphics quality; Ultra
      does not change the physics rate. The lateral tyre is LINEAR to its
      extremum (slip 0.01, 9.81 m/s2); our "sustained 4.6-6.6" is
      chattering across the knee, and driven ay p99 is already 9.6, so the
      lever is planned ay 6.0 -> 6.75 -> 7.5 gated on driven ay p99 < 9.3
      (VAUL plans 12 flat). Warm-up lap is free: no special startup mode.
      Ranked: ay rungs (-4..-8 s/race est.), brake 5.5 profile (-1.5..-2.5),
      vmax 8 -> 11 with the two 9 m/s caps raised (-1..-3), thr_brake_lead_s
      0.08 if the cut returns (+5..+8 but avoids a DQ). TUM mintime: no.

- [x] **1. Unblock speeds above 9 m/s** -- DROPPED (Sep 18): the plant tops
      out at 7.9 m/s on the vmax-8 profile and 8.7 m/s when the plan asks
      11.2; the straights are too short for the ~4.5 m/s2 (decaying) drive
      ceiling, and the fraction of samples where the encoder would exceed
      9.0 is <= 0.1% in every harness cell. SPEED_MAX and max_step_mps never
      bind; raising them buys nothing and adds a real hazard (above 9 the
      estimator freezes then clamps, so the brake trigger fires late at the
      fastest point). No code change.

- [x] **2. Longitudinal envelope** -- MEASURED (Sep 18): the whole reprofile
      grid (vmax 8-12 x brake 4-7 x ax 6.5/7.1) spans 0.48 s of ideal lap; the
      honest ideal with a tapered ax (5.3 - 0.273 v) is 11.5 s, not 11.0.
      Harness at 0.058 (144-lap cells): frozen profile 0 collisions; brake
      5.5 -0.14 s/lap but 2 collisions at s 37.5 (a new corner, kappa ~1.0);
      brake 7.0 unphysical; ax 7.1 or vmax >= 9: 1-6 collisions per 36-108
      laps. brake_a_min 2.5 is a no-op (the brake_dv branch fires first).
      Any extra speed anywhere finds a wall somewhere. One rung worth a live
      slot: `lon_v8b55a65` (~-1.4 s/race at 0.058, -2.5 at 0.094, vs ~+1.4 s
      expected penalty). Lines committed under finals_mesh_lon_*.
 — prerequisite for checkpoint 2.
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

      **Retrace of the Porto 89.6 -> 69.2 s path, re-applied here (Sep 18):**
      of ~20 s gained on Porto, ~2.3 s was dead-time compensation, ~7.2 s
      the mesh map + LiDAR offset + the ay ladder, ~4.6 s mpc3 + the
      speed-cut rescale (e_free/k_e), ~3.7 s the wide line family, ~2 s
      rate_adaptive + the dkappa line, ~1 s the display smoothing/barrier
      work. Everything controller-generic already ships here. Per-track
      knobs re-measured on finals_mesh (corridor uniformly 1.34-1.36 m,
      one 18 m straight, a corner at the steering lock, feasible barrier
      cap 0.385): barrier 0.34 vs 0.40 indistinguishable (keep 0.40);
      steer_tau 0.02/0.03 no gain and worse clearance (keep 0.04); e_free
      0.25 -> 10 collisions/36, k_e 1.0 -> 5 (the defaults sit on a cliff:
      keep 0.18/2.0; k_e 3.0 = +0.025 m for +0.10 s/lap); ay_budget 6.5 no
      gain; max_step_mps 9.0 never binds (v_max 8, 0 m above 8.5 m/s);
      **thr_brake_lead_s 0.08 is NOT a no-op here** (pick_config's "never
      0.08" is a 0.097-s artefact): 0.0 -> 8 collisions/36 at 0.058, 0.05
      -> 0/36 at 0.050 m clearance, 0.08 -> 0/36 at 0.135-0.146 m for
      +0.37 s/lap. **The Porto cr_dk0.8 trick does not transfer**: every
      dkappa <= 1.0 line here has 0.500 min half-width and the best of
      them took 9/18 at 0.058 (clearance 0.000). w_veh is not a lever (the
      0.53 floor binds on all 8 rungs). The checkpoint/gate constraints
      cost 0.56 s of ideal for nothing (the shipping line already passes).
      **The lever left is planned ay on the shipping geometry**: 6.75 ->
      ideal 10.66, 7.5 -> 10.41, 8.0 -> 10.27 (shipping 10.99), all at the
      same clearance and gates; the harness cannot arbitrate (plant clips
      at 6.5), so: live, ascending, abort on driven ay p99 > 9.3 or cte
      p95 rising. Lines: finals_mesh_rt_dr2_ay{675,75,80}_race.csv.

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
