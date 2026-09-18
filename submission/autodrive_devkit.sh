#!/bin/bash
# Start once per container; additional interactive shells never start a race.
set -eo pipefail
# The controller's numpy solves are 20x30 matrices; OpenBLAS spins every core
# on them (measured: mpc3 at 1030% CPU), starving the simulator and the
# bridge on the same host. One thread is faster for this size anyway.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
cd /home/autodrive_devkit

# Exact map from the simulator mesh (tools/map_from_mesh.py) with the LiDAR's
# 0.1933 m forward offset applied in the filter: measured, this removes the
# 0.12-0.17 m inside bias the recon map had in the hairpins. A recon map built
# with PIT_CREW_LIDAR_OFFSET=0 must be run with PIT_CREW_LIDAR_OFFSET=0.
# FINALS (Sep 18): the SRL 2026 IROS track ships as the default map; set
# PIT_CREW_MAP=/home/data/maps/porto_mesh to race the qualification track.
MAP="${PIT_CREW_MAP:-/home/data/maps/finals_mesh}"
SLUG="$(basename "$MAP")"
LIDAR_OFFSET="${PIT_CREW_LIDAR_OFFSET:-0.1933}"
# Wide min-curvature line on the mesh map, driven by mpc3 (see STATUS, Sept 12
# night). PIT_CREW_CONTROLLER=mpc selects the legacy tracker, which needs the
# centerline-like `safe` line; do not pair mpc with the wide line.
CONTROLLER="${PIT_CREW_CONTROLLER:-mpc3}"
# The line default FOLLOWS THE MAP's slug. It used to be two fixed Porto paths,
# so changing PIT_CREW_MAP alone (the documented finals option A) left the
# legacy branch pointing at `track_v2_safe.csv` -- Porto's recon line on the
# finals map. Preflight catches that, but it catches it by refusing to start:
# the container exits 2 and there is no race at all. Deriving from $SLUG means
# one variable changes the whole set, and a track we have no line for fails
# with a name, not with a wrong-track geometry error.
#
# `porto_mesh` keeps its explicitly validated wide line; every other slug gets
# the drill's own output names (`<slug>_race.csv` / `<slug>_safe.csv`), which
# `scripts/finals_drill_mesh.sh` writes.
if [[ -n "${PIT_CREW_LINE:-}" ]]; then
    LINE="$PIT_CREW_LINE"
elif [[ "$CONTROLLER" == mpc3 && "$SLUG" == porto_mesh ]]; then
    LINE=/home/data/racelines/porto_mesh_cr_dk0.8.csv
elif [[ "$CONTROLLER" == mpc3 && "$SLUG" == finals_mesh ]]; then
    # Sep 19: the finals race geometry with the ay 6.75 speed profile except
    # over s 23.5-28.5 and s 34.5-41.5, where the frozen (ay 6.0) speeds are
    # blended back in, PLUS the s 36-42 corner shifted 0.08 m to the inside
    # with its exit capped at 3.0 m/s (`tools/fix_s39_finals.py`).  Pure ay
    # 6.75 ran 110.83 s clean on the display but took 10 collisions in 24 Xvfb
    # laps (0.098 s), all running wide at s 38.6; the un-shifted hybrid passed
    # 100 Xvfb laps but the built image still hit that corner once in 12 (wall
    # margin 0.195 m).  This line: 100 Xvfb laps / 0 / 118.07 s with 0.566 m
    # margin there, display 32 / 0 / 111.59 s (frozen `finals_mesh_race.csv`:
    # 119.51 / 114.04 s).
    LINE=/home/data/racelines/finals_mesh_hyb675b_race.csv
elif [[ "$CONTROLLER" == mpc3 ]]; then
    LINE="/home/data/racelines/${SLUG}_race.csv"
else
    LINE="/home/data/racelines/${SLUG}_safe.csv"
fi
# PREPARED, NOT ENABLED: on a circuit tighter than Porto the validated mpc3
# line is usually `safe`, not `race` (ICRA 2026 ships `safe`; both 2025
# circuits only produce a fast line under the drill's relative kappa gate).
# Do NOT leave the default pointing at a `race.csv` nobody drove.
#
# Preferred fix, and no edit here at all: make the derived name BE the
# validated line before the build --
#     cp data/racelines/<slug>_mesh_safe.csv data/racelines/<slug>_mesh_race.csv
# One file, one COPY, and tests/test_submission_assets.py then checks the exact
# file the entrypoint will load. The alternative is to change the mpc3 branch
# ABOVE (line with `${SLUG}_race.csv`) to `${SLUG}_safe.csv` -- inside the
# branch, so that an explicit PIT_CREW_LINE still wins over it.
# The centerline is derived from the MAP and never from a fixed Porto path.
# It is not a diagnostic: localization seeds its entire particle cloud at
# centerline[0] and resamples along it for EVERY collision recovery, so a
# centerline from another track is a race with no laps in it.
CENTER="${PIT_CREW_CENTERLINE:-/home/data/racelines/${SLUG}_centerline.csv}"
SCALE="${PIT_CREW_SPEED_SCALE:-1.00}"
MODE="${PIT_CREW_MODE:-raceline}"
if [[ "$MODE" != raceline && "$MODE" != reactive ]]; then
    echo "Invalid PIT_CREW_MODE: $MODE" >&2; exit 2
fi
if [[ "$CONTROLLER" != mpc && "$CONTROLLER" != mpc3 ]]; then
    echo "Submission supports mpc or mpc3; use PIT_CREW_MODE=reactive for FTG" >&2
    exit 2
fi
if [[ "$MODE" == raceline ]]; then
    # Name every missing asset by its own name before preflight tries to parse
    # it: `np.loadtxt` on an absent path is an OSError traceback, which on the
    # finals day reads as "our stack crashed" rather than "that file was never
    # COPYed into the image".
    for f in "$MAP.pgm" "$MAP.yaml" "$LINE"; do
        if [[ ! -f "$f" ]]; then
            echo "missing race asset for $SLUG: $f" >&2
            echo "  (is it in the Dockerfile's COPY list? racelines are listed" \
                 "one by one, never globbed)" >&2
            exit 2
        fi
    done
    if [[ ! -f "$CENTER" ]]; then
        echo "no centerline for $SLUG: $CENTER" >&2; exit 2
    fi
    # Preflight now also cross-checks that the line and the centerline lie on
    # THIS map and that the map's own lidar_offset matches the one we localize
    # with -- the two finals mistakes that no per-file check can see.
    python3 -m pit_crew.preflight "$MAP" "$LINE" "$CENTER" "$SCALE" "$LIDAR_OFFSET"
fi

BRIDGE=""
WORKERS=()
# mpc3 and mpc2 take the corridor map explicitly; the legacy mpc does not
# declare the parameter and would refuse to start if it were passed. Deriving
# it from the raceline NAME (the old behaviour) silently disabled the corridor
# barrier whenever the guess did not resolve.
CTRL_ARGS=()
if [[ "$CONTROLLER" == mpc3 ]]; then
    # rate_adaptive scales the timing terms by the measured sensor period
    # (identity at 0.104 s). Without it mpc3 collided 7/32 laps at the
    # 0.088-0.092 s instances; with it and the cr_dk0.8 line, 100 laps
    # clean at 0.091-0.098 s (70.5 s). Sept 12.
    # w_dcmd + cmd_tau (Sept 12): damp the solver's steering limit cycle.
    # Display (0.058 s): 11 collisions/15 laps -> 10/27 with the first ten
    # scored laps taking one; Xvfb (0.095 s): 32 laps clean, 71.17 s vs
    # 70.31 s without (the price of robustness at an unknown period).
    # On a TIGHT circuit cmd_tau costs more (+0.5 s/lap: CDC-TF 116.7 s vs
    # 111.1 s); `-p steer_tau:=0.04` recovers 4.7 s of that (112.0 s) and is
    # opt-in via PIT_CREW_CONTROLLER_ARGS, pending its display run.
    CTRL_ARGS=(-p map_prefix:="$MAP" -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10)
    # --- PREPARED, NOT ENABLED: the real-display corridor margin (Sept 12) ---
    # The organisers run the simulator WINDOWED ON A REAL DISPLAY at Ultra.
    # Measured on this host that is a 0.058 s telemetry period against the
    # 0.088-0.106 s Xvfb gives, and at 0.058 s four successive configurations
    # of this tracker took 10-11 collisions, always at the same hairpin entry,
    # drifting 0.34-0.37 m inside their own line.
    #
    # `barrier_margin` (default 0.24) is the corridor half-width the MPC's
    # barrier keeps between the BODY CORNERS and the wall. Raising it to 0.34
    # moves that hairpin offset outward at every rate (+0.056 m at 0.058 s) for
    # about +0.07 s/lap, and it is the ONLY configuration that has ever driven
    # the real display clean: 32 laps, 0 collisions, OFFICIAL 69.44 s at
    # 0.059 s -- the best official race time measured to date. Margin is thin
    # (worst lap -0.313 m against a wall at ~-0.35 m).
    #
    # FEASIBILITY, before enabling: it needs 0.135 + 0.34 = 0.475 m of
    # half-width. Measured available on every `safe` line (worst 0.520 m at the
    # CDC-TF S-kink; lap sum -0.02 s, 1 collision in 432 harness laps), and NOT
    # available on the fast width-using lines or on Porto's own line
    # (0.40-0.42 m), where the barrier is infeasible and merely biases the car
    # toward the centre -- which is what Porto has been running all along.
    # Generate a tight-circuit line with `--min-halfwidth 0.48` if both are to
    # ship together (tools/make_track_lines.py; see docs/FINALS_RUNBOOK.md 3a).
    #
    # Uncomment the next line -- ONE line, here, nowhere else -- once its
    # four-track Xvfb regression and the finals track's Gate D are clean.
    # steer_tau 0.04 (Sept 12): first-order rack lag in the rollout. Display
    # 32/0 69.36 s (with the barrier); recovers the smoothing's lap cost on
    # tight circuits (CDC-TF 116.7 -> 112.0, ICRA 2026 150.4 -> 148.3);
    # ICRA 2025 fast2_m50 32/0 140.4 s. Keep at 0.04: 0.06+ is a plant
    # mismatch that collides in the harness.
    CTRL_ARGS+=(-p steer_tau:=0.04)
    # ----------------------------------------------------------------------
    # AUDIT NOTE (Sep 19): the next TWO `barrier_margin` statements -- the
    # 0.34 default and the `case *cr_dk0.8.csv` line -- are DEAD on every
    # slug. The FINALS line further down appends `barrier_margin:=0.40`
    # unconditionally, and ROS takes the LAST `-p name:=value`, so the
    # effective value is 0.40 everywhere (verified by
    # tests/test_entrypoint_args.py::TestTheEffectiveArgsAreTheFrozenFinalsSet).
    # They are kept because the Sept 12 provenance below reads in order and
    # deleting them is a behaviour-neutral edit to a Gate-D'd script; do not
    # read either of them as the value that ships.
    # ----------------------------------------------------------------------
    CTRL_ARGS+=(-p barrier_margin:=0.34)  # enabled Sept 12: display 32/0 x2 (69.44, 69.36 s), Porto 100/0 Xvfb 71.50 s, CDC-TF/ICRA25/ICRA26 32/0 each
    # PREPARED (Sept 12 evening, not enabled): barrier 0.40 on Porto and on
    # safe lines. Display 32/0 70.13 s with the hairpin most-inside worst
    # -0.296 m (0/32 laps below -0.30) vs 16/32 at 0.34, for +0.8 s of race;
    # a line's feasible margin is min half-width - 0.135 (Porto 0.285, safe
    # lines 0.385-0.585, the fast2/fast3 lines 0.345), and above it the
    # barrier pushes across the line, so keep 0.34 on fast lines. Enable
    # after the Porto 100-lap Xvfb run at 0.40 (chain6):
    # 0.40 only on Porto's cr_dk0.8 (validated: display 32/0 70.13 s, Xvfb 100/0
    # 71.91 s). The 2025 safe lines' feasible cap is 0.385, so they stay at 0.34
    # until a live regression at 0.40 exists; pick_config.py encodes the rule.
    case "$LINE" in *cr_dk0.8.csv) CTRL_ARGS+=(-p barrier_margin:=0.40) ;; esac
    #
    # --- PREPARED, NOT ENABLED: carry the drill's own pick in the image -----
    # Both decisions above are GEOMETRIC, and on an unseen circuit they are
    # decided by the line the drill generates, not by anything known now. The
    # `case` above pattern-matches a FILENAME; on the finals track the name
    # that matters (`<slug>_race.csv`) says nothing about how much room the
    # line has, and 0.40 above a line's feasible cap makes it worse.
    #
    # `tools/pick_config.py <slug>` -- stage 5/5 of scripts/finals_drill_mesh.sh
    # -- measures the shipping line's min half-width (marched along the path
    # normal, the way the barrier itself measures it) and its tightest
    # curvature, and writes data/racelines/<slug>_pick.json with both decisions
    # already taken: `barrier_margin` 0.40 only where the line's feasible cap
    # (min half-width minus the 0.135 m car half-width) clears it with slack,
    # else 0.34; `thr_brake_lead_s` 0.05 on a tight circuit (tightest kappa
    # > 1.2 or corridor min width < 1.3 m), 0 otherwise -- never 0.08, which is
    # bit-identical to 0.05 at 0.097 s and costs on every other rate.
    #
    # Uncommenting the block between the markers lets the FINALS IMAGE carry
    # that pick as one COPYed asset instead of an edit to this file on the day.
    # It is appended AFTER the built-ins above, so the sidecar's
    # `barrier_margin` overrides both the 0.34 default and the `case` line
    # (last `-p` wins), and BEFORE `EXTRA_CTRL_ARGS`, so a finals-day
    # `-e PIT_CREW_CONTROLLER_ARGS=...` still wins over all of them. A missing
    # or unreadable sidecar changes nothing at all. Read the sidecar before
    # enabling this on a SAFE line: the measured caps there are 0.385-0.585, so
    # on the two 2025 circuits the rule lands on 0.34 where the `case` line
    # above is currently giving them 0.40.
    # tests/test_pick_config.py runs this exact block against a fixture.
    # FINALS (Sep 18, SRL 2026 IROS, 2026-iros-compete): the simulator brakes
    # only when throttle == 0.0 exactly (VehicleController.cs); this stack had
    # never published 0.0. brake_mode zero + the pick (barrier 0.40, brake
    # lead 0.05) on finals_mesh_race: real display (0.058 s) 32 laps / 0
    # collisions / 114.04 s, where every config without the brake took 10
    # collisions in 11-14 laps at the s 19 entry. Applied on every map.
    # barrier 0.40 here is a deliberate exception to pick_config's cap rule
    # (this line's feasible cap is 0.385): the display A/B at 0.34 vs 0.40
    # was indistinguishable (both DQ without the brake, both clean with it),
    # and 0.40 is the value every clean display run was measured at.
    CTRL_ARGS+=(-p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05)
    CTRL_ARGS+=(-p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6)
    # Sep 19: no-anticipation publish gate on straights (the "swivel"): clips
    # the published angle to line-matching + 0.05 rad while no corner is within
    # 1.5 m and |e| <= 0.25.  Straight steering std 0.0242 -> 0.0181 rad live,
    # time-neutral: display 32/0/111.65 s, 100 Xvfb laps / 0 / 117.11 s on
    # hyb675b (111.59 / 118.07 without).
    CTRL_ARGS+=(-p pub_gate_corr:=0.05)
    # PICK_BLOCK_BEGIN
    # PICK="${PIT_CREW_PICK:-/home/data/racelines/${SLUG}_pick.json}"
    # if [[ -f "$PICK" ]]; then
    #     PICK_ARGS="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("controller_args",""))' "$PICK" 2>/dev/null || echo "")"
    #     echo "pick for $SLUG: ${PICK_ARGS:-<none>}" >&2
    #     # shellcheck disable=SC2206
    #     CTRL_ARGS+=(${PICK_ARGS})
    # fi
    # PICK_BLOCK_END
    #
    # REJECTED, do not resurrect: `-p apply_floor_steps:=0.0 -p rate_terms:=...`
    # (the old "fix 1"). It removed the command-slew saturation at 0.058 s
    # exactly as the harness predicted (0.078 -> 0.003) and changed NOTHING at
    # the hairpin: 15 laps, 11 collisions, same corner, same cross-track. With
    # the `w_dcmd`/`cmd_tau` smoothing above, the staircase it targeted is gone
    # (0.0% over-slew commands) and the fix now costs. Also rejected on the
    # display: `nd_mode:=period`, `lead_mode:=spatial`, `steer_tau` alone,
    # `w_dcmd` without `cmd_tau`. See docs/FINALS_RUNBOOK.md 5D.
elif [[ "$CONTROLLER" == mpc2 ]]; then
    CTRL_ARGS=(-p map_prefix:="$MAP")
fi
# Built-in localization parameters beyond the three every race needs. Separate
# from EXTRA_LOC_ARGS so that a finals-day `-e PIT_CREW_LOCALIZATION_ARGS=...`
# still comes LAST and still wins.
LOC_ARGS=()
# --- PREPARED, NOT ENABLED: the long-straight speed cap (Sept 12) -----------
# The motion model's `max_step_m` 0.6 m per update is a 5.77 m/s speed cap at
# `rate_nominal` 0.104. On SRL 2025 ICRA, which has a 13.9 m stretch above that
# (Porto 3.3 m, ICRA 2026 10.3, CDC-TF 11.7), the estimate fell ~4 m behind and
# the car drove into the wall reading cte 0.03: 4 laps, 14 collisions, a DQ.
# The harness reproduces it exactly and is clean at cap >= 6.4, at zero lap
# cost. `publish_on_reloc:=false` suppresses the one pose after a global reinit
# (the centroid of a track-wide cloud is not a pose). Uncomment once the live
# A/B (`icra2025_step9`) plus the Porto and ICRA-2026 regressions are back.
# ENABLED Sept 12 (the line below is live, not prepared): ICRA 2025 went from
# 4 laps / 14 collisions to 32 laps / 0 collisions / 148.8 s, with Porto
# (70.31 s) and ICRA 2026 (148.9 s) unchanged. The cap is now 9.0 m/s; a finals
# profile that asks for MORE than that on a long straight needs it raised again
# -- Gate B in docs/FINALS_RUNBOOK.md measures exactly that, and
# `-e PIT_CREW_LOCALIZATION_ARGS="-p max_step_mps:=12.0"` is the override.
# Caveat: a higher cap also lets the cloud dead-reckon further forward after a
# collision reset (up to 0.89 m in the worst measured window).
LOC_ARGS+=(-p max_step_mps:=9.0 -p publish_on_reloc:=false)
#
# --- PREPARED, NOT ENABLED: telemetry-gap handling (Sept 12) ----------------
# If the organisers' box stalls the simulator (OBS plus a rosbag on the same
# host), two 0.5 s constants coincide and the filter dead-reckons the whole gap
# at the STALE pre-gap speed: 0.9 m of phantom travel for a car that did not
# move, 2.25 m of error and 9 relocalizations for a 1 s telemetry-only gap.
# Replayed on the real collision bag (25 injected stalls x 6 seeds) these bring
# the worst post-gap error to 0.46 m. Widening the cloud works; capping the
# step throws away real motion.
#
# DRIVEN LIVE Sept 12 (real display, Ultra, 0.058 s, docker-pause stalls from
# scripts/stall_inject.sh), and the two halves only work together:
#   * `gap_predict_s:=0.6` ALONE (`stall_gap` arm, 8 sim pauses of 0.4-0.63 s):
#     32 laps, 0 collisions -- but one 21.2 s lap. Seven seconds after the
#     second stall the car STOPPED at s 27.1 for 13 s (throttle 0, steering
#     held, no collision): localization declared itself lost after the stall
#     and, with `publish_on_reloc:=false`, published no pose at all, so the
#     controller had nothing to solve against and resumed only by luck.
#   * BOTH (`stall_both` arm, 8 sim pauses of 0.4-0.6 s): **32 laps, 0
#     collisions, longest lap 7.54 s, OFFICIAL 70.18 s** -- i.e. the same race
#     as the no-stall control (70.13 s). The filter rides the gap out instead
#     of losing itself, so the controller's ride-out has a pose to ride on.
# Enable BOTH lines or NEITHER; the controller line alone is the 13 s stop.
# `gap_noise_mult` is a DOUBLE: it must be written `3.0`, not `3` -- rclpy
# rejects an integer override for a double parameter and the node exits on
# launch (a turn-in probe lost a run to exactly that today).
# The controller line is mpc3-only; the legacy `mpc` does not declare
# `gap_predict_s` and would refuse to start if it were passed.
# Uncomment BOTH lines between the markers once chain8's no-stall Xvfb
# regression (Porto + one 2025 circuit) is clean.
# tests/test_gap_block.py lifts these two lines, uncomments them and runs the
# launch functions with them, and instantiates both real nodes with exactly
# these overrides.
# GAP_BLOCK_BEGIN
# CTRL_ARGS+=(-p gap_predict_s:=0.6)
# LOC_ARGS+=(-p gap_dt:=0.15 -p gap_noise_mult:=3.0 -p lost_patience_min:=3 -p scan_dt_max:=1.2 -p enc_gap_max:=1.5)
# GAP_BLOCK_END
#
# --- PREPARED, NOT ENABLED: teleport detection after a collision reset ------
# The reset moves the car ~0.88 m and nothing notices: the encoder keeps
# reading (+2.1 m/s over-read for 0.4 s) and the filter never declares itself
# lost. The SCAN branch caught 9/9 real resets within one frame at 0 false
# positives on the collision bag (clean max 0.194 m, so 0.35 is the threshold);
# the IMU branch is unusable (1/9, and one frame early) -- keep it at 0.
# Reseeding at checkpoints helped on 0/9 and costs 0.06 m, so hold-only; the
# image also carries no `checkpoints_csv`, which reseeding would need.
# It is ONE line on purpose: a commented `LOC_ARGS+=(` split over two lines is
# an unbalanced array literal, so uncommenting only the first half is a bash
# syntax error and the container never starts. Every commented `*_ARGS+=` line
# in this file must stay individually balanced --
# tests/test_entrypoint_args.py::TestACommentedArgLineCanBeUncommentedAlone.
# TELEPORT_BLOCK_BEGIN
# LOC_ARGS+=(-p teleport_scan_jump:=0.35 -p teleport_imu_g:=0.0 -p teleport_hold_n:=7 -p teleport_reseed:=false)
# TELEPORT_BLOCK_END
# Extra `-p name:=value` parameters for the finals, without editing this file.
# Word-split ON PURPOSE (the only unquoted expansions here) so
# PIT_CREW_CONTROLLER_ARGS="-p rate_adaptive:=true -p ilc_enable:=true" works;
# they are appended verbatim at EVERY launch site, initial and supervisor
# restart, because a node that comes back without its flags is a silent
# mid-race configuration change.
EXTRA_CTRL_ARGS=()
EXTRA_LOC_ARGS=()
# shellcheck disable=SC2206
if [[ -n "${PIT_CREW_CONTROLLER_ARGS:-}" ]]; then
    EXTRA_CTRL_ARGS=(${PIT_CREW_CONTROLLER_ARGS})
fi
# shellcheck disable=SC2206
if [[ -n "${PIT_CREW_LOCALIZATION_ARGS:-}" ]]; then
    EXTRA_LOC_ARGS=(${PIT_CREW_LOCALIZATION_ARGS})
fi

# The reactive stack's parameter file, for PIT_CREW_MODE=reactive and for the
# supervisor's last-resort fallback. The default is the MAPPING profile
# (v_max 1.0 m/s): deliberately slow, and the only profile carrying the
# stuck/escape keys that unwedge a car nobody is watching. `ftg.yaml` is the
# faster racing profile (v_max 1.5) with no escape behaviour. Overridable
# because the right answer depends on how tight the finals circuit is, and
# because Follow-the-Gap is genuinely weak on a narrow one (24 collisions in
# 194 s on SRL 2025 ICRA, which is a DQ).
FTG_PARAMS="${PIT_CREW_FTG_PARAMS:-/home/autodrive_devkit/src/pit_crew/config/ftg_mapping.yaml}"
if [[ "$MODE" == reactive && ! -f "$FTG_PARAMS" ]]; then
    echo "missing Follow-the-Gap parameter file: $FTG_PARAMS" >&2; exit 2
fi

LOC_PID=""; CTRL_PID=""; FTG_PID=""
start_loc() {
    setsid ros2 run pit_crew localization --ros-args \
        -p map_prefix:="$MAP" -p centerline_csv:="$CENTER" \
        -p lidar_offset:="$LIDAR_OFFSET" "${LOC_ARGS[@]}" \
        "${EXTRA_LOC_ARGS[@]}" &
    LOC_PID=$!
}
start_ctrl() {
    setsid ros2 run pit_crew "$CONTROLLER" --ros-args \
        -p raceline_csv:="$LINE" -p speed_scale:="$SCALE" "${CTRL_ARGS[@]}" \
        "${EXTRA_CTRL_ARGS[@]}" &
    CTRL_PID=$!
}
start_ftg() {
    setsid ros2 run pit_crew follow_the_gap --ros-args \
        --params-file "$FTG_PARAMS" &
    FTG_PID=$!
}
stop_group() { kill -TERM -- "-$1" 2>/dev/null || true; }
shutdown() {
    code=$?
    trap - EXIT INT TERM
    # Stop producers first, including ros2-run descendants. Keep the bridge
    # alive while final commands drain so latched controls are cleared.
    for pid in "${WORKERS[@]}"; do stop_group "$pid"; done
    sleep 0.3
    for pid in "${WORKERS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    timeout 4 ros2 topic pub -r 20 --times 20 \
        /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
    ZERO_T=$!
    timeout 4 ros2 topic pub -r 20 --times 20 \
        /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
    ZERO_S=$!
    wait "$ZERO_T" "$ZERO_S" 2>/dev/null || true
    if [[ -n "$BRIDGE" ]]; then stop_group "$BRIDGE"; fi
    exit "$code"
}
trap shutdown EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 launch autodrive_roboracer bringup_headless.launch.py &
BRIDGE=$!
if [[ "$MODE" == reactive ]]; then
    start_ftg
    WORKERS=("$FTG_PID")
else
    start_loc
    start_ctrl
    WORKERS=("$LOC_PID" "$CTRL_PID")
fi

# A child exiting, even successfully, is a failed race stack -- but a dead
# node must not be a dead RACE. The organisers run all 12 laps in one go, so a
# single unhandled exception used to cost every remaining lap and the whole
# score. Restart ONLY the node that died (restarting localization needlessly
# would throw away a converged particle filter mid-lap), a bounded number of
# times, then fall back to the reactive stack, which needs no map, no line and
# no localization at all. A dead ROS bridge is the one thing not recoverable
# from here.
#
# `wait -n` reaps exactly one child, so `kill -0` afterwards identifies it.
RESTARTS=0
MAX_RESTARTS=4
while :; do
    result=0
    wait -n "$BRIDGE" "${WORKERS[@]}" || result=$?
    if ! kill -0 "$BRIDGE" 2>/dev/null; then
        echo "ROS bridge exited (status $result); shutting down" >&2
        exit 1
    fi
    RESTARTS=$((RESTARTS + 1))
    sleep 0.5                       # never spin on a node that dies instantly
    if [[ "$MODE" == reactive ]]; then
        if (( RESTARTS > MAX_RESTARTS )); then break; fi
        echo "follow_the_gap exited ($result); restart $RESTARTS" >&2
        start_ftg
        WORKERS=("$FTG_PID")
        continue
    fi
    if (( RESTARTS > MAX_RESTARTS )); then
        echo "race stack failed $MAX_RESTARTS times; falling back to \
follow_the_gap" >&2
        for pid in "${WORKERS[@]}"; do stop_group "$pid"; done
        sleep 0.3
        MODE=reactive
        start_ftg
        WORKERS=("$FTG_PID")
        RESTARTS=0
        continue
    fi
    kill -0 "$LOC_PID" 2>/dev/null || { echo "localization exited ($result); \
restart $RESTARTS" >&2; start_loc; }
    kill -0 "$CTRL_PID" 2>/dev/null || { echo "$CONTROLLER exited ($result); \
restart $RESTARTS" >&2; start_ctrl; }
    WORKERS=("$LOC_PID" "$CTRL_PID")
done
echo "Reactive fallback exited too; shutting down" >&2
exit 1
