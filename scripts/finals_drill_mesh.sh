#!/bin/bash
# The finals drill, without the recon lap.
#
# `new_track_drill.sh` spends its first 150 s driving an unknown circuit blind
# with Follow-the-Gap so it has a trajectory to build a map from. Everything
# that drill then does is an attempt to recover from what that recon costs:
# pose error baked into the map, teleports where the recon collided, walls the
# LiDAR never saw, and -- twice -- a recon that did not go round at all and
# produced a map of part of the track.
#
# None of that is necessary when the simulator image ships the track. The
# collision mesh IS the map (`tools/map_from_mesh.py`), and the scene's own
# ordered Checkpoints are a better lap definition than a blind lap ever was
# (`extract_centerline.traj_from_checkpoints`). So this drill never starts the
# simulator: it reads the image, rasterises the geometry, and generates lines.
#
# Usage: ./scripts/finals_drill_mesh.sh <slug> <sim_tag> "<track name substring>"
#   e.g. ./scripts/finals_drill_mesh.sh icra2026 2026-iros-practice "2026 ICRA"
#        ./scripts/finals_drill_mesh.sh finals   2026-iros-finals   "Finals"
#
# List what a given image contains first:
#   .venv/bin/python tools/map_from_mesh.py --tag <sim_tag> --list
#
# IF THE IMAGE HAS NO MESH for the finals track -- `--list` shows nothing that
# matches, or the mesh has no "<stem> Checkpoints"/"<stem> Spawn Points" root,
# or `map_from_mesh.py` exits before writing a .pgm -- then this drill cannot
# run and the recon drill is the fallback:
#
#   FTG_ARGS='-p bubble_radius:=0.22 -p extend_margin:=0.18' \
#     ./scripts/new_track_drill.sh <slug> <sim_tag> 150
#
# That path is ~271 s of wall clock plus a live sim; this one is ~45 s and no
# sim at all, so try this first and keep the recon drill in reserve.
set +e
cd "$(dirname "$0")/.."
SLUG="${1:?usage: finals_drill_mesh.sh <slug> <sim_tag> \"<track name substring>\"}"
TAG="${2:-2026-iros-practice}"
NAME="${3:-$SLUG}"
MAP="data/maps/${SLUG}_mesh"
T0=$(date +%s)
stage() { echo ""; echo "=== [$(( $(date +%s) - T0 ))s] $* ==="; }

stage "1/5 rasterise the collision mesh out of ${TAG}"
# docker create + docker cp against an image that is never started: this cannot
# disturb a running simulator, and the extracted assets are cached in $TMPDIR.
.venv/bin/python tools/map_from_mesh.py --tag "$TAG" "$NAME" "$MAP" || {
    echo ""
    echo "No usable mesh for '${NAME}' in ${TAG}."
    echo "Check what the image actually has:"
    echo "  .venv/bin/python tools/map_from_mesh.py --tag ${TAG} --list"
    echo "If the finals track is genuinely absent, fall back to the recon drill:"
    echo "  ./scripts/new_track_drill.sh ${SLUG} ${TAG} 150"
    exit 1
}

stage "2/5 sanity-check the map before anything is built on it"
# A mesh map that came out wrong is silent: it still writes a .pgm. The two
# ways it goes wrong are a missing parent transform (every wall 10 by 14 m out
# of place, so the spawn lands outside the corridor and the flood fill leaves
# the whole grid unknown) and a track whose mesh carries a floor (the whole
# surface rasterises as solid). Both show up as a free fraction far from the
# 30-55% every known circuit produces.
.venv/bin/python - "$MAP" <<'PYEOF' || exit 1
import sys
import numpy as np, yaml
from PIL import Image
p = sys.argv[1]
img = np.array(Image.open(p + '.pgm'))
free = float(np.mean(img == 254))
print(f'  free {free:.1%}, occupied {np.mean(img == 0):.1%}, '
      f'unknown {np.mean(img == 205):.1%}')
if not 0.15 <= free <= 0.70:
    sys.exit(f'  free fraction {free:.1%} is outside the 15-70% every known '
             f'circuit produces -- the mesh or its transform is wrong. '
             f'Re-run map_from_mesh.py with --report and --list.')
meta = yaml.safe_load(open(p + '.yaml'))
assert meta.get('source') == 'mesh', 'map is not stamped source: mesh'
print(f"  lidar_offset {meta['lidar_offset']}, origin {meta['origin'][:2]}")
PYEOF

stage "3/5 centerline + swept racing lines (no recon: checkpoints stand in)"
# make_track_lines.py sees `source: mesh` in the YAML and switches itself over:
# no wall dilation (the walls are already closed, so the corridor keeps the
# 0.15 m the recon repair used to throw away), a trajectory synthesised from
# the scene's checkpoints, a w_veh sweep that reaches far enough to buy back
# clearance, extra lateral-shift smoothing, and the provisional 0.22 m
# clearance floor. Both centerline methods are built and every candidate is
# gated on drivability and measured body clearance, exactly as before.
#
# These flags used to be a paragraph in the runbook that someone had to read
# and re-type on the day, after reading the 3/5 gate output and noticing that
# every `raceline` candidate had been REJECTED on kappa. They are the default
# now, because an unseen circuit tighter than Porto is the case the drill
# exists for and the fallback (silently shipping the centerline) is a whole
# second per lap:
#
#   --kappa-gate safe    a raceline may not ask for more curvature than the
#                        steering lock or than the FALLBACK line already
#                        demands. The absolute 1.0 1/m bar rejects everything
#                        on a circuit whose own centerline peaks at 2.10
#                        (cdctf2025) or 1.63 (icra2025) -- i.e. it rejects the
#                        EASIER line.
#   --dkappa-max 1.5     bound the curvature RATE, which is what the tracker
#                        actually cannot follow.
#   --kappa-bound 1.0    keep the QP itself off the lock.
#   --min-halfwidth-ladder
#                        floor the path-to-wall distance at every generation
#                        stage, not just in the QP. 0.53 is what absorbs the
#                        tracker's hairpin cut (CDC-TF: it cuts ~0.4 m inside
#                        on EVERY lap at the s 39 entry, and 2 collisions in
#                        32 laps were the 2.2-sigma tail of it); 0.50 is the
#                        floor both 2025 circuits solve at; 0.45 is the last
#                        rung before the safe line. The ladder takes the FIRST
#                        rung that still yields an accepted candidate, and if
#                        none does, make_track_lines ships `safe` by itself.
#
#   --checkpoints auto --gate-separation 0.60
#                        OPT-IN, not in the default set. The lap counter keeps
#                        ONE checkpoint latch (LapTimer.cs) and overwrites
#                        PreviousCheckpoint on every trigger EXIT, so a body
#                        inside two trigger boxes at once can drop a
#                        checkpoint and the lap is not scored at all --
#                        measured once in 32 laps on cdctf2025 fast3_m53,
#                        whose gates 18-20 sit 0.39-0.48 m apart along the
#                        line. 0.60 m = body 0.50 + box 0.10. Stage 5 below
#                        MEASURES this on whatever line was chosen and exits 2
#                        if it is short, so add these two flags and regenerate
#                        when it does. On cdctf2025 the constraint costs the
#                        whole width-using raceline (every candidate then
#                        fails the kappa gate) and the safe line is what
#                        ships, so it is a deliberate choice, not a default.
#
# `LINE_ARGS=` (explicitly empty) restores the old flagless behaviour; setting
# it to anything else replaces the set wholesale.
DRILL_LINE_ARGS='--dkappa-max 1.5 --dkappa-refine 3 --kappa-bound 1.0
                 --kappa-gate safe --min-halfwidth-ladder 0.53,0.50,0.45'
# shellcheck disable=SC2086
# mktemp, NOT data/logs: that directory is a bind mount the containers
# create as root, so on a real host the tee fails, the log never exists,
# and the grep below silently decides no retry is needed. Seen on the
# 2026-iros-compete drill (Sep 17).
LINE_LOG="$(mktemp -t "drill_lines_${SLUG}.XXXXXX")"
echo "  line-generation log: $LINE_LOG"
.venv/bin/python tools/make_track_lines.py "${SLUG}_mesh" \
    ${LINE_ARGS-$DRILL_LINE_ARGS} 2>&1 | tee "$LINE_LOG"
[ "${PIPESTATUS[0]}" = "0" ] || exit 1
[ -s "$LINE_LOG" ] || { echo "line-generation log is empty ($LINE_LOG);"\
    " cannot tell whether the floor was cleared" >&2; exit 1; }

# CORNER-OPENING RETRY. `--open-kappa` is deliberately NOT a default: on both
# tuned tracks it changes the line that ships (Porto picks an anchored raceline
# at 7.32 s instead of safe; ICRA-2026 picks an opened line at 13.66 s instead
# of safe at 14.08 s), and those are configurations that have been driven. But
# on a circuit whose SKELETON has a sharp vertex it is the difference between a
# line that clears the evidence floor and one that does not: the medial axis of
# a corridor with a sharp outer wall vertex has a sharp vertex too, however wide
# the corridor -- SRL 2024 IROS carries |kappa| 5.42 (R 0.18 m) at a point
# 1.415 m from the nearest wall, so every min-curvature candidate must cusp
# there. Measured on that track: without it, 20/20 candidates rejected and the
# drill ships safe at 0.265 m effective, UNDER the 0.28 m floor and explicitly
# unproven; with it, the opened line ships at 0.372 m and 12.36 s ideal.
#
# So: only retry when the first pass failed to clear the floor, and only keep
# the retry if it actually did better. A track that already clears the floor is
# never touched, which is why this is safe to leave switched on for the day.
if grep -q "nothing cleared the" "$LINE_LOG"; then
    stage "3b/5 nothing cleared the clearance floor -- retrying with corner opening"
    # 1.0 / 0.45 is the measured operating point and it is a CLIFF, not a slope:
    # on SRL 2024 IROS target 1.0 converges before touching the wall floor
    # (|kappa| 5.42 -> 1.11, wall distance 0.483 m). Every more aggressive
    # setting tried -- target 0.9, 0.8, 0.6, 0.45, and floors 0.50/0.40/0.35 --
    # drives the line ONTO the floor, where the projection back inside the
    # corridor puts the kink straight back: |kappa| 1.97-2.84, all worse. Do not
    # "tune" these two numbers on the day; they get worse in every direction.
    # shellcheck disable=SC2086
    .venv/bin/python tools/make_track_lines.py "${SLUG}_mesh" \
        ${LINE_ARGS-$DRILL_LINE_ARGS} --open-kappa 1.0 --open-wall-floor 0.45 \
        2>&1 | tee "${LINE_LOG}.open"
    [ "${PIPESTATUS[0]}" = "0" ] || exit 1
    if grep -q "nothing cleared the" "${LINE_LOG}.open"; then
        echo "  corner opening did not clear the floor either -- the line that"
        echo "  ships is still UNPROVEN. Drive it before trusting it and keep"
        echo "  PIT_CREW_MODE=reactive as the fallback."
    else
        echo "  corner opening cleared the floor; that is the line now in"
        echo "  data/racelines/${SLUG}_mesh_race.csv"
    fi
fi

stage "4/5 footprint clearance check on both lines we might race"
# `tools/footprint_analysis.py <map> <csv>` cannot read a RACELINE csv: its
# positional branch is `np.genfromtxt(names=True)`, which wants an `x`/`y`
# header, and a raceline's header is `# s_m,x_m,y_m,...`.  Every drill run has
# therefore printed two tracebacks here instead of the report Gate A asks for
# (pre-existing, all tracks -- `set +e` hid it).  Rather than leave the gate
# blind, call the module's OWN functions on the line's xy columns, which is
# exactly what its internal "RACELINE (as planned)" section does.
.venv/bin/python - "$MAP" "data/racelines/${SLUG}_mesh_race.csv" \
    "data/racelines/${SLUG}_mesh_safe.csv" <<'PYEOF' || exit 1
import sys
sys.path.insert(0, 'tools')
import numpy as np
from footprint_analysis import load_dist, analyse
prefix = sys.argv[1]
dist, res, ox, oy = load_dist(prefix)
for path in sys.argv[2:]:
    print(f'--- {path.split("/")[-1]}')
    analyse(path.split('/')[-1], np.loadtxt(path, delimiter=',')[:, 1:3],
            dist, res, ox, oy)
PYEOF

stage "5/5 the config this line needs (barrier margin and brake lead)"
# Two of the finals decisions are geometric and were being taken by hand off
# docs/STATUS.md: how much corridor the MPC's barrier may ask for (a line's
# feasible cap is its min half-width minus the 0.135 m car half-width, and
# above that the barrier pushes the car across its own line), and whether the
# circuit is tight enough for the brake lead to be worth its +0.23..+0.30 s of
# lap time. `pick_config.py` measures both off this map and this line, writes
# data/racelines/${SLUG}_mesh_pick.json, and prints the exact
# PIT_CREW_CONTROLLER_ARGS to race with. The runbook's gate table reads the
# sidecar; the submission entrypoint has a prepared block that reads it too,
# so the finals image can carry the pick without an edit.
.venv/bin/python tools/pick_config.py "${SLUG}_mesh" || exit 1

stage "the exact submission edits for slug '${SLUG}_mesh'"
# The commonest way to lose a finals is an asset that never made it into the
# image, so print the two edits rather than describing them.
cat <<EOM
submission/Dockerfile -- add at the 'ADD THE FINALS LINES HERE' marker
(the centerline is REQUIRED: localization seeds its whole particle cloud at
centerline[0] and resamples along it on every collision recovery):

     data/racelines/${SLUG}_mesh_centerline.csv \\
     data/racelines/${SLUG}_mesh_<race|safe>.csv \\

submission/autodrive_devkit.sh -- one line; LINE and CENTER follow the slug:

  MAP="\${PIT_CREW_MAP:-/home/data/maps/${SLUG}_mesh}"

The map itself needs no Dockerfile edit (data/maps/*_mesh.{pgm,yaml} is
globbed). Racelines are never globbed; if the Dockerfile carries the
FINALS_SLUG build arg (docs/FINALS_RUNBOOK.md step 6) there is no edit at all,
just:  docker build --build-arg FINALS_SLUG=${SLUG}_mesh ...

Verify the whole derivation on the host, BEFORE the Dockerfile is touched:
  FINALS_SLUG=${SLUG}_mesh .venv/bin/python -m pytest -q \\
      tests/test_submission_assets.py -k FinalsSlug

To carry the 5/5 pick into the image instead of typing it on the day, add the
sidecar to the same COPY list and uncomment the PICK block in
submission/autodrive_devkit.sh:

     data/racelines/${SLUG}_mesh_pick.json \\
EOM

stage "DRILL COMPLETE in $(( $(date +%s) - T0 ))s"
cat <<EOM

Drive it (the race line, then the safe line as the fallback):

  CONTROLLER=mpc3 LIDAR_OFFSET=0.1933 ./scripts/run_stanley.sh 1.0 ${SLUG}_mesh race
  CONTROLLER=mpc3 LIDAR_OFFSET=0.1933 ./scripts/run_stanley.sh 1.0 ${SLUG}_mesh safe

Score it:  .venv/bin/python tools/score_race.py <log> data/racelines/${SLUG}_mesh_centerline.csv

\`${SLUG}_mesh_centerline.csv\` is 4 columns (x, y, w_right, w_left) and carries
NO speed profile: it is what localization seeds from, and it can never be passed
as PIT_CREW_LINE. The centerline-shaped thing you RACE is \`_safe.csv\`.

The race line is chosen under a clearance floor of 0.22 m, which is a PHYSICS
bound and not a driven one -- nothing has yet raced a mesh-map line that tight.
If it collides, re-run with a floor that has evidence behind it:

  .venv/bin/python tools/make_track_lines.py ${SLUG}_mesh --min-clearance 0.25

and if that still collides, race \`safe\`, which is the corridor middle.
EOM
