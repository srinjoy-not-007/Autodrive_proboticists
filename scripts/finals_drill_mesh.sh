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
# `LINE_ARGS=` (explicitly empty) restores the old flagless behaviour; setting
# it to anything else replaces the set wholesale.
DRILL_LINE_ARGS='--dkappa-max 1.5 --dkappa-refine 3 --kappa-bound 1.0
                 --kappa-gate safe --min-halfwidth-ladder 0.53,0.50,0.45'
# shellcheck disable=SC2086
.venv/bin/python tools/make_track_lines.py "${SLUG}_mesh" \
    ${LINE_ARGS-$DRILL_LINE_ARGS} || exit 1

stage "4/5 footprint clearance check on both lines we might race"
for kind in race safe; do
    echo "--- ${SLUG}_mesh_${kind}.csv"
    .venv/bin/python tools/footprint_analysis.py "$MAP" \
        "data/racelines/${SLUG}_mesh_${kind}.csv"
done

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
globbed). Verify both with:  .venv/bin/python -m pytest -q tests/test_submission_assets.py

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
