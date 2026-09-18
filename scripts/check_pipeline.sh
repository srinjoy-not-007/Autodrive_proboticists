#!/bin/bash
# Regenerate every track's racing lines and fail if any shipping artefact moved.
#
# This exists because a change validated on Porto shipped having broken ICRA's
# line generation entirely, and nothing said so until the next manual check. The
# offline pipeline is deterministic, so "regenerate and diff" is a complete test
# and it costs about half a minute.
#
# Run before any commit that touches tools/ or src/tools/.
#   ./scripts/check_pipeline.sh
set -u
cd "$(dirname "$0")/.."
# Pin BLAS: unpinned, optimize_raceline.py takes every core (28 on this host,
# load 40) and that disturbed the simulator's frame timing during scored runs
# on Sep 19.  The pipeline is deterministic either way.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
TRACKS="${*:-track_v2 icra2026}"
FAIL=0

if ! git diff --quiet -- data/racelines/ 2>/dev/null; then
    echo "REFUSING: data/racelines/ has uncommitted changes; commit or stash first."
    exit 2
fi

for T in $TRACKS; do
    OUT=$(.venv/bin/python tools/make_track_lines.py "$T" 2>&1)
    if ! echo "$OUT" | grep -q '^race line'; then
        echo "FAIL  $T: no drivable line"
        echo "$OUT" | sed 's/^/        /' | tail -6
        FAIL=1
        continue
    fi
    echo "ok    $T: $(echo "$OUT" | grep '^race line' | sed 's/race line: //')"
done

# The lines we actually race must be bit-for-bit what was validated. QC images
# and per-method intermediates are allowed to move; these are not.
SHIPPED=$(for T in $TRACKS; do
    echo "data/racelines/${T}_safe.csv data/racelines/${T}_race.csv data/racelines/${T}_centerline.csv"
done)
if ! git diff --quiet -- $SHIPPED; then
    echo "FAIL  shipping artefacts changed:"
    git diff --numstat -- $SHIPPED | sed 's/^/        /'
    FAIL=1
fi

git checkout -- data/racelines/ 2>/dev/null
[ "$FAIL" = 0 ] && echo "PIPELINE OK — all tracks drivable, shipping lines unchanged"
exit $FAIL
