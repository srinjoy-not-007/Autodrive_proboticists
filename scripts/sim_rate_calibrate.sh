#!/bin/bash
# Find the CPU pinning that reproduces a wanted simulator telemetry period.
#
# WHY THIS EXISTS
# ---------------
# The period the whole stack sees is decided by ONE CPU-saturated Unity render
# thread (GPU idle, no vsync on Xvfb), so *which cores that thread lands on*
# decides the rate, and an unpinned start is a lottery: 0.088-0.091 s on some
# starts, 0.103-0.106 s on others, stable within a run. mpc3 is rate-sensitive
# (clean at ~0.104 s, collides at ~0.088 s) and the organisers measured
# ~0.097 s, which is between the two -- so a validation run is only evidence if
# it was taken at their period. Pinning makes the lottery a dial:
#   - CCD0 (cpus 0-7,16-23) is the fast complex; CCD1 (8-15,24-31) is ~5% slower
#   - a narrower set slows the render thread further
# This script drives that dial once per candidate set and reports the period,
# so a single cpuset can then be fixed for every scored run.
#
# Usage:
#   ./scripts/sim_rate_calibrate.sh [target_period_s]        # default 0.097
#   SIM_CPUSETS="0-7,16-23 8-11" ./scripts/sim_rate_calibrate.sh 0.097
#   SIM_GPU_LOCK=1 GPU_MHZ=1800 ./scripts/sim_rate_calibrate.sh
#
# Env:
#   SIM_CPUSETS   space-separated candidate cpusets (default: the six below)
#   WAIT_S        settle time before measuring, seconds (default 30)
#   MAP / LINE / SCALE / SIM_QUALITY / CONTROLLER / LIDAR_OFFSET
#                 what to run; defaults are the shipping config at Ultra
#   SIM_GPU_LOCK=1  lock the GPU clocks first (see below -- needs sudo)
#   GPU_MHZ       locked graphics clock in MHz (default 1800)
#   LOAD_OK=1     skip the host-load refusal (do not use for real numbers)
#
# GPU CLOCK LOCKING: with SIM_GPU_LOCK=1 this runs
#   sudo nvidia-smi -pm 1 -lgc <GPU_MHZ>
# which needs sudo (it will prompt) and PERSISTS after the script exits. Undo it
# yourself when you are done measuring:
#   sudo nvidia-smi -rgc
# It is opt-in because the render thread is CPU-bound here, so locking clocks
# mostly removes a source of drift rather than changing the period.
#
# This script starts and stops the real stack once per cpuset, so it takes about
# (WAIT_S + ~45 s of start/stop) per candidate -- ~8 min for the default six.
set -u
cd "$(dirname "$0")/.."

TARGET="${1:-0.097}"
WAIT_S="${WAIT_S:-30}"
MAP="${MAP:-porto_mesh}"
LINE="${LINE:-mc0.63_ay6.75}"
SCALE="${SCALE:-1.0}"
DEFAULT_SETS='0-7,16-23 8-15,24-31 0-3 8-11 0,16 8,24'
read -r -a SETS <<< "${SIM_CPUSETS:-$DEFAULT_SETS}"

LOAD=$(cut -d" " -f1 /proc/loadavg)
if [ "${LOAD_OK:-0}" != 1 ] && awk "BEGIN{exit !($LOAD > 12)}"; then
    echo "REFUSING: host load $LOAD; a rate measurement needs a quiet host"; exit 3
fi

if [ "${SIM_GPU_LOCK:-0}" = "1" ]; then
    GPU_MHZ="${GPU_MHZ:-1800}"
    echo "Locking GPU clocks to ${GPU_MHZ} MHz (sudo; undo with 'sudo nvidia-smi -rgc')"
    sudo nvidia-smi -pm 1 -lgc "$GPU_MHZ" || echo "WARN: could not lock GPU clocks; continuing unlocked"
fi

mkdir -p data/logs
RESULTS=""

measure_dt() {   # $1 = log csv; prints the median of the last 150 row intervals
    .venv/bin/python - "$1" <<'PY'
import csv, sys, numpy as np
rows = list(csv.DictReader(open(sys.argv[1])))
t = np.array([float(r['t']) for r in rows][-150:])
print('%.4f' % np.median(np.diff(t)) if len(t) > 20 else 'nan')
PY
}

for CPUSET in "${SETS[@]}"; do
    echo "=== cpuset $CPUSET ==="
    PREV_LOG=$(ls -t data/logs/race_*.csv 2>/dev/null | head -1)
    SIM_CPUSET="$CPUSET" \
    CONTROLLER="${CONTROLLER:-mpc3}" LIDAR_OFFSET="${LIDAR_OFFSET:-0.1933}" \
    SIM_QUALITY="${SIM_QUALITY:-ultra}" \
        ./scripts/run_stanley.sh "$SCALE" "$MAP" "$LINE" \
        > /tmp/sim_rate_calibrate_start.log 2>&1
    START_RC=$?
    sleep 3
    LOG=$(ls -t data/logs/race_*.csv 2>/dev/null | head -1)
    if [ "$START_RC" != 0 ] || [ -z "$LOG" ] || [ "$LOG" = "$PREV_LOG" ]; then
        # A failed Connection click, a wedged sim, or a logger that never wrote:
        # no run happened, so there is nothing to measure. Skip this candidate.
        echo "  SKIP: stack did not start (rc=$START_RC, log=${LOG:-none})"
        tail -3 /tmp/sim_rate_calibrate_start.log | sed 's/^/    /'
        ./scripts/stop_run.sh >/dev/null 2>&1
        RESULTS="${RESULTS}${CPUSET}\tskipped\n"
        continue
    fi
    echo "  logging to $LOG; settling ${WAIT_S}s"
    sleep "$WAIT_S"
    DT=$(measure_dt "$LOG")
    ./scripts/stop_run.sh >/dev/null 2>&1
    if [ -z "$DT" ] || [ "$DT" = "nan" ]; then
        echo "  SKIP: too few logger rows to measure a period"
        RESULTS="${RESULTS}${CPUSET}\tskipped\n"
        continue
    fi
    echo "  period $DT s"
    RESULTS="${RESULTS}${CPUSET}\t${DT}\n"
done

echo
echo "cpuset            period_s   (target ${TARGET})"
printf "%b" "$RESULTS" | while IFS=$'\t' read -r CS DT; do
    [ -z "$CS" ] && continue
    if [ "$DT" = "skipped" ]; then
        printf "%-16s  %s\n" "$CS" "skipped"
    else
        printf "%-16s  %s   (%+.4f)\n" "$CS" "$DT" \
            "$(awk "BEGIN{printf \"%.4f\", $DT - $TARGET}")"
    fi
done

BEST=$(printf "%b" "$RESULTS" | awk -F'\t' -v tgt="$TARGET" '
    $2 != "skipped" && $2 != "" { d = $2 - tgt; if (d < 0) d = -d;
        if (best == "" || d < best) { best = d; set = $1; dt = $2 } }
    END { if (set != "") printf "%s\t%s\t%.4f", set, dt, best }')
echo
if [ -z "$BEST" ]; then
    echo "No cpuset produced a measurable period -- nothing to choose."
    exit 1
fi
echo "CLOSEST TO ${TARGET} s: SIM_CPUSET=$(echo "$BEST" | cut -f1)" \
     "(measured $(echo "$BEST" | cut -f2) s, off by $(echo "$BEST" | cut -f3) s)"
echo "Use it on every scored run, with the period gate as the check:"
echo "  SIM_CPUSET=$(echo "$BEST" | cut -f1) RATE_MIN_DT=$(awk "BEGIN{printf \"%.3f\", $TARGET-0.003}")" \
     "RATE_MAX_DT=$(awk "BEGIN{printf \"%.3f\", $TARGET+0.003}") \\"
echo "    SIM_QUALITY=ultra CONTROLLER=mpc3 LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh <name> $LINE"
[ "${SIM_GPU_LOCK:-0}" = "1" ] && echo "GPU clocks are still locked: run 'sudo nvidia-smi -rgc' when finished."
exit 0
