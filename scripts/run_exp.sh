#!/bin/bash
# One measured experiment: start the shipping stack with overrides, run until
# N laps (default 32) or a collision budget is blown, stop, score, and append a
# row to data/logs/experiments.tsv so runs from one session are comparable.
#
# Usage:
#   ./scripts/run_exp.sh <name> [line] [extra ros args...]
#   LAPS=32 MAX_COLL=10 MAP=track_v2 SCALE=1.0 LIDAR_OFFSET=0 ./scripts/run_exp.sh ...
# Example:
#   ./scripts/run_exp.sh delay135 safe -p cmd_delay_s:=0.135
set -u
cd "$(dirname "$0")/.."
NAME="$1"; LINE="${2:-safe}"; shift 2 2>/dev/null || shift $#
EXTRA="$*"
LAPS="${LAPS:-32}"; MAX_COLL="${MAX_COLL:-10}"; MAP="${MAP:-track_v2}"
SCALE="${SCALE:-1.0}"; TIMEOUT_S="${TIMEOUT_S:-900}"
mkdir -p data/logs

CONTROLLER="${CONTROLLER:-mpc}" LIDAR_OFFSET="${LIDAR_OFFSET:-0.0}" \
    ./scripts/run_stanley.sh "$SCALE" "$MAP" "$LINE" $EXTRA > /tmp/run_exp_start.log 2>&1 \
    || { echo "start failed"; tail -5 /tmp/run_exp_start.log; exit 1; }
sleep 3
LOG=$(ls -t data/logs/race_*.csv | head -1)
echo "[$NAME] logging to $LOG"

T0=$(date +%s)
while true; do
    sleep 5
    ROWS=$(wc -l < "$LOG")
    [ "$ROWS" -lt 3 ] && continue
    FIRST_LAP=$(sed -n 2p "$LOG" | cut -d, -f7); FIRST_COL=$(sed -n 2p "$LOG" | cut -d, -f9)
    LAST=$(tail -1 "$LOG")
    L=$(( $(echo "$LAST" | cut -d, -f7) - FIRST_LAP ))
    C=$(( $(echo "$LAST" | cut -d, -f9) - FIRST_COL ))
    if [ "$L" -ge "$LAPS" ] || [ "$C" -ge "$MAX_COLL" ] || [ $(( $(date +%s) - T0 )) -ge "$TIMEOUT_S" ]; then
        break
    fi
done
./scripts/stop_run.sh > /dev/null 2>&1

SUMMARY=$(.venv/bin/python - "$LOG" <<'EOF'
import csv, sys, numpy as np
rows = list(csv.DictReader(open(sys.argv[1])))
t = np.array([float(r['t']) for r in rows]); lap = np.array([int(r['lap_count']) for r in rows])
col = np.array([int(r['collision_count']) for r in rows])
idx = np.flatnonzero(np.diff(lap)) + 1
lt = np.diff(t[idx]) if len(idx) > 1 else np.array([np.nan])
print(f"laps={lap[-1]-lap[0]} coll={col[-1]-col[0]} median={np.nanmedian(lt):.3f} "
      f"best={np.nanmin(lt):.3f} mean={np.nanmean(lt):.3f}")
EOF
)
SCORE=$(.venv/bin/python tools/score_race.py "$LOG" 2>&1 | grep -E 'race|UNVERIFIED' | head -1)
echo -e "$(date +%F_%T)\t$NAME\t$LINE\t$EXTRA\t$SUMMARY\t$SCORE\t$LOG" >> data/logs/experiments.tsv
echo "[$NAME] $SUMMARY | $SCORE"
