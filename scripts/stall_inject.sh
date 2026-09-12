#!/usr/bin/env bash
# Inject a realistic TELEMETRY STALL into a LIVE run, from the host, without
# killing or restarting anything.
#
# Why this exists: `gap_predict_s` (mpc3) and the localization gap parameters
# were validated in the offline harness only. The live failure they are meant
# to survive is the organisers' box running OBS and a rosbag alongside the
# simulator, and the shape of that failure decides which fix is right.
#
# Modes, in decreasing order of how well they model the organisers' risk:
#
#   sim     (DEFAULT) `docker pause` the SIMULATOR container. The freezer
#           cgroup SIGSTOPs every thread in it, so Unity's physics, its
#           rendering AND its WebSocket server stop together and restart
#           together. That is exactly what CPU starvation by a screen
#           recorder produces: the car does not move while the stack is
#           blind, and on resume the stamps simply jump. The bridge sees a
#           gap; the commands it keeps publishing are applied to a frozen
#           car, i.e. they are neither lost nor acted on.
#
#   bridge  SIGSTOP the devkit bridge process inside the api container. The
#           SIM KEEPS DRIVING (the last command latches -- see CLAUDE.md),
#           telemetry stops AND commands stop being applied. This is the
#           bidirectional case: strictly worse than `sim`, and the right
#           test for "the car ran on at the last command for a second".
#
#   net     Drop the loopback WebSocket traffic on port 4567 with netem.
#           Telemetry only, in the sense that the socket stalls while both
#           processes keep running; the car keeps driving. Needs NET_ADMIN
#           on the host and is the least faithful of the three (TCP
#           retransmits deliver a BACKLOG on release, which localization
#           then drops via its `now - stamp > 0.35` guard).
#
# Everything is reversible and nothing is restarted, so this is safe to run
# mid-race. The resume is in an EXIT/INT/TERM trap and is re-verified after
# the loop, so a Ctrl-C during a stall still unfreezes the simulator.
#
# Usage:
#   ./scripts/stall_inject.sh --dur 0.6 --every 15 --count 4
#   ./scripts/stall_inject.sh --mode bridge --dur 1.0 --count 1 --delay 30
#   CONTAINER=stall_test ./scripts/stall_inject.sh --dur 0.5 --count 2
#
# The stall windows are appended to a CSV the scorer can read; its `t` column
# is `time.time()` epoch seconds, the same base as race_logger.py's `t`.
set -euo pipefail

MODE=sim
DUR=0.6           # s, length of one stall
EVERY=15          # s, period between stall STARTS
COUNT=1           # number of stalls
DELAY=0           # s, wait before the first stall
LOG=""
DRY=0
PORT=4567

usage() { sed -n '2,44p' "$0"; exit "${1:-0}"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode) MODE="$2"; shift 2;;
        --dur) DUR="$2"; shift 2;;
        --every) EVERY="$2"; shift 2;;
        --count) COUNT="$2"; shift 2;;
        --delay) DELAY="$2"; shift 2;;
        --log) LOG="$2"; shift 2;;
        --port) PORT="$2"; shift 2;;
        --dry-run) DRY=1; shift;;
        -h|--help) usage 0;;
        *) echo "unknown argument: $1" >&2; usage 2;;
    esac
done

case "$MODE" in sim|bridge|net) ;; *) echo "bad --mode: $MODE" >&2; exit 2;; esac

SIM_C="${SIM_CONTAINER:-autodrive_roboracer_sim}"
API_C="${API_CONTAINER:-autodrive_roboracer_api}"
# CONTAINER overrides whichever container this mode touches. It exists so the
# mechanics can be rehearsed on a throwaway container (see the README note at
# the bottom) without going anywhere near a scored run.
if [[ -n "${CONTAINER:-}" ]]; then SIM_C="$CONTAINER"; API_C="$CONTAINER"; fi

TARGET_C="$SIM_C"; [[ "$MODE" == bridge ]] && TARGET_C="$API_C"

running() { [[ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == true ]]; }
paused()  { [[ "$(docker inspect -f '{{.State.Paused}}'  "$1" 2>/dev/null)" == true ]]; }

if ! running "$TARGET_C"; then
    echo "container '$TARGET_C' is not running; refusing to do anything" >&2
    exit 1
fi

LOG="${LOG:-/tmp/stall_$(date +%Y%m%d_%H%M%S).csv}"
if [[ ! -f "$LOG" ]]; then
    echo "i,mode,target,t_start,t_end,dur_s,iso_start" > "$LOG"
fi

BRIDGE_PIDS=""
if [[ "$MODE" == bridge ]]; then
    # The bridge is the devkit's own node; never match our own wrapper (see
    # the bracket-pattern gotcha in CLAUDE.md).
    BRIDGE_PIDS=$(docker exec "$TARGET_C" /bin/bash -c \
        "pgrep -f '[a]utodrive_roboracer' | tr '\n' ' '" || true)
    if [[ -z "${BRIDGE_PIDS// /}" ]]; then
        echo "no devkit bridge process found in '$TARGET_C'" >&2; exit 1
    fi
    echo "bridge pids: $BRIDGE_PIDS"
fi

resume() {
    case "$MODE" in
        sim)    paused "$TARGET_C" && docker unpause "$TARGET_C" >/dev/null || true;;
        bridge) docker exec "$TARGET_C" /bin/bash -c "kill -CONT $BRIDGE_PIDS" \
                    2>/dev/null || true;;
        net)    sudo -n tc qdisc del dev lo root 2>/dev/null || true;;
    esac
}
trap 'resume' EXIT INT TERM

stall_on() {
    if [[ "$DRY" == 1 ]]; then return 0; fi
    case "$MODE" in
        sim)    docker pause "$TARGET_C" >/dev/null;;
        bridge) docker exec "$TARGET_C" /bin/bash -c "kill -STOP $BRIDGE_PIDS";;
        net)    sudo -n tc qdisc add dev lo root handle 1: prio >/dev/null
                sudo -n tc qdisc add dev lo parent 1:3 handle 30: netem loss 100% >/dev/null
                sudo -n tc filter add dev lo protocol ip parent 1:0 prio 3 u32 \
                    match ip dport "$PORT" 0xffff flowid 1:3 >/dev/null;;
    esac
}
stall_off() { [[ "$DRY" == 1 ]] || resume; }

echo "mode=$MODE target=$TARGET_C dur=${DUR}s every=${EVERY}s count=$COUNT log=$LOG"
[[ "$DRY" == 1 ]] && echo "(dry run: timing and logging only, nothing is frozen)"
[[ "$DELAY" != 0 ]] && sleep "$DELAY"

for ((i = 1; i <= COUNT; i++)); do
    T0=$(date +%s.%N)
    stall_on
    sleep "$DUR"
    stall_off
    T1=$(date +%s.%N)
    printf '%d,%s,%s,%.3f,%.3f,%.3f,%s\n' "$i" "$MODE" "$TARGET_C" "$T0" "$T1" \
        "$(echo "$T1 - $T0" | bc -l)" "$(date -u -d "@$T0" +%FT%TZ)" >> "$LOG"
    echo "stall $i/$COUNT: $(printf '%.3f' "$T0") -> $(printf '%.3f' "$T1") " \
         "($(printf '%.3f' "$(echo "$T1 - $T0" | bc -l)") s)"
    if (( i < COUNT )); then
        REST=$(echo "$EVERY - ($T1 - $T0)" | bc -l)
        (( $(echo "$REST > 0" | bc -l) )) && sleep "$REST"
    fi
done

# Belt and braces: a stall that outlives this script is a wrecked run.
resume
if [[ "$MODE" == sim ]] && paused "$TARGET_C"; then
    echo "FAILED TO UNPAUSE '$TARGET_C' -- run: docker unpause $TARGET_C" >&2
    exit 3
fi
echo "resumed; windows logged to $LOG"

# --- rehearsing the mechanics without touching a live run --------------------
#   docker run --rm -d --name stall_test alpine sleep 600
#   CONTAINER=stall_test ./scripts/stall_inject.sh --dur 0.6 --every 2 --count 3
#   docker inspect -f '{{.State.Paused}}' stall_test   # must print false
#   docker rm -f stall_test
