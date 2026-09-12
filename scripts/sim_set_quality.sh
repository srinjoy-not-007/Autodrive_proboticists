#!/bin/bash
# Switch the running simulator's graphics quality by clicking its UI, the way
# the organisers do by hand (they score at **Ultra graphics, GUI**).
#
# WHERE THE CONTROL IS
# --------------------
# The quality control is the 4th item of the simulator's LEFT sidebar -- the
# same panel that holds Connection. On the 1920x1080 Xvfb canvas the column is
# x=75 and the rows are ~99.6 px apart:
#
#     hamburger (toggles the sidebar)   75,  43
#     IP Address field                 190, 186   (used here only to detect
#     Port Number field                190, 310    whether the sidebar is open)
#     Connected                         75, 409   <- sim_restart_connect.sh
#     Autonomous                        75, 509
#     Driver's Eye                      75, 608
#     <level> Quality                   75, 708   <- THIS SCRIPT
#     Scene Light                       75, 807
#     Reset                             75, 907
#     Quit                              75,1006
#
# VERIFIED by screenshot (2026-09-12, 2026-iros-practice): with the sidebar
# open the row at y=708 reads "Low Quality" -- i.e. the button is a CYCLING
# toggle whose label is the CURRENT level, not a drop-down. There is no
# separate settings window and no visible list of options.
#
# UNVERIFIED (a scored run was in progress, so nothing was clicked): the length
# and order of the cycle. The hypothesis below is Unity's six default quality
# levels in ascending order, wrapping around:
#
#     Very Low -> Low -> Medium -> High -> Very High -> Ultra -> (Very Low)
#
# So from "Low Quality" it is 4 clicks to Ultra and 2 more back to the default
# ("Low", which is what the image starts in). If the real cycle differs, adjust
# CYCLE and the level list below -- run `sim_set_quality.sh next --verify` a few
# times and read the labels off the saved crops, then fix the constants here.
# Nothing else in the script depends on the ordering.
#
# THIS SCRIPT CLICKS THE SIMULATOR. Do not run it during a scored run: the
# sidebar overlays the viewport and a mis-click could hit Reset (75,907) or
# Quit (75,1006), one row below the quality button.
#
# TAG / DISPLAY AGNOSTIC
# ----------------------
# Nothing here is track- or TAG-specific; the sidebar is identical on every
# *_sim tag (practice and the *-compete images). Only the canvas geometry
# matters. On :99 there is no window manager, so the canvas is a bare 1920x1080
# at 0,0 and the coordinates above are literal. On a real display (VISIBLE=1
# runs) the window is decorated and smaller, so the coordinates are scaled by
# the window's size and shifted by its origin plus the title-bar height
# (DECOR, default 35 px) -- approximate, hence always --verify there.
#
# Usage:
#   scripts/sim_set_quality.sh show            # just screenshot the label
#   scripts/sim_set_quality.sh next            # one click = next level
#   scripts/sim_set_quality.sh ultra           # CYCLE_TO_ULTRA clicks
#   scripts/sim_set_quality.sh default         # complete the cycle back to Low
#   ... --verify                               # screenshot + label crop after
#                                              # every click, for a human
#   ... --clicks N                             # explicit number of clicks
#
# Env: DISP (default :99), SIM_API (devkit container that owns xdotool),
#      OUTDIR (where screenshots land), DECOR, KEEP_OPEN=1 (leave the sidebar
#      open; by default the sidebar is left exactly as it was found).
set -e

DISP="${DISP:-:99}"
SIM_API="${SIM_API:-autodrive_roboracer_api}"
SIM_WIN="${SIM_WIN:-^AutoDRIVE Simulator\$}"
OUTDIR="${OUTDIR:-/tmp/sim_quality}"
DECOR="${DECOR:-35}"

# --- the cycle (UNVERIFIED, see header) --------------------------------------
CYCLE=3            # verified 2026-09-13: Low -> High -> Ultra -> (Low)
CYCLE_TO_ULTRA=2   # from the default Low

MODE=""
VERIFY=0
CLICKS=""
while [ $# -gt 0 ]; do
    case "$1" in
        show|next|ultra|default) MODE="$1" ;;
        --verify)  VERIFY=1 ;;
        --clicks)  CLICKS="$2"; shift ;;
        -h|--help) sed -n '2,60p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done
MODE="${MODE:-show}"
mkdir -p "$OUTDIR"

api() { docker exec "$SIM_API" /bin/bash -c "$1"; }

# --- geometry ----------------------------------------------------------------
# Canvas coordinates (1920x1080, no decoration) -- then mapped to the display.
C_HAMB_X=75;  C_HAMB_Y=43
C_QUAL_X=75;  C_QUAL_Y=708
C_PROBE_X=190; C_PROBE_Y=186    # the IP Address field: white iff sidebar open

if [ "$DISP" = ":99" ]; then
    map() { echo "$1 $2"; }
else
    WID=$(api "DISPLAY=$DISP xdotool search --name '$SIM_WIN' 2>/dev/null | head -1")
    [ -z "$WID" ] && { echo "ERROR: no simulator window on $DISP"; exit 1; }
    GEO=$(api "DISPLAY=$DISP xdotool getwindowgeometry $WID")
    OX=$(echo "$GEO" | sed -n 's/.*Position: \([0-9]*\),\([0-9]*\).*/\1/p')
    OY=$(echo "$GEO" | sed -n 's/.*Position: \([0-9]*\),\([0-9]*\).*/\2/p')
    WW=$(echo "$GEO" | sed -n 's/.*Geometry: \([0-9]*\)x\([0-9]*\).*/\1/p')
    WH=$(echo "$GEO" | sed -n 's/.*Geometry: \([0-9]*\)x\([0-9]*\).*/\2/p')
    echo "window $WID at ${OX},${OY} size ${WW}x${WH} (coordinates scaled; --verify strongly advised)"
    map() { echo "$((OX + $1 * WW / 1920)) $((OY + DECOR + $2 * WH / 1080))"; }
    # sim_restart_connect.sh leaves the canvas origin it found from a
    # screenshot (tools/find_sim_ui.py); that beats any frame arithmetic.
    if [ -s /tmp/sim_ui_origin ]; then
        read -r CX CY CW < /tmp/sim_ui_origin
        echo "using canvas origin ${CX},${CY} width ${CW} from sim_restart_connect"
        map() { echo "$((CX + $1 * CW / 1920)) $((CY + $2 * CW / 1920))"; }
    fi
fi
read -r HAMB_X HAMB_Y <<<"$(map $C_HAMB_X $C_HAMB_Y)"
read -r QUAL_X QUAL_Y <<<"$(map $C_QUAL_X $C_QUAL_Y)"

# --- primitives --------------------------------------------------------------
# Unity only reacts to a click on a control the pointer has ENTERed, and wants
# press and release in separate frames -- see scripts/sim_restart_connect.sh.
click_at() {
    api "export DISPLAY=$DISP
         xdotool mousemove 900 600; sleep 0.4
         xdotool mousemove $1 $2; sleep 1.2
         xdotool mousedown 1; sleep 0.2; xdotool mouseup 1"
}

# shot <name>: full screenshot + a crop of the quality label, copied to OUTDIR.
shot() {
    api "DISPLAY=$DISP import -window root /tmp/q.png &&
         convert /tmp/q.png -crop 520x80+110+670 +repage /tmp/qlabel.png" >/dev/null
    docker cp "$SIM_API:/tmp/q.png"      "$OUTDIR/$1.png"      >/dev/null
    docker cp "$SIM_API:/tmp/qlabel.png" "$OUTDIR/$1_label.png" >/dev/null
    echo "  screenshot $OUTDIR/$1.png   label crop $OUTDIR/$1_label.png"
}

# The sidebar may be open or collapsed and there is no way to ask which, so
# probe a pixel: the IP Address entry box is pure white when the panel is open
# and the (blue) sky shows through when it is not. Measured 2026-09-12:
# open (255,255,255), closed (44,75,120).
sidebar_open() {
    api "DISPLAY=$DISP import -window root /tmp/probe.png &&
         convert /tmp/probe.png -format '%[fx:int(255*p{$C_PROBE_X,$C_PROBE_Y}.r)] %[fx:int(255*p{$C_PROBE_X,$C_PROBE_Y}.g)] %[fx:int(255*p{$C_PROBE_X,$C_PROBE_Y}.b)]' info:" \
    | awk '{ exit !($1>200 && $2>200 && $3>200) }'
}

# --- act ---------------------------------------------------------------------
if sidebar_open; then WAS_OPEN=1; else WAS_OPEN=0; fi
echo "sidebar was $([ $WAS_OPEN = 1 ] && echo open || echo closed)"
if [ "$WAS_OPEN" = 0 ]; then
    echo "opening sidebar (hamburger at $HAMB_X,$HAMB_Y)"
    click_at $HAMB_X $HAMB_Y
    sleep 1.5
    sidebar_open || { echo "ERROR: sidebar did not open; nothing clicked on the quality button"; exit 1; }
fi

case "$MODE" in
    show)    N=0 ;;
    next)    N=1 ;;
    ultra)   N=$CYCLE_TO_ULTRA ;;
    default) N=$(( (CYCLE - CYCLE_TO_ULTRA) % CYCLE )) ;;
esac
[ -n "$CLICKS" ] && N="$CLICKS"

echo "current state:"
shot "quality_before"

i=0
while [ "$i" -lt "$N" ]; do
    i=$((i + 1))
    echo "click $i/$N on the quality button ($QUAL_X,$QUAL_Y)"
    click_at $QUAL_X $QUAL_Y
    sleep 1.0
    [ "$VERIFY" = 1 ] && shot "quality_step$i"
done

if [ "$N" -gt 0 ] || [ "$VERIFY" = 1 ]; then
    sleep 0.5
    echo "result:"
    shot "quality_after"
fi

# Leave the sidebar as it was found -- it overlays the viewport when open.
if [ "$WAS_OPEN" = 0 ] && [ "${KEEP_OPEN:-0}" != "1" ]; then
    echo "closing sidebar again"
    click_at $HAMB_X $HAMB_Y
fi

echo
LAST="$OUTDIR/quality_after_label.png"
[ -f "$LAST" ] || LAST="$OUTDIR/quality_before_label.png"
echo "Open $LAST and read the label: it names the"
echo "CURRENT quality level. If it is not what you asked for, the cycle order"
echo "in this script's header is wrong -- fix CYCLE / CYCLE_TO_ULTRA, or just"
echo "run '$0 next --verify' until the label reads Ultra."
