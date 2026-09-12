# Shared display plumbing for the scripts that have to CLICK the simulator.
#
# Sourced, never executed.  Two callers need exactly the same mechanism:
#
#   scripts/sim_restart_connect.sh   -- development runs (container
#                                       `autodrive_roboracer_api` owns xdotool)
#   scripts/test_submission_image.sh -- races the BUILT image, so the devkit
#                                       container is the submission under test
#                                       and the helper container `pit_helper`
#                                       owns xdotool instead.
#
# Everything here was derived once, at cost, and is documented in
# sim_restart_connect.sh's header: Unity ignores XSendEvent, needs the pointer
# to ENTER a control, needs press and release in separate frames, and on a real
# display `xdotool getwindowgeometry` reports the FRAME, not the canvas -- so
# the canvas origin comes from a screenshot via tools/find_sim_ui.py.
#
# Nothing in this file starts a container; the caller owns the lifecycle.

# Where the canvas origin is handed to scripts/sim_set_quality.sh.  Overridable
# only so the host tests do not touch a live run's handoff file.
SIM_UI_ORIGIN="${SIM_UI_ORIGIN:-/tmp/sim_ui_origin}"

# display_select -- sets DISP from VISIBLE.  ":99" (private Xvfb inside the sim
# container) unless VISIBLE=1, which puts the sim on the user's own display and
# takes over the pointer, so it is opt-in.
display_select() {
    if [ "${VISIBLE:-0}" = "1" ]; then
        DISP="${DISPLAY:-:0}"
        echo "VISIBLE=1: simulator will open on your display ${DISP}."
        echo "It will move the mouse pointer briefly to click Connection."
        xhost local:root >/dev/null 2>&1 || true
    else
        DISP=":99"
    fi
}

# display_find_window <container> <disp> -- echoes the simulator's window id, or
# nothing.  Never fails, so it is safe under `set -e` in a retry loop.
display_find_window() {
    docker exec "$1" /bin/bash -c \
        "DISPLAY=$2 xdotool search --name '^AutoDRIVE Simulator\$' 2>/dev/null | head -1" || true
}

# display_click_at <container> <disp> <x> <y> -- one real XTEST click.
# Unity's UI only reacts to a click on a control it has seen the pointer ENTER,
# and wants the press and release in separate frames.  Approach from elsewhere,
# then hold the button down briefly; `xdotool click` alone frequently misses.
display_click_at() {
    docker exec "$1" /bin/bash -c \
        "export DISPLAY=$2
         xdotool mousemove 900 600; sleep 0.4
         xdotool mousemove $3 $4; sleep 1.2
         xdotool mousedown 1; sleep 0.2; xdotool mouseup 1"
}

# display_resolve_click_coords <container> <disp> <wid> <repo_dir>
# Sets HAMB_X/HAMB_Y (sidebar toggle) and CONN_X/CONN_Y (Connection), plus
# SHOT=1 on a real display (per-attempt screenshots are worth keeping there).
#
# On :99 there is no window manager: the canvas is a bare 1920x1080 at 0,0.
# On a real display the window is decorated and smaller, so the UI is scaled
# and offset -- take the geometry from the window itself rather than assuming.
display_resolve_click_coords() {
    local container="$1" disp="$2" wid="$3" repo_dir="$4"
    if [ "$disp" = ":99" ]; then
        HAMB_X=75;  HAMB_Y=43;  CONN_X=75;  CONN_Y=409
        return 0
    fi
    # Make the window the 1920x1080 canvas the coordinates were measured on:
    # on a 5120x2880 desktop Unity opens small and the UI is scaled, and
    # fixed offsets (the old +51/+64/+302) missed every click. Resize, then
    # read the geometry back (Position is the client-area origin, i.e. it
    # already includes the window-manager decoration) and fall back to
    # proportional scaling if the window manager refused the size.
    docker exec "$container" /bin/bash -c \
        "DISPLAY=${disp} xdotool windowmove $wid 0 0; sleep 0.5
         DISPLAY=${disp} xdotool windowsize $wid 1920 1080; sleep 1.5
         DISPLAY=${disp} xdotool windowactivate $wid 2>/dev/null; true"
    local GEO OX OY WW WH UI shot
    GEO=$(docker exec "$container" /bin/bash -c \
        "DISPLAY=${disp} xdotool getwindowgeometry $wid")
    OX=$(echo "$GEO" | sed -n 's/.*Position: \([0-9]*\),\([0-9]*\).*/\1/p')
    OY=$(echo "$GEO" | sed -n 's/.*Position: \([0-9]*\),\([0-9]*\).*/\2/p')
    WW=$(echo "$GEO" | sed -n 's/.*Geometry: \([0-9]*\)x\([0-9]*\).*/\1/p')
    WH=$(echo "$GEO" | sed -n 's/.*Geometry: \([0-9]*\)x\([0-9]*\).*/\2/p')
    echo "window $wid at ${OX},${OY} size ${WW}x${WH} (frame)"
    # The frame geometry is not the canvas: find the sim's header bar in a
    # screenshot (tools/find_sim_ui.py) and use its origin. Falls back to the
    # frame with a proportional scale if the bar is not found.
    UI=""
    for shot in 1 2 3 4; do
        sleep 2
        docker exec "$container" /bin/bash -c \
            "DISPLAY=${disp} import -window root /tmp/ui.png 2>/dev/null; true"
        docker cp "$container":/tmp/ui.png /tmp/sim_ui.png >/dev/null 2>&1 || true
        UI=$("${repo_dir}/.venv/bin/python" "${repo_dir}/tools/find_sim_ui.py" /tmp/sim_ui.png 2>/dev/null || true)
        # accept only a settled, full-width canvas (the resize animates)
        if [ -n "$UI" ] && [ "$(echo "$UI" | cut -d" " -f3)" -ge 1900 ]; then break; fi
        UI=""
    done
    if [ -n "$UI" ]; then
        read -r CX CY CW <<<"$UI"
        echo "canvas at ${CX},${CY} width ${CW}"
        echo "$CX $CY $CW" > "$SIM_UI_ORIGIN"
        HAMB_X=$((CX + 75 * CW / 1920)); HAMB_Y=$((CY + 43 * CW / 1920))
        CONN_X=$((CX + 75 * CW / 1920)); CONN_Y=$((CY + 409 * CW / 1920))
    else
        echo "WARN: header bar not found in screenshot; scaling from the frame"
        rm -f "$SIM_UI_ORIGIN"
        HAMB_X=$((OX + 75 * WW / 1920)); HAMB_Y=$((OY + 43 * WH / 1080))
        CONN_X=$((OX + 75 * WW / 1920)); CONN_Y=$((OY + 409 * WH / 1080))
    fi
    # Keep a screenshot per attempt for diagnosis (data/logs/display_click_*.png).
    SHOT=1
}
