"""`VISIBLE=1` on the two scripts that CLICK the simulator.

`scripts/test_submission_image.sh` races the built image the way the organisers
run it, but it could only ever do so on Xvfb :99 -- where the telemetry period
is 0.088-0.106 s.  The organisers run on a real display at 0.058 s, the period
at which four configurations have failed (docs/STATUS.md, Sept 12), so Gate D
has to be runnable against the IMAGE and not only against mounted source.

These tests run both scripts for real under `bash`, with a stub `docker` (and
`sleep`, `xhost`, and the repo's `.venv/bin/python`) on PATH that only records
its argv, and assert on the calls:

* VISIBLE unset  -> Xvfb :99, fixed canvas coordinates (75,43)/(75,409), and
  `tools/find_sim_ui.py` is never invoked;
* VISIBLE=1      -> the host's own $DISPLAY, no Xvfb, the window resized to
  1920x1080, a screenshot, and the canvas located by `find_sim_ui.py`.

Nothing here starts a container: the stub answers every `docker` call.
"""

import os
import shutil
import subprocess

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = [
    os.path.join(REPO, 'scripts', 'lib_display.sh'),
    os.path.join(REPO, 'scripts', 'sim_restart_connect.sh'),
    os.path.join(REPO, 'scripts', 'test_submission_image.sh'),
]

DOCKER_STUB = r'''#!/bin/bash
printf '%s\n' "$*" >> "$CALLS/docker.log"
case "$*" in
  *"ros2 topic echo"*)      exit 1 ;;          # never "connects"
  *"xdotool search"*)       echo 12345 ;;
  *getwindowgeometry*)      printf 'Window 12345\n  Position: 100,200 (screen: 0)\n  Geometry: 1920x1080\n' ;;
esac
exit 0
'''

PY_STUB = r'''#!/bin/bash
printf '%s\n' "$*" >> "$CALLS/python.log"
exit 1
'''

NOOP = "#!/bin/bash\nexit 0\n"


def _write(path, text):
    with open(path, 'w') as fh:
        fh.write(text)
    os.chmod(path, 0o755)


@pytest.fixture
def sandbox(tmp_path):
    """A fake repo root (scripts/tools symlinked) plus a stub PATH."""
    calls = tmp_path / 'calls'
    calls.mkdir()
    stubs = tmp_path / 'stubs'
    stubs.mkdir()
    _write(str(stubs / 'docker'), DOCKER_STUB)
    _write(str(stubs / 'sleep'), NOOP)     # the scripts sleep ~40 s in total
    _write(str(stubs / 'xhost'), NOOP)     # must not touch the real X server
    fake = tmp_path / 'repo'
    (fake / '.venv' / 'bin').mkdir(parents=True)
    (fake / 'data' / 'logs').mkdir(parents=True)
    _write(str(fake / '.venv' / 'bin' / 'python'), PY_STUB)
    for name in ('scripts', 'tools'):
        os.symlink(os.path.join(REPO, name), str(fake / name))
    env = dict(os.environ)
    env.update(PATH='%s:%s' % (stubs, env['PATH']),
               CALLS=str(calls),
               DISPLAY=':77',
               SIM_UI_ORIGIN=str(tmp_path / 'sim_ui_origin'))
    env.pop('VISIBLE', None)
    return fake, calls, env


def run_submission(sandbox, **extra_env):
    fake, calls, env = sandbox
    env = dict(env, **extra_env)
    proc = subprocess.run(
        ['bash', str(fake / 'scripts' / 'test_submission_image.sh'), 'img:tag', '4'],
        cwd=str(fake), env=env, capture_output=True, text=True, timeout=120)
    def log(name):
        p = calls / name
        return p.read_text() if p.exists() else ''
    return proc, log('docker.log'), log('python.log')


# --------------------------------------------------------------- syntax

@pytest.mark.parametrize('path', SCRIPTS, ids=os.path.basename)
def test_shell_syntax(path):
    assert subprocess.run(['bash', '-n', path]).returncode == 0


@pytest.mark.parametrize('path', SCRIPTS[1:], ids=os.path.basename)
def test_both_scripts_share_one_display_mechanism(path):
    """Neither script may grow its own copy of the click geometry."""
    src = open(path).read()
    assert 'lib_display.sh' in src
    assert 'display_select' in src
    assert 'display_resolve_click_coords' in src


# --------------------------------------------------------------- Xvfb path

def test_xvfb_path_is_unchanged(sandbox):
    proc, docker, python = run_submission(sandbox)
    assert 'ERROR: could not connect' in proc.stdout, proc.stdout + proc.stderr
    assert 'Xvfb :99 -screen 0 1920x1080x24' in docker
    assert "DISPLAY=:99 xdotool search --name '^AutoDRIVE Simulator$'" in docker
    # fixed canvas coordinates, no window juggling, no screenshot hunt
    assert 'xdotool mousemove 75 409' in docker
    assert 'xdotool mousemove 75 43' in docker
    assert 'windowsize' not in docker
    assert '/tmp/ui.png' not in docker
    assert 'find_sim_ui.py' not in python
    assert ':77' not in docker


# --------------------------------------------------------------- real display

def test_visible_uses_the_real_display(sandbox):
    proc, docker, python = run_submission(sandbox, VISIBLE='1')
    assert 'ERROR: could not connect' in proc.stdout, proc.stdout + proc.stderr
    assert 'VISIBLE=1: simulator will open on your display :77.' in proc.stdout
    # the simulator joins the host's X server instead of starting its own
    assert 'Xvfb' not in docker
    assert 'DISPLAY=:77 ./AutoDRIVE' in docker
    assert "DISPLAY=:77 xdotool search --name '^AutoDRIVE Simulator$'" in docker
    # window made into the 1920x1080 canvas the coordinates were measured on
    assert 'xdotool windowmove 12345 0 0' in docker
    assert 'xdotool windowsize 12345 1920 1080' in docker
    # canvas origin from a screenshot, with the settled-canvas retry
    assert 'DISPLAY=:77 import -window root /tmp/ui.png' in docker
    hunts = [l for l in python.splitlines() if 'find_sim_ui.py' in l]
    assert len(hunts) == 4, python
    assert hunts[0].endswith('/tmp/sim_ui.png')
    # the stub find_sim_ui never resolves, so the frame fallback applies:
    # Position 100,200 + (75,43)/(75,409) scaled by 1920x1080
    assert 'xdotool mousemove 175 609' in docker
    assert 'xdotool mousemove 175 243' in docker
    # and Ultra is driven through the same helper container
    assert 'exec pit_helper' in docker


def test_visible_never_leaks_into_the_helper_container_choice(sandbox):
    """xdotool lives in `pit_helper`, not in the image under test."""
    for env in ({}, {'VISIBLE': '1'}):
        _, docker, _ = run_submission(sandbox, **env)
        # the stub joins argv with spaces, so a record is "exec <ctr> ... <cmd>"
        for rec in docker.split('\nexec ')[1:]:
            if 'xdotool' in rec:
                assert rec.startswith('pit_helper '), rec
