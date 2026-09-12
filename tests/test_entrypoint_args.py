"""`PIT_CREW_CONTROLLER_ARGS` / `PIT_CREW_LOCALIZATION_ARGS` in the entrypoint.

`submission/autodrive_devkit.sh` cannot be run on the host (no ROS, no devkit
overlay), so -- in the same spirit as `tests/test_mpc3_startup.py` lifting
methods out of `mpc3.py` with `ast` -- these tests lift the two launch
*functions* out of the shipped script with a small brace-matching parse and run
them under `bash` with `setsid` and `ros2` replaced by stubs that only record
their argv.  That exercises the real quoting and word-splitting rather than a
copy of it.

The supervisor loop restarts a dead node by calling those same functions, so a
source-level assertion covers the restart path: what has to stay true is that
there is exactly ONE place that spells out each `ros2 run`.
"""

import os
import re
import subprocess

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENTRY = os.path.join(REPO, 'submission', 'autodrive_devkit.sh')


def source():
    with open(ENTRY) as fh:
        return fh.read()


def lift(name, src=None):
    """Return the text of shell function `name`, by matching its braces."""
    src = source() if src is None else src
    m = re.search(r'^%s\(\)\s*\{' % re.escape(name), src, re.M)
    assert m, 'no %s() in the entrypoint' % name
    depth = 0
    for i in range(m.end() - 1, len(src)):
        if src[i] == '{':
            depth += 1
        elif src[i] == '}':
            depth -= 1
            if depth == 0:
                return src[m.start():i + 1]
    raise AssertionError('unbalanced braces in %s()' % name)


def run(func, env):
    """Run a lifted launch function with stub `setsid`/`ros2`; return argv."""
    prelude = [
        'setsid() { "$@"; }',
        'ros2() { for a in "$@"; do printf "%s\\n" "$a"; done; }',
        'MAP=/m; CENTER=/c.csv; LINE=/l.csv; SCALE=1.00',
        'LIDAR_OFFSET=0.1933; CONTROLLER=mpc3',
        'CTRL_ARGS=(-p map_prefix:="$MAP")',
        'EXTRA_CTRL_ARGS=(); EXTRA_LOC_ARGS=()',
    ]
    # The real script builds the EXTRA_* arrays at top level; lift that too.
    src = source()
    build = re.findall(
        r'^if \[\[ -n "\$\{PIT_CREW_(?:CONTROLLER|LOCALIZATION)_ARGS.*?^fi$',
        src, re.M | re.S)
    assert len(build) == 2, build
    script = '\n'.join(prelude + build + [lift(func), func, 'wait'])
    out = subprocess.run(['bash', '-c', script], env={**os.environ, **env},
                         capture_output=True, text=True, check=True)
    return out.stdout.split('\n')[:-1]


class TestSyntax:
    def test_the_entrypoint_parses(self):
        subprocess.run(['bash', '-n', ENTRY], check=True)


class TestControllerArgs:
    def test_nothing_is_appended_by_default(self):
        argv = run('start_ctrl', {'PIT_CREW_CONTROLLER_ARGS': ''})
        assert argv == ['run', 'pit_crew', 'mpc3', '--ros-args',
                        '-p', 'raceline_csv:=/l.csv',
                        '-p', 'speed_scale:=1.00',
                        '-p', 'map_prefix:=/m']

    def test_an_unset_variable_is_the_same_as_empty(self):
        env = {k: v for k, v in os.environ.items()}
        env.pop('PIT_CREW_CONTROLLER_ARGS', None)
        argv = run('start_ctrl', env)
        assert 'raceline_csv:=/l.csv' in argv
        assert '' not in argv          # an empty array element would be a arg

    def test_the_documented_form_word_splits_into_separate_arguments(self):
        argv = run('start_ctrl', {
            'PIT_CREW_CONTROLLER_ARGS':
                '-p rate_adaptive:=true -p ilc_enable:=true'})
        assert argv[-4:] == ['-p', 'rate_adaptive:=true',
                             '-p', 'ilc_enable:=true']
        assert argv.count('-p') == 5   # 3 built-in + 2 appended
        # ...and they come AFTER the built-ins, so they can override them
        assert argv.index('rate_adaptive:=true') > argv.index('map_prefix:=/m')

    def test_extra_whitespace_does_not_produce_empty_arguments(self):
        argv = run('start_ctrl',
                   {'PIT_CREW_CONTROLLER_ARGS': '  -p  ilc_enable:=true  '})
        assert argv[-2:] == ['-p', 'ilc_enable:=true']
        assert '' not in argv


class TestLocalizationArgs:
    def test_nothing_is_appended_by_default(self):
        argv = run('start_loc', {'PIT_CREW_LOCALIZATION_ARGS': ''})
        assert argv == ['run', 'pit_crew', 'localization', '--ros-args',
                        '-p', 'map_prefix:=/m',
                        '-p', 'centerline_csv:=/c.csv',
                        '-p', 'lidar_offset:=0.1933']

    def test_the_documented_form_word_splits(self):
        argv = run('start_loc',
                   {'PIT_CREW_LOCALIZATION_ARGS': '-p n_particles:=1200'})
        assert argv[-2:] == ['-p', 'n_particles:=1200']
        assert argv.index('n_particles:=1200') > argv.index('lidar_offset:=0.1933')


class TestTheSupervisorRestartsWithTheSameArguments:
    """The failure this guards against: a node that dies mid-race comes back
    without the flags it was started with, changing the configuration silently
    partway through a scored 12-lap run."""

    @pytest.mark.parametrize('func,node', [('start_ctrl', '"$CONTROLLER"'),
                                           ('start_loc', 'localization')])
    def test_only_the_launch_function_spells_out_the_ros2_run(self, func, node):
        src = source()
        body = lift(func, src)
        pat = re.compile(r'ros2 run pit_crew %s' % re.escape(node))
        assert len(pat.findall(src)) == 1, \
            'a second %s launch site would need the EXTRA_* args too' % node
        assert pat.search(body), (func, node)

    @pytest.mark.parametrize('func,var', [('start_ctrl', 'EXTRA_CTRL_ARGS'),
                                          ('start_loc', 'EXTRA_LOC_ARGS')])
    def test_the_launch_function_appends_the_extra_arguments(self, func, var):
        assert '"${%s[@]}"' % var in lift(func, src=source())

    @pytest.mark.parametrize('func', ['start_ctrl', 'start_loc'])
    def test_the_restart_path_calls_the_launch_function(self, func):
        src = source()
        loop = src[src.index('while :; do'):]
        assert func in loop, \
            '%s is not restarted by the supervisor loop' % func

    def test_the_fallback_and_restart_paths_do_not_inline_a_launch(self):
        src = source()
        loop = src[src.index('while :; do'):]
        assert 'ros2 run' not in loop
