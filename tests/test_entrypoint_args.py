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
        'LOC_ARGS=()',
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


class TestTheBuiltInArgArraysComeBeforeTheEnvironment:
    """`CTRL_ARGS` / `LOC_ARGS` hold the parameters the SHIP decision turns on
    (the mpc3 rate fix, the localization speed cap). They must be appended at
    the launch functions -- so a supervisor restart carries them -- and must
    come BEFORE the `PIT_CREW_*_ARGS` environment, so a finals-day override
    still wins."""

    @pytest.mark.parametrize('func,built_in,extra', [
        ('start_ctrl', 'CTRL_ARGS', 'EXTRA_CTRL_ARGS'),
        ('start_loc', 'LOC_ARGS', 'EXTRA_LOC_ARGS'),
    ])
    def test_built_in_then_environment(self, func, built_in, extra):
        body = lift(func, src=source())
        assert '"${%s[@]}"' % built_in in body, (func, built_in)
        assert body.index('${%s[@]}' % built_in) < body.index('${%s[@]}' % extra)

    def test_the_controller_override_wins_over_the_built_in(self):
        """`CTRL_ARGS` carries the parameters the finals ship decision turns on
        (`rate_adaptive`, the smoothing pair, and the prepared
        `barrier_margin`).  A last-minute `-e PIT_CREW_CONTROLLER_ARGS=...` on
        the organisers' box has to be able to change any of them, and ROS takes
        the LAST `-p name:=value` for a repeated name -- so the environment's
        copy must be emitted after the built-in one."""
        src = source()
        script = '\n'.join([
            'setsid() { "$@"; }',
            'ros2() { for a in "$@"; do printf "%s\\n" "$a"; done; }',
            'LINE=/l.csv; SCALE=1.00; CONTROLLER=mpc3',
            'CTRL_ARGS=(-p map_prefix:=/m -p rate_adaptive:=true'
            ' -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p barrier_margin:=0.34)',
        ] + re.findall(
            r'^if \[\[ -n "\$\{PIT_CREW_CONTROLLER_ARGS.*?^fi$',
            src, re.M | re.S) + [lift('start_ctrl', src), 'start_ctrl', 'wait'])
        out = subprocess.run(
            ['bash', '-c', script], capture_output=True, text=True, check=True,
            env={**os.environ,
                 'PIT_CREW_CONTROLLER_ARGS':
                     '-p barrier_margin:=0.30 -p steer_tau:=0.04'})
        argv = out.stdout.split('\n')[:-1]
        assert argv[-4:] == ['-p', 'barrier_margin:=0.30',
                             '-p', 'steer_tau:=0.04']
        # both copies are present, the environment's last
        assert argv.index('barrier_margin:=0.34') \
            < argv.index('barrier_margin:=0.30')
        # ...and the built-ins that are NOT overridden survive untouched
        for kept in ('rate_adaptive:=true', 'w_dcmd:=50.0', 'cmd_tau:=0.10'):
            assert kept in argv, kept

    def test_the_localization_override_wins_over_the_built_in(self):
        """Both arrays populated: the environment's copy is last, and ROS takes
        the last `-p name:=value` for a repeated name."""
        src = source()
        script = '\n'.join([
            'setsid() { "$@"; }',
            'ros2() { for a in "$@"; do printf "%s\\n" "$a"; done; }',
            'MAP=/m; CENTER=/c.csv; LIDAR_OFFSET=0.1933',
            'LOC_ARGS=(-p max_step_mps:=9.0)',
        ] + re.findall(
            r'^if \[\[ -n "\$\{PIT_CREW_LOCALIZATION_ARGS.*?^fi$',
            src, re.M | re.S) + [lift('start_loc', src), 'start_loc', 'wait'])
        out = subprocess.run(
            ['bash', '-c', script], capture_output=True, text=True, check=True,
            env={**os.environ,
                 'PIT_CREW_LOCALIZATION_ARGS': '-p max_step_mps:=7.0'})
        argv = out.stdout.split('\n')[:-1]
        assert argv[-4:] == ['-p', 'max_step_mps:=9.0',
                             '-p', 'max_step_mps:=7.0']


class TestThePreparedFinalsLines:
    """The prepared-but-commented finals knobs must stay reachable.

    Each is one uncommented line away from shipping, and the runbook's
    checklist points at it by name.  This asserts the line is still THERE and
    still spells the parameter correctly -- it deliberately does NOT assert it
    is commented out, because enabling it is the expected outcome of Gate D.
    """

    @pytest.mark.parametrize('param,value', [
        ('barrier_margin', '0.34'),
        ('max_step_mps', '9.0'),
        ('gap_dt', '0.15'),
        ('teleport_scan_jump', '0.35'),
    ])
    def test_the_prepared_line_is_present(self, param, value):
        src = source()
        assert '%s:=%s' % (param, value) in src, param

    @pytest.mark.parametrize('param', ['barrier_margin', 'w_dcmd', 'cmd_tau',
                                       'rate_adaptive', 'steer_tau',
                                       'gap_predict_s', 'max_step_mps',
                                       'publish_on_reloc', 'gap_noise_mult',
                                       'teleport_scan_jump'])
    def test_every_parameter_named_in_the_script_exists_in_the_node(self, param):
        """A prepared line that names a parameter the node does not declare is
        a container that exits on launch, found on the finals day."""
        node = ('localization' if param in (
            'max_step_mps', 'publish_on_reloc', 'gap_dt', 'gap_noise_mult',
            'teleport_scan_jump') else 'mpc3')
        path = os.path.join(REPO, 'src', 'pit_crew', node + '.py')
        with open(path) as fh:
            assert "'%s'" % param in fh.read(), (param, node)


class TestTheLineDefaultFollowsTheMap:
    """The finals mistake this closes: switching PIT_CREW_MAP to the finals
    map alone used to leave the legacy-`mpc` branch on Porto's
    `track_v2_safe.csv`. Preflight refuses that, which means the container
    exits 2 and there is no race at all."""

    def block(self):
        src = source()
        m = re.search(r'^if \[\[ -n "\$\{PIT_CREW_LINE.*?^fi$', src,
                      re.M | re.S)
        assert m, 'the entrypoint no longer derives LINE in one block'
        return m.group(0)

    def line_for(self, slug, controller, env=None):
        script = '\n'.join([
            'SLUG=%s; CONTROLLER=%s' % (slug, controller),
            self.block(), 'printf "%s\\n" "$LINE"'])
        out = subprocess.run(['bash', '-c', script], capture_output=True,
                             text=True, check=True,
                             env={**os.environ, **(env or {})})
        return out.stdout.strip()

    @pytest.mark.parametrize('slug,controller,want', [
        ('porto_mesh', 'mpc3', '/home/data/racelines/porto_mesh_cr_dk0.8.csv'),
        ('porto_mesh', 'mpc', '/home/data/racelines/porto_mesh_safe.csv'),
        ('finals_mesh', 'mpc3', '/home/data/racelines/finals_mesh_hyb675b_race.csv'),
        ('finals_mesh', 'mpc', '/home/data/racelines/finals_mesh_safe.csv'),
    ])
    def test_the_default_is_derived_from_the_slug(self, slug, controller, want):
        env = {k: v for k, v in os.environ.items()}
        env.pop('PIT_CREW_LINE', None)
        assert self.line_for(slug, controller, env) == want

    def test_no_default_names_another_track(self):
        env = {k: v for k, v in os.environ.items()}
        env.pop('PIT_CREW_LINE', None)
        for controller in ('mpc3', 'mpc'):
            got = self.line_for('finals_mesh', controller, env)
            assert 'track_v2' not in got and 'porto' not in got, got

    def test_an_explicit_line_still_wins(self):
        got = self.line_for('finals_mesh', 'mpc3',
                            {'PIT_CREW_LINE': '/home/data/racelines/x.csv'})
        assert got == '/home/data/racelines/x.csv'


# ---------------------------------------------------------------- race-day
# Audit, Sep 19. Everything below pins what the script ACTUALLY does on the
# organisers' box, as opposed to what any one line of it reads like. Three
# failures these close, each of which is a scored race and none of which the
# tests above could see:
#
#   * a `-p name:=value` whose parameter the node does not declare -- rclpy
#     refuses the override and the node exits ON LAUNCH, so the supervisor
#     restarts it four times and then drops the race to Follow-the-Gap. The
#     hand-maintained list in `TestThePreparedFinalsLines` covered ten names
#     and missed every brake parameter and `pub_gate_corr`, i.e. the entire
#     change that made the finals config clean;
#   * an earlier `-p barrier_margin:=...` being read as the shipping value
#     when a later one overrides it (ROS takes the last);
#   * a commented `*_ARGS+=(` split over two lines, where uncommenting one
#     half is a syntax error and the container never starts at all.

ARG_RE = re.compile(r'-p\s+([A-Za-z_][A-Za-z0-9_]*):=(\S+?)(?=[\s)]|$)')


def arg_lines():
    """(node, param, value, raw line) for every `-p` the script can emit.

    Commented lines count: they are the prepared knobs, and the runbook's
    procedure for each of them is "uncomment this one line".
    """
    out = []
    for raw in source().splitlines():
        line = raw.strip()
        body = line[1:].strip() if line.startswith('#') else line
        if 'LOC_ARGS' in body:
            node = 'localization'
        elif 'CTRL_ARGS' in body:
            node = 'mpc3'
        elif 'ros2 run pit_crew localization' in body or 'centerline_csv' in body:
            node = 'localization'
        elif 'raceline_csv' in body or 'speed_scale' in body:
            node = 'mpc3'
        else:
            continue
        for param, value in ARG_RE.findall(body):
            out.append((node, param, value, raw))
    return out


def declared(node):
    with open(os.path.join(REPO, 'src', 'pit_crew', node + '.py')) as fh:
        return fh.read()


class TestEveryParameterTheScriptCanEmitIsDeclared:
    """Generic replacement for the hand-kept list: nothing may be missed."""

    def test_at_least_the_known_set_is_found(self):
        params = {(n, p) for n, p, _v, _l in arg_lines()}
        for want in [('mpc3', 'barrier_margin'), ('mpc3', 'brake_mode'),
                     ('mpc3', 'pub_gate_corr'), ('mpc3', 'raceline_csv'),
                     ('localization', 'max_step_mps'),
                     ('localization', 'teleport_scan_jump')]:
            assert want in params, (want, sorted(params))

    def test_every_one_of_them_exists_in_its_node(self):
        missing = [(n, p, line.strip()) for n, p, _v, line in arg_lines()
                   if "'%s'" % p not in declared(n)]
        assert not missing, (
            'these `-p name:=value` overrides name parameters the node does '
            'not declare; rclpy rejects them and the node exits on launch: %r'
            % missing)

    def test_no_double_is_written_as_an_integer(self):
        """`gap_noise_mult:=3` cost a run: rclpy refuses an int for a double.

        Any value with a decimal point somewhere else in the file must not
        appear without one here, so check the file is self-consistent: a
        parameter written `x.y` anywhere is never written bare elsewhere.
        """
        seen = {}
        for _n, param, value, _l in arg_lines():
            if not re.fullmatch(r'-?\d+(\.\d+)?', value):
                continue                      # "$LINE", true/false, zero, ...
            seen.setdefault(param, set()).add('.' in value)
        mixed = [p for p, kinds in seen.items() if kinds == {True, False}]
        assert not mixed, ('written both as an int and as a double: %r' % mixed)


class TestTheEffectiveArgsAreTheFrozenFinalsSet:
    """What the controller is actually launched with, per map slug.

    Built by running the script's own LINE / CTRL_ARGS / LOC_ARGS blocks, so
    it cannot drift from them, and reduced the way ROS reduces a repeated
    `-p name:=value`: the LAST one wins.
    """

    def blocks(self):
        src = source()
        return '\n'.join([
            re.search(r'^if \[\[ -n "\$\{PIT_CREW_LINE.*?^fi$', src,
                      re.M | re.S).group(0),
            src[src.index('CTRL_ARGS=()'):
                src.index('# Built-in localization parameters')],
            src[src.index('LOC_ARGS=()'):
                src.index('# Extra `-p name:=value` parameters')],
        ])

    def effective(self, slug, controller):
        script = '\n'.join([
            'MAP=/home/data/maps/%s; SLUG=%s; CONTROLLER=%s'
            % (slug, slug, controller),
            self.blocks(),
            'printf "%s\\n" "$LINE"',
            'printf "%s\\n" "${CTRL_ARGS[*]}"',
            'printf "%s\\n" "${LOC_ARGS[*]}"',
        ])
        env = {k: v for k, v in os.environ.items()}
        for k in ('PIT_CREW_LINE', 'PIT_CREW_CONTROLLER_ARGS',
                  'PIT_CREW_LOCALIZATION_ARGS'):
            env.pop(k, None)
        out = subprocess.run(['bash', '-c', script], env=env, check=True,
                             capture_output=True, text=True)
        line, ctrl, loc = out.stdout.split('\n')[:3]
        last = {}
        for text in (ctrl, loc):
            for param, value in ARG_RE.findall(text):
                last[param] = value           # ROS takes the last
        return line, last

    FROZEN = {
        'rate_adaptive': 'true', 'w_dcmd': '50.0', 'cmd_tau': '0.10',
        'steer_tau': '0.04', 'barrier_margin': '0.40',
        'thr_brake_lead_s': '0.05', 'brake_mode': 'zero',
        'brake_a_min': '1.5', 'brake_dv': '0.3', 'brake_a_full': '4.6',
        'pub_gate_corr': '0.05',
        'max_step_mps': '9.0', 'publish_on_reloc': 'false',
    }

    def test_the_finals_default_is_the_frozen_config(self):
        """docs/FINALS_TUNING.md section 1, plus `pub_gate_corr` (Sep 19).

        Gate D was passed on exactly this set. If this test fails, either the
        image and the documentation have diverged or somebody has changed the
        race after the gate.
        """
        line, eff = self.effective('finals_mesh', 'mpc3')
        assert line == '/home/data/racelines/finals_mesh_hyb675b_race.csv', line
        assert eff['map_prefix'] == '/home/data/maps/finals_mesh'
        for param, value in self.FROZEN.items():
            assert eff.get(param) == value, (param, eff.get(param), value)

    def test_the_barrier_margin_that_ships_is_the_last_one_written(self):
        """0.34 and the `case *cr_dk0.8.csv` line are both dead. Reading
        either of them as the shipping value is how a 0.40-only regression
        gets attributed to the wrong knob."""
        src = source()
        assert 'barrier_margin:=0.34' in src      # still present, still dead
        for slug in ('finals_mesh', 'porto_mesh', 'icra2026_mesh'):
            _line, eff = self.effective(slug, 'mpc3')
            assert eff['barrier_margin'] == '0.40', slug

    def test_the_legacy_mpc_is_passed_no_mpc3_only_parameter(self):
        """`mpc` does not declare `map_prefix`, `brake_mode` or the rest, and
        a node handed an undeclared override exits on launch."""
        _line, eff = self.effective('finals_mesh', 'mpc')
        for param in ('map_prefix', 'brake_mode', 'barrier_margin',
                      'pub_gate_corr', 'thr_brake_lead_s'):
            assert param not in eff, param
        assert eff.get('max_step_mps') == '9.0'   # localization still gets its


class TestACommentedArgLineCanBeUncommentedAlone:
    """Every prepared knob's procedure is "strip the leading `# `" from one
    line. A commented `LOC_ARGS+=(` continued onto a second comment line is an
    unbalanced array literal: uncomment the first half and the container dies
    with a syntax error, on the day, with no shell history to read."""

    def test_each_commented_arg_line_is_balanced(self):
        bad = []
        for raw in source().splitlines():
            line = raw.strip()
            if not line.startswith('#'):
                continue
            body = line[1:].strip()
            if not re.match(r'(CTRL_ARGS|LOC_ARGS)\+?=\(', body):
                continue
            if body.count('(') != body.count(')'):
                bad.append(raw)
        assert not bad, (
            'commented argument lines that are not self-contained; '
            'uncommenting one of them alone is a syntax error: %r' % bad)

    def test_uncommenting_any_one_of_them_still_parses(self):
        src = source()
        for raw in src.splitlines():
            body = raw.strip()
            if not body.startswith('#'):
                continue
            inner = body[1:].strip()
            if not re.match(r'(CTRL_ARGS|LOC_ARGS)\+?=\(', inner):
                continue
            indent = raw[:len(raw) - len(raw.lstrip())]
            patched = src.replace(raw, indent + inner, 1)
            proc = subprocess.run(['bash', '-n'], input=patched, text=True,
                                  capture_output=True)
            assert proc.returncode == 0, (raw, proc.stderr)
