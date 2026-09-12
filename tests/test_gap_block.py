"""The prepared `GAP_BLOCK` in `submission/autodrive_devkit.sh`.

Sept 12, real display, Ultra, 0.058 s, stalls injected with
`scripts/stall_inject.sh --mode sim` (docker pause of the simulator):

  * `gap_predict_s:=0.6` on mpc3 ALONE -- 8 pauses of 0.4-0.63 s -- gave 32
    laps and 0 collisions, but ONE 21.2 s lap: the car stopped dead at s 27.1
    for 13 s because localization declared itself lost after a stall and, with
    the shipped `publish_on_reloc:=false`, published no pose for the tracker to
    ride on.
  * the same plus the filter's five gap parameters (`stall_both`, 8 pauses of
    0.4-0.6 s) gave **32 laps, 0 collisions, longest lap 7.54 s, OFFICIAL
    70.18 s** -- the no-stall control's race time.

So the two lines ship together or not at all, and this file covers both halves
of "uncomment them on the day and nothing surprises us":

1. **The parameters are the ones the nodes declare, with the right TYPES.**
   rclpy refuses an integer override for a double parameter and the node exits
   on launch -- today's turn-in probe lost a run to exactly that (`3` for
   `gap_noise_mult`, a double).  The values here are parsed the way
   `--ros-args -p name:=value` parses them and checked against the declared
   defaults, and then the REAL node classes are constructed with them under
   `tests/rclpy_stub` and asked what they ended up with.
2. **Uncommenting the block is all it takes.**  The two lines are lifted out of
   the shipped script, uncommented in a temporary copy, and the real launch
   functions are run over that copy with stub `setsid`/`ros2`, so the argv is
   the argv the container would produce -- after the built-in `CTRL_ARGS` /
   `LOC_ARGS` and before the `PIT_CREW_*_ARGS` environment, which must keep the
   last word.
"""

import ast
import os
import re
import subprocess
import sys

import pytest

import test_entrypoint_args as ea
import test_node_smoke as smoke

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENTRY = os.path.join(REPO, 'submission', 'autodrive_devkit.sh')
SRC = os.environ.get('PIT_CREW_SRC') or os.path.join(REPO, 'src')

# What was driven, so a silent edit to the script cannot quietly change it.
CTRL_WANT = {'gap_predict_s': 0.6}
LOC_WANT = {'gap_dt': 0.15, 'gap_noise_mult': 3.0, 'lost_patience_min': 3,
            'scan_dt_max': 1.2, 'enc_gap_max': 1.5}


# ------------------------------------------------------------ the block

def block_lines():
    """The two prepared lines, comment prefix and all."""
    src = ea.source()
    m = re.search(r'^# GAP_BLOCK_BEGIN\n(.*?)^# GAP_BLOCK_END$', src,
                  re.M | re.S)
    assert m, 'no GAP_BLOCK markers in the entrypoint'
    lines = [ln for ln in m.group(1).splitlines() if ln.strip()]
    assert len(lines) == 2, lines
    return lines


def uncomment(line):
    out = re.sub(r'^#\s?', '', line)
    assert out != line and not out.lstrip().startswith('#'), line
    return out


def ros_literal(text):
    """Parse `-p name:=VALUE` the way rclpy does: YAML scalar, so `3` is an
    integer and `3.0` is a double, and the two are NOT interchangeable."""
    if text in ('true', 'false'):
        return text == 'true'
    try:
        return int(text)                      # int before float, on purpose
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def params_of(line):
    """{name: value} for every `-p name:=value` in one prepared line."""
    return {n: ros_literal(v)
            for n, v in re.findall(r'-p\s+([A-Za-z_][\w]*):=(\S+?)\)?(?:\s|$)',
                                   uncomment(line))}


def ctrl_params():
    return params_of(next(ln for ln in block_lines() if 'CTRL_ARGS' in ln))


def loc_params():
    return params_of(next(ln for ln in block_lines() if 'LOC_ARGS' in ln))


def declared(module):
    """{name: default} for every ('name', default) pair declared in a node."""
    tree = ast.parse(open(os.path.join(SRC, 'pit_crew', module + '.py')).read())
    out = {}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Tuple) and len(n.elts) == 2
                and isinstance(n.elts[0], ast.Constant)
                and isinstance(n.elts[0].value, str)):
            try:
                out[n.elts[0].value] = ast.literal_eval(n.elts[1])
            except ValueError:
                pass
    return out


class TestTheBlockSaysWhatWasDriven:
    def test_one_line_per_array(self):
        lines = block_lines()
        assert sum('CTRL_ARGS+=(' in ln for ln in lines) == 1, lines
        assert sum('LOC_ARGS+=(' in ln for ln in lines) == 1, lines

    def test_the_controller_line_is_the_measured_one(self):
        assert ctrl_params() == CTRL_WANT

    def test_the_localization_line_is_the_measured_set(self):
        assert loc_params() == LOC_WANT

    def test_it_ships_commented_out(self):
        """Both halves, or the 13 s stop.  The runbook's enable step is an
        uncomment of the WHOLE block, so neither line may be live alone."""
        for ln in block_lines():
            assert ln.lstrip().startswith('#'), ln

    def test_the_live_measurement_is_recorded_next_to_the_lines(self):
        src = ea.source()
        head = src[:src.index('# GAP_BLOCK_BEGIN')]
        for number in ('70.18', '13 s', '32 laps'):
            assert number in head, number


class TestRclpyWouldAcceptEveryOverride:
    """The failure mode: `-p gap_noise_mult:=3` for a parameter declared
    `('gap_noise_mult', 1.0)`.  rclpy raises `InvalidParameterTypeException`
    and the node exits on launch -- discovered today on the turn-in probe."""

    @pytest.mark.parametrize('node,getter', [('mpc3', ctrl_params),
                                             ('localization', loc_params)])
    def test_every_name_is_declared_by_the_node(self, node, getter):
        d = declared(node)
        for name in getter():
            assert name in d, (node, name)

    @pytest.mark.parametrize('node,getter', [('mpc3', ctrl_params),
                                             ('localization', loc_params)])
    def test_no_integer_stands_in_for_a_double(self, node, getter):
        d = declared(node)
        for name, value in getter().items():
            want, got = type(d[name]), type(value)
            assert want is got, (
                '%s: %s is declared %s but the script passes %r (%s); rclpy '
                'does not convert' % (node, name, want.__name__, value,
                                      got.__name__))

    def test_the_double_ones_really_are_doubles(self):
        """Named explicitly so the check cannot pass vacuously if the
        declarations themselves drift to ints."""
        d = declared('localization')
        for name in ('gap_dt', 'gap_noise_mult', 'scan_dt_max', 'enc_gap_max'):
            assert isinstance(d[name], float) and not isinstance(d[name], bool)
        assert isinstance(d['lost_patience_min'], int)
        assert isinstance(declared('mpc3')['gap_predict_s'], float)


class TestTheRealNodesTakeThem:
    """Construct the shipped node classes with exactly the overrides the
    entrypoint would pass, and read back what they stored."""

    def localization(self):
        from pit_crew.localization import ParticleFilter
        return smoke.make_node(
            ParticleFilter, map_prefix=smoke.MAP_PREFIX,
            centerline_csv=smoke.CENTERLINE, n_particles=200,
            lidar_offset=smoke.vehicle.LIDAR_FROM_POSE,
            max_step_mps=9.0, publish_on_reloc=False, **loc_params())

    def tracker(self):
        from pit_crew.mpc3 import MPC3Tracker
        return smoke.make_node(
            MPC3Tracker, raceline_csv=smoke.MPC3_LINE,
            map_prefix=smoke.MAP_PREFIX, rate_adaptive=True,
            w_dcmd=50.0, cmd_tau=0.10, steer_tau=0.04, barrier_margin=0.40,
            **ctrl_params())

    def test_localization_stores_the_five_values(self):
        n = self.localization()
        got = {'gap_dt': n.gap_dt, 'gap_noise_mult': n.gap_noise_mult,
               'lost_patience_min': n.lost_patience_min,
               'scan_dt_max': n.scan_dt_max, 'enc_gap_max': n.enc_gap_max}
        assert got == LOC_WANT
        for name, value in got.items():
            assert type(value) is type(LOC_WANT[name]), name
        smoke.assert_no_errors(n)

    def test_the_encoder_gap_reaches_the_speed_estimator(self):
        """`enc_gap_max` only does anything through `SpeedEstimator.max_gap`;
        at the default 0.0 it is left at the vehicle constant."""
        n = self.localization()
        assert n.speed_est.max_gap == pytest.approx(1.5)
        assert smoke.vehicle.MAX_ENC_GAP != 1.5

    def test_localization_still_converges_with_them_on(self):
        n = self.localization()
        line = smoke.Line(smoke.MPC3_LINE)
        _, _, yaw0, _ = line.at(0.0)
        smoke.drive_perception(n, yaw0)
        assert len(n.stub_published('/pit_crew/pose')) >= smoke.CYCLES - 2
        smoke.assert_no_errors(n)

    def test_the_tracker_arms_its_gap_rider(self):
        n = self.tracker()
        assert n.gap.predict_s == pytest.approx(0.6)
        assert n.gap.enabled
        smoke.assert_no_errors(n)

    def test_the_tracker_still_drives_with_it_on(self):
        n = self.tracker()
        smoke.drive_tracker(n, smoke.Line(smoke.MPC3_LINE))
        smoke.assert_commands(n)
        smoke.assert_no_errors(n)


# ------------------------------------------------- uncommenting the block

def run_with_block(func, env=None):
    """Uncomment the GAP_BLOCK in a copy of the script and run `func` from it.

    Same machinery as `test_entrypoint_args.run`, but over the EDITED source,
    so what is exercised is the line as it will be shipped once enabled --
    including its quoting and the array it appends to.
    """
    src = ea.source()
    for line in block_lines():
        src = src.replace(line, uncomment(line), 1)
    prelude = [
        'setsid() { "$@"; }',
        'ros2() { for a in "$@"; do printf "%s\\n" "$a"; done; }',
        'MAP=/m; CENTER=/c.csv; LINE=/l.csv; SCALE=1.00',
        'LIDAR_OFFSET=0.1933; CONTROLLER=mpc3',
        # the built-ins as they ship today
        'CTRL_ARGS=(-p map_prefix:="$MAP" -p rate_adaptive:=true'
        ' -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p barrier_margin:=0.40)',
        'LOC_ARGS=(-p max_step_mps:=9.0 -p publish_on_reloc:=false)',
    ]
    appends = [uncomment(ln) for ln in block_lines()]
    build = re.findall(
        r'^if \[\[ -n "\$\{PIT_CREW_(?:CONTROLLER|LOCALIZATION)_ARGS.*?^fi$',
        src, re.M | re.S)
    assert len(build) == 2, build
    script = '\n'.join(prelude + appends + build
                       + [ea.lift(func, src), func, 'wait'])
    out = subprocess.run(['bash', '-c', script], capture_output=True,
                         text=True, check=True,
                         env={**os.environ, **(env or {})})
    return out.stdout.split('\n')[:-1]


def pairs(argv):
    return [(argv[i + 1].split(':=')[0], argv[i + 1].split(':=')[1])
            for i, a in enumerate(argv) if a == '-p']


class TestUncommentingTheBlockIsEnough:
    def test_the_edited_script_still_parses(self, tmp_path):
        src = ea.source()
        for line in block_lines():
            src = src.replace(line, uncomment(line), 1)
        path = tmp_path / 'autodrive_devkit.sh'
        path.write_text(src)
        subprocess.run(['bash', '-n', str(path)], check=True)

    def test_the_controller_argv_carries_the_gap_parameter(self):
        argv = run_with_block('start_ctrl', {'PIT_CREW_CONTROLLER_ARGS': ''})
        assert ('gap_predict_s', '0.6') in pairs(argv)
        # after the built-ins, so it could override one of them
        assert argv.index('gap_predict_s:=0.6') > argv.index('cmd_tau:=0.10')

    def test_the_localization_argv_carries_all_five(self):
        argv = run_with_block('start_loc', {'PIT_CREW_LOCALIZATION_ARGS': ''})
        got = pairs(argv)
        for name, value in LOC_WANT.items():
            assert (name, str(value)) in got, (name, got)
        assert argv.index('gap_dt:=0.15') > argv.index('max_step_mps:=9.0')

    def test_no_argument_is_split_or_lost(self):
        """`LOC_ARGS+=(...)` on ONE line: a continuation would make `-p` and
        its value separate array elements only if the quoting survives."""
        argv = run_with_block('start_loc', {'PIT_CREW_LOCALIZATION_ARGS': ''})
        assert '' not in argv
        assert argv.count('-p') == 3 + 2 + 5   # built-in 3, LOC_ARGS 2, gap 5
        assert all(a == '-p' or ':=' in a or '/' in a or a in (
            'run', 'pit_crew', 'localization', '--ros-args') for a in argv)

    @pytest.mark.parametrize('func,var,over,name', [
        ('start_ctrl', 'PIT_CREW_CONTROLLER_ARGS',
         '-p gap_predict_s:=0.0', 'gap_predict_s'),
        ('start_loc', 'PIT_CREW_LOCALIZATION_ARGS',
         '-p gap_dt:=0.0', 'gap_dt'),
    ])
    def test_the_environment_still_wins_over_the_block(self, func, var, over,
                                                       name):
        """ROS takes the LAST `-p name:=value`, so turning the block back OFF
        on the organisers' box must not need a rebuild."""
        argv = run_with_block(func, {var: over})
        assert argv[-2:] == ['-p', '%s:=0.0' % name]
        assert argv.index('%s:=0.0' % name) > argv.index(
            '%s:=%s' % (name, dict(CTRL_WANT, **LOC_WANT)[name]))
