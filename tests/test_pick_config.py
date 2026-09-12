"""`tools/pick_config.py` and the entrypoint block that consumes its sidecar.

Two things are covered, and they meet in the middle:

  * the DERIVATION -- feasible margin = min half-width - car half-width, and
    the two rules that read it.  Measured lines are used as the fixtures where
    they exist, so the thresholds are pinned against the geometry they were
    set from (docs/STATUS.md, "Barrier mechanism" and "Pinch-free fast lines").
  * the CONSUMER -- the `PICK_BLOCK` in `submission/autodrive_devkit.sh`, which
    is shipped COMMENTED OUT.  The test lifts it out of the script, strips the
    comment prefix and runs it under bash against a fixture sidecar, so
    enabling it on the day is an uncomment and not a rewrite.
"""

import json
import os
import re
import subprocess
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENTRY = os.path.join(REPO, 'submission', 'autodrive_devkit.sh')
sys.path.insert(0, os.path.join(REPO, 'tools'))
sys.path.insert(0, os.path.join(REPO, 'src', 'pit_crew'))

import pick_config                                            # noqa: E402


def geo(**kw):
    g = {'min_halfwidth': 0.52, 'min_halfwidth_s': 0.0,
         'corridor_min_width': 1.14, 'kappa_max': 1.0, 'clearance': 0.39,
         'ideal_lap': 10.0, 'v_max': 8.0}
    g.update(kw)
    return g


class TestTheBarrierRule:
    def test_the_feasible_margin_is_the_half_width_minus_the_car(self):
        assert pick_config.decide(geo(min_halfwidth=0.480))[
            'feasible_margin'] == pytest.approx(0.345)

    @pytest.mark.parametrize('half,want', [
        (0.420, 0.34),      # Porto cr_dk0.8: cap 0.285
        (0.480, 0.34),      # the fast2_m50 lines: cap 0.345
        (0.520, 0.34),      # the safe lines / fast3_m53: cap 0.385
        (0.560, 0.40),      # cap 0.425, clears 0.40 + slack
        (0.720, 0.40),      # ICRA 2026: cap 0.585
    ])
    def test_040_only_where_the_line_can_pay_for_it(self, half, want):
        assert pick_config.decide(geo(min_halfwidth=half))[
            'barrier_margin'] == want

    def test_a_line_exactly_on_040_does_not_get_040(self):
        """The cap is measured on a 0.02 m march off a 0.025 m map, so sitting
        ON it is not distinguishable from sitting just under it."""
        assert pick_config.decide(geo(min_halfwidth=0.40 + 0.135))[
            'barrier_margin'] == 0.34

    def test_the_slack_is_a_knob_not_a_constant(self):
        assert pick_config.decide(geo(min_halfwidth=0.535),
                                  slack=0.0)['barrier_margin'] == 0.40


class TestTheBrakeLeadRule:
    @pytest.mark.parametrize('kappa,width,want', [
        (1.68, 1.14, 0.05),     # CDC-TF fast3_m53: tight on both tests
        (1.74, 1.10, 0.05),     # ICRA 2025 fast2_m50
        (0.83, 1.12, 0.05),     # Porto: open in curvature, narrow corridor
        (2.21, 1.52, 0.05),     # ICRA 2026: wide corridor, but a hairpin
        (0.90, 2.00, 0.00),     # neither test trips
    ])
    def test_tight_circuits_get_the_lead(self, kappa, width, want):
        assert pick_config.decide(geo(kappa_max=kappa,
                                      corridor_min_width=width))[
            'thr_brake_lead_s'] == want

    def test_008_is_never_produced(self):
        """0.08 is bit-identical to 0.05 at 0.097 s: never pay for it."""
        for kappa in (0.5, 1.2, 1.21, 3.0):
            for width in (0.9, 1.3, 2.5):
                assert pick_config.decide(
                    geo(kappa_max=kappa, corridor_min_width=width)
                )['thr_brake_lead_s'] in (0.0, 0.05)


class TestTheControllerArgsString:
    def test_both_parameters_are_spelled_out(self):
        s = pick_config.controller_args(
            {'barrier_margin': 0.40, 'thr_brake_lead_s': 0.05})
        assert s == '-p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05'

    @pytest.mark.parametrize('param', ['barrier_margin', 'thr_brake_lead_s'])
    def test_the_parameters_exist_in_the_node(self, param):
        """A pick that names a parameter mpc3 does not declare is a container
        that exits on launch."""
        with open(os.path.join(REPO, 'src', 'pit_crew', 'mpc3.py')) as fh:
            assert "'%s'" % param in fh.read()


class TestTheMeasuredGeometry:
    """The definition of "min half-width" is load-bearing: the barrier marches
    along the path normal, while the distance field at a point reports the
    nearest wall in ANY direction (on a hairpin, the inside one). These are the
    numbers docs/STATUS.md's feasible caps were derived from."""

    @pytest.mark.parametrize('track,line,half', [
        ('porto_mesh', 'porto_mesh_cr_dk0.8', 0.420),
        ('cdctf2025_mesh', 'cdctf2025_mesh_fast3_m53_race', 0.520),
        ('icra2025_mesh', 'icra2025_mesh_fast2_m50_race', 0.480),
    ])
    def test_the_shipped_lines_measure_what_status_recorded(self, track, line,
                                                            half):
        csv = os.path.join(REPO, 'data', 'racelines', line + '.csv')
        prefix = os.path.join(REPO, 'data', 'maps', track)
        if not os.path.exists(csv) or not os.path.exists(prefix + '.pgm'):
            pytest.skip('%s not present' % line)
        g = pick_config.geometry(prefix, csv)
        assert g['min_halfwidth'] == pytest.approx(half, abs=0.011)


class TestTheEntrypointPickBlock:
    """The block ships commented out; this runs it as it would be enabled."""

    def block(self):
        with open(ENTRY) as fh:
            src = fh.read()
        m = re.search(r'^\s*# PICK_BLOCK_BEGIN\n(.*?)^\s*# PICK_BLOCK_END$',
                      src, re.M | re.S)
        assert m, 'no PICK_BLOCK markers in the entrypoint'
        body = m.group(1)
        out = []
        for raw in body.splitlines():
            stripped = raw.strip()
            assert stripped.startswith('#'), (
                'the PICK block must stay COMMENTED OUT: %r' % raw)
            out.append(re.sub(r'^(\s*)# ?', r'\1', raw, count=1))
        return '\n'.join(out)

    def test_it_is_still_commented_out(self):
        self.block()            # asserts every line starts with '#'

    def test_it_is_valid_shell(self):
        subprocess.run(['bash', '-n', '-c', self.block()], check=True)

    def run(self, tmp_path, pick=None, slug='finals_mesh'):
        env = dict(os.environ)
        env.pop('PIT_CREW_PICK', None)
        if pick is not None:
            p = tmp_path / ('%s_pick.json' % slug)
            p.write_text(json.dumps(pick))
            env['PIT_CREW_PICK'] = str(p)
        else:
            env['PIT_CREW_PICK'] = str(tmp_path / 'absent.json')
        script = '\n'.join([
            'set -eo pipefail',
            'SLUG=%s' % slug,
            'CTRL_ARGS=(-p map_prefix:=/m -p barrier_margin:=0.34)',
            self.block(),
            'printf "%s\\n" "${CTRL_ARGS[@]}"',
        ])
        r = subprocess.run(['bash', '-c', script], env=env,
                           capture_output=True, text=True, check=True)
        return r.stdout.split('\n')[:-1]

    def test_a_pick_is_appended_after_the_built_ins(self, tmp_path):
        argv = self.run(tmp_path, {'controller_args':
                                   '-p barrier_margin:=0.40 '
                                   '-p thr_brake_lead_s:=0.05'})
        assert argv == ['-p', 'map_prefix:=/m', '-p', 'barrier_margin:=0.34',
                        '-p', 'barrier_margin:=0.40',
                        '-p', 'thr_brake_lead_s:=0.05']
        # last one wins in ros2 --ros-args, so the pick is the effective value
        assert argv.index('barrier_margin:=0.40') > \
            argv.index('barrier_margin:=0.34')

    def test_a_missing_sidecar_changes_nothing(self, tmp_path):
        assert self.run(tmp_path) == ['-p', 'map_prefix:=/m',
                                      '-p', 'barrier_margin:=0.34']

    def test_a_sidecar_without_the_key_changes_nothing(self, tmp_path):
        assert self.run(tmp_path, {'slug': 'finals_mesh'}) == [
            '-p', 'map_prefix:=/m', '-p', 'barrier_margin:=0.34']

    def test_an_unparseable_sidecar_does_not_kill_the_race(self, tmp_path):
        p = tmp_path / 'finals_mesh_pick.json'
        p.write_text('{ not json')
        env = dict(os.environ, PIT_CREW_PICK=str(p))
        script = '\n'.join([
            'set -eo pipefail', 'SLUG=finals_mesh',
            'CTRL_ARGS=(-p map_prefix:=/m)', self.block(),
            'printf "%s\\n" "${CTRL_ARGS[@]}"'])
        r = subprocess.run(['bash', '-c', script], env=env,
                           capture_output=True, text=True, check=True)
        assert r.stdout.split('\n')[:-1] == ['-p', 'map_prefix:=/m']

    def test_the_real_sidecar_name_follows_the_slug(self):
        assert '${SLUG}_pick.json' in self.block()


class TestTheDrillCallsIt:
    def test_the_drill_runs_pick_config_at_the_end(self):
        with open(os.path.join(REPO, 'scripts',
                               'finals_drill_mesh.sh')) as fh:
            src = fh.read()
        assert 'tools/pick_config.py' in src
        assert '--min-halfwidth-ladder 0.53,0.50,0.45' in src
        assert '--kappa-gate safe' in src

    def test_the_drill_never_renames_the_shipping_line(self):
        """`--out-name` would produce a `<something-else>_race.csv`, which is
        not a name the entrypoint can derive from the map slug."""
        with open(os.path.join(REPO, 'scripts',
                               'finals_drill_mesh.sh')) as fh:
            for line in fh:
                if line.lstrip().startswith('#'):
                    continue
                assert '--out-name' not in line, line
