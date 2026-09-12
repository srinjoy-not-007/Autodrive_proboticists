"""Everything the entrypoint can reach by DEFAULT must be in the image.

Three files have to agree for a race to happen, and nothing until now checked
that they did:

* ``submission/autodrive_devkit.sh`` derives ``MAP``, ``LINE`` and ``CENTER``
  from ``PIT_CREW_MAP`` and ``PIT_CREW_CONTROLLER``;
* ``submission/Dockerfile`` copies maps by glob and racelines **one by one**;
* ``pit_crew.preflight`` refuses the race if the three describe two circuits.

The finals-day failure this closes is not exotic: point the entrypoint at the
new map, forget one line in the Dockerfile's explicit list, and the container
exits 2 the first time anyone runs it -- on the organisers' box, with no shell
history to read.  These tests do the derivation by running the entrypoint's own
shell, so they cannot drift from it.
"""

import os
import re
import subprocess

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENTRY = os.path.join(REPO, 'submission', 'autodrive_devkit.sh')
DOCKERFILE = os.path.join(REPO, 'submission', 'Dockerfile')

import sys                                                      # noqa: E402
sys.path.insert(0, os.path.join(REPO, 'src'))
from pit_crew import preflight                                  # noqa: E402


# ------------------------------------------------------------------ parsing

def entry_src():
    with open(ENTRY) as fh:
        return fh.read()


def dockerfile_args():
    """`ARG NAME=default` -> default, for every build arg with a default.

    The finals slug does not exist when the image is described, only when the
    drill has run, so the COPY list cannot name it in advance.  The mechanism
    this supports is a build arg:

        ARG FINALS_SLUG=icra2026_mesh
        COPY data/racelines/${FINALS_SLUG}_centerline.csv \\
             data/racelines/${FINALS_SLUG}_race.csv \\
             data/racelines/${FINALS_SLUG}_safe.csv \\
             data/racelines/${FINALS_SLUG}_pick.json \\
             /home/data/racelines/

    -- defaulted to a slug that already ships (so the tree always builds and
    the default is a no-op duplicate of an existing COPY), and pointed at the
    finals slug on the day with one `--build-arg FINALS_SLUG=finals_mesh`
    instead of an edit to the COPY list.  A wrong slug then fails the BUILD at
    the COPY, loudly, rather than failing the race.  A wildcard
    (`data/racelines/<slug>_*.csv`) is the other option and is rejected here:
    `test_no_glob_in_the_raceline_copies` exists because globbing racelines
    once shipped an undriven line one `PIT_CREW_LINE` typo from being raced.
    """
    out = {}
    for line in open(DOCKERFILE):
        m = re.match(r'\s*ARG\s+([A-Za-z_][A-Za-z0-9_]*)=(\S+)\s*$', line)
        if m:
            out[m.group(1)] = m.group(2)
    return out


def expand_args(path, args):
    """Substitute `${NAME}` / `$NAME` in a COPY source with its ARG default."""
    def sub(m):
        name = m.group(1) or m.group(2)
        assert name in args, \
            ('Dockerfile COPY uses $%s, which has no ARG default; this test '
             'cannot know what the build would substitute' % name)
        return args[name]
    return re.sub(r'\$\{([A-Za-z_][A-Za-z0-9_]*)\}|\$([A-Za-z_][A-Za-z0-9_]*)',
                  sub, path)


def dockerfile_copies():
    """Published path -> source path, for every COPY in the Dockerfile.

    Handles backslash continuations, the `COPY a b c DEST/` form and the
    two-argument rename form, expands the map globs against the repo, and
    substitutes ARG defaults so a build-arg-parameterised COPY is checked too.
    """
    import glob as globmod
    text = re.sub(r'\\\n', ' ', open(DOCKERFILE).read())
    args = dockerfile_args()
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line.upper().startswith('COPY '):
            continue
        parts = line.split()[1:]
        srcs, dest = parts[:-1], parts[-1]
        expanded = []
        for s in srcs:
            s = expand_args(s, args)
            hits = sorted(globmod.glob(os.path.join(REPO, s)))
            assert hits, 'Dockerfile COPYs a path that does not exist: %s' % s
            expanded += hits
        for src in expanded:
            if dest.endswith('/'):
                out[dest + os.path.basename(src)] = src
            else:
                assert len(expanded) == 1, line
                out[dest] = src
    return out


SHIPPED = dockerfile_copies()


def derive(slug, controller):
    """Run the entrypoint's own MAP/LINE/CENTER derivation for one config."""
    src = entry_src()
    blocks = [re.search(r'^MAP="\$\{PIT_CREW_MAP.*?^SLUG=.*?$', src,
                        re.M | re.S).group(0),
              re.search(r'^if \[\[ -n "\$\{PIT_CREW_LINE.*?^fi$', src,
                        re.M | re.S).group(0),
              re.search(r'^CENTER="\$\{PIT_CREW_CENTERLINE.*?$', src,
                        re.M).group(0)]
    env = {k: v for k, v in os.environ.items()}
    for k in ('PIT_CREW_MAP', 'PIT_CREW_LINE', 'PIT_CREW_CENTERLINE'):
        env.pop(k, None)
    if slug is not None:
        env['PIT_CREW_MAP'] = '/home/data/maps/' + slug
    env['PIT_CREW_CONTROLLER'] = controller
    script = '\n'.join(['CONTROLLER="${PIT_CREW_CONTROLLER}"'] + blocks
                       + ['printf "%s\\n%s\\n%s\\n" "$MAP" "$LINE" "$CENTER"'])
    out = subprocess.run(['bash', '-c', script], env=env, check=True,
                         capture_output=True, text=True)
    return out.stdout.split('\n')[:3]


def offset_of(map_prefix):
    """The offset this map MUST be raced at: its own stamped value.

    A mesh map stamps 0.1933; a recon map (`track_v2`) stamps nothing and has
    to be raced at 0, which is exactly the mismatch preflight exists to catch.
    """
    meta = yaml.safe_load(open(map_prefix + '.yaml'))
    return None if 'lidar_offset' not in meta else '%.4f' % float(
        meta['lidar_offset'])


def check(mp, line, center):
    prefix = os.path.splitext(SHIPPED[mp + '.pgm'])[0]
    args = [prefix, SHIPPED[line], SHIPPED[center], '1.00']
    off = offset_of(prefix)
    if off is not None:
        args.append(off)
    preflight.validate(*args)


def reachable_slugs():
    """Slugs a default-derived race could actually name: those whose
    `<slug>_centerline.csv` is in the image at all."""
    return sorted({os.path.basename(p)[:-len('_centerline.csv')]
                   for p in SHIPPED
                   if p.endswith('_centerline.csv')})


# -------------------------------------------------------------------- tests

class TestTheShippingDefaultIsInTheImage:
    @pytest.mark.parametrize('controller', ['mpc3', 'mpc'])
    def test_every_default_asset_is_copied(self, controller):
        mp, line, center = derive(None, controller)
        for path in (mp + '.pgm', mp + '.yaml', line, center):
            assert path in SHIPPED, \
                '%s is an entrypoint default but no Dockerfile COPY produces it' \
                % path

    @pytest.mark.parametrize('controller', ['mpc3', 'mpc'])
    def test_the_default_combination_passes_preflight(self, controller):
        mp, line, center = derive(None, controller)
        assert offset_of(os.path.splitext(SHIPPED[mp + '.pgm'])[0]) == '0.1933', \
            'the shipping default must be a mesh map (recon maps race at 0)'
        check(mp, line, center)


class TestEveryReachableTrackIsComplete:
    """A map with a centerline in the image is a track someone can race with
    one environment variable. Both controller defaults must then resolve."""

    @pytest.mark.parametrize('controller', ['mpc3', 'mpc'])
    def test_both_controller_defaults_resolve_and_agree(self, controller):
        slugs = reachable_slugs()
        assert slugs, 'no centerline ships at all'
        for slug in slugs:
            mp, line, center = derive(slug, controller)
            assert center.endswith('/%s_centerline.csv' % slug), center
            assert line in SHIPPED, \
                ('PIT_CREW_MAP=%s with PIT_CREW_CONTROLLER=%s defaults to %s, '
                 'which is not in the image' % (slug, controller, line))
            check(mp, line, center)

    def test_no_default_crosses_tracks(self):
        """The classic finals mistake: three consistent-looking files that
        describe two circuits. The slug must appear in all three names."""
        for slug in reachable_slugs():
            for controller in ('mpc3', 'mpc'):
                mp, line, center = derive(slug, controller)
                for path in (mp, line, center):
                    assert slug in path, (slug, controller, path)


class TestTheDockerfileStillListsRacelinesOneByOne:
    """Globbing racelines once shipped a line that reversed direction twice and
    asked for a 0.327 m radius against a 0.561 m steering lock -- gated out of
    `race.csv` but one `PIT_CREW_LINE` typo from being driven."""

    def test_no_glob_in_the_raceline_copies(self):
        text = re.sub(r'\\\n', ' ', open(DOCKERFILE).read())
        for line in text.splitlines():
            if 'racelines' in line and line.strip().upper().startswith('COPY '):
                assert '*' not in line and '?' not in line, line

    def test_every_shipped_raceline_is_a_real_line_or_centerline(self):
        for path, src in SHIPPED.items():
            if '/racelines/' not in path:
                continue
            assert os.path.exists(src), src
            assert 'REJECTED' not in path, path


# ------------------------------------------------------- the finals slug

FINALS_SLUG = os.environ.get('FINALS_SLUG', '')


@pytest.mark.skipif(not FINALS_SLUG,
                    reason='set FINALS_SLUG=<slug>_mesh after the drill')
class TestTheFinalsSlugIsRaceable:
    """The Sep 16 rehearsal in one command.

    ``scripts/finals_drill_mesh.sh <slug>`` writes files the Dockerfile cannot
    have named in advance, so nothing in the default suite can check them.
    Run this against the drill's own slug::

        FINALS_SLUG=finals_mesh .venv/bin/python -m pytest -q \\
            tests/test_submission_assets.py -k FinalsSlug

    The first two tests pass as soon as the drill has run (they check the
    entrypoint derivation against the files ON DISK, which is the check worth
    having BEFORE the Dockerfile is touched); the third is the one that only
    goes green once the finals lines are actually COPYed -- by the
    ``FINALS_SLUG`` build arg described in ``dockerfile_args`` or by an
    explicit edit to the raceline list.
    """

    def repo_path(self, published):
        """/home/data/... -> the repository file the drill wrote."""
        return os.path.join(REPO, published.replace('/home/', '', 1))

    @pytest.mark.parametrize('controller', ['mpc3', 'mpc'])
    def test_the_drill_produced_every_derived_asset(self, controller):
        mp, line, center = derive(FINALS_SLUG, controller)
        for path in (mp + '.pgm', mp + '.yaml', line, center):
            assert FINALS_SLUG in path, (FINALS_SLUG, path)
            assert os.path.exists(self.repo_path(path)), (
                '%s is what the entrypoint would load with PIT_CREW_MAP='
                '/home/data/maps/%s and PIT_CREW_CONTROLLER=%s, but the drill '
                'wrote no such file. Re-run scripts/finals_drill_mesh.sh, or '
                '`cp <slug>_safe.csv <slug>_race.csv` if the validated line is '
                'the safe one.' % (path, FINALS_SLUG, controller))

    @pytest.mark.parametrize('controller', ['mpc3', 'mpc'])
    def test_the_three_files_describe_one_circuit(self, controller):
        mp, line, center = derive(FINALS_SLUG, controller)
        prefix = os.path.splitext(self.repo_path(mp + '.pgm'))[0]
        args = [prefix, self.repo_path(line), self.repo_path(center), '1.00']
        off = offset_of(prefix)
        assert off == '0.1933', \
            'a mesh map stamps 0.1933; a recon map must be raced at 0'
        args.append(off)
        preflight.validate(*args)

    @pytest.mark.parametrize('controller', ['mpc3', 'mpc'])
    def test_every_derived_asset_would_be_copied(self, controller):
        mp, line, center = derive(FINALS_SLUG, controller)
        missing = [p for p in (mp + '.pgm', mp + '.yaml', line, center)
                   if p not in SHIPPED]
        assert not missing, (
            '%s would be loaded by the entrypoint but no Dockerfile COPY '
            'produces it. Maps ship by glob; racelines are listed one by one. '
            'Add them with the FINALS_SLUG build arg (see dockerfile_args) or '
            'at the ADD THE FINALS LINES HERE marker.' % missing)
