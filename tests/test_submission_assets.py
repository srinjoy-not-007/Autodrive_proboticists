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


def dockerfile_copies():
    """Published path -> source path, for every COPY in the Dockerfile.

    Handles backslash continuations, the `COPY a b c DEST/` form and the
    two-argument rename form, and expands the map globs against the repo.
    """
    import glob as globmod
    text = re.sub(r'\\\n', ' ', open(DOCKERFILE).read())
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line.upper().startswith('COPY '):
            continue
        parts = line.split()[1:]
        srcs, dest = parts[:-1], parts[-1]
        expanded = []
        for s in srcs:
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
