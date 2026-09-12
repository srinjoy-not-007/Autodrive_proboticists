"""What `pit_crew.preflight` checks about the baked assets.

The submission entrypoint runs preflight before any actuator node starts, and
it is the only gate between `PIT_CREW_MAP` / `PIT_CREW_LINE` /
`PIT_CREW_CENTERLINE` / `PIT_CREW_LIDAR_OFFSET` and a race.  On an unseen track
those are set by hand, so the likeliest finals mistake is not a corrupt file
but a self-consistent set of files describing two different circuits.
"""

import os
import sys

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew import preflight                                  # noqa: E402


RES = 0.05
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def make_map(tmp_path, name='m', w=240, h=240, origin=(-6.0, -6.0),
             lidar_offset=None):
    """A square of free space with a wall border, map_server format."""
    img = np.full((h, w), 254, dtype=np.uint8)
    img[:4, :] = img[-4:, :] = img[:, :4] = img[:, -4:] = 0
    Image.fromarray(img).save(tmp_path / f'{name}.pgm')
    meta = (f"image: {name}.pgm\nresolution: {RES}\n"
            f"origin: [{origin[0]}, {origin[1]}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    if lidar_offset is not None:
        meta += f'lidar_offset: {lidar_offset}\nsource: mesh\n'
    (tmp_path / f'{name}.yaml').write_text(meta)
    return str(tmp_path / name)


def _rows(xy, v=4.0):
    d = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    return np.column_stack([s, xy[:, 0], xy[:, 1], np.zeros(len(xy)),
                            np.zeros(len(xy)), np.full(len(xy), v),
                            np.zeros(len(xy))])


def _circle(R, n, cx, cy, closed):
    t = (np.linspace(0.0, 2 * np.pi, n, endpoint=False) if closed
         else np.linspace(0.0, np.pi, n))            # open: half a lap
    return np.column_stack([cx + R * np.cos(t), cy + R * np.sin(t)])


def make_line(tmp_path, name='line.csv', R=2.0, n=120, v=4.0,
              cx=0.0, cy=0.0, closed=True):
    """A closed circular raceline: s, x, y, psi, kappa, vx, ax."""
    p = tmp_path / name
    np.savetxt(p, _rows(_circle(R, n, cx, cy, closed), v), delimiter=',')
    return str(p)


def make_center(tmp_path, name='center.csv', R=2.2, n=120, cx=0.0, cy=0.0,
                closed=True):
    """A centerline: x, y in the first two columns (what localization reads)."""
    p = tmp_path / name
    np.savetxt(p, _circle(R, n, cx, cy, closed), delimiter=',')
    return str(p)


class TestPreflightRejects:
    def test_accepts_a_consistent_set(self, tmp_path, capsys):
        m = make_map(tmp_path, lidar_offset=0.1933)
        preflight.validate(m, make_line(tmp_path),
                           make_center(tmp_path),
                           '1.0', '0.1933')
        assert 'Preflight OK' in capsys.readouterr().out

    @pytest.mark.parametrize('scale', ['0', '-1', '2.0', 'nan'])
    def test_bad_speed_scale(self, tmp_path, scale):
        m = make_map(tmp_path)
        with pytest.raises(ValueError):
            preflight.validate(m, make_line(tmp_path),
                               make_center(tmp_path), scale)

    def test_missing_map(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            preflight.validate(str(tmp_path / 'nope'), make_line(tmp_path),
                               make_center(tmp_path), '1.0')

    def test_zero_speed_row_is_rejected(self, tmp_path):
        m = make_map(tmp_path)
        line = make_line(tmp_path)
        rows = np.loadtxt(line, delimiter=',')
        rows[5, 5] = 0.0
        np.savetxt(line, rows, delimiter=',')
        with pytest.raises(ValueError):
            preflight.validate(m, line, make_center(tmp_path), '1.0')

    def test_non_monotonic_arc_length_is_rejected(self, tmp_path):
        m = make_map(tmp_path)
        line = make_line(tmp_path)
        rows = np.loadtxt(line, delimiter=',')
        rows[10, 0] = rows[9, 0]
        np.savetxt(line, rows, delimiter=',')
        with pytest.raises(ValueError):
            preflight.validate(m, line, make_center(tmp_path), '1.0')


class TestTheTrackMustBeOneTrack:
    def test_a_line_that_does_not_lie_on_the_map_is_rejected(self, tmp_path):
        """Three env vars used to default to Porto independently; forgetting
        one started the race with a raceline from another circuit."""
        m = make_map(tmp_path, origin=(-6.0, -6.0))       # covers -6..6 m
        far = make_line(tmp_path, 'far.csv', cx=500.0, cy=500.0)
        with pytest.raises(ValueError, match='leaves the map'):
            preflight.validate(m, far, make_center(tmp_path), '1.0')

    def test_a_centerline_from_another_track_is_rejected(self, tmp_path):
        """The centerline is not a diagnostic: it seeds the particle filter
        and is the only prior for collision recovery."""
        m = make_map(tmp_path)
        far = make_center(tmp_path, cx=500.0, cy=500.0)
        with pytest.raises(ValueError, match='leaves the map'):
            preflight.validate(m, make_line(tmp_path), far, '1.0')

    def test_a_line_through_the_walls_is_rejected(self, tmp_path):
        m = make_map(tmp_path)
        through = make_line(tmp_path, 'wall.csv', R=5.9)   # into the border
        with pytest.raises(ValueError, match='walls'):
            preflight.validate(m, through,
                               make_center(tmp_path), '1.0')

    def test_an_open_line_is_rejected(self, tmp_path):
        """RefPath treats the line as cyclic (np.roll heading and curvature,
        index modulo n), so an open line makes its seam a discontinuity."""
        m = make_map(tmp_path)
        with pytest.raises(ValueError, match='closed loop'):
            preflight.validate(m, make_line(tmp_path, 'open.csv', closed=False),
                               make_center(tmp_path), '1.0')


class TestLidarOffsetMustMatchTheMap:
    def test_matching_offset_passes(self, tmp_path):
        m = make_map(tmp_path, lidar_offset=0.1933)
        preflight.validate(m, make_line(tmp_path),
                           make_center(tmp_path), '1.0', '0.1933')

    def test_mismatched_offset_is_rejected(self, tmp_path):
        m = make_map(tmp_path, lidar_offset=0.0)
        with pytest.raises(ValueError, match='mismatch'):
            preflight.validate(m, make_line(tmp_path),
                               make_center(tmp_path), '1.0', '0.1933')

    def test_an_unstamped_map_cannot_be_raced_at_an_offset(self, tmp_path):
        """A recon map built at offset 0 raced at 0.1933 carries a systematic
        0.19 m bias against a line with ~0.29 m of clearance."""
        m = make_map(tmp_path, lidar_offset=None)
        with pytest.raises(ValueError, match='no lidar_offset'):
            preflight.validate(m, make_line(tmp_path),
                               make_center(tmp_path), '1.0', '0.1933')

    def test_an_unstamped_map_is_still_fine_when_no_offset_is_demanded(
            self, tmp_path):
        m = make_map(tmp_path, lidar_offset=None)
        preflight.validate(m, make_line(tmp_path),
                           make_center(tmp_path), '1.0')


class TestShippingAssetsPassPreflight:
    """Every baked combination must survive every new check.

    The entrypoint's own DEFAULTS are covered by
    `tests/test_submission_assets.py`, which derives them by running the
    entrypoint's shell instead of restating them here (this list went stale
    once already: it still called `mc0.60_ay6.0` the shipping mpc3 line three
    lines after the default moved to `cr_dk0.8`).  What this list adds is the
    combinations someone may select BY HAND with `PIT_CREW_LINE`.
    """

    @pytest.mark.parametrize('mp,line,center,offset', [
        ('porto_mesh', 'porto_mesh_cr_dk0.8', 'porto_mesh_centerline',
         '0.1933'),                                     # mpc3, shipping
        ('porto_mesh', 'porto_mesh_mc0.63_ay6.75', 'porto_mesh_centerline',
         '0.1933'),
        ('porto_mesh', 'porto_mesh_mc0.60_ay6.0', 'porto_mesh_centerline',
         '0.1933'),
        ('icra2026_mesh', 'icra2026_mesh_safe', 'icra2026_mesh_centerline',
         '0.1933'),
        ('porto_mesh', 'track_v2_safe', 'porto_mesh_centerline', '0.1933'),
        ('track_v2', 'track_v2_safe', 'track_v2_centerline', None),
        ('icra2026_mesh', 'icra2026_mesh_race', 'icra2026_mesh_centerline',
         '0.1933'),
    ])
    def test_baked_combination(self, mp, line, center, offset):
        args = [os.path.join(REPO, 'data', 'maps', mp),
                os.path.join(REPO, 'data', 'racelines', line + '.csv'),
                os.path.join(REPO, 'data', 'racelines', center + '.csv'),
                '1.00']
        if offset is not None:
            args.append(offset)
        preflight.validate(*args)
