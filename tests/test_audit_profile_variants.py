"""`tools/audit_profile_variants.py` -- the opt-in finals profile rungs.

Two properties decide whether these lines are safe to hand to a live run, and
neither is visible by reading the CSV:

* the GEOMETRY must be byte-identical to `finals_mesh_hyb675b_race.csv`. A
  profile rung that also moves the path is two experiments at once, and the
  finals geometry is the one that passed 100 Xvfb laps and 32 display laps;
* the PROTECTED zones -- the three hairpins and the standing start -- must not
  be raised by a single sample. Every live attempt to raise them collided
  (FINALS_TUNING sections 3 and 4b), so a rung that leaks speed into one of
  them is a rung that has already been refuted.

The third test is the one that makes the whole construction meaningful: with
nothing relaxed, the generator must reproduce the base profile. Without that
control a "variant" is just a different line and its ideal-lap delta means
nothing. An earlier attempt that re-solved the velocity profile from scratch
failed exactly here -- it came out ~1.0 s/lap of ideal FASTER than the line
that was driven, because the base is not a two-pass profile: it carries the
lateral-jerk limiter's split budget, two ay-6.0 blend windows and an explicit
3.0 m/s exit cap at s 38.3-39.8.
"""

import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))

import audit_profile_variants as A                             # noqa: E402

LINES = os.path.join(REPO, 'data', 'racelines')


def base():
    return np.loadtxt(os.path.join(LINES, 'finals_mesh_hyb675b_race.csv'),
                      delimiter=',')


def variant(name):
    path = os.path.join(LINES, f'finals_mesh_audit_{name}_race.csv')
    if not os.path.exists(path):
        pytest.skip('%s not generated; run tools/audit_profile_variants.py'
                    % os.path.basename(path))
    return np.loadtxt(path, delimiter=',')


def protected_mask(s):
    m = np.zeros(len(s), bool)
    for lo, hi in A.PROTECTED:
        m |= (s >= lo) & (s <= hi)
    return m


@pytest.mark.parametrize('name', sorted(A.VARIANTS))
class TestTheVariantsAreProfileOnly:
    def test_the_geometry_is_byte_identical_to_the_shipping_line(self, name):
        r, v = base(), variant(name)
        assert v.shape == r.shape, name
        assert np.array_equal(v[:, :5], r[:, :5]), \
            'audit_%s moved s/x/y/psi/kappa; it is no longer a profile rung' % name

    def test_the_protected_zones_are_never_raised(self, name):
        r, v = base(), variant(name)
        prot = protected_mask(r[:, 0])
        over = np.flatnonzero(v[prot, 5] > r[prot, 5] + 1e-9)
        assert not len(over), \
            ('audit_%s raises the speed at %d protected samples (s = %s); the '
             'hairpins and the start are frozen' %
             (name, len(over), r[prot][over, 0][:5]))

    def test_the_lateral_acceleration_ceiling_does_not_move(self, name):
        r, v = base(), variant(name)
        ay0 = float(np.max(r[:, 5] ** 2 * np.abs(r[:, 4])))
        ay1 = float(np.max(v[:, 5] ** 2 * np.abs(v[:, 4])))
        assert ay1 <= ay0 + 1e-6, (name, ay1, ay0)

    def test_speeds_are_positive_and_arc_length_increases(self, name):
        """The same two conditions `pit_crew.preflight` refuses a line on."""
        v = variant(name)
        assert np.all(v[:, 5] > 0), name
        assert np.all(np.diff(v[:, 0]) > 0), name


class TestTheControlReproducesTheBase:
    """Relax nothing and the generator must hand back the base profile."""

    def test_the_null_variant_is_the_shipping_profile(self):
        r = base()
        s, kappa, v0, ax_col = r[:, 0], r[:, 4], r[:, 5], r[:, 6]
        el = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
        out = A.build(v0, ax_col, kappa, el, protected_mask(s),
                      A.BASE_AX, A.BASE_BRAKE, A.BASE_VMAX, 3.2)
        assert np.max(np.abs(out - v0)) < 1e-3, np.max(np.abs(out - v0))

    def test_every_variant_relaxes_something_or_says_so(self):
        """A rung that moves nothing is a rung nobody should spend a live slot
        on -- `ax50b46` is deliberately one of those and is recorded as such."""
        r = base()
        moved = {}
        for name in A.VARIANTS:
            v = variant(name)
            moved[name] = float(np.mean(np.abs(v[:, 5] / r[:, 5] - 1) > 0.005))
        assert moved, moved
        assert any(m > 0.05 for m in moved.values()), moved
