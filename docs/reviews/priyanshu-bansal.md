# Review response — Priyanshu Bansal, 26 August

Source: `docs/Priyanshu Bansal/25-8-26/Recommenede_changes.md`. Four structural
faults with copy-paste patches. Verdicts re-derived against current `main`.

| Fault | Verdict | Action |
|---|---|---|
| 1a `pf_core` ignores `lidar_offset` | **TRUE** | fixed |
| 1b offline_pf mixes two time bases | **TRUE in code, but measured harmless** | left alone, documented |
| 1c brittle `name[-13]` wheel parsing | **TRUE but functionally inert** | fixed for clarity |
| 2 `lidar_offset` lost on map promotion | **TRUE, and a live finals footgun** | fixed |
| 3 `cp` breaks on Windows | TRUE | fixed |
| 4 teleport jumps corrupt the centerline | real problem, patch untested here | deferred, see below |

## Fault 2 — the one that mattered

`new_track_drill.sh` builds the map both ways and promotes the winner over the
canonical `<track>.yaml/.pgm`. Nothing recorded which variant won, and
`localization.py` defaulted `lidar_offset` to 0.0 — so a map built with the
0.1933 m correction could be localized against with 0.0. That combination is
worse than either done consistently.

This is not hypothetical: earlier the same day I raced the corrected map and had
to pass `LIDAR_OFFSET=0.1933` by hand, having added the plumbing minutes before.
On finals night nobody would have remembered.

`build_map_from_bag.py` now writes `lidar_offset` into the map YAML and
`localization.py` reads it, overriding the parameter. Verified — the two maps
built from the same bag now carry `0.1933` and `0.0` respectively. `pf_core.py`
takes the same value, so the offline filter and the live one finally agree.

## Fault 1 — only one of the three causes is real

The document attributes `offline_pf.py`'s divergence (192 relocalizations
against the live filter's zero) to three causes. Checked individually:

- **1a is real.** `pf_core` cast scan endpoints from the pose point while the
  live node cast them from the LiDAR. Two different filters. Fixed.
- **1b is not happening.** The claim is that host recording timestamps and
  simulator header stamps drift apart when the sim lags. Measured across the
  1843 LiDAR messages of the 175 s Porto mapping bag: **the two clocks agree to
  within 2 ms end to end.** The reasoning is sound and the bug is real in
  principle; it simply is not what is wrong here, so the patch is not applied.
- **1c is inert.** `name[-13]` does yield `'/'` for left and `'r'` for right
  rather than `'L'`/`'R'` — but `SpeedEstimator` uses the argument purely as a
  dict key, so two distinct keys behave identically. It worked by accident.
  Rewritten to say what it means, with no behaviour change.

So the divergence is **still unexplained** after 1a. Do not treat `offline_pf`
as trustworthy yet; `docs/STATUS.md`'s "drive instead" still stands.

## Fault 4 — deferred, deliberately

Recon laps contain teleports (3 in the ICRA recon, 58 in `maplap_short`) because
a recon run collides, and they do corrupt the traced centerline. The proposed fix
— detect the jump, find the nearest past point, excise the failed retry segment —
is a different approach from the two already tried here, both of which were
reverted for breaking the shipping ICRA line (`docs/STATUS.md`).

Not attempted in this session because it is the single highest-risk change in
the offline pipeline and the session had already shipped a large controller
change. `scripts/check_pipeline.sh` is the gate it must pass: it regenerates
every track and fails if any stops producing a drivable line. Worth doing next,
with that guard, and not near a deadline.
