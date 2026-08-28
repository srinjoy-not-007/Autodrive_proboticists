# Review response — Vansh Lohia, 26 August

Source: `docs/Vansh Lohia/26-8-26/` — a rules/format summary and a detailed
spec sheet. Not a code review, so it was read as a **compliance audit**: does
anything we ship violate it, and does anything in it point at lap time.

## Compliance: clean

The rules restrict IPS, odom, lap/collision counters, lap times, TF and
`reset_command` to debug/training, and prohibit "simulation ground-truth data"
at race runtime — instant disqualification.

`submission/autodrive_devkit.sh` starts exactly two nodes (`localization` and
`mpc`, or `follow_the_gap` in reactive mode). Their combined subscriptions are:

    lidar · imu · left_encoder · right_encoder · throttle · /pit_crew/pose

Every one is an allowed **Input**. **No restricted topic is subscribed anywhere
in the submission path.** `race_logger.py` does read `lap_count` and
`collision_count`, and `calibrator.py` publishes `reset_command` — both are
measurement/dev tools and neither is started by the entrypoint. `odometry.py`
publishes to `/pit_crew/odom`, our own namespace, not the restricted
`/autodrive/roboracer_1/odom`, and is not started either.

Also checked and clean: our code is a separate package (`pit_crew`), the devkit
is untouched, the transport is the stock WebSocket, and the image derives from
the official devkit.

## What the spec sheet caught

### A submission bug that would have cost us qualification

`localization`'s `init_x`/`init_y` default to **(0.0, 0.0)**, and
`autodrive_devkit.sh` never passes them — only `run_stanley.sh` does. On Porto
the car spawns at (0.80, 3.16), which is **3.26 m from the seed, more than five
times `init_spread` (0.6 m)**. Every particle would have started somewhere the
car had never been. Every test we have ever run masked this, because the test
harness supplies the pose and the submission does not.

Fixed: with no explicit parameter the filter now seeds at the first point of the
centerline it already loads — track-agnostic, and 0.158 m from the Porto spawn.
Verified by running the exact submission command line with no init pose:
**36 laps, 0 collisions, 9.22 s median.**

### A correction to yesterday's mass change

The spec gives **Total Mass 3.906 kg = 3.470 sprung + 0.436 unsprung**. A
previous review called the planner's `m_veh=3.906` "unsourced" against the
3.47 kg that `compare_sim_builds.py` reads off the Unity rigidbody, and it was
changed to 3.47 on that basis. That was wrong: a Unity rigidbody carries only
the sprung mass, because the wheels are `WheelCollider`s with their own. 3.906
is the figure a velocity profile wants. Restored, with `SPRUNG_MASS` kept
alongside it. Zero effect on any line either way (`drag_coeff=0`, so mass
cancels out of a friction-limited profile) — verified byte-identical.

### Independent confirmation of the LiDAR offset

Frame table: `lidar` at x = 0.2733, `ips`/`imu` at x = 0.08, both from the rear
axle. Difference **0.1933 m**, exactly the `LIDAR_FROM_POSE` we measured out of
the Unity scene. Two independent sources now agree.

### Minor, not acted on

Overhangs are given as front 0.090 / rear 0.080, implying a body extending
0.414 m ahead of the rear axle and 0.080 m behind. `optimize_raceline.py` uses
0.412 and 0.088 — 2 mm short at the front, 8 mm conservative at the rear.
Changing it would move every shipping line for 2 mm and force a full
re-validation, which is not worth it. Noted here so it is not re-discovered.
(The spec is internally inconsistent anyway: 0.090 + 0.324 + 0.080 = 0.494
against a stated length of 0.500.)

## The one strategic point worth acting on

**Qualification is explicitly speed-independent** — "complete 10 consecutive
laps; speed doesn't matter" — while the final is ranked purely on time. They
have opposite risk profiles, and we currently ship one configuration for both.

For **Phase 1 (Sep 12)** the correct submission is the most robust thing we
have, because a faster lap earns nothing and a collision can only hurt. For
**Phase 2 (Sep 19)** it is the fastest clean configuration. This costs nothing
to act on: `PIT_CREW_MODE`, `PIT_CREW_LINE` and `PIT_CREW_SPEED_SCALE` already
select it at runtime.

Nothing else in the rules points at lap time. The format (1 untimed warm-up +
10 scored + 1 optional cool-down) matches how we already project race time as
`10 x median + 10 x collisions`.
