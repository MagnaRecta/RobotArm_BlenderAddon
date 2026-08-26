# Kinematics re-vendoring — 2026-08-03

## Why

The ROS2 side's `build_file.py` loader (new this week, part of Phase 5 —
loading and executing a Blender-exported build file) hard-refuses to run
any build file whose `kinematics_version` field doesn't exactly match the
installed `so_arm_100_kinematics.__version__` on the ROS2 side — by design,
per `BRIDGE_PROTOCOL.md`'s own words: "ROS2 must compare it against its own
and refuse to execute on mismatch." That's a real, intentional check: the
two sides sharing one kinematics module is the entire basis for trusting
Blender's own validation of a build file.

The ROS2 side is at `so_arm_100_kinematics` **1.2.0**. This workspace's
`so_arm_100_kinematics/` staging copy was already at 1.2.0 too (kept in
sync earlier). But the copy the addon's *own Blender interpreter actually
imports* — `so100_builder/kinematics/` — was still at **1.1.0**, and didn't
have `jaw_clearance.py` at all. So any build file exported from Blender
would have been stamped `kinematics_version: "1.1.0"`, and ROS2 would
correctly (if unhelpfully) refuse to run it.

This was already a known, flagged gap — `docs/STATUS.md`'s Open items had
it listed as "not yet re-vendored," with a pointer to the exact steps in
`so_arm_100_kinematics/README.md`'s "Vendoring into the Blender addon"
section. I did that step.

## What I changed

Copied, verbatim, from `so_arm_100_kinematics/so_arm_100_kinematics/` into
`so100_builder/kinematics/`:

- `__init__.py` (bumps `__version__` 1.1.0 → 1.2.0, exports the new
  `jaw_clearance` names)
- `constants.py` (adds `JAW_RADIUS_M`, `STICK_COLLISION_INFLATION_M`,
  `STICK_COLLISION_RADIUS_M`)
- `jaw_clearance.py` (**new file** — this module didn't exist in the
  vendored copy at all before)
- `chain.py`, `envelope.py`, `grasp.py` — copied too, but these were
  already byte-identical to the ROS2 side (confirmed with `diff`), so
  nothing about placement/reachability computation actually changed.
- `VERSION` (1.1.0 → 1.2.0)

Also added `so100_builder/tests/test_kinematics_jaw_clearance_vendored.py`
— the addon-side counterpart to `so_arm_100_kinematics/test/test_jaw_clearance.py`,
re-pointed to import from `so100_builder.kinematics.*` instead of the ROS
package, same as the existing `test_kinematics_vendored.py` and
`test_kinematics_grasp_vendored.py` already do for `chain.py`/`grasp.py`.
This was the third vendored test file the README's own vendoring rules
call for; it didn't exist yet because `jaw_clearance.py` had never been
vendored before now.

## Verified

All 42 tests across the three vendored test files pass, run under
`env -i PATH=/usr/bin:/bin python3 -m unittest ...` — a plain, numpy-free
environment with nothing sourced, deliberately simulating the isolation
Blender's own bundled interpreter has (no external site-packages, no
guaranteed numpy). If these pass here, they'll pass inside Blender too.

## What this means for you

The **next** "Export Build File" from the addon's Build panel will stamp
`kinematics_version: "1.2.0"` automatically — `so100_builder/ops/build.py`
already reads it dynamically from the vendored `kinematics.__version__`,
so no other code needed to change for this. You don't need to hand-edit
that field anymore; a freshly-exported file will just work with the
current ROS2 loader.

## What did *not* change

- No computed result changes. `chain.py`/`envelope.py`/`grasp.py` were
  already identical between the two copies — this was purely "the vendored
  copy was missing a module and was one version behind," not a bug fix to
  any placement math.
- `jaw_clearance.check_jaw_clearance()` is now *available* on this side
  (vendored, tested) but still has **no caller** — `core/validate.py`
  doesn't invoke it during validation yet, same as the ROS2 side's own
  task server doesn't either. Wiring it in is a separate, still-open
  feature, not something this update did.
- I did not touch `bl_info`, any addon-level version string, or anything
  outside `so100_builder/kinematics/` and the one new test file.
