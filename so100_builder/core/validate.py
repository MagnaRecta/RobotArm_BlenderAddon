"""Per-stick verdicts against the vendored kinematics. BLENDER_ADDON_PLAN.md
Sec 7, Phase B.

**Policy (decided with the user 2026-07-28): permissive.** Trust the
closed-form IK's own success/failure as the reachability verdict, including
for sticks tilted out of the arm's own vertical plane -- ROS2's MoveIt does
its own final reachability/collision check on load regardless, so an
over-eager Blender-side rejection only costs the user a design iteration for
nothing. Where this module cannot be fully confident (see below), it warns
rather than blocks, so the user can inspect the stick with the "Check By
Eye" button (``ops/design.py``) and re-plan if needed. Only a placement the
closed-form solver itself proves impossible is a hard error.

⚠ Sec 9.4 of the ROS2 plan claims a stick "tilted out of the arm's plane
(leaning sideways)" is categorically unreachable with 5 DOF. Empirical
testing against the real, hardware-validated ``chain.py`` directly
contradicts this: such placements ARE reachable, verified to <0.05 deg by
round-tripping the solved joints back through ``fk()``. That table entry
should be corrected in the plan; this module does not encode it.

## The grasp-orientation transform lives in the shared kinematics package

Converting a stick's desired 3D orientation into the ``(tool_elevation_rad,
stick_roll_rad)`` pair ``chain.ik()`` takes is genuinely non-trivial. It was
first built here, then promoted into ``so_arm_100_kinematics.grasp`` (version
1.1.0) so the ROS2 side's ``PlaceStick`` action uses the exact same,
self-verifying search this module does -- the two sides sharing one
derivation is the entire point of vendoring rather than reimplementing (see
README.md). This module now only calls
``kinematics.grasp.solve_stick_orientation`` / ``grasp_target``; it does not
derive orientations itself. Two non-obvious findings from building it, both
confirmed empirically against ``fk()``/``ik()`` and documented in full in
``kinematics/grasp.py``:

1. **The reference stick direction points from the grip toward the stick's
   BASE, not its tip.** Get this sign wrong and every roll solve is off by
   ~180 deg, silently pushing ``Wrist_Roll`` out of its limit for placements
   that are actually fine.
2. **The elevation equation has two roots 180 deg apart**, and only one of
   them puts the roll=0 reference direction anywhere near the target. There
   is no way to know which root is right without trying both -- so the
   shared solver tries both elevation roots and both elbow branches and
   **verifies every candidate by feeding the solved joints back through
   ``fk()``**, rather than trusting the closed-form roll formula on its own.

Because the search is exhaustive and self-verifying, "no candidate verifies"
means the placement is genuinely unreachable with this arm -- not merely
unvalidated. There is no leftover "we don't know" case for reachability
itself.

## What this module does NOT check (see Phase C)

* **Jaw clearance / collision against already-placed sticks** (Sec 6 C3,
  Sec 8.2). This needs a real build order to know what "already placed"
  means -- Phase A's angle-based ``tight_clearance`` warning is the current
  proxy for the joint-geometry risk this would catch.
* **Self-collision, the mount platform, or MoveIt's own path planning.** Per
  Sec 7: this is an upper bound, never a final verdict. MoveIt has the last
  word regardless of what this module says.

## Why steep/out-of-plane tilts still get a warning even when verified

A verified "reachable" here is still only a single-point IK check -- exactly
the class of placement Sec 7 calls an upper bound. Out-of-plane tilts are
comparatively untested territory (Sec 9.4 did not even expect them to be
reachable), so a placement in that territory is more likely than a vertical
one to run into something this module does not model (self-collision, an
awkward approach path) once MoveIt actually plans it. The warning exists so
the user notices and can eyeball it, not because the reachability verdict
itself is in doubt.

## Multi-robot (docs/STATUS.md "Multi-robot support kicked off 2026-08-21")

Everything above is ``so_arm_100``-specific -- the exact-root/cardinal-anchor
search, the asymmetric-``Wrist_Roll`` flip suggestion, the out-of-plane
warning -- and stays that way verbatim (this generalization must not change
so_arm_100's own verdicts; see ``core/robots.py``'s module docstring). It now
lives in :func:`_validate_stick_so_arm_100`, reached only when
``robot_id == core_robots.SO_ARM_100_ID`` (the default).

``kr10_r900_2`` (vendored 2026-08-22) gets its own path too --
:func:`_validate_stick_kr10_r900_2` -- rather than the generic fallback,
because that robot's stock is round with a genuinely free roll DOF: the
single-roll ``solve_stick_placement`` under-reports reachability for it, so
this path calls ``solve_stick_placement_any_roll`` instead (sweeps roll x
branch; see ``core/robots.py``'s module docstring for why this could not be
folded into one shared "contract" function name across robots).

Any *other* robot id -- none registered yet -- falls through to
:func:`_validate_stick_generic`, which uses only ``solve_stick_placement``
positionally (``base``, ``tip``) and turns any failure into a clean
``Verdict``. It is a safety net for a future third robot before it has
earned its own richer path, not a design to converge on.
"""

import math

from . import robots as core_robots
from .transform import v_dot, v_normalized

_SO_ARM_100 = core_robots.get_robot(core_robots.SO_ARM_100_ID)
_so_arm_100_kinematics = _SO_ARM_100.kinematics

# Finding 4 (empirical, 2026-07-29): near a genuinely degenerate direction
# (both a_r and a_z small -- i.e. close to horizontal-tangential), the
# elevation equation is numerically ill-conditioned: a small amount of
# geometric noise in the target direction (routinely produced by the
# mesh-expansion relaxation solve, which is iterative and rarely lands on
# exact symmetry -- and grows with how asymmetric the drawn mesh itself
# is, not just float noise) can swing the atan2-derived elevation by
# ~90 deg, onto a branch that is reachable in EXACT math but not in
# practice (e.g. it demands a wrist reach far outside the sub-chain's
# envelope), while the natural, documented pose for that direction (Sec
# 9.4: vertical/horizontal-radial/horizontal-tangential all sit at
# elevation 0 or +-90 deg exactly) reaches it fine with only a few degrees
# of orientation error. So the exact roots alone are not a reliable
# search -- ``kinematics.grasp`` also tries the four cardinal anchors.
#
# The tolerance for accepting an anchor (``kgrasp.ANCHOR_TOLERANCE_DEG``) is
# grounded in the joint's own physical slack, not chosen to make a specific
# failing case pass: the glue gap (3.25-6.5 mm, JOINT_ALLOWANCE_M) already
# absorbs some imprecision, and over a ~110 mm stick that gap alone
# corresponds to asin(3.25/110)..asin(6.5/110) =~ 1.7..3.4 deg of angular
# slack before the joint's own designed-in gap is exceeded. 5 deg has
# comfortable margin above that (covers a real case found at 2.08 deg)
# while staying far below where an actual bug shows up (every wrong-branch
# bug found so far was off by 15-180 deg, not single digits) -- see
# tests/test_validate.py's TestNearDegenerateElevationBranch for the case
# that set this number.

# Below this, an accepted candidate's residual error is treated as "the
# exact math basically worked" and not called out separately -- it is
# indistinguishable from the numerical round-trip noise
# kgrasp.VERIFY_TOLERANCE_DEG already tolerates for the exact-root path.
_APPROXIMATION_NOTEWORTHY_DEG = 0.5

# Sec 9.4: warn when a stick's tilt has a meaningful component outside the
# arm's own vertical plane -- untested territory even though reachable.
# Matches kgrasp.ANCHOR_TOLERANCE_DEG's physical grounding (deliberately,
# not by coincidence): a stick a few degrees off pure vertical/radial/tangential
# from ordinary hand-drawn imprecision is not a genuine mixed tilt worth
# flagging as untested territory -- it is the same normal geometric slack
# the joint's own glue gap already absorbs. Found too low at 2 deg
# (2026-07-30): a user's own hand-drawn, unquestionably-safe near-
# tangential stick (2.077 deg off) tripped it, alongside the unrelated
# WARN_ORIENTATION_APPROXIMATED -- redundant noise for the same underlying
# cause, not two distinct things worth separately flagging.
OUT_OF_PLANE_WARN_THRESHOLD_RAD = math.radians(_so_arm_100_kinematics.ANCHOR_TOLERANCE_DEG)

WARN_OUT_OF_PLANE_TILT = "out_of_plane_tilt"
# The accepted orientation is a nearby approximation (an anchor pose),
# not an exact solution -- see kgrasp.ANCHOR_TOLERANCE_DEG's derivation in
# kinematics/grasp.py. The stick will be placed a few degrees off the drawn
# direction; still well inside the joint's own glue-gap slack, but worth
# being visible about rather than silent.
WARN_ORIENTATION_APPROXIMATED = "orientation_approximated"
# Applied by ops/design.py, not this module -- kept here so every warning
# code the UI can show has one home. Set when `suggested_flip` above caused
# the extraction operator to flip this stick automatically (Finding 6):
# reachability was the ONLY reason, this was not the design's original
# base/tip choice, and once a real build order exists (Phase C, where
# "base" gains structural meaning -- the end that attaches to an
# already-placed stick) that choice may need reconciling again.
WARN_AUTO_FLIPPED = "auto_flipped"


class Verdict:
    __slots__ = ("buildable", "reason", "warnings", "suggested_flip")

    def __init__(self, buildable, reason=None, warnings=(), suggested_flip=False):
        self.buildable = buildable
        self.reason = reason
        self.warnings = tuple(warnings)
        # True only when THIS assignment is unreachable and the opposite
        # end-as-base is confirmed (not guessed) to work -- see Finding 6.
        # A structured flag rather than callers string-matching `reason`.
        self.suggested_flip = suggested_flip

    def __repr__(self):
        return "Verdict(buildable=%r, reason=%r, warnings=%r, suggested_flip=%r)" % (
            self.buildable, self.reason, self.warnings, self.suggested_flip,
        )


def stick_grasp_target(stick, kinematics_module=_so_arm_100_kinematics):
    """TCP target for PLACING this stick: the grip point along its own
    axis, GRASP_OFFSET_M from the base end -- ROS2_IMPLEMENTATION_PLAN.md
    Sec 8.2's ``T_target_tcp`` formula. Thin domain-object wrapper around
    ``kinematics_module.grasp_target`` (which takes plain coordinates, not a
    ``StickSpec``). ``kinematics_module`` defaults to ``so_arm_100`` --
    every caller of this helper today is the so_arm_100-specific path below.
    """
    return kinematics_module.grasp_target(stick.base, stick.tip)


def _validate_stick_so_arm_100(stick):
    """Sec 7's reachability + orientation checks for one stick, against
    so_arm_100's own kinematics. Does not include jaw clearance / collision
    against neighbours -- see module docstring; that is Phase C, once a
    build order gives it something real to check against.
    """
    kgrasp = _so_arm_100_kinematics
    target = stick_grasp_target(stick, kgrasp)
    axis = stick.axis()

    basis = kgrasp.azimuth_frame(target)
    if basis is None:
        return Verdict(False, "target lies on the Shoulder_Rotation axis -- azimuth undefined")
    r_hat, _z_hat, tan_hat = basis

    warnings = []
    # Sec 9.4's table lists FOUR safe, well-documented directions: purely
    # vertical, purely radial, purely tangential, and any purely in-plane
    # tilt between vertical and radial. All of those sit at one of the two
    # extremes of "how tangential is the axis" -- so "how aligned is the
    # axis with tan_hat" alone is the WRONG test (it flags the safe pure-
    # tangential case as maximally risky, since that axis basically equals
    # tan_hat). What is genuinely untested territory (the corrected Sec 9.4
    # row) is a stick with a MEANINGFUL lean in BOTH the in-plane direction
    # AND the tangential direction at once -- angularly far from *both*
    # extremes, not merely far from one of them.
    tangential_component = max(-1.0, min(1.0, abs(v_dot(axis, tan_hat))))
    angle_from_in_plane = math.asin(tangential_component)       # 0 when in-plane, 90deg when tangential
    angle_from_tangential = math.acos(tangential_component)     # 0 when tangential, 90deg when in-plane
    if min(angle_from_in_plane, angle_from_tangential) > OUT_OF_PLANE_WARN_THRESHOLD_RAD:
        warnings.append(WARN_OUT_OF_PLANE_TILT)

    solved = kgrasp.solve_stick_orientation(target, axis)
    if solved is None:
        # Finding 6 (empirical, 2026-07-30): Wrist_Roll's limit is
        # ASYMMETRIC (roughly -157..+68 deg, since WRIST_ROLL_AT_ZERO_
        # STICK_ROLL_RAD=+90deg eats most of the positive headroom).
        # Swapping which end of a stick counts as "base" negates the grip
        # direction, which typically negates the needed roll too -- so for
        # a stick sitting right at this asymmetric edge, ONE of the two
        # base/tip assignments can be comfortably reachable while the
        # OTHER needs a roll on the wrong side of the limit. For a
        # near-horizontal stick (both ends at nearly the same height) this
        # is a routine, easily-hit case: Sec 5.1's own "base = lower Z"
        # tie-break is then decided by microscopic mesh-expansion noise,
        # and can pick either end. `sticks.py` already exposes a per-stick
        # `flip` override for exactly this -- check it here so the
        # suggestion is concrete and verified, not a guess.
        flipped_solved = kgrasp.solve_stick_orientation(target, tuple(-c for c in axis))
        if flipped_solved is not None:
            return Verdict(
                False,
                "unreachable with this end as base (the needed Wrist_Roll exceeds "
                "its limit), but the OPPOSITE end works -- toggle this stick's "
                "'Flip' setting",
                tuple(warnings),
                suggested_flip=True,
            )

        # Neither assignment works. Report a concrete reason from the
        # closest attempt, so "impossible" is never left unexplained.
        # Re-run the position/elevation-only check (roll=0, always
        # in-limits -- see module docstring) to get is_reachable's own
        # specific message when that is the blocker; otherwise it is a
        # Wrist_Roll-limit case the exhaustive search itself already ruled
        # out for both assignments.
        grip_dir = v_normalized(tuple(-c for c in axis))
        a_r, a_z = v_dot(grip_dir, r_hat), grip_dir[2]
        elevation0 = 0.0 if (abs(a_r) < 1e-12 and abs(a_z) < 1e-12) else math.atan2(-a_r, a_z)
        reachable, reason = kgrasp.is_reachable(target, elevation0, stick_roll_rad=0.0)
        if not reachable:
            return Verdict(False, reason, tuple(warnings))
        return Verdict(
            False,
            "reachable in position but no wrist-roll value places the stick "
            "at its designed orientation (with either end as base) without "
            "exceeding Wrist_Roll's limit",
            tuple(warnings),
        )

    _elevation, _roll, _elbow_up, error_deg = solved
    if error_deg > _APPROXIMATION_NOTEWORTHY_DEG:
        warnings.append(WARN_ORIENTATION_APPROXIMATED)
    return Verdict(True, None, tuple(warnings))


def _validate_stick_kr10_r900_2(stick, kinematics_module):
    """Reachability check for kr10_r900_2. Unlike so_arm_100's square stock
    (roll fixed by the design, always 0 -- see ``io/build_file.py``'s own
    comment on ``roll_deg``), this robot's stock is round (KUKA_IMPLEMENTATION_
    PLAN.md KQ6) with NO physically-preferred roll for grasping alone (see
    that package's own ``grasp.py`` docstring) -- so a single fixed-roll
    attempt would report plenty of genuinely-buildable sticks as impossible.
    ``solve_stick_placement_any_roll`` sweeps roll x (elbow_up, wrist_flip)
    and is exact per attempt (no round-trip verification needed, unlike
    so_arm_100 -- see that package's own ``chain.py`` docstring), so "every
    combination fails" means genuinely unreachable, matching this file's own
    so_arm_100 section's same standard.

    Tries the flipped base/tip assignment on failure for the same reason the
    so_arm_100 path does: which end a design calls "base" is a structural
    tie-break (Sec 5.1), not a guarantee that assignment is the reachable one.
    """
    try:
        kinematics_module.solve_stick_placement_any_roll(stick.base, stick.tip)
        return Verdict(True)
    except kinematics_module.Unreachable as forward_exc:
        try:
            kinematics_module.solve_stick_placement_any_roll(stick.tip, stick.base)
        except kinematics_module.Unreachable:
            return Verdict(False, str(forward_exc))
        return Verdict(
            False,
            "%s -- but the opposite end works, toggle this stick's 'Flip' "
            "setting" % forward_exc,
            suggested_flip=True,
        )


def _validate_stick_generic(stick, kinematics_module):
    """Contract-only reachability check for any robot besides so_arm_100 --
    see the module docstring's "Multi-robot" section. Uses nothing but
    ``solve_stick_placement`` (``core/robots.py``'s interface contract), and
    tries the flipped base/tip assignment on failure the same way the
    so_arm_100 path does, for the same reason (Sec 5.1's "base = lower Z"
    default is a tie-break, not a guarantee either end is the right choice).

    Deliberately broad ``except Exception``: a not-yet-vendored placeholder
    package (``kinematics/kr10_r900_2/``) raises a plain ``RuntimeError`` the
    moment any of its attributes are touched, not necessarily the contract's
    own ``Unreachable`` -- and this function's whole job is to turn *any*
    failure from an arbitrary robot module into a clean ``Verdict`` rather
    than let it crash the extraction pipeline.
    """
    try:
        kinematics_module.solve_stick_placement(stick.base, stick.tip)
        return Verdict(True)
    except Exception as forward_exc:
        try:
            kinematics_module.solve_stick_placement(stick.tip, stick.base)
        except Exception:
            return Verdict(False, str(forward_exc))
        return Verdict(
            False,
            "%s -- but the opposite end works, toggle this stick's 'Flip' "
            "setting" % forward_exc,
            suggested_flip=True,
        )


def validate_stick(stick, robot_id=core_robots.SO_ARM_100_ID):
    """Sec 7's reachability + orientation checks for one stick, against the
    given robot. Does not include jaw clearance / collision against
    neighbours -- see module docstring; that is Phase C, once a build order
    gives it something real to check against.

    ``robot_id`` defaults to so_arm_100 so every pre-existing caller (tests
    included) keeps its exact original behaviour unchanged.
    """
    if robot_id == core_robots.SO_ARM_100_ID:
        return _validate_stick_so_arm_100(stick)
    kinematics_module = core_robots.get_robot(robot_id).kinematics
    if robot_id == core_robots.KR10_R900_2_ID:
        return _validate_stick_kr10_r900_2(stick, kinematics_module)
    return _validate_stick_generic(stick, kinematics_module)


def validate_sticks(sticks, robot_id=core_robots.SO_ARM_100_ID):
    """Validate a whole extraction result. Sec 7 (order-aware jaw
    clearance) is Phase C's addition once a build order exists; until then
    this is per-stick, independent of the others -- see module docstring.
    """
    return {stick.id: validate_stick(stick, robot_id) for stick in sticks}
