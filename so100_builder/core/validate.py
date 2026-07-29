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

## The grasp-orientation transform -- built here, not yet on the ROS2 side

Converting a stick's desired 3D orientation into the ``(tool_elevation_rad,
stick_roll_rad)`` pair ``chain.ik()`` takes is genuinely non-trivial, and is
explicitly unbuilt on the ROS2 side too (its own Phase 1 checklist: "the
transform helper ... are not written yet"). The derivation below is grounded
entirely in facts ``chain.py`` already states and tests
(``Shoulder_Pitch``/``Elbow``/``Wrist_Pitch`` is exactly planar; the stick is
held perpendicular to the tool axis; ``Wrist_Roll`` never moves the TCP), not
in anything new about the physical arm. Two non-obvious findings from
building it, both confirmed empirically against ``fk()``/``ik()``, not
assumed:

1. **The reference stick direction points from the grip toward the stick's
   BASE, not its tip.** ``rot(joints) @ (0, 0, 1)`` (the tool frame's local Z
   column) is a valid representation of "which way the stick points" --
   confirmed by matching all five tuned poses' known-vertical stick, all of
   which come out as *negative* Z. That makes physical sense once you notice
   the grip point sits *above* the stick's base (``GRASP_OFFSET_M`` up from
   it) -- the direction from grip to base is downward. Get this sign wrong
   and every roll solve is off by ~180 deg, silently pushing ``Wrist_Roll``
   out of its limit for placements that are actually fine.
2. **The elevation equation has two roots 180 deg apart**, and only one of
   them puts the roll=0 reference direction anywhere near the target (the
   other requires ~180 deg of roll to compensate, which is usually
   unreachable). There is no way to know which root is right without trying
   both -- so ``_solve_orientation`` below tries both elevation roots and
   both elbow branches (4 combinations; ``Shoulder_Rotation`` itself has no
   branch ambiguity, so this is exhaustive) and **verifies every candidate by
   feeding the solved joints back through ``fk()`` and checking the achieved
   direction**, rather than trusting the closed-form roll formula on its own.
   A candidate is only ever accepted once measured, not derived and assumed.

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
"""

import math

from ..kinematics import chain as kchain
from ..kinematics import envelope as kenvelope
from ..kinematics.constants import CHAIN, GRASP_OFFSET_M
from .transform import v_add, v_cross, v_dot, v_normalized, v_scale

# Shoulder_Rotation's own origin -- every azimuth is measured from here.
# Imported from the submodule directly since the curated kinematics
# __init__ doesn't re-export CHAIN; this reads the vendored package's own
# public data, it does not modify or fork it.
_SHOULDER_AXIS_POINT = CHAIN[0][1]

# How far a candidate's achieved direction may miss the target before it's
# rejected rather than accepted (Sec "empirically verified" above). Tight:
# this is a numerical round-trip check, not a manufacturing tolerance.
_VERIFY_TOLERANCE_DEG = 0.05

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
# search -- see _CARDINAL_ANCHORS below.
#
# The tolerance for accepting an anchor is grounded in the joint's own
# physical slack, not chosen to make a specific failing case pass: the
# glue gap (3.25-6.5 mm, JOINT_ALLOWANCE_M) already absorbs some
# imprecision, and over a ~110 mm stick that gap alone corresponds to
# asin(3.25/110)..asin(6.5/110) =~ 1.7..3.4 deg of angular slack before
# the joint's own designed-in gap is exceeded. 5 deg has comfortable
# margin above that (covers a real case found at 2.08 deg) while staying
# far below where an actual bug shows up (every wrong-branch bug found so
# far was off by 15-180 deg, not single digits) -- see
# tests/test_validate.py's TestNearDegenerateElevationBranch for the case
# that set this number.
_ANCHOR_TOLERANCE_DEG = 5.0
_CARDINAL_ANCHORS_RAD = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)

# Below this, an accepted candidate's residual error is treated as "the
# exact math basically worked" and not called out separately -- it is
# indistinguishable from the numerical round-trip noise _VERIFY_TOLERANCE_
# DEG already tolerates for the exact-root path.
_APPROXIMATION_NOTEWORTHY_DEG = 0.5

# Sec 9.4: warn when a stick's tilt has a meaningful component outside the
# arm's own vertical plane -- untested territory even though reachable.
# Matches _ANCHOR_TOLERANCE_DEG's physical grounding (deliberately, not by
# coincidence): a stick a few degrees off pure vertical/radial/tangential
# from ordinary hand-drawn imprecision is not a genuine mixed tilt worth
# flagging as untested territory -- it is the same normal geometric slack
# the joint's own glue gap already absorbs. Found too low at 2 deg
# (2026-07-30): a user's own hand-drawn, unquestionably-safe near-
# tangential stick (2.077 deg off) tripped it, alongside the unrelated
# WARN_ORIENTATION_APPROXIMATED -- redundant noise for the same underlying
# cause, not two distinct things worth separately flagging.
OUT_OF_PLANE_WARN_THRESHOLD_RAD = math.radians(_ANCHOR_TOLERANCE_DEG)

WARN_OUT_OF_PLANE_TILT = "out_of_plane_tilt"
# The accepted orientation is a nearby approximation (an anchor pose),
# not an exact solution -- see the _ANCHOR_TOLERANCE_DEG derivation above.
# The stick will be placed a few degrees off the drawn direction; still
# well inside the joint's own glue-gap slack, but worth being visible
# about rather than silent.
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


def stick_grasp_target(stick):
    """TCP target for PLACING this stick: the grip point along its own
    axis, GRASP_OFFSET_M from the base end -- ROS2_IMPLEMENTATION_PLAN.md
    Sec 8.2's ``T_target_tcp`` formula. This is the position half of the
    transform (unaffected by the orientation-sign finding above: the grip
    point is offset from base *toward* tip, regardless of which direction
    counts as the IK's own "roll=0 reference").
    """
    axis = stick.axis()
    return v_add(stick.base, v_scale(axis, GRASP_OFFSET_M))


def _stick_axis_from_joints(joint_angles_rad):
    """The tool frame's local Z column -- see module docstring finding 1:
    this points from the grip toward the stick's BASE."""
    _pos, rot = kchain.fk(joint_angles_rad)
    return (rot[0][2], rot[1][2], rot[2][2])


def _wrap_angle(angle_rad):
    """Wrap to (-pi, pi]. ik() does not wrap its own joint-angle solutions
    before comparing them against joint limits (chain.py never needs to,
    since its own callers always pass a small, already-sane elevation) --
    so ``elevation0 + pi`` landing near +2*pi instead of near 0 makes a
    perfectly fine configuration look like it exceeds a limit it doesn't
    (e.g. a Wrist_Pitch solution of "-424.84 deg" that is really -64.84 deg
    once wrapped). Always pass ik() a canonical-range elevation."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _azimuth_basis(target_xyz_m):
    """r_hat (radial, horizontal), z_hat (vertical), tan_hat (tangential /
    out-of-plane horizontal) at the target's azimuth from
    Shoulder_Rotation's axis."""
    dx = target_xyz_m[0] - _SHOULDER_AXIS_POINT[0]
    dy = target_xyz_m[1] - _SHOULDER_AXIS_POINT[1]
    rho = math.hypot(dx, dy)
    if rho < 1e-9:
        return None
    r_hat = (dx / rho, dy / rho, 0.0)
    tan_hat = (-r_hat[1], r_hat[0], 0.0)
    return r_hat, (0.0, 0.0, 1.0), tan_hat


def _try_elevation(target_xyz_m, elevation, grip_dir, tolerance_deg):
    """One (elevation, both elbow branches) probe. Returns the best
    ``(elevation, roll, elbow_up, error_deg)`` of the branches that verify
    within ``tolerance_deg``, or ``None``."""
    best = None
    for elbow_up in (True, False):
        try:
            q_ref = kchain.ik(target_xyz_m, elevation, stick_roll_rad=0.0, elbow_up=elbow_up)
        except kchain.Unreachable:
            continue
        ref_axis = _stick_axis_from_joints(q_ref)
        tool = kchain.tool_axis(q_ref)
        # Sign convention finding 3 (empirical, like findings 1-2 above):
        # increasing stick_roll_rad rotates the stick CLOCKWISE around
        # tool_axis(), opposite the standard right-hand convention a naive
        # cross-product formula assumes. Invisible in-plane (there
        # sin(roll) is always ~0, so only |roll| -- sign-independent -- is
        # ever exercised); only shows up once grip_dir has a genuine
        # tangential component. Caught by round-trip verification below,
        # not by this line looking "obviously" right -- do not simplify
        # this back to +tool without re-running that verification.
        roll = math.atan2(
            -v_dot(v_cross(ref_axis, grip_dir), tool), v_dot(ref_axis, grip_dir)
        )
        try:
            q = kchain.ik(target_xyz_m, elevation, stick_roll_rad=roll, elbow_up=elbow_up)
        except kchain.Unreachable:
            continue
        achieved = _stick_axis_from_joints(q)
        cos_err = max(-1.0, min(1.0, v_dot(grip_dir, achieved)))
        error_deg = math.degrees(math.acos(cos_err))
        if error_deg > tolerance_deg:
            continue
        if best is None or error_deg < best[3]:
            best = (elevation, roll, elbow_up, error_deg)
    return best


def _solve_orientation(target_xyz_m, base_to_tip_axis):
    """Exhaustive, self-verifying search for ``(tool_elevation_rad,
    stick_roll_rad, elbow_up)`` placing the stick along ``base_to_tip_axis``
    at ``target_xyz_m``. Returns ``None`` if no candidate reproduces the
    target direction within tolerance -- see module docstring for why that
    means genuinely unreachable, not merely unchecked.

    Tries two families of candidate elevation, for the reason in Finding 4:

    * The two **exact roots** of the perpendicularity equation (elevation0,
      elevation0 + pi) -- verified tight (0.05 deg). For a well-conditioned
      target (the common case) this is exact, and is preferred whenever it
      succeeds.
    * The four **cardinal anchors** (0, +-90, 180 deg) -- Sec 9.4's own
      documented poses for vertical/horizontal-radial/horizontal-tangential
      -- verified against the physically-grounded ``_ANCHOR_TOLERANCE_DEG``
      (see its own derivation), to absorb the geometric noise the
      mesh-expansion solver routinely introduces near exactly these
      directions without accepting a genuinely wrong orientation.

    All verified candidates compete on smallest achieved error first (an
    exact root beats an anchor whenever both succeed), then smallest |roll|
    to break remaining ties.

    Returns ``(elevation, roll, elbow_up, error_deg)`` -- the caller decides
    whether ``error_deg`` (always < ``_VERIFY_TOLERANCE_DEG`` for an exact
    root, possibly up to ``_ANCHOR_TOLERANCE_DEG`` for an anchor) is worth
    surfacing to the user.
    """
    basis = _azimuth_basis(target_xyz_m)
    if basis is None:
        return None
    r_hat, _z_hat, _tan_hat = basis

    # Finding 1: the reference direction points grip-to-base, not base-tip.
    grip_dir = v_normalized(tuple(-c for c in base_to_tip_axis))
    a_r = v_dot(grip_dir, r_hat)
    a_z = grip_dir[2]
    elevation0 = 0.0 if (abs(a_r) < 1e-12 and abs(a_z) < 1e-12) else math.atan2(-a_r, a_z)

    candidates = []
    for elevation in (_wrap_angle(elevation0), _wrap_angle(elevation0 + math.pi)):
        found = _try_elevation(target_xyz_m, elevation, grip_dir, _VERIFY_TOLERANCE_DEG)
        if found is not None:
            candidates.append(found)
    for elevation in _CARDINAL_ANCHORS_RAD:
        found = _try_elevation(target_xyz_m, elevation, grip_dir, _ANCHOR_TOLERANCE_DEG)
        if found is not None:
            candidates.append(found)

    if not candidates:
        return None
    candidates.sort(key=lambda c: (c[3], abs(c[1])))
    return candidates[0]


def validate_stick(stick):
    """Sec 7's reachability + orientation checks for one stick. Does not
    include jaw clearance / collision against neighbours -- see module
    docstring; that is Phase C, once a build order gives it something real
    to check against.
    """
    target = stick_grasp_target(stick)
    axis = stick.axis()

    basis = _azimuth_basis(target)
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

    solved = _solve_orientation(target, axis)
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
        flipped_solved = _solve_orientation(target, tuple(-c for c in axis))
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
        reachable, reason = kenvelope.is_reachable(target, elevation0, stick_roll_rad=0.0)
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


def validate_sticks(sticks):
    """Validate a whole extraction result. Sec 7 (order-aware jaw
    clearance) is Phase C's addition once a build order exists; until then
    this is per-stick, independent of the others -- see module docstring.
    """
    return {stick.id: validate_stick(stick) for stick in sticks}
