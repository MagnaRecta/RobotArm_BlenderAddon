"""The grasp-orientation transform: turn a stick's physical placement into
IK-consumable parameters -- mirrors ``so_arm_100_kinematics.grasp``'s role
(``ROS2_IMPLEMENTATION_PLAN.md`` Sec 8.2's "transform helper"), adapted for
this arm's genuine 6-DOF spherical wrist.

One fact this builds on, derived from real data, not assumed
-------------------------------------------------------------
**``gripper_tcp``'s local Y axis is the physical stick's own base->tip
direction.** Computed via this package's own ``fk()`` at BOTH of the two
independently hand-tuned real poses in
``kuka_pick_and_place/config/pick_and_place.yaml`` (``pregrasp`` and
``lower``) -- local Y comes out to world ``(-0.99997, -0.00013,
-0.00714)`` and ``(-0.99997, -0.00034, -0.00752)`` respectively, agreeing
with each other to <0.02 deg and matching world **-X** (the physical
stick's own known direction, KQ5: "sticks extend from that coordinate
towards -X") to within the same margin. Two independently-tuned poses
agreeing this tightly is real evidence, not a guess -- see
``KUKA_IMPLEMENTATION_PLAN.md`` Sec 3 Phase 3. This is the KUKA-side
analogue of SO-100's own Finding 1 (the stick points along the tool frame's
local Z there) -- same idea, different axis, because this robot's tool
frame is built differently (see ``constants.py``'s ``GRIPPER_MOUNT_PITCH_RAD``
comment).

Unlike SO-100's 5-DOF arm (1 free orientation DOF -- roll -- with a
numerically ill-conditioned elevation search that needed round-trip
verification against ``fk()`` before trusting any candidate), this arm's
closed-form ``chain.ik()`` is **exact and unconditional**: given ANY valid
target orientation (subject only to joint limits and the 4-branch
elbow/wrist-flip structure -- see ``chain.py``), it returns a solution that
reproduces that exact orientation, verified over 20000+ random round-trip
samples to <1e-11 (see ``test/test_chain.py``). So placing a stick reduces
to (a) fixing ``gripper_tcp``'s local Y axis to the desired stick direction
(the ONE real physical constraint), and (b) picking the remaining free "roll"
about that axis -- which, for this robot's **round** 2mm stock (KQ6,
unlike SO-100's square stock), has **no physically-preferred value at
all** for grasping alone. ``roll_rad`` is exposed as a free parameter for
Phase 5's own future needs (e.g. avoiding a jaw/table collision, or a real
glue-joint alignment concern once that geometry is worked out), not because
grasping a round stick needs a particular one.
"""

import math

from .chain import Unreachable, fk, ik
from .constants import GRASP_OFFSET_M, JAW_CONTACT_HALF_LENGTH_M, MIN_GRASP_OFFSET_M

# How many roll_rad values solve_stick_placement_any_roll tries (evenly
# spaced across a full turn) before giving up -- see that function's own
# docstring for why a sweep is meaningful here (unlike SO-100, roll is a
# genuinely free parameter for this robot's round stock, not a numerical
# workaround).
DEFAULT_ROLL_SWEEP_STEPS = 12


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v):
    n = math.sqrt(_dot(v, v))
    if n < 1e-12:
        raise ValueError("cannot normalize a zero-length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def grasp_offset_for_length(length_m, default_offset_m=GRASP_OFFSET_M,
                            jaw_contact_half_length_m=JAW_CONTACT_HALF_LENGTH_M,
                            min_offset_m=MIN_GRASP_OFFSET_M):
    """The grasp offset (see :func:`grasp_target`) to use for a stick of
    this length -- mirrors ``so_arm_100_kinematics.grasp.grasp_offset_for_length``
    exactly (same formula, same intent: never close the jaws past a short
    stick's own tip).

    **Not needed for Phase 1's feeder grasp** -- this robot's grasp point is
    fixed in the fixture's own frame regardless of stick length
    (``KUKA_IMPLEMENTATION_PLAN.md`` Sec 0 point 2), and ``GRASP_OFFSET_M``
    (18.8mm) plus ``JAW_CONTACT_HALF_LENGTH_M`` already sits safely inside
    even the shortest allowed stick (35mm, ``STICK_LENGTH_RANGE_M``). Kept
    for Phase 5, where a stick may need re-gripping elsewhere.

    Raises :class:`chain.Unreachable` if even the largest possible offset
    falls below ``min_offset_m``.
    """
    max_offset = length_m - jaw_contact_half_length_m
    offset = min(default_offset_m, max_offset)
    if offset < min_offset_m:
        raise Unreachable(
            "stick length %.4fm is too short to grip safely: the largest "
            "usable offset (%.4fm, capped by jaw contact length) is below "
            "the %.4fm floor-clearance floor" % (length_m, offset, min_offset_m))
    return offset


def grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M):
    """TCP target for gripping/placing a stick between these physical ends:
    the point ``grasp_offset_m`` from the base end, along the stick's own
    axis toward the tip. Identical formula to
    ``so_arm_100_kinematics.grasp.grasp_target`` -- position-only, unaffected
    by anything orientation-related below.
    """
    dx = tip_xyz_m[0] - base_xyz_m[0]
    dy = tip_xyz_m[1] - base_xyz_m[1]
    dz = tip_xyz_m[2] - base_xyz_m[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-12:
        raise ValueError("base and tip coincide -- the stick has zero length")
    return (
        base_xyz_m[0] + dx / length * grasp_offset_m,
        base_xyz_m[1] + dy / length * grasp_offset_m,
        base_xyz_m[2] + dz / length * grasp_offset_m,
    )


def stick_axis(joint_angles_rad):
    """The physical stick's base->tip direction for this joint solution --
    ``gripper_tcp``'s local Y column, ``fk(joint_angles_rad)[1]``'s second
    column. See the module docstring for how this axis assignment (Y, not
    e.g. Z) was derived from real tuned-pose data, not assumed."""
    _pos, rot = fk(joint_angles_rad)
    return (rot[0][1], rot[1][1], rot[2][1])


def orientation_from_stick_axis(stick_axis_hat, roll_rad=0.0, reference_up=(0.0, 0.0, 1.0)):
    """Build a full ``gripper_tcp`` target rotation matrix (3x3 tuple of
    tuples, columns = X/Y/Z) whose local Y column equals
    ``stick_axis_hat`` (need not be unit length; normalized here) -- the one
    real physical constraint (module docstring). The remaining 1 DOF is
    ``roll_rad``, rotation about that same axis, measured from a
    deterministic reference built via Gram-Schmidt against
    ``reference_up`` (world +Z by default; automatically swapped to world
    +X if ``stick_axis_hat`` is itself nearly vertical, i.e. nearly
    parallel to the default reference, so the reference never degenerates).

    ``roll_rad = 0`` is therefore an arbitrary-but-deterministic choice, NOT
    the ``pregrasp``/``lower`` poses' own roll (recovering that exactly
    would need the actual X/Z column data at those poses, not just Y) --
    for this robot's round stock there is no physical reason to prefer one
    roll over another anyway (module docstring).
    """
    y = _normalize(stick_axis_hat)
    ref = reference_up if abs(_dot(reference_up, y)) < 0.999 else (1.0, 0.0, 0.0)
    z0 = _normalize(_v_sub(ref, _v_scale(y, _dot(ref, y))))
    x0 = _cross(y, z0)  # x0 x y = z0, established below; keeps the frame right-handed

    c, s = math.cos(roll_rad), math.sin(roll_rad)
    x = (x0[0] * c + z0[0] * s, x0[1] * c + z0[1] * s, x0[2] * c + z0[2] * s)
    z = (-x0[0] * s + z0[0] * c, -x0[1] * s + z0[1] * c, -x0[2] * s + z0[2] * c)

    return (
        (x[0], y[0], z[0]),
        (x[1], y[1], z[1]),
        (x[2], y[2], z[2]),
    )


def orientation_pointing_down(roll_rad=0.0, direction_hat=(0.0, 0.0, -1.0)):
    """Added 2026-09-04 for the manual jog tool (operator_gui.py's "move to
    a coordinate" option): a full ``gripper_tcp`` target rotation matrix
    whose local **Z** column (the tool's own approach/pointing axis --
    ``motion.py``'s own ``retreat`` handling calls this "the gripper's own
    approach axis, local -Z") equals ``direction_hat``, default straight
    down (world ``-Z``). Deliberately Z here, NOT :func:`orientation_from_stick_axis`'s
    Y -- that function aligns the axis a physically HELD STICK runs along
    (this gripper's local Y, derived from real tuned-pose data -- see this
    module's own docstring), a different, perpendicular axis from the
    direction the tool itself points/travels along to get there. Confusing
    the two would build a gripper that holds a stick pointing straight
    down, not one that itself points down.

    Built by reusing :func:`orientation_from_stick_axis`'s exact
    Gram-Schmidt/roll construction (aligning ITS Y column to
    ``direction_hat`` instead) and then cyclically permuting the resulting
    right-handed columns ``(X, Y, Z) -> (Z, X, Y)`` so ``direction_hat``
    lands in the Z slot -- a cyclic permutation of a right-handed frame's
    own columns is still right-handed (verified: 2000 random rolls, exact
    orthonormality and ``x cross y == z`` to float precision; round-tripped
    through ``chain.ik()`` + ``chain.fk()`` at a real reachable point,
    position and Z-axis-direction error both at floating-point noise).
    This is a relabelling of the SAME deterministic frame, not a second,
    independently-written derivation that could disagree with the first.
    """
    m = orientation_from_stick_axis(direction_hat, roll_rad)
    x0 = (m[0][0], m[1][0], m[2][0])
    y0 = (m[0][1], m[1][1], m[2][1])  # == normalize(direction_hat)
    z0 = (m[0][2], m[1][2], m[2][2])
    return (
        (z0[0], x0[0], y0[0]),
        (z0[1], x0[1], y0[1]),
        (z0[2], x0[2], y0[2]),
    )


def iter_vertical_poses(position_m, roll_rad=0.0, direction_hat=(0.0, 0.0, -1.0)):
    """Every (``elbow_up``, ``wrist_flip``) branch that puts ``gripper_tcp``
    at ``position_m`` with its local Z axis pointing along ``direction_hat``
    (world ``-Z``, straight down, by default) -- the general "point the
    tool in a fixed world direction" analogue of :func:`iter_stick_placements`,
    for the manual jog tool rather than stick placement (see
    :func:`orientation_pointing_down`'s own docstring for how the two
    differ). Same reason for a generator rather than a single winner as
    :func:`iter_stick_placements`: ``chain.ik()`` has no notion of the
    robot's own links colliding with EACH OTHER, so a caller doing
    joint-space planning needs every reachable branch to retry when
    MoveIt's own collision-aware planner rejects one.

    Never raises. Yields ``(joints, elbow_up, wrist_flip)``; an empty
    iteration means unreachable at this exact position/roll.
    """
    target_rot = orientation_pointing_down(roll_rad, direction_hat)
    for elbow_up in (True, False):
        for wrist_flip in (False, True):
            try:
                joints = ik(position_m, target_rot, elbow_up=elbow_up, wrist_flip=wrist_flip)
            except Unreachable:
                continue
            yield joints, elbow_up, wrist_flip


def solve_stick_placement(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M,
                          roll_rad=0.0, elbow_up=True, wrist_flip=False):
    """The full solve for a single, specific branch/roll choice: given a
    stick's physical base and tip, return the 6 joint angles
    (radians, ``chain.JOINT_NAMES`` order) that grip/place it.

    Raises :class:`chain.Unreachable` (from ``chain.ik()``) if this exact
    branch/roll combination cannot reach the target -- try
    :func:`solve_stick_placement_any_roll` for a placement where the
    specific roll doesn't matter (the common case for this robot's round
    stock -- see the module docstring).
    """
    target_pos = grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m)
    axis = tuple(t - b for t, b in zip(tip_xyz_m, base_xyz_m))
    target_rot = orientation_from_stick_axis(axis, roll_rad)
    return ik(target_pos, target_rot, elbow_up=elbow_up, wrist_flip=wrist_flip)


def iter_stick_placements(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M, roll_rad=0.0):
    """Every (``elbow_up``, ``wrist_flip``) branch that reaches this exact
    placement/roll, in the same preference order :func:`solve_stick_placement`
    itself would try them.

    ``chain.ik()``'s branch choice has no notion of the robot's own links
    colliding with EACH OTHER -- only MoveIt's collision-aware planner
    catches that (found 2026-08-23 on a real build: a kinematically valid
    branch put ``link_4`` into contact with ``link_6``). A caller doing
    joint-space planning needs every reachable branch to retry against when
    MoveIt rejects one, not just the first -- hence this generator instead
    of :func:`solve_stick_placement` picking a single winner.

    Never raises. Yields ``(joints, elbow_up, wrist_flip)``; an empty
    iteration means unreachable at this exact roll.
    """
    target_pos = grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m)
    axis = tuple(t - b for t, b in zip(tip_xyz_m, base_xyz_m))
    target_rot = orientation_from_stick_axis(axis, roll_rad)
    for elbow_up in (True, False):
        for wrist_flip in (False, True):
            try:
                joints = ik(target_pos, target_rot, elbow_up=elbow_up, wrist_flip=wrist_flip)
            except Unreachable:
                continue
            yield joints, elbow_up, wrist_flip


def iter_stick_placements_any_roll(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M,
                                   roll_steps=DEFAULT_ROLL_SWEEP_STEPS):
    """Like :func:`iter_stick_placements`, swept across ``roll_steps`` evenly
    spaced roll values (the round stock's free roll DOF -- module
    docstring), in the same order :func:`solve_stick_placement_any_roll`
    tries them. Yields every reachable ``(joints, roll_rad, elbow_up,
    wrist_flip)``, not just the first -- see :func:`iter_stick_placements`
    for why a caller needs more than one candidate.

    Never raises. An empty iteration means unreachable at every roll tried.
    """
    target_pos = grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m)
    axis = tuple(t - b for t, b in zip(tip_xyz_m, base_xyz_m))
    for step in range(roll_steps):
        roll_rad = 2.0 * math.pi * step / roll_steps
        target_rot = orientation_from_stick_axis(axis, roll_rad)
        for elbow_up in (True, False):
            for wrist_flip in (False, True):
                try:
                    joints = ik(target_pos, target_rot, elbow_up=elbow_up, wrist_flip=wrist_flip)
                except Unreachable:
                    continue
                yield joints, roll_rad, elbow_up, wrist_flip


def solve_stick_placement_any_roll(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M,
                                   roll_steps=DEFAULT_ROLL_SWEEP_STEPS):
    """Like :func:`solve_stick_placement`, but exploits the round stock's
    genuinely free roll DOF (module docstring): tries ``roll_steps`` evenly
    spaced roll values, each against all 4 (``elbow_up``, ``wrist_flip``)
    branches, and returns the first joint solution that satisfies every
    joint limit. Since ``chain.ik()`` is exact (not a numerical search),
    "no combination works" here means the placement is genuinely
    unreachable for EVERY roll, not merely unchecked.

    Returns ``(joints, roll_rad, elbow_up, wrist_flip)`` on success. Raises
    :class:`chain.Unreachable` if every combination fails. A caller that
    wants every candidate (e.g. to retry past a self-collision MoveIt
    rejects) should use :func:`iter_stick_placements_any_roll` directly.
    """
    for joints, roll_rad, elbow_up, wrist_flip in iter_stick_placements_any_roll(
            base_xyz_m, tip_xyz_m, grasp_offset_m, roll_steps):
        return joints, roll_rad, elbow_up, wrist_flip
    raise Unreachable(
        "no roll (%d values tried) x branch combination places a stick from "
        "%r to %r" % (roll_steps, base_xyz_m, tip_xyz_m))
