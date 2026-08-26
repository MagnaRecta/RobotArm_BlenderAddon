"""Closed-form forward and inverse kinematics for the KUKA KR10 R900-2, a
genuine 6-DOF articulated ("elbow manipulator") arm -- pure Python (stdlib
``math`` only, no numpy), mirroring ``so_arm_100_kinematics.chain``'s own
zero-ROS-dependency constraint.

Geometry, derived from the URDF
--------------------------------
``joint1`` yaws about vertical (axis (0,0,-1)) with no radial offset (its
own origin sits directly above ``base_link``'s origin). ``joint2``/``joint3``
both pitch about a horizontal Y axis -- the classic shoulder/elbow pair --
and, exactly like SO-100's own Shoulder_Pitch/Elbow/Wrist_Pitch sub-chain
(see that package's chain.py docstring), every joint origin from ``joint2``
onward has zero local-Y component, so a rotation about Y never introduces a
Y component into a vector that starts without one: **the whole
joint2/joint3/wrist-centre sub-chain is exactly planar**, living in the
single vertical plane fixed by ``joint1``'s azimuth. Unlike SO-100, this
plane is reached via a real offset vector (``joint3``'s wrist-centre offset
has a nonzero component off ``joint3``'s own local X axis), not a pure
2-link arm -- solved below with the same "offset angle folded into a
constant phase shift" technique SO-100's own ``_ik_planar`` uses.

``joint4``/``joint5``/``joint6`` form a genuine spherical wrist: ``joint4``'s
axis is X, and the joint4->joint5 translation is *along that same axis* --
so rotating ``joint4`` never moves ``joint5``'s own origin, which sits fixed
on ``joint4``'s own axis line. ``joint6``'s axis is also X (well, -X: see
below), and the joint5->joint6 translation is along *that* axis too -- so
``joint6``'s own rotation line, extended, passes back through ``joint5``'s
origin as well. **All three axis lines intersect at the single point
``joint5``'s own origin** -- this is the Pieper condition, verified
numerically (not just algebraically) in ``test/test_chain.py``. This lets
the 6-DOF problem decouple cleanly, Pieper-style:

  1. The wrist centre (``joint5``'s origin) is recovered from the target
     TCP pose by subtracting the *fixed-offset, orientation-carried* vector
     from wrist centre to TCP -- see ``_wrist_center_from_target`` below;
     this needs the TARGET orientation, not any unknown joint value, because
     that offset vector, expressed in world frame, is exactly
     ``R_target @ (fixed local offset)`` regardless of ``joint4``/``joint5``/
     ``joint6``'s actual values (the same fact SO-100's own Wrist_Roll
     invariance rests on, generalized to a rotation about the *same axis*
     the offset lies along not moving that offset at all).
  2. ``joint1`` (azimuth) and ``joint2``/``joint3`` (the offset 2-link
     planar solve) position that wrist centre exactly, closed form, two
     elbow branches.
  3. ``joint4``/``joint5``/``joint6`` orient the tool exactly, closed form,
     via a standard X-Y-X proper-Euler decomposition of the rotation that
     remains once ``joint1``/``joint2``/``joint3``'s own contribution is
     backed out -- see ``_extract_xyx`` below. (``joint4``/``joint6``'s axes
     are both literally ``(-1,0,0)`` in the URDF; folding the sign into the
     decomposition once, here, is simpler than re-deriving a mirrored
     formula, so this module treats the *magnitude/axis-line* as X-Y-X and
     negates ``joint4``/``joint6`` at the very end.)

No iteration, no seeds, no timeouts: the full 6-DOF solve is closed form
(subject to the elbow up/down branch and, at the joint4/joint5/joint6 wrist
singularity, an explicit tie-breaking choice -- see ``_extract_xyx``).
Verified by round-tripping ``fk() -> ik() -> fk()`` over 20000+ random
joint-space samples (both elbow branches) to <1e-12 position/orientation
error -- see ``test/test_chain.py``.
"""

import math

from .constants import (
    CHAIN,
    GRIPPER_CLOCK_ANGLE_RAD,
    GRIPPER_CLOCK_XYZ_M,
    GRIPPER_MOUNT_PITCH_RAD,
    GRIPPER_TCP_Z_M,
)

# --- minimal 3D vector + 3x3 matrix helpers (no numpy) ----------------------


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mat_vec(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _mat_mat(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mat_transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _axis_angle_matrix(axis, angle):
    n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    ux, uy, uz = axis[0] / n, axis[1] / n, axis[2] / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return (
        (c + ux * ux * C, ux * uy * C - uz * s, ux * uz * C + uy * s),
        (uy * ux * C + uz * s, c + uy * uy * C, uy * uz * C - ux * s),
        (uz * ux * C - uy * s, uz * uy * C + ux * s, c + uz * uz * C),
    )


def _rotate_2d(v, theta):
    """Rotate a (u, v) pair by theta, standard CCW convention -- matches
    Ry(theta) acting on (v_component, 0, u_component) when u is the pair's
    first element and v its second (see the module docstring's planar
    sub-chain and ``_A``/``_B``/``_C`` below, which use (height, radial)
    pairs in that order)."""
    c, s = math.cos(theta), math.sin(theta)
    u, w = v
    return (u * c - w * s, u * s + w * c)


class Unreachable(Exception):
    """Raised by ik() when no joint solution exists for the target."""


# --- forward kinematics ------------------------------------------------------

# Fixed joint6/link_6 -> gripper_tcp transform, folded into one constant
# (translation, rotation) pair at import time -- three fixed URDF joints in
# series (gripper_clock_joint, gripper_mount_joint, gripper_tcp_joint).
_TOOL_ROT = _mat_mat(_rot_x(GRIPPER_CLOCK_ANGLE_RAD), _rot_y(GRIPPER_MOUNT_PITCH_RAD))
_TOOL_XYZ = _v_add(
    GRIPPER_CLOCK_XYZ_M,
    _mat_vec(_mat_mat(_rot_x(GRIPPER_CLOCK_ANGLE_RAD), _rot_y(GRIPPER_MOUNT_PITCH_RAD)), (0.0, 0.0, GRIPPER_TCP_Z_M)),
)
_TOOL_ROT_T = _mat_transpose(_TOOL_ROT)

# joint6's own translation (0.090, 0, 0) lies along joint6's own axis line
# (see module docstring point 1) -- pulled out here since both fk() and the
# wrist-centre back-solve in ik() need it.
_WRIST_TO_LINK6 = CHAIN[5][1]


def fk(joint_angles_rad):
    """Full FK. Returns (position_xyz_m, rotation_matrix) of ``gripper_tcp``
    in ``base`` (this robot's URDF root link -- NOT ``base_link``; see
    ``kuka_pick_and_place``'s own ``base_frame`` parameter). rotation_matrix
    is a 3x3 tuple of tuples; columns are the frame's X/Y/Z axes expressed
    in ``base``."""
    pos = (0.0, 0.0, 0.0)
    rot = _IDENTITY
    for (_name, xyz, axis, _lo, _hi), q in zip(CHAIN, joint_angles_rad):
        pos = _v_add(pos, _mat_vec(rot, xyz))
        rot = _mat_mat(rot, _axis_angle_matrix(axis, q))
    pos = _v_add(pos, _mat_vec(rot, _TOOL_XYZ))
    rot = _mat_mat(rot, _TOOL_ROT)
    return pos, rot


def _link3_fk(q1, q2, q3):
    """Position + orientation of ``link_3`` (after joint1/joint2/joint3
    only) -- internal, used by both the wrist-centre position formula and
    the joint4/5/6 orientation solve below."""
    pos = (0.0, 0.0, 0.0)
    rot = _IDENTITY
    for (_name, xyz, axis, _lo, _hi), q in zip(CHAIN[:3], (q1, q2, q3)):
        pos = _v_add(pos, _mat_vec(rot, xyz))
        rot = _mat_mat(rot, _axis_angle_matrix(axis, q))
    return pos, rot


# --- geometry used by the position IK (joint1/joint2/joint3) ----------------

_SHOULDER_AXIS_POINT = CHAIN[0][1]  # joint1's own origin xyz

# Planar sub-chain offsets, expressed as (height, radial) = (local-z,
# local-x) pairs -- see module docstring: joint2/joint3 both rotate about Y,
# and every offset below has zero local-Y component, so the whole sub-chain
# stays in one vertical plane, exactly as SO-100's own does (just via a true
# elbow-manipulator layout rather than SO-100's simpler chain).
_A = (0.0, CHAIN[1][1][0])  # joint2 origin offset: (0, 0.025)
_B = (0.0, CHAIN[2][1][0])  # joint3 origin offset: (0, 0.455)
# Wrist-centre offset from joint3's own origin: joint4's translation
# (0,0,0.025) plus joint5's translation (0.420,0,0) -- the latter is along
# joint4's own axis, so it survives joint4's rotation unchanged (module
# docstring point 1); together (0.420, 0, 0.025) in local (x, y, z).
_C = (CHAIN[3][1][2], CHAIN[4][1][0])  # (0.025, 0.420)

_L1 = math.hypot(*_B)
_L2 = math.hypot(*_C)
_PHI_B = math.atan2(_B[1], _B[0])
_PHI_C = math.atan2(_C[1], _C[0])
_DELTA = _PHI_C - _PHI_B


def _wrist_center_from_target(target_xyz_m, target_rot):
    """Back-solve the wrist centre (joint5's own origin) from the desired
    TCP pose -- module docstring point 1. ``target_rot`` is used, not any
    unknown joint value: the vector from wrist-centre to ``gripper_tcp``,
    expressed in world frame, equals ``target_rot`` applied to a FIXED local
    offset, because joint6's own rotation is about the same axis line that
    offset lies along (verified in ``test/test_chain.py``)."""
    link6_rot = _mat_mat(target_rot, _TOOL_ROT_T)
    link6_pos = _v_sub(target_xyz_m, _mat_vec(link6_rot, _TOOL_XYZ))
    return _v_sub(link6_pos, _mat_vec(link6_rot, _WRIST_TO_LINK6))


def _solve_position(wrist_center_xyz_m, elbow_up=True):
    """Closed-form joint1/joint2/joint3 placing the wrist centre exactly at
    ``wrist_center_xyz_m``. Raises Unreachable if out of the sub-chain's
    reach envelope."""
    dx = wrist_center_xyz_m[0] - _SHOULDER_AXIS_POINT[0]
    dy = wrist_center_xyz_m[1] - _SHOULDER_AXIS_POINT[1]
    dz = wrist_center_xyz_m[2] - _SHOULDER_AXIS_POINT[2]

    rho = math.hypot(dx, dy)
    if rho < 1e-9:
        raise Unreachable("wrist centre lies on joint1's own axis -- azimuth undefined")
    phi = math.atan2(dy, dx)
    q1 = -phi  # joint1's axis is (0,0,-1): rotation by q1 about it equals
    # rotation by -q1 about (0,0,1), i.e. azimuth = -q1.

    v_target = (dz, rho)
    g = (v_target[0] - _A[0], v_target[1] - _A[1])
    r = math.hypot(*g)

    cos_delta = (r * r - _L1 * _L1 - _L2 * _L2) / (2.0 * _L1 * _L2)
    if cos_delta < -1.0 - 1e-9 or cos_delta > 1.0 + 1e-9:
        raise Unreachable(
            f"wrist centre {wrist_center_xyz_m} at reach {r:.4f} m is outside the "
            f"joint2/joint3/wrist-centre sub-chain's envelope "
            f"({abs(_L1 - _L2):.4f}..{_L1 + _L2:.4f} m)"
        )
    cos_delta = max(-1.0, min(1.0, cos_delta))
    delta_angle = math.acos(cos_delta)
    if not elbow_up:
        delta_angle = -delta_angle

    beta1 = math.atan2(g[1], g[0]) - math.atan2(
        _L2 * math.sin(delta_angle), _L1 + _L2 * math.cos(delta_angle)
    )
    q2 = beta1 - _PHI_B
    q3 = delta_angle - _DELTA
    return q1, q2, q3


# --- geometry used by the orientation IK (joint4/joint5/joint6) -------------


def _wrap_angle(angle_rad):
    """Wrap to (-pi, pi]."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _extract_xyx(r, flip=False):
    """Recover (a, b, c) such that ``Rx(a) @ Ry(b) @ Rx(c) == r`` (a proper
    X-Y-X Euler decomposition) -- the standard closed-form extraction,
    verified numerically over 200000 random matrices to <1e-15 reconstruction
    error (not hand-checked-only; see ``test/test_chain.py``).

    Like any 3-DOF proper-Euler wrist, every orientation generically has
    **two** decompositions ("wrist flip", the orientation analogue of
    ``elbow_up``): ``(a, b, c)`` with ``b`` in ``[0, pi]``, and
    ``(a+pi, -b, c+pi)`` with ``b`` in ``[-pi, 0]`` -- algebraically
    equivalent (verified numerically, same tolerance), but landing on
    opposite sides of a joint's own limits is common, so both must be tried
    before declaring a target unreachable (see ``ik()``'s ``wrist_flip``
    parameter). ``flip=True`` selects the second branch directly.

    At the ``sin(b) == 0`` wrist singularity (``joint4``/``joint6`` axes
    coincide), only ``a + c`` (b=0) or ``a - c`` (b=pi) is determined -- this
    picks ``a = 0`` and puts the whole remainder on ``c``, an arbitrary but
    explicit tie-break, matching the kind of singularity-handling choice
    SO-100's own Wrist_Roll convention documents rather than hides. Both
    ``flip`` values return the same (degenerate) result at the singularity.
    """
    r00, r01, r02 = r[0][0], r[0][1], r[0][2]
    r10, r20 = r[1][0], r[2][0]
    sin_b = math.hypot(r01, r02)
    b = math.atan2(sin_b, r00)
    if sin_b > 1e-9:
        a = math.atan2(r10, -r20)
        c = math.atan2(r01, r02)
    else:
        a = 0.0
        if r00 > 0.0:
            c = math.atan2(r[2][1], r[1][1])
        else:
            c = -math.atan2(r[2][1], r[1][1])
        return a, b, c
    if flip:
        return _wrap_angle(a + math.pi), -b, _wrap_angle(c + math.pi)
    return a, b, c


def _solve_orientation(link3_rot, target_rot, wrist_flip=False):
    """Closed-form joint4/joint5/joint6 producing ``target_rot`` given the
    orientation joint1/joint2/joint3 already established. joint4 and
    joint6's URDF axes are both literally ``(-1,0,0)``, i.e. ``Rx(-q)``, and
    joint5's is ``(0,1,0)``, i.e. standard ``Ry(q)`` -- so the remaining
    rotation ``link3_rot.T @ link6_rot == Rx(-q4) @ Ry(q5) @ Rx(-q6)``, an
    X-Y-X proper Euler form with ``a=-q4, b=q5, c=-q6``. ``link6_rot`` is
    recovered from ``target_rot`` by undoing the fixed tool rotation
    (``target_rot`` is ``gripper_tcp``'s orientation, not ``link_6``'s)."""
    link6_rot = _mat_mat(target_rot, _TOOL_ROT_T)
    r_wrist = _mat_mat(_mat_transpose(link3_rot), link6_rot)
    a, b, c = _extract_xyx(r_wrist, flip=wrist_flip)
    return -a, b, -c


# --- public IK ---------------------------------------------------------------


def ik(target_xyz_m, target_rot, elbow_up=True, wrist_flip=False):
    """Closed-form IK. ``target_xyz_m``/``target_rot`` are the desired
    ``gripper_tcp`` position and 3x3 rotation matrix (columns = its X/Y/Z
    axes) in ``base``.

    Unlike SO-100's ``ik()`` (5-DOF, elevation + roll), this arm's spherical
    wrist gives full 3-DOF orientation freedom, so the caller must supply a
    complete target orientation, not just an elevation angle -- see
    ``grasp.orientation_from_stick_axis`` for building one from a desired
    stick direction plus a free roll.

    Two independent binary branch choices, like any 6-DOF elbow-manipulator
    solve: ``elbow_up`` (joint2/joint3, position) and ``wrist_flip``
    (joint4/joint5/joint6, orientation -- see ``_extract_xyx``). A caller
    that only cares "is this reachable at all" should try all four
    combinations, exactly as ``envelope.is_reachable`` does.

    Returns the 6 joint angles (radians, ``CHAIN``/``JOINT_NAMES`` order) on
    success, or raises Unreachable with a specific reason (never returns a
    silently-wrong answer).
    """
    wrist_center = _wrist_center_from_target(target_xyz_m, target_rot)
    q1, q2, q3 = _solve_position(wrist_center, elbow_up=elbow_up)
    _pos3, rot3 = _link3_fk(q1, q2, q3)
    q4, q5, q6 = _solve_orientation(rot3, target_rot, wrist_flip=wrist_flip)

    joints = (q1, q2, q3, q4, q5, q6)
    # Epsilon set to ~0.06 deg, not float-noise-tight -- matches
    # so_arm_100_kinematics.chain.ik()'s own reasoning: absorbs rounding
    # from a 2-decimal-degree yaml round trip without hiding a real
    # out-of-limits solution.
    limit_eps = 1e-3
    for (name, _xyz, _axis, lo, hi), q in zip(CHAIN, joints):
        if not (lo - limit_eps <= q <= hi + limit_eps):
            raise Unreachable(f"{name} solution {math.degrees(q):.2f} deg exceeds limits")
    return joints


def translate_holding_wrist(joints_rad, delta_xyz_m, elbow_up=True, iterations=6):
    """Move ``gripper_tcp`` by ``delta_xyz_m`` (world frame) while holding
    ``joint4``/``joint5``/``joint6`` EXACTLY at their current values --
    only ``joint1``/``joint2``/``joint3`` are re-solved. An approximate,
    deliberately DIFFERENT alternative to ``ik()`` for a small local move:
    added 2026-08-25 as the fallback for exactly the case ``ik()`` (and
    MoveIt's own numerical Cartesian planner) handle badly.

    Why this exists, not just "call ik() with the same target_rot"
    -----------------------------------------------------------------
    Requesting the EXACT same orientation at a nearby position is only
    "the same problem, slightly moved" far from a singularity. Near one
    (this arm's spherical wrist: ``sin(joint5) == 0`` makes ``joint4``/
    ``joint6`` mutually redistributable, module docstring point 3 /
    ``_extract_xyx``'s own docstring), it is not: ``joint2``/``joint3``
    shift a little to reach the new position, which rotates ``link_3``'s
    own frame a little, and because ``joint4``/``joint5``/``joint6`` are
    measured RELATIVE to that frame, reproducing the exact SAME absolute
    orientation from a slightly rotated ``link_3`` can require a
    wildly different individual ``joint4``/``joint6`` split -- confirmed
    directly (not assumed): for this arm's own hand-tuned ``lower`` pose,
    ``ik()`` needs joint4/joint6 near (104, -143) deg to hold the exact
    orientation just 5cm higher, versus the pose's own (175, 146). MoveIt's
    numerical Cartesian planner hits the exact same wall from the numerical
    side, and can even find a way to interpolate SMOOTHLY across that huge
    a change (found live: joint6 drifting +146 -> -146 deg, well under a
    degree per waypoint, over an ostensibly-tiny 5cm cartesian nudge --
    "successful" by every check MoveIt runs, and still a ~290deg wrist
    spin). See ``motion.MotionController._reject_wild_cartesian_swing``
    for where that gets caught on the ROS side.

    This function sidesteps the whole problem instead of solving it: if
    ``joint4``/``joint5``/``joint6`` are not allowed to move AT ALL, there
    is nothing to redistribute. The trade is TCP position accuracy, not
    safety -- since ``joint4``/``joint5``/``joint6`` no longer track the
    "requested" orientation exactly (link_3's own small rotation still
    reaches the caller, just unfought), the returned pose's orientation
    drifts by a small, second-order amount from the nominal target, refined
    by fixed-point iteration (recomputing the wrist-centre target from the
    ACTUAL orientation reached so far, each pass) until the resulting TCP
    position converges -- ~17mm error after 0 iterations for a 5cm lift at
    this arm's own worst-observed pose, ~4mm after 2, geometrically
    shrinking further with each additional one; the default of 6 leaves it
    well under 1mm. Good enough for "lift the stick clear of the feeder,
    then let the next step's own from-scratch, collision-checked plan take
    over" -- NOT a substitute for ``ik()`` wherever real precision or an
    intentional orientation change is actually needed.

    Raises :class:`Unreachable` if the wrist centre for the (approximate)
    target ever falls outside the position sub-chain's own reach envelope.
    """
    _pos, rot = fk(joints_rad)
    target_pos = tuple(p + d for p, d in zip(_pos, delta_xyz_m))
    q4, q5, q6 = joints_rad[3], joints_rad[4], joints_rad[5]
    joints = joints_rad
    for _ in range(iterations):
        wrist_center = _wrist_center_from_target(target_pos, rot)
        q1, q2, q3 = _solve_position(wrist_center, elbow_up=elbow_up)
        joints = (q1, q2, q3, q4, q5, q6)
        _new_pos, rot = fk(joints)
    return joints
