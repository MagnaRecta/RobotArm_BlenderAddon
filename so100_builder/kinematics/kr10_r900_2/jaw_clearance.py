"""Jaw-clearance check: can the gripper actually reach into a joint without
clipping already-placed structure -- port of
``so_arm_100_kinematics.jaw_clearance``, same model, same math, only the
constants differ (round stock here vs. SO-100's square stock). See that
module's own docstring for the full rationale; not repeated here.

## What differs from SO-100's version

* **The stick capsule radius** (``STICK_COLLISION_RADIUS_M``) uses the round
  stock's own radius directly, no circumscribed-square correction needed.
* **``JAW_RADIUS_M``/``JAW_CONTACT_HALF_LENGTH_M`` are rougher estimates**
  than SO-100's own (see ``constants.py``'s own caveat on both) -- treat a
  "clear" result here as even more provisional than SO-100's own
  not-yet-measured caveat already implies.

## What this module does NOT decide

Identical to ``so_arm_100_kinematics.jaw_clearance``: which already-placed
sticks are this joint's own expected neighbours (caller's job -- exclude
them before calling); self-collision, the mount platform, the table, or
MoveIt's own path planning (MoveIt has the last word regardless).
"""

import math

from .constants import GRASP_OFFSET_M, JAW_RADIUS_M, STICK_COLLISION_RADIUS_M
from .grasp import grasp_target


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def segment_distance(p1, q1, p2, q2):
    """Closest distance between segment ``p1``-``q1`` and segment
    ``p2``-``q2`` in 3D -- identical implementation to
    ``so_arm_100_kinematics.jaw_clearance.segment_distance`` (Ericson,
    *Real-Time Collision Detection* Sec 5.1.9)."""
    d1 = _v_sub(q1, p1)
    d2 = _v_sub(q2, p2)
    r = _v_sub(p1, p2)
    a = _dot(d1, d1)
    e = _dot(d2, d2)
    f = _dot(d2, r)
    eps = 1e-12

    if a <= eps and e <= eps:
        s, t = 0.0, 0.0
    elif a <= eps:
        s = 0.0
        t = _clamp(f / e, 0.0, 1.0)
    else:
        c = _dot(d1, r)
        if e <= eps:
            t = 0.0
            s = _clamp(-c / a, 0.0, 1.0)
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = _clamp((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = _clamp(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = _clamp((b - c) / a, 0.0, 1.0)

    c1 = _v_add(p1, _v_scale(d1, s))
    c2 = _v_add(p2, _v_scale(d2, t))
    return math.sqrt(_dot(_v_sub(c1, c2), _v_sub(c1, c2)))


def jaw_swept_capsule(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M):
    """The ``(grip_xyz_m, vertex_xyz_m)`` capsule endpoints for placing a
    stick with these physical ends -- the segment the jaws occupy."""
    return grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m), tuple(base_xyz_m)


def check_jaw_clearance(base_xyz_m, tip_xyz_m, placed_sticks,
                         grasp_offset_m=GRASP_OFFSET_M,
                         jaw_radius_m=JAW_RADIUS_M,
                         stick_collision_radius_m=STICK_COLLISION_RADIUS_M):
    """Can the jaws reach this placement's joint without clipping
    already-placed structure? Identical contract to
    ``so_arm_100_kinematics.jaw_clearance.check_jaw_clearance`` -- see that
    function's own docstring for the full parameter/return description.
    """
    jaw_grip, jaw_vertex = jaw_swept_capsule(base_xyz_m, tip_xyz_m, grasp_offset_m)

    tightest_index = None
    tightest_clearance = None
    for index, (p_base, p_tip) in enumerate(placed_sticks):
        dist = segment_distance(jaw_grip, jaw_vertex, p_base, p_tip)
        clearance = dist - jaw_radius_m - stick_collision_radius_m
        if tightest_clearance is None or clearance < tightest_clearance:
            tightest_clearance = clearance
            tightest_index = index

    if tightest_index is None:
        return True, None, None, None

    if tightest_clearance < 0.0:
        reason = (
            "jaw capsule for base=%r tip=%r overlaps placed_sticks[%d] by "
            "%.1f mm" % (base_xyz_m, tip_xyz_m, tightest_index, -tightest_clearance * 1000.0)
        )
        return False, reason, tightest_clearance, tightest_index

    return True, None, tightest_clearance, tightest_index
