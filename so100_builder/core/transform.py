"""Blender world space <-> base_link metres.

BLENDER_ADDON_PLAN.md Sec 8. This module is deliberately free of ``bpy`` and
``mathutils`` imports so it can be unit-tested in a bare interpreter as well
as inside Blender. It accepts any object that supports ``m[row][col]``
indexing, which includes ``mathutils.Matrix`` and a plain nested tuple.

Two facts that are load-bearing and are commonly "fixed" by mistake:

* Blender and ROS are **both right-handed and Z-up**. There is no axis flip
  between them. Do not add one.
* ``scene.unit_settings.scale_length`` applies to **translation only**, and
  only **after** the inverse transform into the base empty's frame
  (constraint B6). Applying it before, or to the rotation part, silently
  scales the whole design by a factor most users never notice because
  scale_length is 1.0 by default.

Quaternion order conversion (Blender ``(w,x,y,z)`` vs ROS ``(x,y,z,w)``,
constraint B5) lives here and nowhere else -- though the build file's
base/tip stick format sidesteps it for every case the addon currently has.
"""

import math

IDENTITY_4X4 = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


class SingularMatrix(Exception):
    """Raised when a matrix cannot be inverted (degenerate object scale)."""


def to_tuple_4x4(m):
    """Normalise anything indexable as ``m[row][col]`` into a nested tuple.

    ``mathutils.Matrix`` rows are row-vectors under this indexing, which is
    the same convention as the tuples used throughout this module.
    """
    return tuple(tuple(float(m[r][c]) for c in range(4)) for r in range(4))


def mat_mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
        for i in range(4)
    )


def invert_4x4(m):
    """General 4x4 inverse by Gauss-Jordan elimination with partial pivoting.

    Written out rather than delegating to ``mathutils.Matrix.inverted()`` so
    this module stays importable outside Blender. A general inverse (not the
    cheaper rigid-transform shortcut) is required because the base empty may
    legitimately carry a non-uniform scale from its parent (constraint B6).
    """
    m = to_tuple_4x4(m)
    aug = [list(m[r]) + [1.0 if c == r else 0.0 for c in range(4)] for r in range(4)]

    for col in range(4):
        pivot_row = max(range(col, 4), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot_row][col]) < 1e-12:
            raise SingularMatrix(
                "matrix is not invertible -- the SO100_Base empty (or a parent) "
                "probably has a zero scale on one axis"
            )
        aug[col], aug[pivot_row] = aug[pivot_row], aug[col]

        pivot = aug[col][col]
        aug[col] = [v / pivot for v in aug[col]]

        for r in range(4):
            if r == col:
                continue
            factor = aug[r][col]
            if factor:
                aug[r] = [v - factor * p for v, p in zip(aug[r], aug[col])]

    return tuple(tuple(row[4:]) for row in aug)


def apply_point(m, v):
    """Transform a point (w=1) by a 4x4."""
    x, y, z = v[0], v[1], v[2]
    w = m[3][0] * x + m[3][1] * y + m[3][2] * z + m[3][3]
    if abs(w) < 1e-12:
        raise SingularMatrix("point projected to infinity")
    return (
        (m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3]) / w,
        (m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3]) / w,
        (m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]) / w,
    )


def apply_direction(m, v):
    """Transform a direction (w=0) by a 4x4 -- no translation, no scale_length."""
    x, y, z = v[0], v[1], v[2]
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def blender_to_robot(vec_world, base_matrix_world, scale_length=1.0):
    """Blender world-space point -> metres in ``base_link``.

    ``base_matrix_world`` is the SO100_Base empty's ``matrix_world``. The
    empty *is* base_link, so moving it repositions the whole design relative
    to the robot with no re-authoring (Sec 9.1).
    """
    local = apply_point(invert_4x4(base_matrix_world), vec_world)
    return (local[0] * scale_length, local[1] * scale_length, local[2] * scale_length)


def blender_to_robot_batch(points_world, base_matrix_world, scale_length=1.0):
    """Same as ``blender_to_robot`` for many points, inverting the base matrix
    once. Extraction runs this over every mesh vertex, so the saving is real."""
    inv = invert_4x4(base_matrix_world)
    out = []
    for p in points_world:
        local = apply_point(inv, p)
        out.append(
            (local[0] * scale_length, local[1] * scale_length, local[2] * scale_length)
        )
    return out


def robot_to_blender(vec_robot_m, base_matrix_world, scale_length=1.0):
    """Metres in ``base_link`` -> Blender world space. The exact inverse of
    ``blender_to_robot``; used to draw overlays and the derived build mesh
    back in the user's scene."""
    if abs(scale_length) < 1e-12:
        raise SingularMatrix("scene.unit_settings.scale_length is zero")
    local = (
        vec_robot_m[0] / scale_length,
        vec_robot_m[1] / scale_length,
        vec_robot_m[2] / scale_length,
    )
    return apply_point(to_tuple_4x4(base_matrix_world), local)


# --- quaternion order (constraint B5) ---------------------------------------
# The ONLY place in the addon that reorders quaternion components. The build
# file uses base/tip endpoints instead of poses precisely so this stays
# unused on the hot path -- keep it that way.


def quat_blender_to_ros(q_wxyz):
    """Blender ``(w, x, y, z)`` -> ROS ``(x, y, z, w)``."""
    w, x, y, z = q_wxyz
    return (x, y, z, w)


def quat_ros_to_blender(q_xyzw):
    """ROS ``(x, y, z, w)`` -> Blender ``(w, x, y, z)``."""
    x, y, z, w = q_xyzw
    return (w, x, y, z)


# --- small vector helpers shared by core/ -----------------------------------
# core/ is numpy-free (constraint B4) and must not depend on mathutils either,
# so these live here rather than being re-typed in every module.


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_length(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def v_dist(a, b):
    return v_length(v_sub(a, b))


def v_normalized(a):
    n = v_length(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def angle_between(a, b):
    """Angle in radians between two vectors, clamped against float noise."""
    na, nb = v_length(a), v_length(b)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    c = max(-1.0, min(1.0, v_dot(a, b) / (na * nb)))
    return math.acos(c)
