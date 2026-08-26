"""Robot mirror rig geometry. BLENDER_ADDON_PLAN.md Sec 10.4 / Phase E.

Pure Python -- no ``bpy`` -- like the rest of ``core/``, so it is testable
outside Blender. ``ops/mirror.py`` is the ``bpy``-facing half that turns
this into a scene object.

Multi-robot (docs/STATUS.md "Multi-robot support"): this module only ever
calls the given robot's own vendored ``fk()`` -- never so_arm_100's
specifically -- so it works unmodified for any robot whose ``fk()`` has the
same shape: loop over ``zip(CHAIN, joint_angles_rad)``, then unconditionally
add ONE fixed final transform (translation, and optionally rotation) once,
after the loop, regardless of how many joints were actually given. so_arm_100
and kr10_r900_2 both have this shape (confirmed by reading each), even
though kr10_r900_2's fixed transform is a composed (translation, rotation)
pair with no plain constant name exported for it, unlike so_arm_100's own
``EE_OFFSET``. Rather than depend on a named constant that not every robot
exposes, the fixed offset is read off ``fk(())`` itself (see
:func:`_tool_offset`'s own docstring) -- a call into the robot's own public,
tested ``fk()``, never a re-derivation of its math. A 6-DOF arm's chain is
simply longer; nothing here is hardcoded to 5 joints.
"""


def _rot_apply(rot, v):
    return (
        rot[0][0] * v[0] + rot[0][1] * v[1] + rot[0][2] * v[2],
        rot[1][0] * v[0] + rot[1][1] * v[1] + rot[1][2] * v[2],
        rot[2][0] * v[0] + rot[2][1] * v[1] + rot[2][2] * v[2],
    )


def _tool_offset(fk):
    """The fixed translation ``fk()`` adds, unconditionally, once after its
    joint loop -- derived by calling ``fk(())`` (zero joints): the loop
    (``for (...), q in zip(CHAIN, joint_angles_rad)``) then runs zero times,
    leaving position at the origin and rotation at identity, so whatever
    ``fk`` returns at that point *is* the fixed final transform in isolation
    -- no named ``EE_OFFSET``-style constant needed, and nothing here
    re-derives or assumes what that transform actually is (confirmed
    empirically to match so_arm_100's own ``EE_OFFSET`` exactly; works
    identically for kr10_r900_2, whose composed tool transform has no
    exported constant at all).
    """
    pos, _rot = fk(())
    return pos


def joint_frames(kinematics_module, joint_angles_rad):
    """The origin (``base_link`` for so_arm_100, ``base`` for kr10_r900_2 --
    each robot's own URDF root, metres) of every joint frame in
    ``kinematics_module``'s chain, in order, plus the true end-effector
    position last -- ``len(joint_angles_rad) + 1`` points, forming that many
    segments a preview rig can draw as bones.

    Not a second FK implementation: every point comes directly from
    ``kinematics_module.fk()`` itself, called on successively longer
    prefixes of ``joint_angles_rad``. A shorter prefix makes ``fk()``'s own
    joint loop stop exactly there -- the position/rotation it has
    accumulated at that point already *is* the correct partial-chain frame.
    ``fk()``'s only postprocessing step, adding the fixed tool offset
    unconditionally at the end (see :func:`_tool_offset`), is correct for
    the full chain (the true TCP) but wrong for a shorter prefix (that
    offset belongs only after the final joint), so it is subtracted back
    out for every prefix but the last. Any robot module whose ``fk()`` has
    this shape gets this for free -- so_arm_100 and kr10_r900_2 both do.
    """
    fk = kinematics_module.fk
    tool_offset = _tool_offset(fk)

    points = [(0.0, 0.0, 0.0)]
    total = len(joint_angles_rad)
    for count in range(1, total + 1):
        pos, rot = fk(joint_angles_rad[:count])
        if count < total:
            offset = _rot_apply(rot, tool_offset)
            pos = tuple(p - o for p, o in zip(pos, offset))
        points.append(pos)
    return points
