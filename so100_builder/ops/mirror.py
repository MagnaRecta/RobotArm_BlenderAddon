"""Robot mirror rig operators & scene wiring. BLENDER_ADDON_PLAN.md Sec 10.4,
Phase E.

All ``bpy`` access happens here, matching ``ops/design.py``'s own rule --
``core/mirror.py`` stays pure so its FK-derivation math is testable outside
Blender. Everything here runs inside an operator's ``execute()`` on the
main thread (constraint B2); nothing here is driven by property ``update=``
callbacks, deliberately -- ``properties.py`` stays free of any dependency on
``ops/``, matching the existing layering (see ``ops/design.py``'s own
one-way import of ``..properties``, never the reverse).
"""

import bpy
from bpy.props import IntProperty
from bpy.types import Operator

from ..core import mirror as core_mirror
from ..core import robots as core_robots
from ..core import transform as core_transform

MIRROR_OBJECT_NAME = "SO100_Mirror"


def _segments(point_count):
    """Consecutive bone pairs for ``point_count`` points -- point 0 is the
    robot's own URDF-root origin, the last is the true TCP, in between are
    the joint frames (``core.mirror.joint_frames()``'s own docstring).
    Robot-agnostic on purpose: so_arm_100's 5 joints give 6 points/5
    segments, kr10_r900_2's 6 joints give 7 points/6 segments -- neither is
    hardcoded here.
    """
    return [(i, i + 1) for i in range(point_count - 1)]


def _ordered_items(props):
    return sorted((item for item in props.sticks if item.order >= 0),
                 key=lambda item: item.order)


def _stick_base_tip_robot(context, props, edge_index):
    """The stick at ``build_mesh`` edge ``edge_index``, as
    ``(base, tip)`` in robot (``base_link``) coordinates -- read back from
    the build mesh rather than re-deriving from the design, since the
    build mesh already reflects any ``flip`` (Sec 6 makes 'base' structural
    once an order exists). ``None`` if the build mesh is missing or stale.
    """
    obj = props.build_mesh
    if obj is None or obj.name not in bpy.data.objects:
        return None
    if props.base_empty is None or props.base_empty.name not in bpy.data.objects:
        return None
    edges = obj.data.edges
    if edge_index >= len(edges) or len(edges) != len(props.sticks):
        return None

    scale_length = context.scene.unit_settings.scale_length
    base_matrix = core_transform.to_tuple_4x4(props.base_empty.matrix_world)
    v0, v1 = edges[edge_index].vertices
    a = tuple(obj.data.vertices[v0].co)
    b = tuple(obj.data.vertices[v1].co)
    return (
        core_transform.blender_to_robot(a, base_matrix, scale_length),
        core_transform.blender_to_robot(b, base_matrix, scale_length),
    )


def _ensure_mirror_object(context):
    obj = bpy.data.objects.get(MIRROR_OBJECT_NAME)
    if obj is None or obj.name not in bpy.data.objects:
        mesh = bpy.data.meshes.new(MIRROR_OBJECT_NAME)
        obj = bpy.data.objects.new(MIRROR_OBJECT_NAME, mesh)
        context.collection.objects.link(obj)
        # Preview only -- never the thing the user hand-edits, and never in
        # a render (matches build_mesh's own hide_select convention).
        obj.hide_select = True
        obj.hide_render = True
    return obj


def _hide_mirror_object():
    obj = bpy.data.objects.get(MIRROR_OBJECT_NAME)
    if obj is not None:
        obj.hide_set(True)


def _solve_placement(robot_id, kinematics_module, base, tip):
    """Joint angles placing a stick from ``base`` to ``tip``, for whichever
    robot is selected. kr10_r900_2's round stock has a genuinely free roll
    DOF (core/validate.py's own note on the same distinction) so a single
    fixed-roll attempt under-reports reachability for it -- use its own
    ``solve_stick_placement_any_roll`` (returns a 4-tuple; only the joints
    are needed here) instead of the shared ``solve_stick_placement`` name.
    """
    if robot_id == core_robots.KR10_R900_2_ID:
        joints, _roll_rad, _elbow_up, _wrist_flip = (
            kinematics_module.solve_stick_placement_any_roll(base, tip))
        return joints
    return kinematics_module.solve_stick_placement(base, tip)


def update_mirror_rig(context, props):
    """Pose (or hide) the mirror rig for the current ``mirror_index``.
    Returns a short status string for the panel / operator report.
    """
    if not props.show_mirror:
        _hide_mirror_object()
        return ""

    ordered = _ordered_items(props)
    if not ordered:
        _hide_mirror_object()
        return "No build order yet -- Compute Build Order first."

    index = max(0, min(props.mirror_index, len(ordered) - 1))
    if index != props.mirror_index:
        props.mirror_index = index
    item = ordered[index]

    edge_index = next(
        (i for i, stick in enumerate(props.sticks) if stick.stick_id == item.stick_id),
        None,
    )
    pair = None if edge_index is None else _stick_base_tip_robot(context, props, edge_index)
    if pair is None:
        _hide_mirror_object()
        return "Build mesh is out of sync -- re-run Extract Sticks."
    base, tip = pair

    kinematics_module = core_robots.get_robot(props.robot_id).kinematics
    try:
        joints = _solve_placement(props.robot_id, kinematics_module, base, tip)
    except Exception as exc:
        # Broad on purpose: a not-yet-vendored robot (kinematics/kr10_r900_2/)
        # raises a plain RuntimeError the moment any attribute is touched,
        # not necessarily the contract's own Unreachable -- this rig must
        # degrade cleanly (a status message) for either kind of failure,
        # never crash the operator. See core/validate.py's generic path for
        # the same reasoning.
        _hide_mirror_object()
        return "%s: %s" % (item.stick_id, exc)

    points_robot = core_mirror.joint_frames(kinematics_module, joints)
    scale_length = context.scene.unit_settings.scale_length
    base_matrix = core_transform.to_tuple_4x4(props.base_empty.matrix_world)
    points_blender = [
        core_transform.robot_to_blender(p, base_matrix, scale_length)
        for p in points_robot
    ]

    obj = _ensure_mirror_object(context)
    mesh = obj.data
    mesh.clear_geometry()
    mesh.from_pydata(points_blender, _segments(len(points_blender)), [])
    mesh.update()
    # robot_to_blender already produced world-space coordinates (matches
    # rebuild_build_mesh's own convention).
    obj.matrix_world.identity()
    obj.hide_set(False)

    return "%s -- build order %d of %d" % (item.stick_id, index + 1, len(ordered))


class SO100_OT_mirror_toggle(Operator):
    """Show or hide the robot mirror rig, posing it if newly shown."""

    bl_idname = "so100.mirror_toggle"
    bl_label = "Toggle Robot Mirror"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def execute(self, context):
        props = context.scene.so100
        props.show_mirror = not props.show_mirror
        props.mirror_status = update_mirror_rig(context, props)
        return {"FINISHED"}


class SO100_OT_mirror_step(Operator):
    """Move the mirror rig's build-order position by +/-1 (wrapping)."""

    bl_idname = "so100.mirror_step"
    bl_label = "Step Robot Mirror"
    bl_options = {"REGISTER"}

    direction: IntProperty(default=1)

    @classmethod
    def poll(cls, context):
        props = context.scene.so100
        return props.show_mirror and any(item.order >= 0 for item in props.sticks)

    def execute(self, context):
        props = context.scene.so100
        ordered = _ordered_items(props)
        if not ordered:
            return {"CANCELLED"}
        props.mirror_index = (props.mirror_index + self.direction) % len(ordered)
        props.mirror_status = update_mirror_rig(context, props)
        return {"FINISHED"}


_CLASSES = (SO100_OT_mirror_toggle, SO100_OT_mirror_step)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
    obj = bpy.data.objects.get(MIRROR_OBJECT_NAME)
    if obj is not None:
        mesh = obj.data
        bpy.data.objects.remove(obj)
        bpy.data.meshes.remove(mesh)
