"""GPU viewport overlay: build volume, robot base box, and per-stick status
colours. BLENDER_ADDON_PLAN.md Sec 10.4.

Kept in its own module with a hard on/off switch (``scene.so100.show_overlay``)
per Sec 10.4's explicit warning: *"draw handlers are the most common cause of
addon crashes across Blender versions."* Consequences of that, followed
throughout this module:

* The draw callback returns immediately, before touching any geometry, if
  the toggle is off, the addon's scene properties aren't present, or there
  is no build mesh yet -- never assume the state it wants is there.
* ``register()``/``unregister()`` are idempotent and paired: registering
  twice replaces rather than duplicates the handle, and unregistering when
  nothing is registered is a no-op. There is no code path that can leave a
  stale handler installed after the addon is disabled.
* Positions are read straight from ``build_mesh.data`` -- that object's
  ``matrix_world`` is always identity (``ops/design.py`` sets it explicitly),
  so its vertex coordinates already *are* Blender world-space, needing no
  further transform for a ``POST_VIEW`` callback.
* Build-mesh edge *i* corresponds to ``scene.so100.sticks[i]`` by
  construction (``ops/design.py`` appends both lists in the same
  ``result.sticks`` order on every extraction) -- reused here rather than
  duplicating stick geometry into the property group.

⚠ Not exercised by the automated test suite: ``gpu`` drawing needs a real
GPU context, unavailable under ``blender --background`` (confirmed: even
``gpu.shader.from_builtin`` raises ``GPU functions ... requires the gpu
module to be initialized`` there). Manually smoke-tested with a real,
windowed Blender instance instead (register -> extract -> force a redraw ->
confirm no error and no stale handler after unregister).
"""

import bpy
import gpu
from gpu_extras.batch import batch_for_shader

from ..core import robots as core_robots
from ..core import state as core_state
from ..core import transform as core_transform

_handle = None

_VOLUME_COLOR = (0.7, 0.7, 0.75, 0.6)
_BASE_BOX_COLOR = (0.6, 0.45, 0.3, 0.6)

_STATUS_COLOR = {
    core_state.STATUS_PENDING: (0.55, 0.55, 0.55, 1.0),
    core_state.STATUS_BUILDABLE: (0.15, 0.85, 0.2, 1.0),
    core_state.STATUS_IMPOSSIBLE: (0.95, 0.15, 0.15, 1.0),
    core_state.STATUS_PLACED: (0.2, 0.45, 0.95, 1.0),
    core_state.STATUS_FAILED: (0.95, 0.55, 0.05, 1.0),
    core_state.STATUS_SKIPPED: (0.35, 0.35, 0.35, 1.0),
}
_DEFAULT_COLOR = (0.8, 0.8, 0.8, 1.0)

_VOLUME_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def _box_edge_points(lo, hi, base_matrix_world, scale_length):
    """12-edge wireframe line list for an axis-aligned box given in the
    robot's own frame, transformed into Blender world space."""
    corners_local = (
        (lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]), (hi[0], hi[1], lo[2]), (lo[0], hi[1], lo[2]),
        (lo[0], lo[1], hi[2]), (hi[0], lo[1], hi[2]), (hi[0], hi[1], hi[2]), (lo[0], hi[1], hi[2]),
    )
    corners = [
        core_transform.robot_to_blender(c, base_matrix_world, scale_length)
        for c in corners_local
    ]
    points = []
    for a, b in _VOLUME_EDGES:
        points.append(corners[a])
        points.append(corners[b])
    return points


def _volume_box_points(base_matrix_world, scale_length, robot_id):
    """The selected robot's own build volume (``core/robots.py``'s
    ``RobotProfile``), multi-robot support (docs/STATUS.md 2026-08-21) --
    previously always drew so_arm_100's box regardless of ``robot_id``.
    Returns ``[]`` (drawing nothing, not a crash or a made-up box) for a
    robot with no confirmed build volume yet -- neither registered robot
    hits that today, but a future one might before it earns real numbers.
    """
    profile = core_robots.get_robot(robot_id)
    if not profile.has_build_volume:
        return []
    return _box_edge_points(
        profile.build_volume_min_m, profile.build_volume_max_m,
        base_matrix_world, scale_length)


def _base_box_points(base_matrix_world, scale_length, robot_id):
    """The selected robot's own mounting pedestal (``RobotProfile.
    base_box_min_m/max_m``), 2026-08-23 user request -- a viewport
    reference only, drawn for whichever robot has one (kr10_r900_2 today;
    so_arm_100 has none, same ``[]``-means-"don't draw" convention as
    ``_volume_box_points`` above)."""
    profile = core_robots.get_robot(robot_id)
    if not profile.has_base_box:
        return []
    return _box_edge_points(
        profile.base_box_min_m, profile.base_box_max_m,
        base_matrix_world, scale_length)


def _stick_status_points_and_colors(props):
    obj = props.build_mesh
    if obj is None or obj.name not in bpy.data.objects:
        return [], []
    mesh = obj.data
    edges = mesh.edges
    sticks = props.sticks
    if len(edges) != len(sticks):
        return [], []  # stale relative to the stick list -- draw nothing rather than mismatched colours

    points, colors = [], []
    for index, edge in enumerate(edges):
        color = _STATUS_COLOR.get(sticks[index].status, _DEFAULT_COLOR)
        for vertex_index in edge.vertices:
            points.append(tuple(mesh.vertices[vertex_index].co))
            colors.append(color)
    return points, colors


def _draw():
    scene = bpy.context.scene
    if scene is None:
        return
    props = getattr(scene, "so100", None)
    if props is None or not props.show_overlay:
        return

    gpu.state.line_width_set(2.0)
    gpu.state.blend_set("ALPHA")

    if props.base_empty is not None and props.base_empty.name in bpy.data.objects:
        base_matrix = core_transform.to_tuple_4x4(props.base_empty.matrix_world)
        scale_length = scene.unit_settings.scale_length
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")

        volume_points = _volume_box_points(base_matrix, scale_length, props.robot_id)
        if volume_points:
            batch = batch_for_shader(shader, "LINES", {"pos": volume_points})
            shader.uniform_float("color", _VOLUME_COLOR)
            batch.draw(shader)

        base_box_points = _base_box_points(base_matrix, scale_length, props.robot_id)
        if base_box_points:
            batch = batch_for_shader(shader, "LINES", {"pos": base_box_points})
            shader.uniform_float("color", _BASE_BOX_COLOR)
            batch.draw(shader)

    stick_points, stick_colors = _stick_status_points_and_colors(props)
    if stick_points:
        shader = gpu.shader.from_builtin("FLAT_COLOR")
        batch = batch_for_shader(shader, "LINES", {"pos": stick_points, "color": stick_colors})
        batch.draw(shader)

    gpu.state.blend_set("NONE")


def register():
    global _handle
    if _handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_handle, "WINDOW")
    _handle = bpy.types.SpaceView3D.draw_handler_add(_draw, (), "WINDOW", "POST_VIEW")


def unregister():
    global _handle
    if _handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_handle, "WINDOW")
        _handle = None
