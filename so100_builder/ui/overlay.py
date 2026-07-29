"""GPU viewport overlay: build volume + per-stick status colours.
BLENDER_ADDON_PLAN.md Sec 10.4.

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

from ..core import state as core_state
from ..core import transform as core_transform
from ..kinematics.constants import BUILD_VOLUME_MAX_M, BUILD_VOLUME_MIN_M

_handle = None

_VOLUME_COLOR = (0.7, 0.7, 0.75, 0.6)

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


def _volume_box_points(base_matrix_world, scale_length):
    lo, hi = BUILD_VOLUME_MIN_M, BUILD_VOLUME_MAX_M
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
        volume_points = _volume_box_points(base_matrix, scale_length)
        if volume_points:
            shader = gpu.shader.from_builtin("UNIFORM_COLOR")
            batch = batch_for_shader(shader, "LINES", {"pos": volume_points})
            shader.uniform_float("color", _VOLUME_COLOR)
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
