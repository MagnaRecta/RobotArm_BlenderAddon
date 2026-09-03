"""GPU viewport overlay: build volume, robot base box, per-stick status
colours, and the Check By Eye selected-edge highlight.
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

import bmesh
import bpy
import gpu
from gpu_extras.batch import batch_for_shader

from ..core import robots as core_robots
from ..core import state as core_state
from ..core import transform as core_transform

_handle = None

_VOLUME_COLOR = (0.7, 0.7, 0.75, 0.6)
_BASE_BOX_COLOR = (0.6, 0.45, 0.3, 0.6)
# Check By Eye (2026-08-24 user request): a distinct, high-contrast colour
# for whichever build-mesh edge is currently selected -- bright yellow
# reads clearly against every _STATUS_COLOR below and both box colours.
_SELECTED_EDGE_COLOR = (1.0, 0.95, 0.0, 1.0)
_SELECTED_EDGE_WIDTH = 4.0
# "Highlight Previous Sticks" (2026-08-24 user request) -- a muted gold,
# same hue family as the bright current-edge yellow above (reads as "part
# of the same highlighting system, just already built") but visibly dimmer
# so the CURRENT stick still stands out as the most prominent one. Not
# reused from _STATUS_COLOR's own orange (STATUS_FAILED) to avoid reading
# as "these sticks failed".
_PREVIOUS_EDGE_COLOR = (0.85, 0.72, 0.1, 0.9)
_PREVIOUS_EDGE_WIDTH = 3.0

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


def _volume_box_points(base_matrix_world, scale_length, robot_id, plate_offset_m=0.0):
    """The selected robot's own build volume (``core/robots.py``'s
    ``RobotProfile``), multi-robot support (docs/STATUS.md 2026-08-21) --
    previously always drew so_arm_100's box regardless of ``robot_id``.
    Returns ``[]`` (drawing nothing, not a crash or a made-up box) for a
    robot with no confirmed build volume yet -- neither registered robot
    hits that today, but a future one might before it earns real numbers.

    ``plate_offset_m`` (2026-08-24, user request: "I want to be able to
    move that box from the addon, and then make the bottom of that box
    the build plate reference height") -- ``props.build_plate_height_mm``
    converted to metres, shifting the box's Z so its own bottom face is
    always exactly the height ``ops/design.py``'s own
    ``effective_ground_height_m()`` uses for grounding. The two were
    previously independent numbers (this box never moved when that field
    changed) that only happened to agree by coincidence for so_arm_100;
    passing the same value here is what makes moving the box and setting
    the plate height the SAME action rather than two that can disagree.
    """
    profile = core_robots.get_robot(robot_id)
    if not profile.has_build_volume:
        return []
    lo, hi = profile.build_volume_min_m, profile.build_volume_max_m
    lo = (lo[0], lo[1], lo[2] + plate_offset_m)
    hi = (hi[0], hi[1], hi[2] + plate_offset_m)
    return _box_edge_points(lo, hi, base_matrix_world, scale_length)


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


def _selected_edge_indices(props):
    """Indices of the build mesh's own CURRENTLY selected edges, read live
    from its edit-mode bmesh. Returns ``[]`` whenever the build mesh is not
    currently in Edit Mode (nothing selected, and ``bmesh.from_edit_mesh``
    would raise if the object has no live edit-mesh) -- Check By Eye is
    what puts it there in the first place. Shared by ``_selected_edge_
    points`` and ``_previous_edge_points`` so both agree on which edge is
    "current" without querying the live bmesh twice per redraw.
    """
    obj = props.build_mesh
    if obj is None or obj.name not in bpy.data.objects or obj.mode != "EDIT":
        return []
    bm = bmesh.from_edit_mesh(obj.data)
    return [edge.index for edge in bm.edges if edge.select]


def _selected_edge_points(props, selected_indices):
    """The build mesh's own CURRENTLY selected edges' endpoints -- 2026-08-24
    user request: highlight the edge Check By Eye selects, "and revert it
    when the user clicks something else in the viewer."

    Deliberately reads live selection state (``selected_indices``, from
    ``_selected_edge_indices``) fresh on every redraw rather than tracking
    "which stick did Check By Eye select" as its own bit of state: Blender
    already redraws after every click, and clicking a different edge (or
    empty space, clearing the selection) already updates the edit-mesh's
    own ``.select`` flags through Blender's normal selection handling --
    so the highlight following or disappearing needs no event handler,
    modal operator, or extra property of its own, only reading whatever is
    selected right now.
    """
    obj = props.build_mesh
    if not selected_indices or obj is None or obj.name not in bpy.data.objects:
        return []
    mesh = obj.data
    points = []
    for index in selected_indices:
        edge = mesh.edges[index]
        for vertex_index in edge.vertices:
            points.append(tuple(mesh.vertices[vertex_index].co))
    return points


def _previous_edge_points(props, selected_indices):
    """Every OTHER build-mesh edge that comes BEFORE the single currently
    selected one -- 2026-08-24 user request: "a checkbox before the check
    by eye button that makes all the previous edges highlighted", so the
    path already built up to this point is visible at a glance, not just
    the current stick alone. Off by default (``props.
    highlight_previous_sticks``) -- the caller checks that before calling
    this at all, so behaviour is unchanged unless the user opts in.

    "Before" means BUILD order (``item.order``) once one has been
    computed, falling back to extraction order otherwise -- the exact same
    fallback ``SO100_OT_step_stick`` already uses, so "previous" means the
    same thing here as it does when stepping through the list.

    Only activates for EXACTLY one selected edge -- with zero or several
    selected, "the current one" is ambiguous, so nothing extra is drawn
    (the ordinary per-edge selection highlight still shows whatever IS
    selected either way).
    """
    obj = props.build_mesh
    sticks = props.sticks
    if (
        len(selected_indices) != 1
        or obj is None or obj.name not in bpy.data.objects
        or len(obj.data.edges) != len(sticks)
    ):
        return []
    current_index = selected_indices[0]
    if not (0 <= current_index < len(sticks)):
        return []

    current_order = sticks[current_index].order
    if props.has_order and current_order >= 0:
        previous_indices = [
            i for i, item in enumerate(sticks)
            if i != current_index and item.order >= 0 and item.order < current_order
        ]
    else:
        previous_indices = list(range(current_index))

    mesh = obj.data
    points = []
    for index in previous_indices:
        edge = mesh.edges[index]
        for vertex_index in edge.vertices:
            points.append(tuple(mesh.vertices[vertex_index].co))
    return points


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

        volume_points = _volume_box_points(
            base_matrix, scale_length, props.robot_id,
            props.build_plate_height_mm / 1000.0)
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

    selected_indices = _selected_edge_indices(props)

    if props.highlight_previous_sticks:
        previous_points = _previous_edge_points(props, selected_indices)
        if previous_points:
            gpu.state.line_width_set(_PREVIOUS_EDGE_WIDTH)
            shader = gpu.shader.from_builtin("UNIFORM_COLOR")
            batch = batch_for_shader(shader, "LINES", {"pos": previous_points})
            shader.uniform_float("color", _PREVIOUS_EDGE_COLOR)
            batch.draw(shader)

    # Drawn AFTER "previous" so the current stick's brighter highlight
    # stays visually on top / most prominent of the two.
    selected_points = _selected_edge_points(props, selected_indices)
    if selected_points:
        gpu.state.line_width_set(_SELECTED_EDGE_WIDTH)
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")
        batch = batch_for_shader(shader, "LINES", {"pos": selected_points})
        shader.uniform_float("color", _SELECTED_EDGE_COLOR)
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
