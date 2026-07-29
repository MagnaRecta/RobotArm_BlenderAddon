"""Design-stage operators. BLENDER_ADDON_PLAN.md Sec 10.1, Phase A.

All ``bpy`` access happens here and in ``ui/`` -- ``core/`` stays pure so it
can be tested outside Blender. Everything in this module runs on the main
thread inside an operator's ``execute()``; nothing here ever touches ``bpy``
from a background thread (constraint B2).
"""

import bmesh
import bpy
from bpy.props import BoolProperty, IntProperty, StringProperty
from bpy.types import Operator
from bpy_extras.io_utils import ExportHelper

from ..core import sticks as core_sticks
from ..core import state as core_state
from ..core import transform as core_transform
from ..core import validate as core_validate
from ..properties import EDGE_ID_LAYER

# Sec 9.1: the empty named this marks base_link.
BASE_EMPTY_NAME = "SO100_Base"
BUILD_MESH_NAME = "SO100_BuildMesh"


def _report_error(operator, message):
    operator.report({"ERROR"}, message)
    return {"CANCELLED"}


# --- stable ids (Sec 9.2) ----------------------------------------------------


def ensure_edge_id_layer(mesh):
    """Fetch (creating if needed) the INT attribute layer on the EDGE domain.

    Edge *indices* are not stable across mesh edits, so the id has to live
    with the edge itself. An attribute layer is Blender's own mechanism for
    that and it survives the edits that renumber indices.
    """
    attribute = mesh.attributes.get(EDGE_ID_LAYER)
    if attribute is None:
        attribute = mesh.attributes.new(name=EDGE_ID_LAYER, type="INT", domain="EDGE")
    elif attribute.domain != "EDGE" or attribute.data_type != "INT":
        mesh.attributes.remove(attribute)
        attribute = mesh.attributes.new(name=EDGE_ID_LAYER, type="INT", domain="EDGE")
    return attribute


def assign_stable_ids(mesh, props):
    """Give every edge a stable id, minting new ones only where needed.

    Returns ``(ids, reassigned_count)`` where ``ids`` is one string id per
    edge, in current edge-index order.
    """
    attribute = ensure_edge_id_layer(mesh)
    raw = [attribute.data[i].value for i in range(len(mesh.edges))]

    numbers, next_number, reassigned = core_state.allocate_ids(
        raw, props.next_stick_number
    )
    for index, number in enumerate(numbers):
        if attribute.data[index].value != number:
            attribute.data[index].value = number
    props.next_stick_number = next_number

    return [core_state.format_id(n) for n in numbers], reassigned


# --- mesh -> base_link metres (Sec 5.1, constraints B6) ----------------------


def read_design_mesh(context, props):
    """Return ``(points_m, edge_pairs, edge_ids, reassigned)`` in base_link.

    Modifiers are deliberately **not** evaluated: the stable-id attribute
    layer lives on the original mesh, and a generative modifier would produce
    edges that have no id and cannot be tracked between sessions. Apply
    modifiers before extracting if you need them.
    """
    obj = props.design_mesh
    mesh = obj.data
    base = props.base_empty

    edge_ids, reassigned = assign_stable_ids(mesh, props)

    # B6: always read matrix_world (never object.location) and apply the
    # scene's unit scale.
    scale_length = context.scene.unit_settings.scale_length
    world = obj.matrix_world
    points_world = [tuple(world @ v.co) for v in mesh.vertices]

    base_matrix = core_transform.to_tuple_4x4(base.matrix_world)
    points_m = core_transform.blender_to_robot_batch(
        points_world, base_matrix, scale_length
    )

    edge_pairs = [tuple(edge.vertices) for edge in mesh.edges]
    return points_m, edge_pairs, edge_ids, reassigned


def _fmt_dims(bounds):
    if bounds is None:
        return "--"
    lo, hi = bounds
    return "%.1f x %.1f x %.1f mm" % tuple((hi[i] - lo[i]) * 1000.0 for i in range(3))


# --- shared extraction pipeline ---------------------------------------------
# Module-level rather than operator methods so ops/order.py can run the exact
# same pipeline. Phase C needs the StickSpec geometry (endpoints, vertex
# indices), which the PropertyGroup deliberately does not store -- re-running
# extraction is both cheaper than duplicating that state and guarantees the
# order is computed against the CURRENT settings, not a stale snapshot.


def check_design_ready(props):
    """Returns an error string, or None when extraction can proceed."""
    if props.design_mesh is None:
        return "Pick a design mesh first"
    if props.base_empty is None:
        return "Pick (or create) an SO100_Base empty first"
    if props.design_mesh.mode != "OBJECT":
        # Mesh attribute layers cannot be written while the object is in
        # Edit Mode, and `mesh.vertices` still reports the pre-edit state --
        # so extracting here would both fail to store ids and silently use
        # stale coordinates.
        return "Leave Edit Mode on '%s' before extracting" % props.design_mesh.name
    if not props.design_mesh.data.edges:
        return "The design mesh has no edges"
    return None


def extract_and_validate(points_m, edge_pairs, edge_ids, extract_kwargs, flips):
    """``extract_sticks()`` + ``validate_sticks()``, which always run
    together. One call site so auto-flip's second pass cannot drift out of
    sync with the first."""
    result = core_sticks.extract_sticks(
        points_m, edge_pairs, edge_ids, flips=flips, **extract_kwargs
    )
    # Only sticks that passed the geometric checks: a stick whose length or
    # residual is already wrong has nothing meaningful to say about
    # reachability, and its endpoints may not be what would get built.
    geometrically_bad = {stick_id for stick_id, _code, _msg in result.errors if stick_id}
    verdicts = core_validate.validate_sticks(
        [s for s in result.sticks if s.id not in geometrically_bad]
    )
    return result, verdicts


def extract_with_autoflip(context, props):
    """The full extraction pipeline including Finding 6's auto-flip.

    Returns ``(result, verdicts, auto_flipped, reassigned)``. Raises
    ``ValueError`` (bad stock lengths / limits) or
    ``core_transform.SingularMatrix`` (degenerate base empty).
    """
    stock = props.stock_lengths_m()
    points_m, edge_pairs, edge_ids, reassigned = read_design_mesh(context, props)

    extract_kwargs = dict(
        joint_allowance_m=props.joint_allowance_mm / 1000.0,
        growth_mode=props.growth_mode,
        ground_mode=props.ground_mode,
        stock_lengths_m=stock,
        min_stick_length_m=props.min_stick_length_mm / 1000.0,
        max_stick_length_m=props.max_stick_length_mm / 1000.0,
        section_m=props.section_mm / 1000.0,
        merge_tolerance_m=props.merge_tolerance_mm / 1000.0,
        ground_epsilon_m=props.ground_epsilon_mm / 1000.0,
        residual_tolerance_m=props.residual_tolerance_mm / 1000.0,
    )
    flips = {item.stick_id: item.flip for item in props.sticks if item.flip}

    result, verdicts = extract_and_validate(
        points_m, edge_pairs, edge_ids, extract_kwargs, flips)

    # Finding 6: a stick can be unreachable ONLY because of which end got
    # picked as "base" -- Wrist_Roll's limit is asymmetric, so the opposite
    # assignment can work fine. validate_stick() confirms the flip works
    # before suggesting it, so applying it never guesses. Flip choices don't
    # interact (core/sticks.py applies `flip` per edge independently, after
    # the shared expansion solve), so one extra pass converges.
    auto_flipped = {
        stick_id for stick_id, verdict in verdicts.items() if verdict.suggested_flip
    }
    if auto_flipped:
        for stick_id in auto_flipped:
            flips[stick_id] = not flips.get(stick_id, False)
        result, verdicts = extract_and_validate(
            points_m, edge_pairs, edge_ids, extract_kwargs, flips)

    return result, verdicts, auto_flipped, reassigned


def store_results(props, result, verdicts=None, auto_flipped=(), order_result=None):
    """Write an extraction (and optionally a build order) into the Scene.

    ``props.sticks`` stays in **extraction** order, never build order --
    ``ops`` relies on "build-mesh edge *i* is ``props.sticks[i]``" for the
    Check By Eye highlight, and the UIList sorts for display instead. The
    build index lives in ``item.order``.
    """
    verdicts = verdicts or {}
    errors_by_id = {}
    for stick_id, code, message in result.errors:
        if stick_id:
            errors_by_id.setdefault(stick_id, []).append((code, message))

    ordered_by_id = order_result.by_id() if order_result is not None else {}
    order_errors = {}
    order_warn_codes = {}
    if order_result is not None:
        for stick_id, code, message in order_result.errors:
            if stick_id:
                order_errors[stick_id] = message
        for entry in order_result.ordered:
            order_warn_codes[entry.id] = entry.warnings

    props.sticks.clear()
    for stick in result.sticks:
        item = props.sticks.add()
        item.stick_id = stick.id
        entry = ordered_by_id.get(stick.id)
        item.order = entry.order if entry is not None else -1
        item.stick_length_mm = stick.length_mm
        item.expanded_edge_mm = stick.solved_edge_m * 1000.0
        item.residual_mm = stick.residual_m * 1000.0
        item.shared_ends = stick.shared_ends
        # Sec 6 makes "base" structural, so an order supersedes extraction's
        # reachability-driven auto-flip (see core/order.py's module docstring).
        item.flip = entry.stick.flipped if entry is not None else stick.flipped

        warning_codes = list(stick.warnings)
        if stick.id in auto_flipped and entry is None:
            warning_codes.append(core_validate.WARN_AUTO_FLIPPED)
        warning_codes.extend(order_warn_codes.get(stick.id, ()))

        problems = errors_by_id.get(stick.id)
        if problems:
            item.status = core_state.STATUS_IMPOSSIBLE
            item.reason = problems[0][1]
        elif stick.id in order_errors:
            item.status = core_state.STATUS_IMPOSSIBLE
            item.reason = order_errors[stick.id]
        else:
            verdict = verdicts.get(stick.id)
            if verdict is None:
                item.status = core_state.STATUS_PENDING
                item.reason = ""
            elif verdict.buildable:
                item.status = core_state.STATUS_BUILDABLE
                item.reason = ""
                warning_codes.extend(verdict.warnings)
            else:
                item.status = core_state.STATUS_IMPOSSIBLE
                item.reason = verdict.reason
                warning_codes.extend(verdict.warnings)

        # De-duplicate while preserving order: a stick can pick up the same
        # code from both its extraction warnings and its placement.
        seen, unique = set(), []
        for code in warning_codes:
            if code and code not in seen:
                seen.add(code)
                unique.append(code)
        item.warnings = ",".join(unique)

    props.topology_signature = core_state.topology_signature(
        [s.id for s in result.sticks], [s.length_m for s in result.sticks]
    )
    props.design_dimensions_mm = _fmt_dims(result.design_bounds)
    props.expanded_dimensions_mm = _fmt_dims(result.expanded_bounds)

    buildable = sum(
        1 for item in props.sticks if item.status == core_state.STATUS_BUILDABLE
    )
    impossible = sum(
        1 for item in props.sticks if item.status == core_state.STATUS_IMPOSSIBLE
    )
    props.summary = "%d sticks - %d buildable - %d impossible" % (
        len(result.sticks), buildable, impossible
    )

    if result.expansion is not None:
        props.solver_report = "%s, %d iteration%s, max residual %.3f mm%s" % (
            result.expansion.method,
            result.expansion.iterations,
            "" if result.expansion.iterations == 1 else "s",
            result.expansion.max_residual_m * 1000.0,
            "" if result.expansion.converged else " (HIT ITERATION CAP)",
        )
    else:
        props.solver_report = ""

    props.active_stick_index = 0
    _store_order_report(props, order_result)


def _store_order_report(props, order_result):
    props.order_warnings.clear()
    if order_result is None:
        props.order_summary = ""
        props.has_order = False
        return

    for stick_id, code, message in order_result.errors:
        entry = props.order_warnings.add()
        entry.stick_id = stick_id or ""
        entry.code = code
        entry.message = message
        entry.is_error = True
    for stick_id, code, message in order_result.warnings:
        entry = props.order_warnings.add()
        entry.stick_id = stick_id or ""
        entry.code = code
        entry.message = message
        entry.is_error = False

    props.has_order = bool(order_result.ordered)
    props.order_summary = "%d in order - %d warning%s - %d error%s%s" % (
        len(order_result.ordered),
        len(order_result.warnings), "" if len(order_result.warnings) == 1 else "s",
        len(order_result.errors), "" if len(order_result.errors) == 1 else "s",
        "" if order_result.complete else "  (INCOMPLETE)",
    )


def rebuild_build_mesh(context, props, result):
    """Sec 5.2.3 point 3: generate a derived build mesh as a separate object
    and never modify the user's design mesh -- they will iterate.

    Its edges are the PHYSICAL stick ends, so the glue gaps at every joint
    are visible directly in the viewport.
    """
    scale_length = context.scene.unit_settings.scale_length
    base_matrix = core_transform.to_tuple_4x4(props.base_empty.matrix_world)

    vertices, edges = [], []
    for stick in result.sticks:
        a = core_transform.robot_to_blender(stick.base, base_matrix, scale_length)
        b = core_transform.robot_to_blender(stick.tip, base_matrix, scale_length)
        edges.append((len(vertices), len(vertices) + 1))
        vertices.extend((a, b))

    obj = props.build_mesh
    if obj is None or obj.name not in bpy.data.objects:
        mesh = bpy.data.meshes.new(BUILD_MESH_NAME)
        obj = bpy.data.objects.new(BUILD_MESH_NAME, mesh)
        context.collection.objects.link(obj)
        props.build_mesh = obj

    # Re-armed on every extraction, not just at creation: "select in
    # viewport" deliberately unlocks this for one highlight, and a fresh
    # extraction is the natural point to put the lock back -- the mesh it
    # protects is about to be replaced anyway.
    obj.hide_select = True

    mesh = obj.data
    mesh.clear_geometry()
    mesh.from_pydata(vertices, edges, [])
    mesh.update()
    # robot_to_blender already produced world-space coordinates, so the
    # object itself must carry no transform of its own.
    obj.matrix_world.identity()


# --- operators ---------------------------------------------------------------


class SO100_OT_create_base_empty(Operator):
    bl_idname = "so100.create_base_empty"
    bl_label = "Create SO-100 Base"
    bl_description = ("Add an Empty marking base_link and point the addon at it. "
                      "Move it to reposition the whole design relative to the robot")
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        props = context.scene.so100
        empty = bpy.data.objects.new(BASE_EMPTY_NAME, None)
        empty.empty_display_type = "ARROWS"
        empty.empty_display_size = 0.1
        context.collection.objects.link(empty)
        props.base_empty = empty
        self.report({"INFO"}, "Created %s at the world origin" % empty.name)
        return {"FINISHED"}


class SO100_OT_extract_sticks(Operator):
    bl_idname = "so100.extract_sticks"
    bl_label = "Extract Sticks"
    bl_description = ("Turn the design mesh's edges into physical sticks: snap "
                      "lengths, grow the design so fixed-length sticks fit with a "
                      "glue gap at every joint, and report what will not fit")
    bl_options = {"REGISTER", "UNDO"}

    rebuild_mesh: BoolProperty(
        name="Generate Build Mesh", default=True,
        description="Create/refresh a separate object showing the expanded, "
                    "physically-inset sticks. The design mesh is never modified",
    )

    @classmethod
    def poll(cls, context):
        props = context.scene.so100
        return props.design_mesh is not None and props.base_empty is not None

    def execute(self, context):
        props = context.scene.so100

        problem = check_design_ready(props)
        if problem:
            return _report_error(self, problem)
        if props.design_mesh.data.polygons:
            self.report(
                {"WARNING"},
                "'%s' has faces. Only its edges become sticks -- the faces are "
                "ignored, and a face's boundary edges are all sticks"
                % props.design_mesh.name,
            )

        try:
            result, verdicts, auto_flipped, reassigned = extract_with_autoflip(
                context, props)
        except (ValueError, core_transform.SingularMatrix) as exc:
            return _report_error(self, str(exc))

        store_results(props, result, verdicts, auto_flipped)

        if self.rebuild_mesh:
            rebuild_build_mesh(context, props, result)

        if reassigned:
            self.report(
                {"WARNING"},
                "%d duplicated edge id%s reassigned -- those sticks are new to the "
                "build and any existing order for them is stale"
                % (reassigned, "" if reassigned == 1 else "s"),
            )

        if auto_flipped:
            self.report(
                {"WARNING"},
                "%d stick%s auto-flipped to make %s reachable -- see the "
                "'auto_flipped' badge in the Sticks list"
                % (len(auto_flipped), "" if len(auto_flipped) == 1 else "s",
                   "it" if len(auto_flipped) == 1 else "them"),
            )

        level = {"INFO"}
        if any(item.status == core_state.STATUS_IMPOSSIBLE for item in props.sticks):
            level = {"WARNING"}
        self.report(level, props.summary)
        return {"FINISHED"}

class SO100_OT_select_stick_in_viewport(Operator):
    """"Check it by eye": select one stick's edge on the build mesh and frame
    it. This is the eyes-on complement to the automated buildability checks
    -- Sec 10.3 says the current stick should be highlighted "in list and
    viewport"; this is that half, ahead of Phase C assigning a real build
    order and Phase E's animated preview.

    Edge index == list index is a deliberate invariant: ``_rebuild_build_mesh``
    appends one edge per stick, in ``result.sticks`` order, and nothing else
    in Phase A reorders either list. Once Phase C computes a build order this
    still holds, because the build mesh is regenerated from the (now
    reordered) stick list on every extraction -- so this operator needs no
    change when order lands.
    """

    bl_idname = "so100.select_stick_in_viewport"
    bl_label = "Select && Frame in Viewport"
    bl_description = ("Select this stick's edge on the build mesh and frame it "
                      "in the 3D view")
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        props = context.scene.so100
        return (
            props.build_mesh is not None
            and props.build_mesh.name in bpy.data.objects
            and 0 <= props.active_stick_index < len(props.sticks)
        )

    def execute(self, context):
        props = context.scene.so100
        index = props.active_stick_index
        obj = props.build_mesh

        if len(obj.data.edges) != len(props.sticks):
            return _report_error(
                self,
                "Build mesh is out of sync with the stick list -- re-run Extract Sticks",
            )

        if context.object is not None and context.object.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")

        for other in context.selected_objects:
            other.select_set(False)

        # The build mesh is normally unselectable (Sec 5.2.3: never the
        # thing the user hand-edits, since it is discarded and regenerated
        # on every extraction). Lift that just long enough for this one
        # deliberate, button-driven select -- extract_sticks re-locks it.
        obj.hide_select = False
        obj.select_set(True)
        context.view_layer.objects.active = obj

        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="DESELECT")

        bm = bmesh.from_edit_mesh(obj.data)
        bm.edges.ensure_lookup_table()
        bm.edges[index].select = True
        bmesh.update_edit_mesh(obj.data)

        area = next((a for a in context.screen.areas if a.type == "VIEW_3D"), None)
        if area is not None:
            region = next((r for r in area.regions if r.type == "WINDOW"), None)
            if region is not None:
                with context.temp_override(area=area, region=region):
                    bpy.ops.view3d.view_selected()

        return {"FINISHED"}


class SO100_OT_step_stick(Operator):
    """Move the active stick by +/-1 and re-run the selection/frame above, so
    stepping through build order is a single repeated click (or a single
    hotkey) rather than list-scroll-then-button each time."""

    bl_idname = "so100.step_stick"
    bl_label = "Step Stick"
    bl_options = {"REGISTER"}

    direction: IntProperty(default=1)

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def execute(self, context):
        props = context.scene.so100
        count = len(props.sticks)
        props.active_stick_index = (props.active_stick_index + self.direction) % count
        if bpy.ops.so100.select_stick_in_viewport.poll():
            bpy.ops.so100.select_stick_in_viewport()
        return {"FINISHED"}


class SO100_OT_clear_results(Operator):
    bl_idname = "so100.clear_results"
    bl_label = "Clear Results"
    bl_description = "Discard the extracted sticks and their state"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        props = context.scene.so100
        props.sticks.clear()
        props.summary = ""
        props.solver_report = ""
        props.design_dimensions_mm = ""
        props.expanded_dimensions_mm = ""
        props.topology_signature = ""
        self.report({"INFO"}, "Cleared extracted sticks (stable ids are kept)")
        return {"FINISHED"}


_CLASSES = (
    SO100_OT_create_base_empty,
    SO100_OT_extract_sticks,
    SO100_OT_select_stick_in_viewport,
    SO100_OT_step_stick,
    SO100_OT_clear_results,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
