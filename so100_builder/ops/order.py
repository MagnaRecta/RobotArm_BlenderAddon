"""Build-order operators. BLENDER_ADDON_PLAN.md Sec 6 / Sec 10.2, Phase C.

Two execution paths, deliberately:

* ``invoke()`` -- the interactive path. Drives ``OrderSolver.step()`` from a
  ``bpy.app`` timer via a modal operator, so a long solve never blocks the
  UI (constraint B3). This is the phase where chunking actually earns its
  keep: extraction is sub-10 ms, but ordering runs IK inside a backtracking
  search.
* ``execute()`` -- the synchronous path. Runs the same solver to completion
  in one call. Used by the test suite and anything running under
  ``blender --background``, where modal operators and timers do not run at
  all. Calling ``bpy.ops.so100.compute_build_order()`` from a script hits
  this path by default (``EXEC_DEFAULT``); the button in the panel hits
  ``invoke``.

Both paths share ``_prepare`` and ``_finish`` so the two cannot drift.
"""

import bpy
from bpy.props import IntProperty
from bpy.types import Operator

from ..core import order as core_order
from ..core import robots as core_robots
from ..core import transform as core_transform
from .design import (
    check_design_ready,
    effective_ground_height_m,
    extract_with_autoflip,
    ordered_ids,
    rebuild_build_mesh,
    store_results,
)

# Interval between solver chunks. Short enough to feel responsive, long
# enough that Blender gets its own redraw work done in between.
_TIMER_INTERVAL_S = 0.03


def _report_error(operator, message):
    operator.report({"ERROR"}, message)
    return {"CANCELLED"}


def build_solver(context, props):
    """Returns ``(solver, extraction)``.

    Re-runs extraction rather than reading ``props.sticks``: the solver needs
    the full StickSpec geometry (endpoints, topology vertex indices) that the
    PropertyGroup deliberately does not store, and re-extracting also
    guarantees the order is computed against the settings as they are *now*.
    """
    result, verdicts, auto_flipped, _reassigned = extract_with_autoflip(context, props)
    profile = core_robots.get_robot(props.robot_id)
    solver = core_order.OrderSolver(
        result.sticks,
        ground_epsilon_m=props.ground_epsilon_mm / 1000.0,
        ground_height_m=effective_ground_height_m(profile, props),
        ground_required=props.require_build_plate,
        backtrack_limit=props.backtrack_limit,
        robot_id=props.robot_id,
        jaw_width_m=props.jaw_width_mm / 1000.0,
        section_m=props.section_mm / 1000.0,
        check_jaw_clearance=props.check_jaw_clearance,
    )
    return solver, (result, verdicts, auto_flipped)


class SO100_OT_compute_build_order(Operator):
    bl_idname = "so100.compute_build_order"
    bl_label = "Compute Build Order"
    bl_description = ("Work out which stick the robot places first, second, ... "
                      "so it never has to reach into a space it has already "
                      "walled off. Validates each placement against the "
                      "already-built structure as it goes")
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        props = context.scene.so100
        return props.design_mesh is not None and props.base_empty is not None

    # --- shared ---------------------------------------------------------------

    def _prepare(self, context):
        props = context.scene.so100
        problem = check_design_ready(props)
        if problem:
            return problem
        try:
            self._solver, self._extraction = build_solver(context, props)
        except (ValueError, core_transform.SingularMatrix) as exc:
            return str(exc)
        return None

    def _finish(self, context):
        props = context.scene.so100
        result, verdicts, auto_flipped = self._extraction
        order_result = self._solver.result
        store_results(props, result, verdicts, auto_flipped, order_result)
        # This operator re-extracts, so it owns the build mesh too -- both so
        # it works standalone (without pressing Extract Sticks first) and so
        # the "edge i is sticks[i]" invariant Check By Eye relies on cannot
        # be left pointing at a stale mesh.
        rebuild_build_mesh(context, props, result)

        level = {"INFO"}
        if order_result.errors:
            level = {"WARNING"}
        self.report(level, props.order_summary)

    # --- synchronous (background / scripted) ---------------------------------

    def execute(self, context):
        problem = self._prepare(context)
        if problem:
            return _report_error(self, problem)
        self._solver.solve()
        self._finish(context)
        return {"FINISHED"}

    # --- chunked modal (interactive) -----------------------------------------

    def invoke(self, context, event):
        problem = self._prepare(context)
        if problem:
            return _report_error(self, problem)

        window_manager = context.window_manager
        self._timer = window_manager.event_timer_add(
            _TIMER_INTERVAL_S, window=context.window)
        window_manager.modal_handler_add(self)
        window_manager.progress_begin(0.0, 1.0)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type in {"ESC"}:
            self._teardown(context)
            self.report({"WARNING"}, "Build order cancelled -- no order stored")
            return {"CANCELLED"}

        if event.type != "TIMER":
            return {"PASS_THROUGH"}

        # All bpy access here is on the main thread inside a timer callback,
        # never a worker thread (constraint B2).
        done = self._solver.step()
        context.window_manager.progress_update(self._solver.progress)
        if context.area is not None:
            context.area.header_text_set(
                "Build order: %d%%  (Esc to cancel)"
                % int(self._solver.progress * 100.0))

        if not done:
            return {"RUNNING_MODAL"}

        self._teardown(context)
        self._finish(context)
        return {"FINISHED"}

    def _teardown(self, context):
        window_manager = context.window_manager
        timer = getattr(self, "_timer", None)
        if timer is not None:
            window_manager.event_timer_remove(timer)
            self._timer = None
        window_manager.progress_end()
        if context.area is not None:
            context.area.header_text_set(None)


class SO100_OT_clear_build_order(Operator):
    bl_idname = "so100.clear_build_order"
    bl_label = "Clear Build Order"
    bl_description = "Discard the computed build order, keeping the sticks"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.scene.so100.has_order

    def execute(self, context):
        props = context.scene.so100
        for item in props.sticks:
            item.order = -1
        props.order_warnings.clear()
        props.order_summary = ""
        props.has_order = False
        self.report({"INFO"}, "Cleared the build order")
        return {"FINISHED"}


class SO100_OT_move_build_step(Operator):
    """Swap the active stick's build position with its immediate neighbour
    (2026-08-23 user request: "I want to be able to move the steps before
    or after its position" -- e.g. ``[s1,s3,s4,s2]`` moving ``s3`` earlier
    gives ``[s3,s1,s4,s2]``, later gives ``[s1,s4,s3,s2]``). Only ever an
    ADJACENT swap, matching the request's own examples -- not an arbitrary
    move to any position.

    Re-runs the pipeline through ``OrderSolver.replay()`` (never
    ``solve()``) right after the swap, so the Scene's warnings/errors and
    the build mesh reflect the NEW order immediately -- if the move breaks
    C1 support or introduces a jaw-clearance clash, that shows up right
    away rather than only at export time. See ``core/order.py``'s
    ``replay()`` docstring for exactly what is and is not re-validated.
    """

    bl_idname = "so100.move_build_step"
    bl_label = "Move Build Step"
    bl_description = ("Swap this stick with its neighbour earlier/later in the "
                      "build order, then re-validate the new order")
    bl_options = {"REGISTER", "UNDO"}

    direction: IntProperty(default=-1)

    @classmethod
    def poll(cls, context):
        props = context.scene.so100
        index = props.active_stick_index
        return (
            props.has_order
            and 0 <= index < len(props.sticks)
            and props.sticks[index].order >= 0
        )

    def execute(self, context):
        props = context.scene.so100
        current = props.sticks[props.active_stick_index]
        target_order = current.order + self.direction
        if not (0 <= target_order < len(props.sticks)):
            self.report({"INFO"}, "%s is already at that end of the build order"
                        % current.stick_id)
            return {"CANCELLED"}
        neighbour = next(
            (item for item in props.sticks if item.order == target_order), None)
        if neighbour is None:
            return _report_error(
                self, "No stick currently holds build position %d" % target_order)

        current.order, neighbour.order = neighbour.order, current.order

        try:
            solver, (result, verdicts, auto_flipped) = build_solver(context, props)
        except (ValueError, core_transform.SingularMatrix) as exc:
            return _report_error(self, str(exc))
        order_result = solver.replay(ordered_ids(props))
        store_results(props, result, verdicts, auto_flipped, order_result)
        rebuild_build_mesh(context, props, result)

        self.report(
            {"INFO"} if not order_result.errors else {"WARNING"},
            "Moved %s %s -- %s"
            % (current.stick_id, "earlier" if self.direction < 0 else "later",
               props.order_summary),
        )
        return {"FINISHED"}


_CLASSES = (
    SO100_OT_compute_build_order,
    SO100_OT_clear_build_order,
    SO100_OT_move_build_step,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
