"""Export and the build loop. BLENDER_ADDON_PLAN.md Sec 10.2/10.3, Phase D.

The build file is the whole integration surface with ROS2 (Option C), so
export re-runs the pipeline rather than serialising whatever happens to be
in the PropertyGroup: the file must be self-contained and consistent with
the settings as they are *now*, and the StickSpec geometry it needs is not
stored on the Scene.

Status flows both ways. ROS2 writes the sidecar as it builds (A.3/A.4);
Blender writes it when the operator marks a stick by hand (Sec 10.3's "Mark
placed / Mark failed"). Both read it. Sec 9.3 requires that when the
``.blend`` and the sidecar disagree, **both** are shown and the user
chooses -- never a silent merge -- so ``Sync Status`` reports conflicts
rather than resolving them.
"""

import os

import bpy
from bpy.props import EnumProperty, StringProperty
from bpy.types import Operator
from bpy_extras.io_utils import ExportHelper, ImportHelper

from .. import kinematics
from ..core import state as core_state
from ..core import transform as core_transform
from ..io import build_file as build_io
from ..kinematics import constants as kc
from .design import check_design_ready, rebuild_build_mesh, store_results
from .order import build_solver  # the same pipeline, so export cannot drift


def _report_error(operator, message):
    operator.report({"ERROR"}, message)
    return {"CANCELLED"}


def _mm_to_m(millimetres):
    """Rounded to the micrometre. Blender stores these as float32, so
    6.45 mm comes back as 0.006449999809265136 -- unrounded that lands
    verbatim in a file whose A.1 design rule is "human-readable and
    diffable"."""
    return round(millimetres / 1000.0, 6)


def _stock_block(props):
    return {
        "section_m": [_mm_to_m(props.section_mm), _mm_to_m(props.section_mm)],
        "joint_allowance_m": _mm_to_m(props.joint_allowance_mm),
        "length_range_m": [_mm_to_m(props.min_stick_length_mm),
                           _mm_to_m(props.max_stick_length_mm)],
    }


def _build_volume_block():
    return {
        "min": list(kc.BUILD_VOLUME_MIN_M),
        "max": list(kc.BUILD_VOLUME_MAX_M),
    }


def collect_statuses(props):
    """Per-stick status straight off the Scene, in sidecar shape."""
    statuses = {}
    for item in props.sticks:
        statuses[item.stick_id] = {
            "status": item.status if item.status in build_io.SIDECAR_STATUSES
            else build_io.STATUS_PENDING,
            "at": "",
            "reason": item.reason,
        }
    return statuses


def ordered_ids(props):
    """Stick ids in build order. Empty when no order has been computed."""
    ordered = [item for item in props.sticks if item.order >= 0]
    ordered.sort(key=lambda item: item.order)
    return [item.stick_id for item in ordered]


def write_status_sidecar(props, current_index=0):
    """Rewrite the sidecar next to the build file. Returns its path, or None
    when no build file has been exported yet."""
    if not props.build_file_path:
        return None
    build_path = bpy.path.abspath(props.build_file_path)
    status_path = build_io.status_path_for(build_path)
    document = build_io.status_document(
        os.path.basename(build_path), collect_statuses(props), current_index)
    build_io.write_status_file(status_path, document)
    return status_path


class SO100_OT_export_build_file(Operator, ExportHelper):
    bl_idname = "so100.export_build_file"
    bl_label = "Export Build File"
    bl_description = ("Write the build file ROS2 executes. Everything needed is "
                      "in the file -- metres in base_link, already in build "
                      "order, with the validation verdicts")

    filename_ext = ".build.json"
    filter_glob: StringProperty(default="*.json", options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        props = context.scene.so100
        return props.design_mesh is not None and props.base_empty is not None

    def execute(self, context):
        props = context.scene.so100

        problem = check_design_ready(props)
        if problem:
            return _report_error(self, problem)

        try:
            solver, (result, verdicts, auto_flipped) = build_solver(context, props)
        except (ValueError, core_transform.SingularMatrix) as exc:
            return _report_error(self, str(exc))
        order_result = solver.solve()

        if not order_result.ordered:
            return _report_error(
                self, "Nothing could be ordered -- see the Plan panel")

        # Store what was exported, so the Scene and the file cannot disagree
        # about ids, order or verdicts -- and so the Build panel has
        # something to track progress against straight after an export.
        store_results(props, result, verdicts, auto_flipped, order_result)
        rebuild_build_mesh(context, props, result)

        document = build_io.build_document(
            order_result.ordered,
            verdicts,
            source=os.path.basename(bpy.data.filepath) or "(unsaved.blend)",
            kinematics_version=kinematics.__version__,
            stock=_stock_block(props),
            build_volume=_build_volume_block(),
        )

        try:
            build_io.write_build_file(self.filepath, document)
        except OSError as exc:
            return _report_error(self, "Could not write %s: %s" % (self.filepath, exc))

        props.build_file_path = self.filepath

        unbuildable = sum(
            1 for entry in document["sticks"] if not entry["validation"]["buildable"])
        level = {"WARNING"} if unbuildable else {"INFO"}
        self.report(
            level,
            "Wrote %d sticks to %s%s"
            % (len(document["sticks"]), os.path.basename(self.filepath),
               " (%d marked unbuildable -- they are still in the file, in order)"
               % unbuildable if unbuildable else ""),
        )
        return {"FINISHED"}


class SO100_OT_sync_status(Operator, ImportHelper):
    bl_idname = "so100.sync_status"
    bl_label = "Sync Status From Sidecar"
    bl_description = ("Read back what the robot actually placed. Where the "
                      ".blend and the sidecar disagree, both are reported and "
                      "nothing is changed for those sticks")

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def invoke(self, context, event):
        props = context.scene.so100
        # Default to the sidecar beside the known build file, so the common
        # case is one click rather than a file hunt.
        if props.build_file_path:
            self.filepath = build_io.status_path_for(
                bpy.path.abspath(props.build_file_path))
        return super().invoke(context, event)

    def execute(self, context):
        props = context.scene.so100
        try:
            with open(self.filepath, encoding="utf-8") as handle:
                text = handle.read()
        except OSError as exc:
            return _report_error(self, "Could not read %s: %s" % (self.filepath, exc))

        try:
            _current_index, entries = build_io.load_status_file(text)
        except build_io.ProtocolError as exc:
            return _report_error(self, str(exc))

        states = [
            core_state.StickState(id=item.stick_id, status=item.status,
                                  reason=item.reason)
            for item in props.sticks
        ]
        updated, conflicts = core_state.merge_sidecar(states, entries)

        by_id = {state.id: state for state in states}
        for item in props.sticks:
            state = by_id.get(item.stick_id)
            if state is not None and state.status != item.status:
                item.status = state.status
                item.reason = state.reason

        props.status_conflicts = "; ".join(
            "%s: .blend=%s sidecar=%s" % conflict for conflict in conflicts)

        if conflicts:
            # Sec 9.3: show both, let the user choose. Nothing was changed
            # for these sticks.
            self.report(
                {"WARNING"},
                "%d stick%s disagree between the .blend and the sidecar -- "
                "left unchanged, see the Build panel"
                % (len(conflicts), "" if len(conflicts) == 1 else "s"),
            )
        else:
            self.report({"INFO"}, "Synced %d stick%s from the sidecar"
                        % (len(updated), "" if len(updated) == 1 else "s"))
        return {"FINISHED"}


class SO100_OT_mark_stick(Operator):
    bl_idname = "so100.mark_stick"
    bl_label = "Mark Stick"
    bl_description = "Record what actually happened to this stick"
    bl_options = {"REGISTER", "UNDO"}

    status: EnumProperty(
        items=[
            (build_io.STATUS_PLACED, "Placed", "Glued in place"),
            (build_io.STATUS_FAILED, "Failed", "The robot could not place it"),
            (build_io.STATUS_SKIPPED, "Skipped", "Deliberately passed over"),
            (build_io.STATUS_PENDING, "Pending", "Not yet built"),
        ],
        default=build_io.STATUS_PLACED,
    )
    stick_id: StringProperty(default="")

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def execute(self, context):
        props = context.scene.so100
        target = self.stick_id or next_stick_id(props)
        if not target:
            return _report_error(self, "No stick to mark -- the build is finished")

        item = next((i for i in props.sticks if i.stick_id == target), None)
        if item is None:
            return _report_error(self, "No stick with id %r" % target)

        item.status = self.status
        if self.status != build_io.STATUS_FAILED:
            item.reason = ""

        # Sec 9.3 / QB4: the sidecar is rewritten on every status change, so
        # it survives "don't save changes" and stays the thing ROS2 reads.
        path = write_status_sidecar(props, _current_index(props))
        suffix = " (sidecar updated)" if path else ""
        self.report({"INFO"}, "%s marked %s%s" % (target, self.status, suffix))
        return {"FINISHED"}


def _current_index(props):
    """Build index of the next stick to load -- A.3's ``current_index``."""
    ids = ordered_ids(props)
    statuses = collect_statuses(props)
    upcoming = build_io.next_stick_to_load(ids, statuses)
    if upcoming is None:
        return len(ids)
    return ids.index(upcoming)


def next_stick_id(props):
    return build_io.next_stick_to_load(ordered_ids(props), collect_statuses(props))


class SO100_OT_write_status_sidecar(Operator):
    bl_idname = "so100.write_status_sidecar"
    bl_label = "Write Status Sidecar"
    bl_description = ("Write the current placed/failed state next to the build "
                      "file, so ROS2 can resume from it")
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        props = context.scene.so100
        return bool(props.build_file_path) and len(props.sticks) > 0

    def execute(self, context):
        props = context.scene.so100
        try:
            path = write_status_sidecar(props, _current_index(props))
        except OSError as exc:
            return _report_error(self, "Could not write the sidecar: %s" % exc)
        if path is None:
            return _report_error(self, "Export a build file first")
        self.report({"INFO"}, "Wrote %s" % os.path.basename(path))
        return {"FINISHED"}


class SO100_OT_reset_build_progress(Operator):
    bl_idname = "so100.reset_build_progress"
    bl_label = "Reset Build Progress"
    bl_description = ("Set every stick back to its pre-build state, keeping the "
                      "sticks and the order")
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def execute(self, context):
        props = context.scene.so100
        for item in props.sticks:
            if item.status in (build_io.STATUS_PLACED, build_io.STATUS_FAILED,
                               build_io.STATUS_SKIPPED):
                item.status = core_state.STATUS_BUILDABLE
                item.reason = ""
        props.status_conflicts = ""
        self.report({"INFO"}, "Build progress reset")
        return {"FINISHED"}


class SO100_OT_export_cut_list(Operator, ExportHelper):
    """Sec 5.5. Supersedes the Phase A version: now carries the build order
    and a saw-friendly tally."""

    bl_idname = "so100.export_cut_list"
    bl_label = "Export Cut List"
    bl_description = ("Write the ordered cut list a human needs at the saw and "
                      "at the feeder")

    filename_ext = ".csv"
    filter_glob: StringProperty(default="*.csv", options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def execute(self, context):
        props = context.scene.so100

        rows = sorted(
            props.sticks,
            key=lambda item: (item.order < 0, item.order, item.stick_id))
        lines = ["build_order,stick_id,stick_length_mm,shared_ends,status,warnings"]
        for item in rows:
            lines.append(
                "%s,%s,%.2f,%d,%s,%s"
                % ("" if item.order < 0 else item.order, item.stick_id,
                   item.stick_length_mm, item.shared_ends, item.status,
                   " ".join(item.warning_list())))

        # Sec 5.5: under fixed-stock-length mode this collapses to a tally,
        # "which is what you actually want at a saw".
        tally = {}
        for item in props.sticks:
            key = round(item.stick_length_mm * 2.0) / 2.0
            tally[key] = tally.get(key, 0) + 1
        lines.append("")
        lines.append("# tally")
        lines.append("count,stick_length_mm")
        for length, count in sorted(tally.items()):
            lines.append("%d,%.1f" % (count, length))

        try:
            with open(self.filepath, "w", encoding="utf-8", newline="\n") as handle:
                handle.write("\n".join(lines) + "\n")
        except OSError as exc:
            return _report_error(self, "Could not write %s: %s" % (self.filepath, exc))

        self.report({"INFO"}, "Wrote %d sticks to %s"
                    % (len(props.sticks), os.path.basename(self.filepath)))
        return {"FINISHED"}


_CLASSES = (
    SO100_OT_export_build_file,
    SO100_OT_export_cut_list,
    SO100_OT_sync_status,
    SO100_OT_write_status_sidecar,
    SO100_OT_mark_stick,
    SO100_OT_reset_build_progress,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
