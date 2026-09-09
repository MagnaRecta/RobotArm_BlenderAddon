"""3D View sidebar, category "RA130" (Robot Arm 130 -- the addon's own name,
not either supported robot's; multi-robot support means it is no longer
just "SO-100"). BLENDER_ADDON_PLAN.md Sec 10.

Phase A ships the `Design` panel (Sec 10.1) plus the stick list from the
`Build` panel (Sec 10.3), because seeing per-stick lengths and residuals is
what makes the expansion behaviour checkable by hand. `Plan` (Sec 10.2),
`Build`'s next-stick-to-load display and `Preview` (Sec 10.4) arrive with
Phases C/D/E.
"""

import bpy
from bpy.types import Panel, UIList

from ..core import robots as core_robots
from ..core import state as core_state
from ..ops.design import base_empty_name
# so_arm_100-only reachability caveats (GRASP_OFFSET_M) in SO100_PT_reference
# below -- core/sticks.py / core/order.py have the same, still-open
# limitation (multi-robot support, docs/STATUS.md 2026-08-21 / 2026-08-23).
from ..kinematics.so_arm_100 import constants as kc

# Each robot's own URDF root frame name (used only for this label -- see
# each kinematics package's own README; not part of the interface contract
# in core/robots.py, so not read from there).
_FRAME_NAMES = {
    core_robots.SO_ARM_100_ID: "base_link",
    core_robots.KR10_R900_2_ID: "base",
}

CATEGORY = "RA130"


class SO100_UL_sticks(UIList):
    """Sec 10.3: index, id, stick length, status icon, reason tooltip."""

    def draw_item(self, _context, layout, _data, item, _icon, _active_data,
                  _active_prop, index):
        if self.layout_type in {"DEFAULT", "COMPACT"}:
            row = layout.row(align=True)
            row.label(text="%d" % (item.order if item.order >= 0 else index))
            row.label(text=item.stick_id)
            row.label(text="%.1f mm" % item.stick_length_mm)

            if item.warning_list():
                row.label(text="", icon="ERROR")
            if item.reason:
                # The reason has to be reachable, and a UIList row cannot
                # carry a per-item tooltip -- so it is shown in full in the
                # detail box below the list.
                row.label(text="", icon="INFO")
            row.prop(item, "status", text="", emboss=False, icon_only=True)
        else:
            layout.label(text=item.stick_id)

    def filter_items(self, context, data, propname):
        """Display in build order without reordering the underlying
        collection.

        ``props.sticks`` must stay in extraction order: the Check By Eye
        highlight relies on "build-mesh edge *i* is ``sticks[i]``". Blender's
        own display-order hook keeps those two concerns apart.
        """
        props = context.scene.so100
        if not props.sort_by_build_order or not props.has_order:
            return [], []
        return [], core_state.build_order_permutation(
            [item.order for item in getattr(data, propname)])


class SO100PanelBase:
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = CATEGORY


class SO100_PT_design(SO100PanelBase, Panel):
    bl_idname = "SO100_PT_design"
    bl_label = "Design"

    def draw(self, context):
        layout = self.layout
        props = context.scene.so100

        layout.prop(props, "robot_id", text="Robot", icon="ARMATURE_DATA")
        if props.robot_id != core_robots.DEFAULT_ROBOT_ID:
            profile = core_robots.get_robot(props.robot_id)
            if not profile.is_vendored:
                layout.label(
                    text="Not vendored yet -- validation/export will report why",
                    icon="ERROR",
                )

        column = layout.column(align=True)
        if props.base_empty is None:
            column.operator(
                "so100.create_base_empty",
                text="Create %s" % base_empty_name(props.robot_id),
                icon="EMPTY_ARROWS",
            )
        column.prop(props, "base_empty", text="Base", icon="EMPTY_ARROWS")
        column.prop(props, "design_mesh", text="Mesh", icon="MESH_DATA")

        if context.scene.unit_settings.scale_length != 1.0:
            info = layout.box()
            info.label(text="Scene unit scale %.4g applied"
                            % context.scene.unit_settings.scale_length, icon="INFO")

        box = layout.box()
        box.label(text="Stock", icon="MESH_CUBE")
        column = box.column(align=True)
        column.prop(props, "section_mm")
        column.prop(props, "joint_allowance_mm")
        box.operator("so100.reset_stock_to_robot_defaults", icon="LOOP_BACK")

        box = layout.box()
        box.label(text="Stick Length", icon="DRIVER_DISTANCE")
        box.prop(props, "length_mode", text="")
        if props.length_mode == "STOCK":
            box.prop(props, "stock_lengths_mm", text="")
        column = box.column(align=True)
        column.prop(props, "min_stick_length_mm")
        column.prop(props, "max_stick_length_mm")

        box = layout.box()
        box.label(text="Mesh Expansion", icon="MOD_MESHDEFORM")
        box.label(text="Sticks keep their length; the design grows.", icon="INFO")
        box.prop(props, "growth_mode", text="Growth")
        box.prop(props, "require_build_plate")
        if props.require_build_plate:
            box.prop(props, "ground_mode", text="Ground")
            box.prop(props, "build_plate_height_mm")
            box.operator("so100.drop_to_build_plate", icon="TRIA_DOWN_BAR")
        else:
            box.label(
                text="No plate check -- you supply the support", icon="INFO")

        layout.prop(props, "show_advanced", icon="PREFERENCES")
        if props.show_advanced:
            box = layout.box()
            column = box.column(align=True)
            column.prop(props, "merge_tolerance_mm")
            column.prop(props, "residual_tolerance_mm")
            column.prop(props, "ground_epsilon_mm")
            column.prop(props, "layer_tolerance_mm")

        row = layout.row(align=True)
        row.scale_y = 1.4
        row.operator("so100.extract_sticks", icon="MOD_BUILD")
        row.operator("so100.clear_results", text="", icon="TRASH")

        layout.prop(props, "show_overlay", icon="OVERLAY")


class SO100_PT_summary(SO100PanelBase, Panel):
    bl_idname = "SO100_PT_summary"
    bl_label = "Summary"
    bl_parent_id = "SO100_PT_design"

    @classmethod
    def poll(cls, context):
        return bool(context.scene.so100.summary)

    def draw(self, context):
        layout = self.layout
        props = context.scene.so100

        layout.label(text=props.summary, icon="INFO")

        # Sec 5.2.3 point 2: show BOTH, so the growth is never a surprise.
        box = layout.box()
        box.label(text="Dimensions", icon="FIXED_SIZE")
        column = box.column(align=True)
        column.label(text="Design:   %s" % props.design_dimensions_mm)
        column.label(text="Built:    %s" % props.expanded_dimensions_mm)
        box.label(text="The built sculpture is larger -- ~%.2f mm per joint."
                       % props.joint_allowance_mm, icon="ERROR")

        if props.solver_report:
            layout.label(text="Solver: %s" % props.solver_report, icon="CON_KINEMATIC")

        errors = [item for item in props.sticks
                  if item.status == core_state.STATUS_IMPOSSIBLE]
        warned = [item for item in props.sticks if item.warning_list()]
        if errors:
            box = layout.box()
            box.alert = True
            box.label(text="%d stick(s) impossible -- see reason" % len(errors), icon="CANCEL")
        if warned:
            layout.label(text="%d stick(s) carry warnings" % len(warned), icon="ERROR")


class SO100_PT_plan(SO100PanelBase, Panel):
    """Sec 10.2. Export lands in Phase D."""

    bl_idname = "SO100_PT_plan"
    bl_label = "Plan"

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def draw(self, context):
        layout = self.layout
        props = context.scene.so100

        row = layout.row(align=True)
        row.scale_y = 1.4
        row.operator("so100.compute_build_order", icon="SORTSIZE")
        row.operator("so100.clear_build_order", text="", icon="TRASH")

        if not props.has_order:
            layout.label(text="No build order yet.", icon="INFO")
            return

        layout.label(text=props.order_summary, icon="PRESET")
        layout.prop(props, "sort_by_build_order")

        errors = [w for w in props.order_warnings if w.is_error]
        warnings = [w for w in props.order_warnings if not w.is_error]

        if errors:
            box = layout.box()
            box.alert = True
            box.label(text="Errors", icon="CANCEL")
            for entry in errors[:6]:
                self._draw_entry(box, entry)
            if len(errors) > 6:
                box.label(text="... and %d more" % (len(errors) - 6))

        if warnings:
            box = layout.box()
            box.label(text="Warnings (Sec 6.2)", icon="ERROR")
            counts = {}
            for entry in warnings:
                counts[entry.code] = counts.get(entry.code, 0) + 1
            for code, count in sorted(counts.items()):
                box.label(text="%s x %d" % (code.replace("_", " "), count))

    @staticmethod
    def _draw_entry(box, entry):
        column = box.column(align=True)
        header = entry.code.replace("_", " ")
        if entry.stick_id:
            header = "%s: %s" % (entry.stick_id, header)
        column.label(text=header)
        for line in _wrap(entry.message, 44):
            column.label(text="   " + line)


class SO100_PT_build(SO100PanelBase, Panel):
    """Sec 10.3. Under Option C there is no live action button -- the panel
    shows the **next stick to load** (the one thing the human needs at the
    feeder) and syncs status from the JSON sidecar the ROS2 side writes."""

    bl_idname = "SO100_PT_build"
    bl_label = "Build"

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def draw(self, context):
        layout = self.layout
        props = context.scene.so100

        row = layout.row(align=True)
        row.operator("so100.export_build_file", icon="EXPORT")
        row.operator("so100.export_cut_list", text="", icon="FILE_TEXT")

        if props.build_file_path:
            box = layout.box()
            box.label(text=bpy.path.basename(props.build_file_path), icon="FILE")
            row = box.row(align=True)
            row.operator("so100.sync_status", text="Sync", icon="IMPORT")
            row.operator("so100.write_status_sidecar", text="Write", icon="EXPORT")

        if not props.has_order:
            layout.label(text="Compute a build order first.", icon="INFO")
            return

        upcoming = _next_stick_item(props)
        box = layout.box()
        if upcoming is None:
            box.label(text="Build complete.", icon="CHECKMARK")
        else:
            # The one thing the operator needs at the feeder: which stick,
            # and how long to cut it (Q1 / A.4 step 1).
            box.label(text="Next to load", icon="FORWARD")
            column = box.column(align=True)
            column.scale_y = 1.3
            column.label(text="%s   %.1f mm" % (upcoming.stick_id,
                                                upcoming.stick_length_mm))
            sub = box.column(align=True)
            sub.label(text="build order %d of %d"
                           % (upcoming.order + 1, _ordered_count(props)))
            for code in upcoming.warning_list():
                sub.label(text=code.replace("_", " "), icon="ERROR")
            if upcoming.reason:
                warn = box.box()
                warn.alert = True
                for line in _wrap(upcoming.reason, 44):
                    warn.label(text=line)

            row = box.row(align=True)
            row.operator("so100.mark_stick", text="Placed",
                         icon="KEYFRAME_HLT").status = "placed"
            row.operator("so100.mark_stick", text="Failed",
                         icon="ERROR").status = "failed"
            row.operator("so100.mark_stick", text="Skip",
                         icon="X").status = "skipped"

        counts = {}
        for item in props.sticks:
            counts[item.status] = counts.get(item.status, 0) + 1
        summary = layout.row(align=True)
        summary.label(text="placed %d / %d"
                           % (counts.get("placed", 0), len(props.sticks)))
        if counts.get("failed"):
            summary.label(text="%d failed" % counts["failed"], icon="ERROR")

        if props.status_conflicts:
            # Sec 9.3: never silently pick one -- show both and let the user
            # decide.
            box = layout.box()
            box.alert = True
            box.label(text="Sidecar disagrees with this .blend", icon="ERROR")
            for line in _wrap(props.status_conflicts, 44):
                box.label(text=line)
            box.label(text="Nothing was changed for those sticks.")

        layout.operator("so100.reset_build_progress", icon="LOOP_BACK")


def _ordered_count(props):
    return sum(1 for item in props.sticks if item.order >= 0)


def _next_stick_item(props):
    """The first stick in build order that is not already placed."""
    ordered = sorted((i for i in props.sticks if i.order >= 0),
                     key=lambda i: i.order)
    for item in ordered:
        if item.status != "placed":
            return item
    return None


class SO100_PT_preview(SO100PanelBase, Panel):
    """Sec 10.4: a rig posed by the vendored FK, scrubbed through the build
    order -- a printer-style preview of the whole job before anything
    moves. No telemetry / live robot link (Option C has none, Sec 2)."""

    bl_idname = "SO100_PT_preview"
    bl_label = "Preview"

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def draw(self, context):
        layout = self.layout
        props = context.scene.so100

        row = layout.row(align=True)
        row.operator(
            "so100.mirror_toggle",
            text="Hide Robot Mirror" if props.show_mirror else "Show Robot Mirror",
            icon="ARMATURE_DATA",
            depress=props.show_mirror,
        )

        if not props.has_order:
            layout.label(text="Compute a build order first.", icon="INFO")
            return

        if props.show_mirror:
            row = layout.row(align=True)
            row.operator("so100.mirror_step", text="", icon="TRIA_LEFT").direction = -1
            row.label(text="%d / %d" % (props.mirror_index + 1, _ordered_count(props)))
            row.operator("so100.mirror_step", text="", icon="TRIA_RIGHT").direction = 1
            if props.mirror_status:
                layout.label(text=props.mirror_status)


class SO100_PT_sticks(SO100PanelBase, Panel):
    bl_idname = "SO100_PT_sticks"
    bl_label = "Sticks"

    @classmethod
    def poll(cls, context):
        return len(context.scene.so100.sticks) > 0

    def draw(self, context):
        layout = self.layout
        props = context.scene.so100

        layout.template_list(
            "SO100_UL_sticks", "", props, "sticks", props, "active_stick_index", rows=8
        )

        if 0 <= props.active_stick_index < len(props.sticks):
            item = props.sticks[props.active_stick_index]
            box = layout.box()
            box.label(text=item.stick_id, icon="IPO_LINEAR")

            column = box.column(align=True)
            # Sec 5.2.3: show design edge -> shared ends -> required edge ->
            # residual, per stick.
            column.label(text="Stick length:  %.2f mm  (cut this)" % item.stick_length_mm)
            column.label(text="Shared ends:   %d" % item.shared_ends)
            column.label(text="Expanded edge: %.3f mm" % item.expanded_edge_mm)
            column.label(text="Residual:      %+.4f mm" % item.residual_mm)

            box.prop(item, "flip")

            box.prop(props, "highlight_previous_sticks")
            row = box.row(align=True)
            row.operator("so100.step_stick", text="", icon="TRIA_LEFT").direction = -1
            row.operator("so100.select_stick_in_viewport", text="Check By Eye",
                        icon="VIEWZOOM")
            row.operator("so100.step_stick", text="", icon="TRIA_RIGHT").direction = 1

            if props.has_order and item.order >= 0:
                box.label(text="Build position: %d of %d" % (
                    item.order + 1, len(props.sticks)))
                row = box.row(align=True)
                row.operator("so100.move_build_step", text="Move Earlier",
                             icon="TRIA_UP").direction = -1
                row.operator("so100.move_build_step", text="Move Later",
                             icon="TRIA_DOWN").direction = 1

            for code in item.warning_list():
                box.label(text=code.replace("_", " "), icon="ERROR")
            if item.reason:
                sub = box.box()
                sub.alert = True
                for line in _wrap(item.reason, 44):
                    sub.label(text=line)

        layout.operator("so100.export_cut_list", icon="FILE_TEXT")


class SO100_PT_reference(SO100PanelBase, Panel):
    bl_idname = "SO100_PT_reference"
    bl_label = "Reference"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        props = context.scene.so100
        profile = core_robots.get_robot(props.robot_id)
        frame = _FRAME_NAMES.get(props.robot_id, props.robot_id)

        column = layout.column(align=True)
        column.label(text="Build volume (%s, %s):" % (frame, props.robot_id))
        if profile.has_build_volume:
            lo, hi = profile.build_volume_min_m, profile.build_volume_max_m
            column.label(text="  X %+.0f .. %+.0f mm" % (lo[0] * 1000.0, hi[0] * 1000.0))
            column.label(text="  Y %+.0f .. %+.0f mm" % (lo[1] * 1000.0, hi[1] * 1000.0))
            column.label(text="  Z %+.0f .. %+.0f mm" % (lo[2] * 1000.0, hi[2] * 1000.0))
        else:
            column.label(text="  Not confirmed for this robot yet", icon="ERROR")

        if profile.has_base_box:
            column = layout.column(align=True)
            column.label(text="Robot base box (viewport reference only):")
            lo, hi = profile.base_box_min_m, profile.base_box_max_m
            column.label(text="  X %+.0f .. %+.0f mm" % (lo[0] * 1000.0, hi[0] * 1000.0))
            column.label(text="  Y %+.0f .. %+.0f mm" % (lo[1] * 1000.0, hi[1] * 1000.0))
            column.label(text="  Z %+.0f .. %+.0f mm" % (lo[2] * 1000.0, hi[2] * 1000.0))

        # so_arm_100-specific empirical facts (ROS2 doc Sec 9.4's N2 finding,
        # and its own GRASP_OFFSET_M caveat) -- do not generalize these to a
        # robot they were never measured against.
        if props.robot_id == core_robots.SO_ARM_100_ID:
            # Not a %-format string, so a literal "%" here (not "%%") --
            # found while adding translations (i18n.py), which is why the
            # dict key below has to match this exact corrected text.
            layout.label(text="98% reachable for vertical sticks.", icon="INFO")
            box = layout.box()
            box.label(text="Grip height %.0f mm (derived, not measured)"
                           % (kc.GRASP_OFFSET_M * 1000.0), icon="ERROR")


def _wrap(text, width):
    words, lines, current = text.split(), [], ""
    for word in words:
        candidate = (current + " " + word).strip()
        if len(candidate) > width and current:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    return lines


_CLASSES = (
    SO100_UL_sticks,
    SO100_PT_design,
    SO100_PT_summary,
    SO100_PT_plan,
    SO100_PT_build,
    SO100_PT_preview,
    SO100_PT_sticks,
    SO100_PT_reference,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
