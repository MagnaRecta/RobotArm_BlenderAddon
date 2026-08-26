"""PropertyGroups on Scene and Object. BLENDER_ADDON_PLAN.md Sec 9.

These *are* the ``.blend`` half of QB4's dual persistence: Blender saves
PropertyGroups with the file, so the live state travels with the design.
The JSON sidecar half is written by ``io/`` using ``core.state``.
"""

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import PropertyGroup

from .core import robots as core_robots
from .core import sticks as core_sticks
from .core import state as core_state
# ⚠ Hardcoded to so_arm_100, same as core/sticks.py -- the UI defaults below
# (stock section, length range, ...) stay so_arm_100's numbers regardless of
# `robot_id` until a second robot's real numbers exist to switch to (see
# core/robots.py's module docstring and BLENDER_ADDON_PLAN.md's registry
# section for what is and is not yet robot-parameterized).
from .kinematics.so_arm_100 import constants as kc

# The integer attribute layer that makes ids stable across mesh edits
# (Sec 9.2). Lives on the EDGE domain of the design mesh.
EDGE_ID_LAYER = "so100_stick_id"

STATUS_ITEMS = [
    (core_state.STATUS_PENDING, "Pending", "Not yet validated", "DOT", 0),
    (core_state.STATUS_BUILDABLE, "Buildable", "Validated and reachable", "CHECKMARK", 1),
    (core_state.STATUS_IMPOSSIBLE, "Impossible", "Cannot be built -- see the reason", "CANCEL", 2),
    (core_state.STATUS_PLACED, "Placed", "Glued in place on the real sculpture", "KEYFRAME_HLT", 3),
    (core_state.STATUS_FAILED, "Failed", "The robot could not place it", "ERROR", 4),
    (core_state.STATUS_SKIPPED, "Skipped", "Deliberately passed over", "X", 5),
]


def _mesh_object_poll(_self, obj):
    return obj is not None and obj.type == "MESH"


def _empty_object_poll(_self, obj):
    return obj is not None and obj.type == "EMPTY"


class SO100StickItem(PropertyGroup):
    """One stick, as shown in the Build panel's UIList (Sec 9.2's table)."""

    stick_id: StringProperty(
        name="ID", description="Stable identifier -- never reused", default=""
    )
    order: IntProperty(
        name="Order", description="Build order index; -1 until an order is computed",
        default=-1,
    )
    status: EnumProperty(name="Status", items=STATUS_ITEMS,
                         default=core_state.STATUS_PENDING)
    reason: StringProperty(
        name="Reason",
        description="Specific, actionable explanation for impossible/failed",
        default="",
    )
    stick_length_mm: FloatProperty(
        name="Stick Length",
        description="PHYSICAL length to cut -- the design edge length, after any "
                    "stock snapping. This is what the human cuts and loads",
        default=0.0, unit="NONE", precision=2,
    )
    expanded_edge_mm: FloatProperty(
        name="Expanded Edge",
        description="Solved edge length after mesh expansion -- longer than the "
                    "stick by the joint gaps, and NOT what gets cut",
        default=0.0, precision=3,
    )
    residual_mm: FloatProperty(
        name="Residual",
        description="How far the solved edge missed its required length",
        default=0.0, precision=4,
    )
    shared_ends: IntProperty(name="Shared Ends", default=0, min=0, max=2)
    flip: BoolProperty(
        name="Flip",
        description="Manual override of which end is the base (the end that seats down)",
        default=False,
    )
    warnings: StringProperty(
        name="Warnings", description="Comma-separated warning codes", default=""
    )

    def warning_list(self):
        return [w for w in self.warnings.split(",") if w]


class SO100WarningItem(PropertyGroup):
    """One entry in the Plan panel's warnings list (Sec 10.2 / Sec 6.2)."""

    stick_id: StringProperty(name="Stick", default="")
    code: StringProperty(name="Code", default="")
    message: StringProperty(name="Message", default="")
    is_error: BoolProperty(name="Is Error", default=False)


class SO100SceneProps(PropertyGroup):
    """Everything the Design panel drives (Sec 10.1)."""

    # --- multi-robot (BRIDGE_PROTOCOL.md Sec A.1.1) -------------------------
    robot_id: EnumProperty(
        name="Robot",
        description="Which robot this design targets. Stamped into the "
                    "exported build file as `robot` (BRIDGE_PROTOCOL.md Sec "
                    "A.1.1/A.2) -- the executor refuses to run a file meant "
                    "for a different arm. Selecting a robot whose kinematics "
                    "package isn't vendored yet degrades cleanly: validation, "
                    "the mirror rig and Export Build File all report a clear "
                    "reason instead of silently using the wrong arm's geometry",
        items=[
            (core_robots.SO_ARM_100_ID, "SO-100 (5-DOF)",
             "so_arm_100_kinematics -- testing/dev rig. Closed-form IK, "
             "vendored and hardware-validated"),
            (core_robots.KR10_R900_2_ID, "KUKA KR10 R900-2 (6-DOF)",
             "kr10_r900_2_kinematics -- production rig. Vendored 2026-08-22, "
             "genuine 6-DOF closed-form IK"),
        ],
        default=core_robots.DEFAULT_ROBOT_ID,
    )

    # --- scene pointers (Sec 9.1) ------------------------------------------
    base_empty: PointerProperty(
        type=bpy.types.Object, poll=_empty_object_poll,
        name="Robot Base",
        description="An Empty marking the selected robot's own URDF root frame "
                    "(`base_link` for so_arm_100, `base` for kr10_r900_2 -- see "
                    "each kinematics package's own README). Moving it repositions "
                    "the whole design relative to the robot with no re-authoring",
    )
    design_mesh: PointerProperty(
        type=bpy.types.Object, poll=_mesh_object_poll,
        name="Design Mesh",
        description="The wireframe mesh. Every edge becomes one wooden stick",
    )
    build_mesh: PointerProperty(
        type=bpy.types.Object, poll=_mesh_object_poll,
        name="Build Mesh",
        description="Derived, expanded mesh generated by the addon. Never edit it "
                    "by hand -- it is regenerated on every extraction",
    )

    # --- stock (Sec 10.1) ---------------------------------------------------
    section_mm: FloatProperty(
        name="Stock Section", default=kc.STICK_SECTION_M * 1000.0,
        min=0.5, max=50.0, precision=3,
        description="Square stock cross-section. 6.45 mm is the project's stock",
    )
    joint_allowance_mm: FloatProperty(
        name="Joint Allowance", default=kc.JOINT_ALLOWANCE_M * 1000.0,
        min=0.0, max=50.0, precision=3,
        description="How far each stick stops short of a shared vertex. The default "
                    "is half the stock section, which is exact for a 90 degree "
                    "joint. Raise it if shallow joints are warned about",
    )

    # --- Sec 5.3: length modes ---------------------------------------------
    length_mode: EnumProperty(
        name="Length Mode",
        items=[
            ("DESIGN", "Design-driven",
             "Stick length = the edge length as drawn. Maximum design freedom, "
             "every stick potentially unique", "MOD_LENGTH", 0),
            ("STOCK", "Fixed stock lengths",
             "Snap each edge to the nearest available stock length BEFORE "
             "expanding. The whole build then uses a handful of pre-cut lengths",
             "SNAP_INCREMENT", 1),
        ],
        default="DESIGN",
    )
    stock_lengths_mm: StringProperty(
        name="Stock Lengths", default="80, 100, 120, 150",
        description="Comma-separated lengths available at the saw",
    )

    # --- Sec 5.2.1 / N7 -----------------------------------------------------
    growth_mode: EnumProperty(
        name="Growth",
        items=[
            ("PER_EDGE", "Per-edge",
             "required_edge = stick + allowance x shared_ends. Free ends land "
             "exactly on the design vertex", "MOD_EDGESPLIT", 0),
            ("UNIFORM", "Uniform",
             "Allowance at every end, so every edge grows by exactly 2x the "
             "allowance. Free ends overhang harmlessly; far better conditioned "
             "on looped structures", "MOD_ARRAY", 1),
        ],
        default="PER_EDGE",
    )
    ground_mode: EnumProperty(
        name="Grounded Vertices",
        items=[
            ("SLIDE", "Slide on the plate",
             "Grounded vertices stay at z=0 -- nothing expands below the physical "
             "base plate -- but may slide across it as the design grows. Solves "
             "flat first layers exactly", "SNAP_FACE", 0),
            ("PIN", "Pin",
             "Grounded vertices are fully immobile. A first layer that forms a "
             "closed ring cannot expand at all under this mode and every one of "
             "its edges reports as impossible", "PINNED", 1),
        ],
        default="SLIDE",
    )
    require_build_plate: BoolProperty(
        name="Require Build Plate", default=True,
        description="Uncheck for a design held by something this addon does not "
                    "model at all -- a stick's own base used as a jig, a non-flat "
                    "fixture -- rather than by a flat plate at any height. No "
                    "vertex is then checked against a plate, nothing is ever "
                    "reported as floating, and each disconnected part of the "
                    "design starts from an arbitrary point instead of a grounded "
                    "one. Does not verify the result is physically self-"
                    "supporting -- you are responsible for how it is actually "
                    "held during the build",
    )

    # --- Sec 5.4: limits ----------------------------------------------------
    min_stick_length_mm: FloatProperty(
        name="Min Stick Length", default=kc.STICK_LENGTH_RANGE_M[0] * 1000.0,
        # The widget's own min= bound is fixed at class-registration time,
        # so it cannot depend on which robot is selected -- it uses the
        # SAFEST (lowest) floor across every registered robot, permissive
        # enough to never block a value valid for whichever one is actually
        # selected. The real, robot-SPECIFIC floor is enforced by
        # extract_sticks() itself at extraction time (core/robots.py's
        # module docstring; core/sticks.py's own note on the same limit).
        min=core_sticks.safe_min_stick_length_bound_m() * 1000.0, max=1000.0, precision=1,
        description="Stock threshold. Checked against the SELECTED robot's own "
                    "hard physical floor (min grasp offset + jaw contact "
                    "half-length) at extraction time -- below that, no offset "
                    "both fits within the stick and clears the floor-clearance "
                    "floor",
    )
    max_stick_length_mm: FloatProperty(
        name="Max Stick Length", default=kc.STICK_LENGTH_RANGE_M[1] * 1000.0,
        min=1.0, max=1000.0, precision=1,
    )

    # --- solver tolerances --------------------------------------------------
    merge_tolerance_mm: FloatProperty(
        name="Merge Distance", default=core_sticks.DEFAULT_MERGE_TOLERANCE_M * 1000.0,
        min=0.0, max=10.0, precision=3,
        description="Vertices closer than this are the same structural joint",
    )
    residual_tolerance_mm: FloatProperty(
        name="Residual Tolerance",
        default=core_sticks.DEFAULT_RESIDUAL_TOLERANCE_M * 1000.0,
        min=0.0, max=10.0, precision=3,
        description="Per-edge length error the solver may leave before the edge "
                    "is reported as one the sticks will not physically fit",
    )
    ground_epsilon_mm: FloatProperty(
        name="Ground Tolerance", default=core_sticks.DEFAULT_GROUND_EPSILON_M * 1000.0,
        min=0.0, max=10.0, precision=3,
        description="A vertex within this distance of the build plate seats on it",
    )
    build_plate_height_mm: FloatProperty(
        name="Build Plate Height", default=0.0,
        min=0.0, max=1000.0, precision=1,
        description="The physical build plate's own Z, in the selected robot's "
                    "base frame. 0 (default) matches every design so far. The "
                    "plate is height-adjustable, so a design that sits entirely "
                    "above Z=0 -- reported as a floating component -- is not "
                    "necessarily unbuildable, just unbuildable at the plate's "
                    "CURRENT height here. Raise this to the component's own "
                    "lowest point (named in the error) to treat it as resting "
                    "on the plate",
    )

    # --- results ------------------------------------------------------------
    sticks: CollectionProperty(type=SO100StickItem)
    active_stick_index: IntProperty(default=0)

    next_stick_number: IntProperty(
        default=1, min=1,
        description="Next stable id to hand out. Never decreases -- ids are "
                    "never reused",
    )
    topology_signature: StringProperty(
        default="",
        description="Fingerprint of the design the current results were computed "
                    "against. A mismatch means the results are stale",
    )

    show_overlay: BoolProperty(
        name="Show Overlay", default=True,
        description="GPU viewport overlay: build volume + per-stick status colours "
                    "(Sec 10.4). Hard off switch -- kept in its own module",
    )

    # --- Phase E: robot mirror (Sec 10.4) -----------------------------------
    show_mirror: BoolProperty(
        name="Show Robot Mirror", default=False,
        description="A rig posed by the vendored FK, showing the arm at the "
                    "current build-order position -- a printer-style preview "
                    "of the whole job before anything moves",
    )
    mirror_index: IntProperty(
        default=0, min=0,
        description="Position within the build order the mirror rig is posed at",
    )
    mirror_status: StringProperty(
        default="", description="Which stick the mirror rig is showing, or why it "
                                "isn't showing anything",
    )

    # --- Sec 6 / Phase C: build order --------------------------------------
    order_warnings: CollectionProperty(type=SO100WarningItem)
    order_summary: StringProperty(default="")
    has_order: BoolProperty(
        default=False,
        description="Whether a build order has been computed for the current "
                    "extraction",
    )
    sort_by_build_order: BoolProperty(
        name="Sort by Build Order", default=True,
        description="List sticks in the order the robot will place them, "
                    "rather than in extraction order",
    )
    backtrack_limit: IntProperty(
        name="Backtrack Limit", default=500, min=0, max=100000,
        description="How hard the order solver may search before reporting "
                    "honestly that it could not find a valid order",
    )
    check_jaw_clearance: BoolProperty(
        name="Check Jaw Clearance", default=True,
        description="Check the grip region against already-placed sticks "
                    "(Sec 6 C3). The jaw envelope is an ESTIMATE until Phase 0 "
                    "measures it, so this warns rather than blocks",
    )
    jaw_width_mm: FloatProperty(
        name="Jaw Width", default=20.0, min=0.0, max=200.0, precision=1,
        description="How far across the jaws are. ESTIMATE -- Phase 0 measures "
                    "the real envelope",
    )

    # --- Phase D: the build file and its status sidecar --------------------
    build_file_path: StringProperty(
        name="Build File", subtype="FILE_PATH", default="",
        description="The exported build file. Its status sidecar sits next to "
                    "it and is what ROS2 writes progress into",
    )
    status_conflicts: StringProperty(
        default="",
        description="Sticks whose .blend state and status sidecar disagree. "
                    "Sec 9.3: show both and let the user choose -- never "
                    "silently pick one",
    )

    summary: StringProperty(default="")
    design_dimensions_mm: StringProperty(default="")
    expanded_dimensions_mm: StringProperty(default="")
    solver_report: StringProperty(default="")
    show_advanced: BoolProperty(name="Advanced", default=False)

    def stock_lengths_m(self):
        """Parse the stock-length field. Returns ``None`` on design-driven."""
        if self.length_mode != "STOCK":
            return None
        values = []
        for chunk in self.stock_lengths_mm.replace(";", ",").split(","):
            chunk = chunk.strip()
            if not chunk:
                continue
            try:
                values.append(float(chunk) / 1000.0)
            except ValueError:
                raise ValueError("%r is not a number in Stock Lengths" % chunk)
        if not values:
            raise ValueError("Fixed stock length mode needs at least one length")
        return values

    def results_are_stale(self, signature):
        return bool(self.topology_signature) and self.topology_signature != signature


_CLASSES = (SO100StickItem, SO100WarningItem, SO100SceneProps)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.so100 = PointerProperty(type=SO100SceneProps)


def unregister():
    del bpy.types.Scene.so100
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
