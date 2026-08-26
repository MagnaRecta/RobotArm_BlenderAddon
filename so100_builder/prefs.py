"""AddonPreferences. BLENDER_ADDON_PLAN.md Sec 4.

Deliberately thin. Almost everything the user tunes is per-design and lives
on the Scene (``properties.py``) so it travels with the ``.blend``; only
machine-level choices belong here.
"""

import bpy
from bpy.props import BoolProperty, StringProperty
from bpy.types import AddonPreferences

from .core import robots as core_robots


class SO100BuilderPreferences(AddonPreferences):
    # Must match the extension's package name. Under the 5.2 extensions
    # platform that is the full `bl_ext.<repo>.<id>` module path, which
    # __package__ gives us for free.
    bl_idname = __package__

    build_file_dir: StringProperty(
        name="Build File Folder", subtype="DIR_PATH", default="//",
        description="Where Export Build File starts. The build file is the ONLY "
                    "interface to ROS2 -- move it by shared folder, USB, git or "
                    "scp, whichever is convenient",
    )
    warn_on_unverified_roll: BoolProperty(
        name="Warn on non-zero stick roll", default=True,
        description="The stick_roll -> Wrist_Roll sign convention has never been "
                    "tested on hardware; every test so far used roll = 0",
    )

    def draw(self, _context):
        layout = self.layout

        column = layout.column(align=True)
        column.prop(self, "build_file_dir")
        column.prop(self, "warn_on_unverified_roll")

        box = layout.box()
        box.label(text="Vendored kinematics", icon="CON_KINEMATIC")
        for profile in core_robots.ROBOTS.values():
            row = box.row()
            row.label(text="%s: %s" % (profile.id, profile.kinematics.__version__))
        box.label(
            text="Written into every build file as robot/kinematics_version. "
                "ROS2 refuses to execute on a mismatch.",
            icon="INFO",
        )

        so_arm_100 = core_robots.get_robot(core_robots.SO_ARM_100_ID).kinematics
        caveats = layout.box()
        caveats.label(text="Unverified assumptions (so_arm_100)", icon="ERROR")
        column = caveats.column(align=True)
        column.label(text="GRASP_OFFSET_M = %.3f m is derived from FK, not measured."
                          % so_arm_100.GRASP_OFFSET_M)
        column.label(text="The stick_roll -> Wrist_Roll sign is untested on hardware.")
        column.label(text="Validation here is an upper bound; MoveIt has the final say.")


def register():
    bpy.utils.register_class(SO100BuilderPreferences)


def unregister():
    bpy.utils.unregister_class(SO100BuilderPreferences)
