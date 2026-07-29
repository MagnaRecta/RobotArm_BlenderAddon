"""SO-100 Stick Builder -- design and slicing tool for a stick-gluing robot.

The user models a sculpture as a wireframe mesh (every edge = one 6.45 mm
square wooden stick, 80-150 mm long). This addon extracts the sticks, grows
the design so fixed-length sticks fit with a glue gap at every joint,
computes a build order, validates every placement against the robot's real
kinematics, and exports a build file that a separate ROS2 process executes.

**There is deliberately no live connection to ROS2** (BLENDER_ADDON_PLAN.md
Sec 2, "Option C"). The build file is the entire integration surface. The
kinematics module in ``kinematics/`` is a verbatim vendored copy of the
robot side's ``so_arm_100_kinematics`` -- never fork it here; see that
package's README.

Metadata lives in ``blender_manifest.toml``, not in a legacy ``bl_info``
dict (constraint B8, verified against the Blender 5.2 manual).
"""

# ``core/`` and ``kinematics/`` are pure Python by design, so this package
# must stay importable in a bare interpreter for the offline test run
# (tests/run.py). Only the UI layer actually needs Blender.
try:
    import bpy
except ImportError:  # pragma: no cover -- the bare-CPython test path
    bpy = None

from . import kinematics

__version__ = "0.1.0"
KINEMATICS_VERSION = kinematics.__version__

if bpy is not None:
    from . import prefs, properties
    from .ops import build as ops_build
    from .ops import design as ops_design
    from .ops import order as ops_order
    from .ui import overlay, panels

    _MODULES = (prefs, properties, ops_design, ops_order, ops_build, panels,
                overlay)

    def register():
        for module in _MODULES:
            module.register()

    def unregister():
        for module in reversed(_MODULES):
            module.unregister()

else:  # pragma: no cover

    def register():
        raise RuntimeError("so100_builder.register() requires Blender")

    def unregister():
        raise RuntimeError("so100_builder.unregister() requires Blender")
