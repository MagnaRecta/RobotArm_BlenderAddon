"""SO-100 Stick Builder -- design and slicing tool for a stick-gluing robot.

The user models a sculpture as a wireframe mesh (every edge = one wooden
stick, 50-150 mm long -- BLENDER_ADDON_PLAN.md Sec 5.4). This addon extracts
the sticks, grows the design so fixed-length sticks fit with a glue gap at
every joint, computes a build order, validates every placement against the
target robot's real kinematics, and exports a build file that a separate
ROS2 process executes.

**There is deliberately no live connection to ROS2** (BLENDER_ADDON_PLAN.md
Sec 2, "Option C"). The build file is the entire integration surface.

**Not tied to one robot** (BRIDGE_PROTOCOL.md Sec A.1.1, multi-robot support
added 2026-08-21): ``kinematics/`` holds one vendored, verbatim kinematics
package per supported robot id (``kinematics/so_arm_100/`` today, plus a
``kinematics/kr10_r900_2/`` placeholder) -- never fork one; see that
sub-package's own README/docstring -- and ``core/robots.py`` is the registry
+ interface contract that lets the rest of the addon target whichever one
the Scene's ``robot_id`` property selects.

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

from . import kinematics  # noqa: F401 -- import side effect: registers the sub-packages

__version__ = "0.1.0"

if bpy is not None:
    from . import i18n, prefs, properties
    from .ops import build as ops_build
    from .ops import design as ops_design
    from .ops import mirror as ops_mirror
    from .ops import order as ops_order
    from .ui import overlay, panels

    _MODULES = (i18n, prefs, properties, ops_design, ops_order, ops_build,
                ops_mirror, panels, overlay)

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
