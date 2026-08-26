"""Vendored kinematics packages -- one sub-package per supported robot.

This directory holds a verbatim copy of each robot's own pure-Python
kinematics package (BLENDER_ADDON_PLAN.md Sec 4, "the kinematics/ folder is
a verbatim copy, never a fork"), one sub-directory per BRIDGE_PROTOCOL.md
Sec A.1.1 robot id:

    kinematics/
    +-- so_arm_100/    vendored from so_arm_100_kinematics -- hardware-validated
    +-- kr10_r900_2/   placeholder -- kr10_r900_2_kinematics does not exist yet

Neither this package nor its sub-packages know how to pick one for a given
design; that is ``core/robots.py``'s job (the registry + the interface
contract every one of these sub-packages must satisfy). Import a specific
robot's package directly (``from .so_arm_100 import constants``) or, for
robot-agnostic code, go through ``core.robots.get_robot(robot_id).kinematics``.

Adding a third robot means: a new sub-directory here (vendored per that
package's own README, same rules as ``so_arm_100_kinematics``'s), a new
``RobotProfile`` entry in ``core/robots.py``, and a new row in
BRIDGE_PROTOCOL.md Sec A.1.1's table. Nothing else in this directory changes.
"""
