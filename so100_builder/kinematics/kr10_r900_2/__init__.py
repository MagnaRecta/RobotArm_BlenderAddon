"""Closed-form FK/IK for the KUKA KR10 R900-2, a genuine 6-DOF spherical-
wrist arm. Pure Python, stdlib only (``math``), zero ROS imports -- mirrors
``so_arm_100_kinematics``'s own vendoring constraint (see README.md);
not currently vendored anywhere, since only SO-100 feeds the Blender addon
today.
"""

# Bump on ANY change to chain.py, constants.py, grasp.py or jaw_clearance.py
# that alters computed results -- matches so_arm_100_kinematics'
# kinematics_version convention (BRIDGE_PROTOCOL.md), for the day this
# robot's own build files start recording one.
#
# 1.0.0 (2026-08-21): initial release -- closed-form 6-DOF FK/IK (Pieper
# spherical-wrist decoupling, 4 branches: elbow_up x wrist_flip), grasp
# orientation transform (round-stock, free-roll), jaw clearance, envelope.
__version__ = "1.0.0"

from .chain import Unreachable, fk, ik, translate_holding_wrist
from .constants import (
    CHAIN,
    GRASP_OFFSET_M,
    JAW_CONTACT_HALF_LENGTH_M,
    JAW_RADIUS_M,
    JOINT_ALLOWANCE_M,
    JOINT_NAMES,
    MIN_GRASP_OFFSET_M,
    STICK_COLLISION_RADIUS_M,
    STICK_LENGTH_RANGE_M,
    STICK_SECTION_M,
)
from .envelope import is_reachable, sweep_envelope
from .grasp import (
    DEFAULT_ROLL_SWEEP_STEPS,
    grasp_offset_for_length,
    grasp_target,
    iter_stick_placements,
    iter_stick_placements_any_roll,
    iter_vertical_poses,
    orientation_from_stick_axis,
    orientation_pointing_down,
    solve_stick_placement,
    solve_stick_placement_any_roll,
    stick_axis,
)
from .jaw_clearance import check_jaw_clearance, jaw_swept_capsule, segment_distance

__all__ = [
    "__version__",
    "fk",
    "ik",
    "translate_holding_wrist",
    "Unreachable",
    "is_reachable",
    "sweep_envelope",
    "JOINT_NAMES",
    "CHAIN",
    "GRASP_OFFSET_M",
    "STICK_SECTION_M",
    "STICK_LENGTH_RANGE_M",
    "JOINT_ALLOWANCE_M",
    "grasp_target",
    "grasp_offset_for_length",
    "stick_axis",
    "orientation_from_stick_axis",
    "solve_stick_placement",
    "solve_stick_placement_any_roll",
    "iter_stick_placements",
    "iter_stick_placements_any_roll",
    "orientation_pointing_down",
    "iter_vertical_poses",
    "DEFAULT_ROLL_SWEEP_STEPS",
    "check_jaw_clearance",
    "jaw_swept_capsule",
    "segment_distance",
    "JAW_RADIUS_M",
    "STICK_COLLISION_RADIUS_M",
    "JAW_CONTACT_HALF_LENGTH_M",
    "MIN_GRASP_OFFSET_M",
]
