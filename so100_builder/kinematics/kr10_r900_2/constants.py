"""Geometry and limits for the KUKA KR10 R900-2, 6-DOF arm.

Every number here is read off
``kr10_r900_2_description/urdf/kr10_r900_2_description.urdf.xacro`` -- this
file has no other source of truth. If the URDF changes, update here and
re-run ``test/test_chain.py``.

Pure Python, stdlib only, zero ROS imports -- mirrors
``so_arm_100_kinematics``'s own vendoring constraint (see this package's own
README) in case a future Blender-side validation for this robot is ever
wanted; not currently vendored anywhere.

Unlike SO-100's 5-DOF arm, all six of this robot's joint origins have
``rpy="0 0 0"`` in the URDF -- every axis below is a literal, unrotated
+/-X/+/-Y/+/-Z vector, not something folded in through an origin rotation.
This is a completely standard 6R "elbow manipulator" layout: joint1 yaws
about vertical, joint2/joint3 pitch about a horizontal axis (the classic
shoulder/elbow pair), joint4/joint5/joint6 form a spherical wrist (see
KUKA_IMPLEMENTATION_PLAN.md Sec 1.3 and this package's own chain.py
docstring for why joint4/5/6's axis lines all intersect at one point despite
the nonzero link lengths between them).
"""

import math

# Each entry: (name, origin_xyz_m, axis, limit_lower_rad, limit_upper_rad).
# origin_xyz is the URDF <joint><origin> value (child frame relative to
# parent; origin rpy is always identity for these six joints, so it is not
# a field here); axis is the URDF <joint><axis>, already a literal
# +/-1 unit vector (never rotated by an origin rpy).
CHAIN = (
    ("joint1", (0.0, 0.0, 0.400), (0.0, 0.0, -1.0), math.radians(-170.0), math.radians(170.0)),
    ("joint2", (0.025, 0.0, 0.0), (0.0, 1.0, 0.0), math.radians(-190.0), math.radians(45.0)),
    ("joint3", (0.455, 0.0, 0.0), (0.0, 1.0, 0.0), math.radians(-120.0), math.radians(156.0)),
    ("joint4", (0.0, 0.0, 0.025), (-1.0, 0.0, 0.0), math.radians(-185.0), math.radians(185.0)),
    ("joint5", (0.420, 0.0, 0.0), (0.0, 1.0, 0.0), math.radians(-120.0), math.radians(120.0)),
    # Was +/-350deg (a generic assumption). Confirmed against real hardware
    # 2026-08-25: the physical A6 axis does not move past +/-180deg -- a
    # real hardware limit, not a software one. Must always match
    # kr10_r900_2_description.urdf.xacro's own joint6 <limit> (this
    # package's own "no other source of truth" rule, module docstring).
    ("joint6", (0.090, 0.0, 0.0), (-1.0, 0.0, 0.0), math.radians(-180.0), math.radians(180.0)),
)

JOINT_NAMES = tuple(entry[0] for entry in CHAIN)

# --- Fixed gripper attachment, joint6/link_6 -> gripper_tcp -----------------
# Three fixed (non-actuated) transforms in series, all read directly off the
# URDF -- gripper_clock_joint (mount clocking), gripper_mount_joint (mesh
# alignment), gripper_tcp_joint (the derived TCP offset, see
# KUKA_IMPLEMENTATION_PLAN.md Sec 3 Phase 2 for how its z=0.0805m was
# back-solved). Folded into one constant (translation, rotation) pair by
# chain.py at import time -- see TOOL_OFFSET_XYZ_M / TOOL_OFFSET_ROT below.
GRIPPER_CLOCK_ANGLE_RAD = math.radians(-45.0)
GRIPPER_CLOCK_XYZ_M = (0.03, 0.0, 0.0)
GRIPPER_MOUNT_PITCH_RAD = math.radians(90.0)
GRIPPER_TCP_Z_M = 0.0805

# --- Grasp / stick geometry -------------------------------------------------
# Axial offset from the stick's BASE end (the physical stop,
# KUKA_IMPLEMENTATION_PLAN.md KQ5) to the point the jaws close on, along the
# stick's own axis. Derived 2026-08-21 from compute_fk of gripper_tcp at the
# hand-tuned 'lower' pose vs. the physical-stop coordinate -- NOT yet
# confirmed with a ruler; see KUKA_IMPLEMENTATION_PLAN.md Sec 3 Phase 2 for
# the full derivation (including a ~1.25mm unexplained Y residual).
GRASP_OFFSET_M = 0.0188

# Jaw geometry, from this project's earlier gripper CAD analysis (finger
# mesh bounding box 23.65 x 15.39 x 36.8 mm -- see
# kr10_r900_2_description/meshes/*/gripper_*.stl). Which raw bbox dimension
# maps to "along the stick axis" vs. "the jaws' own radial reach" was not
# independently re-measured against the mesh's own local axes while writing
# this file -- both constants below are therefore rougher estimates than
# GRASP_OFFSET_M above, in the same spirit as so_arm_100_kinematics'
# JAW_RADIUS_M/JAW_CONTACT_HALF_LENGTH_M (see that package's own README
# caveats). Confirm against real hardware or a fresh mesh-axis check before
# trusting either for a real build.
JAW_CONTACT_HALF_LENGTH_M = 0.008
MIN_GRASP_OFFSET_M = 0.010

# The KUKA feeder's grasp point is FIXED in the fixture's own frame --
# KUKA_IMPLEMENTATION_PLAN.md Sec 0 point 2: "the gripper always grasps at
# the same fixed vertical-hole location regardless of stick length." Unlike
# SO-100 (where the grip point is chosen per-stick, up from whichever
# stick's own base), grasp_offset_for_length() is therefore NOT needed for
# Phase 1's feeder grasp -- GRASP_OFFSET_M (18.8mm) plus
# JAW_CONTACT_HALF_LENGTH_M (8mm) = 26.8mm sits safely inside even the
# shortest allowed stick (35mm, STICK_LENGTH_RANGE_M below) with margin to
# spare. The function is still provided (mirroring so_arm_100_kinematics'
# public API) for Phase 5's general stick-placement grasp, where a stick may
# need to be re-gripped somewhere other than the fixed feeder location.

STICK_SECTION_M = 0.002  # round stock, 2mm diameter (KQ6) -- see stick_axis()
STICK_LENGTH_RANGE_M = (0.035, 0.150)  # KQ3, 2026-08-21
JOINT_ALLOWANCE_M = 0.001  # 1mm/end, given directly by the user (KQ3) -- NOT
# re-derived from so_arm_100_kinematics' square-stock w/(2*tan(theta/2))
# formula, which assumes flat mating faces and does not describe round-on-
# round contact the same way. Revisit only if real joints show gaps or
# interference (KUKA_IMPLEMENTATION_PLAN.md Sec 3 Phase 2).

# --- Jaw clearance -----------------------------------------------------------
# Half the finger mesh's own narrowest bbox dimension (15.39mm), the same
# "rough estimate, not measured" status as JAW_CONTACT_HALF_LENGTH_M above.
JAW_RADIUS_M = 0.0077

# Already-placed sticks are checked as capsules too: the round stock's own
# radius (half of STICK_SECTION_M) plus a small inflation, matching
# so_arm_100_kinematics' "consider 1-2mm inflation" convention -- no
# circumscribed-square correction needed here since the stock is round.
STICK_COLLISION_INFLATION_M = 0.0015
STICK_COLLISION_RADIUS_M = STICK_SECTION_M / 2.0 + STICK_COLLISION_INFLATION_M
