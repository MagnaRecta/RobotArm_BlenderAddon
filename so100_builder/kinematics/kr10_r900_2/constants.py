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
# KUKA_IMPLEMENTATION_PLAN.md Sec 3 Phase 2 for how its original z=0.0805m
# was back-solved). Folded into one constant (translation, rotation) pair by
# chain.py at import time -- see TOOL_OFFSET_XYZ_M / TOOL_OFFSET_ROT below.
GRIPPER_CLOCK_ANGLE_RAD = math.radians(-45.0)
GRIPPER_CLOCK_XYZ_M = (0.03, 0.0, 0.0)
GRIPPER_MOUNT_PITCH_RAD = math.radians(90.0)

# GRIPPER_TCP_Z_M -- re-derived 2026-09-09 after the 2026-09-04 finger STL
# swap (kr10_r900_2_description.urdf.xacro's own gripper_tcp_joint comment
# has the full derivation: both old/new finger meshes' local Z start at the
# same 48.2mm mounting end, but the new mesh's tip is only 79.5mm out vs.
# the old mesh's 85.0mm -- 5.5mm shorter). Real-hardware/RViz finding: the
# held/placed stick rendered visibly lower than the physical new, shorter
# fingers reach. Lowered 2mm (0.0805 -> 0.0785) per the user's own visual
# judgement, not the full 5.5mm a tip-relative rederivation would suggest --
# revisit (up to that 5.5mm figure) if it still looks low.
#
# A further 1mm taken off 2026-09-09 (0.0785 -> 0.0775), same reasoning --
# still short of the 5.5mm ceiling, still just the user's own visual call,
# not a rederivation. Must always match the xacro's own gripper_tcp_joint
# origin z (this module's own "no other source of truth" rule, module
# docstring).
GRIPPER_TCP_Z_M = 0.0775

# --- Grasp / stick geometry -------------------------------------------------
# Axial offset from the stick's BASE end (the physical stop,
# KUKA_IMPLEMENTATION_PLAN.md KQ5) to the point the jaws close on, along the
# stick's own axis. Originally derived 2026-08-21 from compute_fk of
# gripper_tcp at the hand-tuned 'lower' pose vs. the physical-stop
# coordinate then assumed for pick_and_place.yaml's steps.stick.base_xyz_m
# -- NOT yet confirmed with a ruler; see KUKA_IMPLEMENTATION_PLAN.md Sec 3
# Phase 2 for that original derivation.
#
# Re-derived 2026-09-02 (0.0188 -> 0.021072): real-hardware finding -- every
# PLACED stick landed ~2mm off, traced to base_xyz_m's assumed physical
# stop being ~2mm short of the real one (corrected there, 0.0665 ->
# 0.0685m on X) -- this constant, derived FROM that assumption, inherited
# the same 2mm error into every placement computed from it (grasp_target/
# grasped_stick_center -- NOT the feeder grasp itself: pregrasp/lower are
# independent, hand-tuned joint targets, never derived from this constant).
#
# NOTE: an earlier same-day revision of this comment (0.023050) used the
# WRONG 'lower' pose for this recomputation -- a transcription slip, not a
# real hardware retune (pick_and_place.yaml's 'lower' has NOT changed since
# 2026-08-25; confirmed via `git diff HEAD -- .../pick_and_place.yaml`
# showing zero delta on that line). Recomputed again here from the ACTUAL,
# unchanged 'lower':
#     fk(lower_rad)'s gripper_tcp X, at pick_and_place.yaml's real
#     'lower' pose (-83.45, -32.25, 126.35, 173.05, 3.55, 148.04) deg:
#     (0.047428, 0.399898, 0.021598)
#     GRASP_OFFSET_M = corrected_base_xyz_m.x - tcp.x
#                     = 0.0685 - 0.047428 = 0.021072
# Off-axis residual at this pose: Y -0.10mm, Z -1.90mm (unexplained, same
# spirit as the original derivation's own "not yet confirmed with a ruler"
# caveat -- re-verify against real hardware, not just this recomputation,
# before fully trusting it).

# Recalibrated 2026-09-09 after GRIPPER_TCP_Z_M's own 2026-09-09 re-derivation
# (see that constant's own comment) shifted where fk() places gripper_tcp at
# every pose, including 'lower'. Recomputed via this same section's own
# formula at pick_and_place.yaml's live 'lower' pose (-83.62, -32.33, 126.72,
# 174.62, 4.45, 147.31) deg and stick.base_xyz_m (0.067229, 0.400186,
# 0.024080): new tcp.x = 0.046149, GRASP_OFFSET_M = 0.067229 - 0.046149 =
# 0.021080 (was 0.021065 before GRIPPER_TCP_Z_M's first change).
#
# Recalibrated again 2026-09-09, same day, after GRIPPER_TCP_Z_M's further
# 1mm reduction (0.0785 -> 0.0775): new tcp.x = 0.046142, GRASP_OFFSET_M =
# 0.067229 - 0.046142 = 0.021087.
GRASP_OFFSET_M = 0.021087

# Jaw geometry -- re-derived 2026-09-05 after a gripper finger STL swap
# (kr10_r900_2_description/meshes/{visual,collision}/gripper_{left,right}_finger.stl;
# old meshes kept alongside as old_gripper_*_finger.stl). Unlike when this
# section was first written, WHICH mesh axis is "along the stick" is no
# longer a guess: gripper_tcp_joint's own origin is `rpy="0 0 0"` off
# gripper_mount_link (a pure translation, kr10_r900_2_description's own
# xacro), and both gripper_finger_*_joint origins are ALSO identity off the
# same link -- so the finger mesh's own local Y axis, exactly as authored
# in the STL, IS gripper_tcp's local Y with zero rotation in between, the
# same axis grasp.py's own docstring already confirms (from real
# tuned-pose FK data, not a guess) is the physical stick's base->tip
# direction. Half the finger mesh's own narrowest bbox dimension (its Y
# extent) is therefore, PROVABLY, the same physical quantity at both call
# sites below -- not two independent measurements that happen to be close:
#   - grasp_offset_for_length's use of JAW_CONTACT_HALF_LENGTH_M: how far
#     the jaw's own contact face extends to either side of the grasp
#     point, along the stick's axis (never grip closer to a short stick's
#     own tip than this).
#   - jaw_clearance.check_jaw_clearance's use of JAW_RADIUS_M: the swept
#     capsule's radius for a coarse "does the jaw assembly clip a
#     neighbouring placed stick" check -- an axial extent standing in for
#     a true perpendicular radius, an already-known shortcut this
#     re-derivation does not change, only re-measures.
# Defined as one literal with the other set equal to it, rather than as two
# independently-guessed numbers (the OLD 7.7mm/8.0mm split) that can
# silently drift apart on the next mesh swap.
#
# New mesh bbox, measured directly from the STL vertices, 2026-09-05:
# 18.65 x 11.0 x 31.3mm (was 23.65 x 15.39 x 36.8mm) -- narrowest/Y-axis
# dimension 11.0mm, half = 5.5mm.
JAW_RADIUS_M = 0.0055
JAW_CONTACT_HALF_LENGTH_M = JAW_RADIUS_M

# MIN_GRASP_OFFSET_M has no equally rigorous derivation -- this re-derivation
# does not change its "rough estimate, not measured" status (same as the
# original 10mm value). Kept PROPORTIONAL to the jaw geometry above rather
# than picked independently: the old value (10mm) was ~1.299x the old
# JAW_RADIUS_M (7.7mm); applying that same ratio to the new JAW_RADIUS_M
# preserves whatever safety margin the original author intended relative to
# jaw size, rather than an arbitrary fresh guess. Re-measure against real
# hardware before trusting this for a short-stick (Phase 5) grasp.
MIN_GRASP_OFFSET_M = 0.00714

# The KUKA feeder's grasp point is FIXED in the fixture's own frame --
# KUKA_IMPLEMENTATION_PLAN.md Sec 0 point 2: "the gripper always grasps at
# the same fixed vertical-hole location regardless of stick length." Unlike
# SO-100 (where the grip point is chosen per-stick, up from whichever
# stick's own base), grasp_offset_for_length() is therefore NOT needed for
# Phase 1's feeder grasp -- GRASP_OFFSET_M (21.09mm as of 2026-09-09) plus
# JAW_CONTACT_HALF_LENGTH_M (5.5mm as of 2026-09-05) = 26.59mm sits safely
# inside even the shortest allowed stick (35mm, STICK_LENGTH_RANGE_M below)
# with margin to spare. The function is still provided (mirroring so_arm_100_kinematics'
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
# JAW_RADIUS_M is defined earlier alongside JAW_CONTACT_HALF_LENGTH_M --
# see that section's own comment for why they share one derivation.

# Already-placed sticks are checked as capsules too: the round stock's own
# radius (half of STICK_SECTION_M) plus a small inflation, matching
# so_arm_100_kinematics' "consider 1-2mm inflation" convention -- no
# circumscribed-square correction needed here since the stock is round.
STICK_COLLISION_INFLATION_M = 0.0015
STICK_COLLISION_RADIUS_M = STICK_SECTION_M / 2.0 + STICK_COLLISION_INFLATION_M
