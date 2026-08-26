"""The robot registry, and the interface contract every vendored kinematics
package must satisfy. BRIDGE_PROTOCOL.md Sec A.1.1 / docs/STATUS.md's
"Multi-robot support" entry.

The addon originally targeted exactly one robot (the SO-100) and every
module that needed its kinematics simply imported ``kinematics/`` directly.
This module is the seam that generalizes that: a small, explicit
:class:`RobotProfile` per supported ``robot`` id (BRIDGE_PROTOCOL.md's own
term for the field), each wrapping one vendored kinematics package under
``kinematics/<robot_id>/``. Everything downstream (``core/validate.py``,
``core/mirror.py``, ``ops/mirror.py``, ``ops/build.py``'s export) is meant to
go through :func:`get_robot` rather than importing a specific robot's
package by name -- that is what makes adding a robot a one-place change here
plus a new vendored directory, not a grep-and-replace across the addon.

Pure Python, no ``bpy`` -- like the rest of ``core/``, this stays testable
outside Blender (constraint B4/B2 -- see ``BLENDER_ADDON_PLAN.md`` Sec 3).

## The kinematics-package interface contract

Every module handed to a caller as "this robot's kinematics" must expose,
at minimum:

| Name | Contract |
|---|---|
| ``__version__`` | ``str``. Written into every build file as `kinematics_version` (BRIDGE_PROTOCOL.md Sec A.2) -- the executor refuses to run a file whose version does not match its own copy. |
| ``STICK_LENGTH_RANGE_M`` | ``(min_m, max_m)`` -- this robot's own stock-length limits (`BLENDER_ADDON_PLAN.md` Sec 5.4). |
| ``grasp_offset_for_length(length_m)`` | -> ``float``. The grip offset (measured from the stick's base) to use for a stick this long. Raises ``Unreachable`` if no offset both fits the stick and clears the robot's own floor-clearance floor. |
| ``fk(joint_angles_rad)`` | -> ``(position_xyz_m, rotation_matrix)``. Loop over ``zip(CHAIN, joint_angles_rad)`` then unconditionally append ONE fixed final transform, so a shorter prefix still returns a valid partial-chain frame -- ``core/mirror.py``'s `joint_frames()` (the preview rig) depends on exactly this shape, and derives the fixed offset via ``fk(())`` rather than a named constant (see its own docstring), so a robot need not export one. |
| ``check_jaw_clearance(base_xyz_m, tip_xyz_m, placed_sticks, ...)`` | -> ``(clear, reason, clearance_m, offending_index)``. The swept-gripper collision pre-filter (`BLENDER_ADDON_PLAN.md` Sec 6 C3). |
| ``Unreachable`` | Exception class raised by the above. Callers catch it *by this name*, never by string-matching a message. |
| ``JOINT_NAMES`` | ``tuple`` of ``str``, this robot's joint order. |
| ``STICK_SECTION_M`` / ``JAW_RADIUS_M`` | This robot's own default stock cross-section and gripper radial extent -- what ``core/order.py``'s jaw-clearance heuristic and ``core/sticks.py``'s geometry currently read (today hardcoded to ``so_arm_100``'s copies; see the note in ``BLENDER_ADDON_PLAN.md``'s registry section on what is and is not yet robot-parameterized). |

**"Solve a stick placement" is deliberately NOT in the table above.**
so_arm_100's `solve_stick_placement(base, tip, grasp_offset_m=...)` and
kr10_r900_2's own function of the same name have compatible-by-coincidence
signatures (both accept just ``base``/``tip`` positionally and return the
joint tuple directly, raising ``Unreachable`` on failure) but that is where
the similarity ends -- kr10_r900_2's stock is round with a genuinely free
roll DOF (unlike so_arm_100's square stock), so the single-roll
``solve_stick_placement`` under-reports reachability for it; the robot's own
`solve_stick_placement_any_roll(base, tip)` (sweeps roll x branch, returns
``(joints, roll_rad, elbow_up, wrist_flip)``) is what a real placement needs.
`core/validate.py` and `ops/mirror.py` therefore dispatch explicitly per
robot id for this one operation (`_validate_stick_so_arm_100` /
`_validate_stick_kr10_r900_2`) rather than pretending a single shared
function name is enough -- see each module's own docstring. A future third
robot may need its own path too; there is no reason to expect this part to
generalize further than two robots' worth of evidence justifies.

``so_arm_100_kinematics`` (vendored at ``kinematics/so_arm_100/``) and
``kr10_r900_2_kinematics`` (vendored at ``kinematics/kr10_r900_2/``) are
both real, are structured the same way (``chain.py`` / ``constants.py`` /
``grasp.py`` / ``jaw_clearance.py``) but differ substantially *internally* --
so_arm_100 is 5-DOF with 1 free orientation DOF (roll) and a numerically
delicate closed-form search needing round-trip verification;
kr10_r900_2 is a genuine 6-DOF spherical-wrist arm with an exact, Pieper-
decoupled closed form needing no verification step at all, and a round
stock with no preferred roll. Only the public surface above (plus whatever
each robot-specific validate/mirror path needs, per the paragraph above) is
load-bearing to this addon; anything else is that package's own business.

## Physical defaults are optional, deliberately

Per BRIDGE_PROTOCOL.md Sec A.1.1: "Each robot's own geometric constants...
are that robot's kinematics package's own business, never hardcoded in the
addon." :class:`RobotProfile`'s ``build_volume_min_m`` / ``build_volume_max_m``
/ ``stock_section_m`` fields exist so the addon has *somewhere* to read a
default from for the exported build file's ``build_volume`` block -- they
are not a second source of truth, and a profile with no confirmed numbers
yet leaves them ``None`` rather than inventing placeholders. ``kr10_r900_2``
now has both: `build_volume_min_m`/`max_m` were given directly by the user
(2026-08-22, ahead of its kinematics package being vendored, so set
directly on the profile rather than read from the package -- see its own
comment) and `stock_section_m` now comes from the real vendored package's
own `STICK_SECTION_M`, exactly like so_arm_100's.

## ``is_vendored``

``True`` unless the kinematics module explicitly sets its own module-level
``VENDORED = False``. No registered robot does this today -- both
so_arm_100 and kr10_r900_2 are real, vendored packages -- but
``ops/build.py``'s export refusal and ``ui/panels.py``'s Design-panel
warning both check it (never `has_build_volume`, a separate, independent
fact) so a *future* not-yet-vendored robot still degrades cleanly the same
way `kr10_r900_2` did before 2026-08-22, without any other code needing to
change.
"""

from ..kinematics import kr10_r900_2 as _kr10_r900_2_kinematics
from ..kinematics import so_arm_100 as _so_arm_100_kinematics

SO_ARM_100_ID = "so_arm_100"
KR10_R900_2_ID = "kr10_r900_2"


class RobotProfile:
    """One registered robot. See the module docstring for what
    ``kinematics`` must expose."""

    __slots__ = ("id", "kinematics", "build_volume_min_m", "build_volume_max_m",
                 "stock_section_m", "frame", "base_box_min_m", "base_box_max_m")

    def __init__(self, id, kinematics, frame, build_volume_min_m=None,
                build_volume_max_m=None, stock_section_m=None,
                base_box_min_m=None, base_box_max_m=None):
        self.id = id
        self.kinematics = kinematics
        # No default, unlike build_volume_min_m/max_m/stock_section_m below
        # -- this is the robot's own URDF root-link NAME, always known for a
        # real, integrated robot (never a "not yet measured" physical
        # quantity the way those three are), and BRIDGE_PROTOCOL.md's A.2
        # `frame` field is exactly this value: "already in robot
        # coordinates... Blender does the transform before writing" (A.1
        # rule 2) is meaningless without knowing which frame that is. Found
        # missing 2026-08-23 when a real kr10_r900_2 export carried
        # `"base_link"` (io/build_file.py's old flat FRAME constant) instead
        # of this robot's actual root link `"base"` -- the executor
        # correctly refused the file. See io/build_file.py's own
        # `_frame_for_robot` for how this gets into an exported document.
        self.frame = frame
        # None means "not confirmed yet" -- see the module docstring's
        # "Physical defaults are optional" section. Never a placeholder
        # number standing in for a real measurement.
        self.build_volume_min_m = build_volume_min_m
        self.build_volume_max_m = build_volume_max_m
        self.stock_section_m = stock_section_m
        # A purely visual/viewport reference (2026-08-23, user request): the
        # robot's own physical mounting pedestal, so a design in progress can
        # be checked by eye against it without a collision it never actually
        # models. None (so_arm_100's own default -- no comparable pedestal
        # given) means "don't draw one", same convention as build_volume_*
        # above. Never exported to the build file -- ROS2 already knows its
        # own robot's footprint from its URDF; this is a Blender-side design
        # aid only.
        self.base_box_min_m = base_box_min_m
        self.base_box_max_m = base_box_max_m

    @property
    def has_build_volume(self):
        return self.build_volume_min_m is not None and self.build_volume_max_m is not None

    @property
    def has_base_box(self):
        return self.base_box_min_m is not None and self.base_box_max_m is not None

    @property
    def is_vendored(self):
        """False only for a placeholder package that explicitly says so
        (``VENDORED = False``, see ``kinematics/kr10_r900_2/__init__.py``).
        Every real vendored package -- so_arm_100 included -- doesn't bother
        setting this, so it defaults True.
        """
        return getattr(self.kinematics, "VENDORED", True)

    def __repr__(self):
        return "RobotProfile(id=%r, kinematics_version=%r)" % (
            self.id, getattr(self.kinematics, "__version__", "?"))


ROBOTS = {
    SO_ARM_100_ID: RobotProfile(
        SO_ARM_100_ID,
        _so_arm_100_kinematics,
        frame="base_link",
        build_volume_min_m=_so_arm_100_kinematics.BUILD_VOLUME_MIN_M,
        build_volume_max_m=_so_arm_100_kinematics.BUILD_VOLUME_MAX_M,
        stock_section_m=_so_arm_100_kinematics.STICK_SECTION_M,
    ),
    # Build volume given directly by the user 2026-08-22 (docs/STATUS.md):
    # a 300x300x300mm cube, centred 450mm from the robot origin along -Y --
    # same X/Y convention as so_arm_100's own BUILD_VOLUME_MIN_M/MAX_M
    # above (X centred on 0, Y centred on the given distance) and the same
    # Z convention (sits ON the base plate, Z in [0, height], not centred
    # vertically -- nothing builds floating or below the plate). Set
    # directly here, not read off the kinematics module, since it did not
    # come from kr10_r900_2_kinematics (which has no build-volume concept
    # at all -- BRIDGE_PROTOCOL.md Sec A.1.1 treats it as this addon's own
    # concern, unlike GRASP_OFFSET_M-style grasp geometry).
    KR10_R900_2_ID: RobotProfile(
        KR10_R900_2_ID,
        _kr10_r900_2_kinematics,
        # This robot's own URDF root link -- NOT "base_link" (kuka_control's
        # kr10_r900_2_description.urdf.xacro has "base_link" as a
        # DIFFERENT, downstream link, the physical robot body; confirmed via
        # check_urdf, kuka_control's own motion.py header comment, and that
        # repo's kuka_pick_and_place/build_file.py FRAME constant).
        frame="base",
        # 300x300x300mm cube, centred 450mm from the origin toward +X (not
        # -Y -- corrected 2026-08-23, the user's own build setup faces +X).
        # Z lowered by 20mm (2026-08-23, user request) to account for the
        # robot's own 320x320x20mm mounting pedestal (base_box_min_m/max_m
        # below): the robot's coordinate origin sits AT THE TOP of that
        # pedestal, 20mm above the actual table surface everything else
        # (this cube included) rests on, so the cube's own floor is at
        # Z=-20mm in the robot's frame, not Z=0.
        build_volume_min_m=(0.30, -0.15, -0.02),
        build_volume_max_m=(0.60, 0.15, 0.28),
        stock_section_m=_kr10_r900_2_kinematics.STICK_SECTION_M,
        # The robot's own mounting pedestal (2026-08-23, user request): a
        # 320x320x20mm box centred on the robot's own origin in X/Y, sitting
        # directly beneath it -- Z in [-20mm, 0], i.e. up to but not
        # including the robot's own coordinate origin. A viewport reference
        # only (module docstring's own note above); not a real collision
        # model and not exported to the build file.
        base_box_min_m=(-0.16, -0.16, -0.02),
        base_box_max_m=(0.16, 0.16, 0.0),
    ),
}

DEFAULT_ROBOT_ID = SO_ARM_100_ID


def get_robot(robot_id):
    """The :class:`RobotProfile` for ``robot_id``. Raises ``KeyError`` (with
    the known ids in the message) for anything not in :data:`ROBOTS` --
    every caller of this addon's own registry is trusted to pass a real id,
    unlike the build-file loader on the ROS2 side, which has to handle
    arbitrary input from a file it did not write.
    """
    try:
        return ROBOTS[robot_id]
    except KeyError:
        raise KeyError(
            "unknown robot id %r -- known ids: %s"
            % (robot_id, ", ".join(sorted(ROBOTS)))
        )
