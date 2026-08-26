"""The `make sync-kinematics` check, as a test (BLENDER_ADDON_PLAN.md Sec 4).

Each ``kinematics/<robot_id>/`` folder is a **verbatim copy, never a fork**.
If a Blender-side edit ever creeps in, the two sides silently disagree about
where sticks go -- the exact failure the shared-module design exists to
prevent. This test makes that loud, for every vendored robot (multi-robot
support, docs/STATUS.md 2026-08-21).

Each robot's check skips (rather than fails) independently when its own
upstream package is not present alongside the addon, so a user who installed
only the packaged extension -- or who only has one of the two source repos
checked out -- still gets a green run.
"""

import os
import unittest

ADDON_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(ADDON_DIR)
KINEMATICS_DIR = os.path.join(ADDON_DIR, "kinematics")

# One entry per vendored robot: (robot_id, module filenames, vendored dir,
# upstream inner-package dir, upstream VERSION file). Upstream paths mirror
# each source repo's own layout (so_arm_100_kinematics/so_arm_100_kinematics/
# vs. kr10_r900_2_kinematics/kr10_r900_2_kinematics/) -- see each package's
# own README "Vendoring into the Blender addon" section.
_SO_ARM_100_MODULES = ("__init__.py", "chain.py", "constants.py", "envelope.py",
                       "grasp.py", "jaw_clearance.py")
_KR10_MODULES = ("__init__.py", "chain.py", "constants.py", "envelope.py",
                 "grasp.py", "jaw_clearance.py")

ROBOTS = {
    "so_arm_100": {
        "modules": _SO_ARM_100_MODULES,
        "vendored": os.path.join(KINEMATICS_DIR, "so_arm_100"),
        "upstream": os.path.join(REPO_DIR, "so_arm_100_kinematics", "so_arm_100_kinematics"),
        "upstream_version": os.path.join(REPO_DIR, "so_arm_100_kinematics", "VERSION"),
    },
    "kr10_r900_2": {
        "modules": _KR10_MODULES,
        "vendored": os.path.join(KINEMATICS_DIR, "kr10_r900_2"),
        # Lives in the sibling ROS2 colcon workspace, not next to this repo --
        # see docs/STATUS.md's "Multi-robot support" entry for the path.
        "upstream": os.path.expanduser(
            "~/ros2_ws/src/kuka_control/kr10_r900_2_kinematics/kr10_r900_2_kinematics"),
        "upstream_version": os.path.expanduser(
            "~/ros2_ws/src/kuka_control/kr10_r900_2_kinematics/VERSION"),
    },
}

# Backward-compatible aliases -- kept because other tests in this file
# (TestVersionConsistency, TestNoForbiddenImports) only ever cared about
# so_arm_100 and there is no reason to touch their working code.
VENDORED = ROBOTS["so_arm_100"]["vendored"]
MODULES = ROBOTS["so_arm_100"]["modules"]


def _read(path):
    with open(path, "rb") as handle:
        return handle.read()


class TestVendoredCopyIsVerbatim(unittest.TestCase):
    def test_every_module_is_byte_identical(self):
        for robot_id, info in ROBOTS.items():
            if not os.path.isdir(info["upstream"]):
                continue  # this robot's upstream isn't checked out here
            for name in info["modules"]:
                with self.subTest(robot=robot_id, module=name):
                    self.assertEqual(
                        _read(os.path.join(info["vendored"], name)),
                        _read(os.path.join(info["upstream"], name)),
                        msg=(
                            "%s/%s has DRIFTED from its upstream original. Do not "
                            "fix it here: fix it in the upstream kinematics package "
                            "and re-vendor, or the two sides will disagree about "
                            "where sticks go." % (robot_id, name)
                        ),
                    )
        if not any(os.path.isdir(info["upstream"]) for info in ROBOTS.values()):
            self.skipTest("no upstream kinematics package present alongside the addon")

    def test_no_ros_packaging_files_were_copied(self):
        # README "Vendoring into the Blender addon": package.xml, setup.py,
        # setup.cfg and resource/ are ROS packaging only.
        for info in ROBOTS.values():
            for forbidden in ("package.xml", "setup.py", "setup.cfg", "resource"):
                self.assertFalse(
                    os.path.exists(os.path.join(info["vendored"], forbidden)),
                    "%s is ROS packaging and must not be vendored" % forbidden,
                )

    def test_version_file_matches_upstream(self):
        found_any = False
        for robot_id, info in ROBOTS.items():
            if not os.path.isfile(info["upstream_version"]):
                continue
            found_any = True
            with self.subTest(robot=robot_id):
                self.assertEqual(
                    _read(os.path.join(info["vendored"], "VERSION")).strip(),
                    _read(info["upstream_version"]).strip(),
                )
        if not found_any:
            self.skipTest("no upstream VERSION file present alongside the addon")


class TestVersionConsistency(unittest.TestCase):
    def test_version_file_matches_dunder_version(self):
        # README rule 3: bump __version__ AND VERSION together. Build files
        # record kinematics_version, and ROS2 refuses to execute on mismatch.
        # Checked for every vendored robot, not just so_arm_100.
        from so100_builder.kinematics import kr10_r900_2, so_arm_100

        for robot_id, kinematics in (("so_arm_100", so_arm_100),
                                     ("kr10_r900_2", kr10_r900_2)):
            with self.subTest(robot=robot_id):
                vendored = ROBOTS[robot_id]["vendored"]
                with open(os.path.join(vendored, "VERSION")) as handle:
                    file_version = handle.read().strip()
                self.assertEqual(kinematics.__version__, file_version)


class TestNoForbiddenImports(unittest.TestCase):
    def test_kinematics_imports_only_stdlib_math(self):
        # Constraint B4: no numpy anywhere, especially not here -- Blender
        # bundles no numpy guarantee.
        for robot_id, info in ROBOTS.items():
            for name in info["modules"]:
                source = _read(os.path.join(info["vendored"], name)).decode("utf-8")
                for banned in ("import numpy", "import rclpy", "from numpy", "from rclpy"):
                    self.assertNotIn(
                        banned, source, "%s/%s must not %s" % (robot_id, name, banned))

    def test_core_modules_do_not_import_bpy(self):
        # core/ must stay testable outside Blender, and must never be the
        # reason a background thread touches bpy (constraint B2).
        core = os.path.join(ADDON_DIR, "core")
        for name in sorted(os.listdir(core)):
            if not name.endswith(".py"):
                continue
            source = _read(os.path.join(core, name)).decode("utf-8")
            for banned in ("import bpy", "import numpy", "import mathutils"):
                self.assertNotIn(
                    banned, source, "core/%s must not %s" % (name, banned)
                )


if __name__ == "__main__":
    unittest.main()
