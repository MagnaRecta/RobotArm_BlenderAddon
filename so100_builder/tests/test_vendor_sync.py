"""The `make sync-kinematics` check, as a test (BLENDER_ADDON_PLAN.md Sec 4).

``kinematics/`` is a **verbatim copy, never a fork**. If a Blender-side edit
ever creeps in, the two sides silently disagree about where sticks go -- the
exact failure the shared-module design exists to prevent. This test makes
that loud.

It skips (rather than fails) when the ROS2 package is not next to the addon,
so a user who installed only the packaged extension still gets a green run.
"""

import os
import unittest

ADDON_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENDORED = os.path.join(ADDON_DIR, "kinematics")
UPSTREAM = os.path.join(
    os.path.dirname(ADDON_DIR), "so_arm_100_kinematics", "so_arm_100_kinematics"
)
UPSTREAM_VERSION_FILE = os.path.join(
    os.path.dirname(ADDON_DIR), "so_arm_100_kinematics", "VERSION"
)

MODULES = ("__init__.py", "chain.py", "constants.py", "envelope.py")


def _read(path):
    with open(path, "rb") as handle:
        return handle.read()


class TestVendoredCopyIsVerbatim(unittest.TestCase):
    def setUp(self):
        if not os.path.isdir(UPSTREAM):
            self.skipTest("ROS2 so_arm_100_kinematics package not present alongside the addon")

    def test_every_module_is_byte_identical(self):
        for name in MODULES:
            with self.subTest(module=name):
                self.assertEqual(
                    _read(os.path.join(VENDORED, name)),
                    _read(os.path.join(UPSTREAM, name)),
                    msg=(
                        "%s has DRIFTED from the ROS2 original. Do not fix it here: "
                        "fix it in so_arm_100_kinematics and re-vendor, or the two "
                        "sides will disagree about where sticks go." % name
                    ),
                )

    def test_no_ros_packaging_files_were_copied(self):
        # README "Vendoring into the Blender addon": package.xml, setup.py,
        # setup.cfg and resource/ are ROS packaging only.
        for forbidden in ("package.xml", "setup.py", "setup.cfg", "resource"):
            self.assertFalse(
                os.path.exists(os.path.join(VENDORED, forbidden)),
                "%s is ROS packaging and must not be vendored" % forbidden,
            )

    def test_version_file_matches_upstream(self):
        self.assertEqual(
            _read(os.path.join(VENDORED, "VERSION")).strip(),
            _read(UPSTREAM_VERSION_FILE).strip(),
        )


class TestVersionConsistency(unittest.TestCase):
    def test_version_file_matches_dunder_version(self):
        # README rule 3: bump __version__ AND VERSION together. Build files
        # record kinematics_version, and ROS2 refuses to execute on mismatch.
        from so100_builder import kinematics

        with open(os.path.join(VENDORED, "VERSION")) as handle:
            file_version = handle.read().strip()
        self.assertEqual(kinematics.__version__, file_version)


class TestNoForbiddenImports(unittest.TestCase):
    def test_kinematics_imports_only_stdlib_math(self):
        # Constraint B4: no numpy anywhere, especially not here -- Blender
        # bundles no numpy guarantee.
        for name in MODULES:
            source = _read(os.path.join(VENDORED, name)).decode("utf-8")
            for banned in ("import numpy", "import rclpy", "from numpy", "from rclpy"):
                self.assertNotIn(banned, source, "%s must not %s" % (name, banned))

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
