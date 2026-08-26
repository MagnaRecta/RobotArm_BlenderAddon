"""core/robots.py -- the robot registry + kinematics-package interface
contract. docs/STATUS.md "Multi-robot support kicked off 2026-08-21".
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core import robots as R  # noqa: E402


class TestRegistry(unittest.TestCase):
    def test_default_robot_id_is_so_arm_100(self):
        self.assertEqual(R.DEFAULT_ROBOT_ID, R.SO_ARM_100_ID)

    def test_get_robot_returns_the_matching_profile(self):
        profile = R.get_robot(R.SO_ARM_100_ID)
        self.assertEqual(profile.id, R.SO_ARM_100_ID)

    def test_get_robot_raises_a_clear_error_for_an_unknown_id(self):
        with self.assertRaises(KeyError) as caught:
            R.get_robot("does_not_exist")
        message = str(caught.exception)
        self.assertIn("does_not_exist", message)
        self.assertIn("so_arm_100", message)

    def test_both_robots_are_registered(self):
        self.assertEqual(set(R.ROBOTS), {R.SO_ARM_100_ID, R.KR10_R900_2_ID})


class TestSoArm100Profile(unittest.TestCase):
    def test_is_vendored(self):
        self.assertTrue(R.get_robot(R.SO_ARM_100_ID).is_vendored)

    def test_has_a_confirmed_build_volume_and_stock_section(self):
        profile = R.get_robot(R.SO_ARM_100_ID)
        self.assertTrue(profile.has_build_volume)
        self.assertIsNotNone(profile.stock_section_m)

    def test_kinematics_module_matches_the_contract(self):
        # "At minimum" from the module docstring -- every name a
        # robot-agnostic caller is allowed to rely on.
        kinematics = R.get_robot(R.SO_ARM_100_ID).kinematics
        for name in ("__version__", "STICK_LENGTH_RANGE_M",
                    "grasp_offset_for_length", "solve_stick_placement",
                    "check_jaw_clearance", "Unreachable", "JOINT_NAMES"):
            self.assertTrue(hasattr(kinematics, name), name)


class TestKr10Profile(unittest.TestCase):
    """kr10_r900_2_kinematics was vendored 2026-08-22 (kuka_control) -- a
    real, tested 6-DOF kinematics package, not a placeholder."""

    def test_is_vendored(self):
        self.assertTrue(R.get_robot(R.KR10_R900_2_ID).is_vendored)

    def test_has_a_confirmed_build_volume_and_stock_section(self):
        profile = R.get_robot(R.KR10_R900_2_ID)
        self.assertTrue(profile.has_build_volume)
        # Round 2mm stock, from the vendored package's own STICK_SECTION_M.
        self.assertAlmostEqual(profile.stock_section_m, 0.002, places=6)

    def test_build_volume_is_a_300mm_cube_centred_450mm_on_plus_x(self):
        profile = R.get_robot(R.KR10_R900_2_ID)
        lo, hi = profile.build_volume_min_m, profile.build_volume_max_m
        for axis in range(3):
            self.assertAlmostEqual(hi[axis] - lo[axis], 0.30, places=6)
        centre = tuple((a + b) / 2.0 for a, b in zip(lo, hi))
        self.assertAlmostEqual(centre[0], 0.45, places=6)
        self.assertAlmostEqual(centre[1], 0.0, places=6)
        # Floor is 20mm below the robot's own coordinate origin, not AT it
        # (2026-08-23) -- the robot sits on its own 20mm pedestal
        # (base_box_min_m/max_m), so the table surface everything else
        # rests on is at Z=-20mm in the robot's own frame.
        self.assertAlmostEqual(lo[2], -0.02, places=6)

    def test_has_a_confirmed_base_box_below_the_robot_origin(self):
        profile = R.get_robot(R.KR10_R900_2_ID)
        self.assertTrue(profile.has_base_box)
        lo, hi = profile.base_box_min_m, profile.base_box_max_m
        self.assertAlmostEqual(hi[0] - lo[0], 0.32, places=6)
        self.assertAlmostEqual(hi[1] - lo[1], 0.32, places=6)
        self.assertAlmostEqual(hi[2] - lo[2], 0.02, places=6)
        centre = tuple((a + b) / 2.0 for a, b in zip(lo, hi))
        self.assertAlmostEqual(centre[0], 0.0, places=6)
        self.assertAlmostEqual(centre[1], 0.0, places=6)
        # Sits directly BELOW the robot's own origin, not centred on it.
        self.assertAlmostEqual(hi[2], 0.0, places=6)

    def test_so_arm_100_has_no_base_box(self):
        self.assertFalse(R.get_robot(R.SO_ARM_100_ID).has_base_box)

    def test_kinematics_module_matches_the_contract(self):
        kinematics = R.get_robot(R.KR10_R900_2_ID).kinematics
        for name in ("__version__", "STICK_LENGTH_RANGE_M",
                    "grasp_offset_for_length", "fk",
                    "check_jaw_clearance", "Unreachable", "JOINT_NAMES"):
            self.assertTrue(hasattr(kinematics, name), name)
        self.assertEqual(len(kinematics.JOINT_NAMES), 6)

    def test_solve_stick_placement_any_roll_exists_for_the_kr10_specific_path(self):
        # Not part of the universal contract (see module docstring) -- but
        # core/validate.py's and ops/mirror.py's kr10-specific paths need it
        # by exact name.
        kinematics = R.get_robot(R.KR10_R900_2_ID).kinematics
        self.assertTrue(callable(kinematics.solve_stick_placement_any_roll))


if __name__ == "__main__":
    unittest.main()
