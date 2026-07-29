"""core/transform.py -- BLENDER_ADDON_PLAN.md Sec 8, constraints B5/B6."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core.transform import (  # noqa: E402
    IDENTITY_4X4,
    SingularMatrix,
    apply_direction,
    apply_point,
    blender_to_robot,
    blender_to_robot_batch,
    invert_4x4,
    mat_mul,
    quat_blender_to_ros,
    quat_ros_to_blender,
    robot_to_blender,
)


def translation(x, y, z):
    return ((1.0, 0.0, 0.0, x), (0.0, 1.0, 0.0, y), (0.0, 0.0, 1.0, z), (0.0, 0.0, 0.0, 1.0))


def rot_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return ((c, -s, 0.0, 0.0), (s, c, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def scale(sx, sy, sz):
    return ((sx, 0.0, 0.0, 0.0), (0.0, sy, 0.0, 0.0), (0.0, 0.0, sz, 0.0), (0.0, 0.0, 0.0, 1.0))


class TestMatrixMath(unittest.TestCase):
    def test_inverse_of_identity(self):
        for row_a, row_b in zip(invert_4x4(IDENTITY_4X4), IDENTITY_4X4):
            for a, b in zip(row_a, row_b):
                self.assertAlmostEqual(a, b, places=12)

    def test_inverse_times_original_is_identity(self):
        m = mat_mul(mat_mul(translation(0.3, -0.7, 1.1), rot_z(0.9)), scale(2.0, 0.5, 3.0))
        product = mat_mul(m, invert_4x4(m))
        for r in range(4):
            for c in range(4):
                self.assertAlmostEqual(product[r][c], 1.0 if r == c else 0.0, places=10)

    def test_singular_matrix_is_reported_not_silently_wrong(self):
        with self.assertRaises(SingularMatrix):
            invert_4x4(scale(1.0, 0.0, 1.0))

    def test_direction_ignores_translation(self):
        m = translation(5.0, 5.0, 5.0)
        self.assertEqual(apply_direction(m, (1.0, 0.0, 0.0)), (1.0, 0.0, 0.0))
        self.assertEqual(apply_point(m, (1.0, 0.0, 0.0)), (6.0, 5.0, 5.0))


class TestBlenderToRobot(unittest.TestCase):
    def test_identity_base_is_a_pass_through(self):
        self.assertEqual(blender_to_robot((0.1, -0.37, 0.2), IDENTITY_4X4, 1.0),
                         (0.1, -0.37, 0.2))

    def test_base_empty_translation_is_subtracted(self):
        # Moving SO100_Base repositions the whole design relative to the
        # robot without re-authoring anything (Sec 9.1).
        base = translation(1.0, 2.0, 3.0)
        got = blender_to_robot((1.1, 1.7, 3.2), base, 1.0)
        for a, b in zip(got, (0.1, -0.3, 0.2)):
            self.assertAlmostEqual(a, b, places=12)

    def test_base_empty_rotation_is_applied(self):
        base = rot_z(math.radians(90.0))
        got = blender_to_robot((0.0, 1.0, 0.0), base, 1.0)
        for a, b in zip(got, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(a, b, places=10)

    def test_scale_length_applies_to_translation_after_the_inverse(self):
        # Constraint B6. With scale_length = 0.001 (millimetre scene units) a
        # point 100 units from the base is 0.1 m away, NOT 100 m.
        base = translation(10.0, 0.0, 0.0)
        got = blender_to_robot((110.0, 0.0, 0.0), base, 0.001)
        self.assertAlmostEqual(got[0], 0.1, places=12)

    def test_scale_length_order_matters(self):
        # Scaling BEFORE the inverse would give a different answer whenever
        # the base empty is not at the origin -- this test is what catches
        # that specific bug.
        base = translation(10.0, 0.0, 0.0)
        correct = blender_to_robot((110.0, 0.0, 0.0), base, 0.001)
        wrong = apply_point(invert_4x4(base), (110.0 * 0.001, 0.0, 0.0))
        self.assertNotAlmostEqual(correct[0], wrong[0], places=6)

    def test_no_axis_flip_between_blender_and_ros(self):
        # Both are right-handed Z-up. A point above the base stays above it.
        got = blender_to_robot((0.0, 0.0, 0.5), IDENTITY_4X4, 1.0)
        self.assertGreater(got[2], 0.0)
        self.assertAlmostEqual(got[1], 0.0, places=12)

    def test_batch_matches_single(self):
        base = mat_mul(translation(0.2, -0.1, 0.05), rot_z(0.4))
        points = [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (-0.5, 0.25, 0.125)]
        batch = blender_to_robot_batch(points, base, 0.01)
        for point, got in zip(points, batch):
            want = blender_to_robot(point, base, 0.01)
            for a, b in zip(got, want):
                self.assertAlmostEqual(a, b, places=12)

    def test_round_trip_through_robot_to_blender(self):
        base = mat_mul(translation(0.2, -0.1, 0.05), rot_z(0.4))
        original = (1.5, -2.25, 0.75)
        there = blender_to_robot(original, base, 0.01)
        back = robot_to_blender(there, base, 0.01)
        for a, b in zip(back, original):
            self.assertAlmostEqual(a, b, places=10)


class TestQuaternionOrder(unittest.TestCase):
    def test_blender_wxyz_to_ros_xyzw(self):
        # Constraint B5, and the only place in the addon that does this.
        self.assertEqual(quat_blender_to_ros((0.1, 0.2, 0.3, 0.4)), (0.2, 0.3, 0.4, 0.1))

    def test_round_trip(self):
        q = (0.7071, 0.0, 0.7071, 0.0)
        self.assertEqual(quat_ros_to_blender(quat_blender_to_ros(q)), q)


if __name__ == "__main__":
    unittest.main()
