"""Validates the VENDORED copy of kr10_r900_2_kinematics.jaw_clearance at
so100_builder/kinematics/kr10_r900_2/ -- mechanical port of that package's
own test/test_jaw_clearance.py (same cases, same numbers), re-pointed per
so_arm_100_kinematics/README.md's vendoring rules (same rules, new robot).
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.kinematics.kr10_r900_2.constants import (
    GRASP_OFFSET_M,
    JAW_RADIUS_M,
    STICK_COLLISION_RADIUS_M,
)
from so100_builder.kinematics.kr10_r900_2.jaw_clearance import (
    check_jaw_clearance,
    jaw_swept_capsule,
    segment_distance,
)


class TestSegmentDistance(unittest.TestCase):
    def test_parallel_segments(self):
        d = segment_distance((0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1))
        self.assertAlmostEqual(d, 1.0, places=9)

    def test_perpendicular_skew_segments(self):
        d = segment_distance((-1, 0, 0), (1, 0, 0), (0, -1, 1), (0, 1, 1))
        self.assertAlmostEqual(d, 1.0, places=9)

    def test_intersecting_segments_zero_distance(self):
        d = segment_distance((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
        self.assertAlmostEqual(d, 0.0, places=9)

    def test_degenerate_point_segments(self):
        d = segment_distance((0, 0, 0), (0, 0, 0), (3, 4, 0), (3, 4, 0))
        self.assertAlmostEqual(d, 5.0, places=9)

    def test_point_to_segment(self):
        d = segment_distance((0, 2, 0), (0, 2, 0), (-5, 0, 0), (5, 0, 0))
        self.assertAlmostEqual(d, 2.0, places=9)

    def test_closest_approach_beyond_endpoint_clamps(self):
        d = segment_distance((0, 0, 0), (1, 0, 0), (3, 0, 1), (4, 0, 1))
        expected = (2.0 ** 2 + 1.0 ** 2) ** 0.5
        self.assertAlmostEqual(d, expected, places=9)


class TestJawSweptCapsule(unittest.TestCase):
    def test_endpoints_are_grip_point_and_base(self):
        base = (0.062, 0.398, 0.020)
        tip = (base[0] - 0.110, base[1], base[2])
        grip, vertex = jaw_swept_capsule(base, tip)
        self.assertEqual(vertex, base)
        self.assertAlmostEqual(grip[0], base[0] - GRASP_OFFSET_M, places=9)
        self.assertAlmostEqual(grip[1], base[1], places=9)
        self.assertAlmostEqual(grip[2], base[2], places=9)


class TestCheckJawClearance(unittest.TestCase):
    BASE = (0.062, 0.398, 0.020)
    TIP = (BASE[0] - 0.110, BASE[1], BASE[2])

    def test_empty_scene_is_clear(self):
        clear, reason, clearance_m, index = check_jaw_clearance(self.BASE, self.TIP, [])
        self.assertTrue(clear)
        self.assertIsNone(reason)
        self.assertIsNone(clearance_m)
        self.assertIsNone(index)

    def test_distant_stick_is_clear(self):
        far_away = [((0.30, 0.20, 0.0), (0.30, 0.10, 0.0))]
        clear, reason, clearance_m, index = check_jaw_clearance(self.BASE, self.TIP, far_away)
        self.assertTrue(clear)
        self.assertIsNone(reason)
        self.assertEqual(index, 0)
        self.assertGreater(clearance_m, 0.0)

    def test_stick_crowding_the_joint_cone_is_blocked(self):
        offset = (JAW_RADIUS_M + STICK_COLLISION_RADIUS_M) * 0.3
        crowding = [((self.BASE[0], self.BASE[1] + offset, self.BASE[2]),
                      (self.TIP[0], self.TIP[1] + offset, self.TIP[2]))]
        clear, reason, clearance_m, index = check_jaw_clearance(self.BASE, self.TIP, crowding)
        self.assertFalse(clear)
        self.assertIsNotNone(reason)
        self.assertEqual(index, 0)
        self.assertLess(clearance_m, 0.0)

    def test_tightest_of_several_sticks_is_reported(self):
        far_away = ((0.30, 0.20, 0.0), (0.30, 0.10, 0.0))
        offset = (JAW_RADIUS_M + STICK_COLLISION_RADIUS_M) * 0.3
        crowding = ((self.BASE[0], self.BASE[1] + offset, self.BASE[2]),
                     (self.TIP[0], self.TIP[1] + offset, self.TIP[2]))
        clear, reason, clearance_m, index = check_jaw_clearance(
            self.BASE, self.TIP, [far_away, crowding])
        self.assertFalse(clear)
        self.assertEqual(index, 1)

    def test_caller_must_exclude_the_actual_joint_neighbour(self):
        joint_neighbour = [((self.BASE[0], self.BASE[1], self.BASE[2]),
                             (self.BASE[0], self.BASE[1] + 0.10, self.BASE[2]))]
        clear, reason, clearance_m, index = check_jaw_clearance(
            self.BASE, self.TIP, joint_neighbour)
        self.assertFalse(clear)  # naive result: looks blocked -- caller's job to exclude
        self.assertEqual(index, 0)


if __name__ == "__main__":
    unittest.main()
