"""Validates the VENDORED copy of kr10_r900_2_kinematics.grasp at
so100_builder/kinematics/kr10_r900_2/ -- mechanical port of that package's
own test/test_grasp.py (same cases, same numbers), re-pointed per
so_arm_100_kinematics/README.md's vendoring rules (same rules, new robot).
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.kinematics.kr10_r900_2.chain import Unreachable, fk
from so100_builder.kinematics.kr10_r900_2.constants import GRASP_OFFSET_M
from so100_builder.kinematics.kr10_r900_2.grasp import (
    grasp_offset_for_length,
    grasp_target,
    orientation_from_stick_axis,
    solve_stick_placement,
    solve_stick_placement_any_roll,
    stick_axis,
)


class TestGraspTarget(unittest.TestCase):
    def test_grasp_target_sits_offset_toward_tip(self):
        base = (0.062, 0.398, 0.020)
        tip = (base[0] - 0.100, base[1], base[2])  # stock runs toward -X, KQ5
        target = grasp_target(base, tip)
        self.assertAlmostEqual(target[0], base[0] - GRASP_OFFSET_M, places=9)
        self.assertAlmostEqual(target[1], base[1], places=9)
        self.assertAlmostEqual(target[2], base[2], places=9)

    def test_zero_length_stick_raises(self):
        with self.assertRaises(ValueError):
            grasp_target((0, 0, 0), (0, 0, 0))


class TestGraspOffsetForLength(unittest.TestCase):
    def test_default_offset_used_for_long_stick(self):
        self.assertAlmostEqual(grasp_offset_for_length(0.150), GRASP_OFFSET_M, places=9)

    def test_capped_for_short_stick(self):
        # A stick barely above the floor: offset must stay below length -
        # jaw_contact_half_length, not the fixed default.
        offset = grasp_offset_for_length(0.020)
        self.assertLess(offset, 0.020)

    def test_too_short_raises(self):
        with self.assertRaises(Unreachable):
            grasp_offset_for_length(0.005)


class TestOrientationFromStickAxis(unittest.TestCase):
    def test_orthonormal_and_right_handed(self):
        random.seed(50)
        for _ in range(200):
            axis = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
            if math.sqrt(sum(c * c for c in axis)) < 1e-3:
                continue
            roll = random.uniform(-math.pi, math.pi)
            r = orientation_from_stick_axis(axis, roll)
            cols = [tuple(r[i][j] for i in range(3)) for j in range(3)]
            for i in range(3):
                for j in range(3):
                    expect = 1.0 if i == j else 0.0
                    got = sum(cols[i][k] * cols[j][k] for k in range(3))
                    self.assertAlmostEqual(got, expect, places=9)
            cross = (
                cols[0][1] * cols[1][2] - cols[0][2] * cols[1][1],
                cols[0][2] * cols[1][0] - cols[0][0] * cols[1][2],
                cols[0][0] * cols[1][1] - cols[0][1] * cols[1][0],
            )
            for got, want in zip(cross, cols[2]):
                self.assertAlmostEqual(got, want, places=9, msg="X x Y must equal Z")

    def test_y_column_matches_requested_axis(self):
        r = orientation_from_stick_axis((-3.0, 0.0, 0.0), roll_rad=0.7)
        y_col = (r[0][1], r[1][1], r[2][1])
        self.assertAlmostEqual(y_col[0], -1.0, places=9)
        self.assertAlmostEqual(y_col[1], 0.0, places=9)
        self.assertAlmostEqual(y_col[2], 0.0, places=9)

    def test_reference_swap_handles_vertical_axis(self):
        # stick_axis_hat parallel to the default reference_up must not
        # degenerate (division by ~zero) -- reference auto-swaps.
        r = orientation_from_stick_axis((0.0, 0.0, 1.0))
        y_col = (r[0][1], r[1][1], r[2][1])
        self.assertAlmostEqual(y_col[2], 1.0, places=9)


class TestSolveStickPlacement(unittest.TestCase):
    def test_round_trip_reaches_target_and_axis(self):
        random.seed(60)
        tested = 0
        for _ in range(300):
            base = (random.uniform(-0.2, 0.2), random.uniform(0.25, 0.45), random.uniform(0.0, 0.08))
            tip = tuple(base[i] + random.uniform(-0.12, 0.12) for i in range(3))
            try:
                joints, _roll, _eu, _wf = solve_stick_placement_any_roll(base, tip, roll_steps=8)
            except Unreachable:
                continue
            tested += 1
            target = grasp_target(base, tip)
            pos, _rot = fk(joints)
            for got, want in zip(pos, target):
                self.assertAlmostEqual(got, want, delta=1e-6)
            axis = tuple(t - b for t, b in zip(tip, base))
            n = math.sqrt(sum(c * c for c in axis))
            axis_hat = tuple(c / n for c in axis)
            achieved = stick_axis(joints)
            cos_err = max(-1.0, min(1.0, sum(a * b for a, b in zip(axis_hat, achieved))))
            self.assertLess(math.degrees(math.acos(cos_err)), 0.01)
        self.assertGreater(tested, 250, "unexpectedly low reachability in sample workspace")

    def test_specific_branch_matches_any_roll_result(self):
        # solve_stick_placement (one exact branch/roll) and
        # solve_stick_placement_any_roll (search) must agree when pointed
        # at the same, known-working combination -- discovered via the
        # search first, since not every (roll, branch) reaches every
        # target (see test_all_combinations_exhausted_raises).
        base = (0.062, 0.398, 0.020)
        tip = (base[0] - 0.080, base[1], base[2])
        _joints, roll, elbow_up, wrist_flip = solve_stick_placement_any_roll(base, tip, roll_steps=8)
        joints_direct = solve_stick_placement(
            base, tip, roll_rad=roll, elbow_up=elbow_up, wrist_flip=wrist_flip)
        pos, _rot = fk(joints_direct)
        target = grasp_target(base, tip)
        for got, want in zip(pos, target):
            self.assertAlmostEqual(got, want, delta=1e-9)

    def test_all_combinations_exhausted_raises(self):
        with self.assertRaises(Unreachable):
            solve_stick_placement_any_roll((10.0, 10.0, 10.0), (10.0, 10.0, 10.5), roll_steps=4)


if __name__ == "__main__":
    unittest.main()
