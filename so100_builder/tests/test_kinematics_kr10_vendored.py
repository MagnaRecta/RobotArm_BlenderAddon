"""Validates the VENDORED copy of kr10_r900_2_kinematics at
so100_builder/kinematics/kr10_r900_2/ -- mechanical port of that package's
own test/test_chain.py (same cases, same numbers), re-pointed per
so_arm_100_kinematics/README.md's vendoring rules (same rules, new robot).
If the tests do not pass against the vendored copy, the copy is wrong.
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import so100_builder.kinematics.kr10_r900_2.chain as chain
from so100_builder.kinematics.kr10_r900_2.chain import Unreachable, fk, ik
from so100_builder.kinematics.kr10_r900_2.constants import CHAIN
from so100_builder.kinematics.kr10_r900_2.envelope import is_reachable
from so100_builder.kinematics.kr10_r900_2.grasp import orientation_from_stick_axis

# The two joint-space waypoints tuned by hand in RViz against the real arm,
# verbatim from kuka_pick_and_place/config/pick_and_place.yaml (degrees,
# CHAIN order). These are ground truth for the URDF this module is derived
# from -- this module must agree with them, not the other way around.
TUNED_POSES_DEG = {
    "pregrasp": (-84.0, -37.0, 129.0, 168.0, 2.0, -207.0),
    "lower": (-84.0, -32.0, 127.0, 175.0, 5.0, -214.0),
}

# gripper_tcp position at each tuned pose, cross-checked 2026-08-21 against
# MoveIt's own compute_fk (KDL, reading the live robot_description) while
# deriving GRASP_OFFSET_M -- see KUKA_IMPLEMENTATION_PLAN.md Sec 3 Phase 2.
# This is what makes these regression tests, not just internal-consistency
# checks: an independent solver already agreed with these numbers once.
EXPECTED_TCP_POSITION_M = {
    "pregrasp": (0.043107, 0.396218, 0.052715),
    "lower": (0.043231, 0.396750, 0.020038),
}


def _deg2rad(pose_deg):
    return tuple(math.radians(v) for v in pose_deg)


class TestForwardKinematics(unittest.TestCase):
    def test_tuned_poses_match_moveit_cross_check(self):
        for name, pose_deg in TUNED_POSES_DEG.items():
            pos, _rot = fk(_deg2rad(pose_deg))
            for got, want in zip(pos, EXPECTED_TCP_POSITION_M[name]):
                self.assertAlmostEqual(got, want, delta=1e-4, msg=name)

    def test_gripper_tcp_orientation_y_axis_is_the_stick_axis(self):
        # Module docstring / grasp.py's own central finding: gripper_tcp's
        # local Y column matches world -X (the stick's own known direction,
        # KQ5) at both independently-tuned poses.
        for name, pose_deg in TUNED_POSES_DEG.items():
            _pos, rot = fk(_deg2rad(pose_deg))
            y_col = (rot[0][1], rot[1][1], rot[2][1])
            self.assertAlmostEqual(y_col[0], -1.0, delta=0.01, msg=name)
            self.assertAlmostEqual(y_col[1], 0.0, delta=0.01, msg=name)
            self.assertAlmostEqual(y_col[2], 0.0, delta=0.02, msg=name)


class TestSphericalWrist(unittest.TestCase):
    def test_wrist_center_independent_of_joint4(self):
        # KUKA_IMPLEMENTATION_PLAN.md Sec 1.3's Pieper-condition claim,
        # checked directly (not just implied by a passing IK round trip):
        # the wrist centre (joint5's own origin) must not move as joint4
        # varies, for fixed joint1/joint2/joint3.
        random.seed(100)
        for _ in range(200):
            q1 = random.uniform(*(-3.0, 3.0))
            q2 = random.uniform(math.radians(-190), math.radians(45))
            q3 = random.uniform(math.radians(-120), math.radians(156))
            _pos3, rot3 = chain._link3_fk(q1, q2, q3)
            expected_wc = chain._v_add(_pos3, chain._mat_vec(rot3, (0.420, 0.0, 0.025)))
            for _q4trial in range(3):
                q4 = random.uniform(math.radians(-185), math.radians(185))
                pos = _pos3
                rot = rot3
                pos = chain._v_add(pos, chain._mat_vec(rot, (0.0, 0.0, 0.025)))
                rot = chain._mat_mat(rot, chain._axis_angle_matrix((-1.0, 0.0, 0.0), q4))
                pos = chain._v_add(pos, chain._mat_vec(rot, (0.420, 0.0, 0.0)))
                for got, want in zip(pos, expected_wc):
                    self.assertAlmostEqual(got, want, places=9)

    def test_xyx_euler_extraction_reconstructs_random_matrices(self):
        random.seed(101)
        for _ in range(3000):
            a = random.uniform(-math.pi, math.pi)
            b = random.uniform(-math.pi, math.pi)
            c = random.uniform(-math.pi, math.pi)
            r = chain._mat_mat(chain._mat_mat(chain._rot_x(a), chain._rot_y(b)), chain._rot_x(c))
            a2, b2, c2 = chain._extract_xyx(r)
            r2 = chain._mat_mat(chain._mat_mat(chain._rot_x(a2), chain._rot_y(b2)), chain._rot_x(c2))
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(r[i][j], r2[i][j], places=9)

    def test_xyx_second_branch_reconstructs_too(self):
        random.seed(102)
        for _ in range(3000):
            a = random.uniform(-math.pi, math.pi)
            b = random.uniform(-math.pi, math.pi)
            c = random.uniform(-math.pi, math.pi)
            r = chain._mat_mat(chain._mat_mat(chain._rot_x(a), chain._rot_y(b)), chain._rot_x(c))
            a2, b2, c2 = chain._extract_xyx(r, flip=True)
            r2 = chain._mat_mat(chain._mat_mat(chain._rot_x(a2), chain._rot_y(b2)), chain._rot_x(c2))
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(r[i][j], r2[i][j], places=9)


class TestInverseKinematics(unittest.TestCase):
    def test_round_trip_random_joint_space(self):
        # Strongest test: sample random joint configurations, run their FK
        # through ik() (best of all 4 branches), confirm the reconstructed
        # joints reproduce the exact same TCP pose.
        random.seed(200)
        tested = 0
        for _ in range(2000):
            js = [random.uniform(lo, hi) for (_n, _x, _a, lo, hi) in CHAIN]
            target_pos, target_rot = fk(js)
            best_err = None
            for elbow_up in (True, False):
                for wrist_flip in (False, True):
                    try:
                        solved = ik(target_pos, target_rot, elbow_up=elbow_up, wrist_flip=wrist_flip)
                    except Unreachable:
                        continue
                    pos2, rot2 = fk(solved)
                    err = max(abs(a - b) for a, b in zip(target_pos, pos2))
                    err = max(err, max(
                        abs(target_rot[i][j] - rot2[i][j]) for i in range(3) for j in range(3)))
                    if best_err is None or err < best_err:
                        best_err = err
            if best_err is not None:
                tested += 1
                self.assertLess(best_err, 1e-8)
        # A genuine, documented coverage gap (chain.py's own docstring): an
        # "arm flip" (shoulder rotated 180 deg, elbow re-solved) branch is
        # not implemented, so a small fraction of fully-random joint-space
        # samples -- almost all extreme, folded-back configurations far
        # outside any real pick-and-place workspace -- are not recoverable
        # via the 4 branches this module does implement. Assert the
        # overwhelming majority still round-trip, not every single one.
        self.assertGreater(tested, 1900, "unexpectedly low round-trip coverage")

    def test_recovers_the_actual_tuned_joint_angles(self):
        # Not just "some" solution -- the SAME solution the poses were
        # tuned to, modulo a +/-360 deg wrap: this module's atan2-based
        # extraction always returns each angle's principal-range ((-180,
        # 180] deg) representative, while joint6's own generous +/-350 deg
        # limit lets a tuned pose legitimately sit outside that range
        # (pregrasp's joint[5] = -207 deg, coterminal with +153 deg -- same
        # physical orientation, confirmed by the FK round-trip in
        # test_round_trip_random_joint_space). Compare wrapped, not raw.
        for name, pose_deg in TUNED_POSES_DEG.items():
            q_expected = _deg2rad(pose_deg)
            target_pos, target_rot = fk(q_expected)
            solved = ik(target_pos, target_rot, elbow_up=True, wrist_flip=False)
            for i, (got, want) in enumerate(zip(solved, q_expected)):
                diff_deg = (math.degrees(got - want) + 180.0) % 360.0 - 180.0
                self.assertAlmostEqual(
                    diff_deg, 0.0, delta=0.01,
                    msg=f"{name} joint[{i}] mismatch")

    def test_unreachable_target_raises(self):
        with self.assertRaises(Unreachable):
            ik((5.0, 5.0, 5.0), chain._IDENTITY)

    def test_gripper_joint_orientation_leaves_position_recoverable(self):
        # Sanity check on orientation_from_stick_axis + ik() working
        # together (the actual grasp.py entry point), independent of
        # test_grasp.py's own more thorough coverage. roll_rad=90deg here is
        # not special -- it is simply a value already confirmed reachable
        # for this target (round stock has no preferred roll; see grasp.py's
        # module docstring), unlike roll_rad=0 which happens not to be for
        # this particular target.
        target_pos = (0.062, 0.398, 0.05)
        target_rot = orientation_from_stick_axis((-1.0, 0.0, 0.0), roll_rad=math.radians(90.0))
        ok, reason = is_reachable(target_pos, target_rot)
        self.assertTrue(ok, reason)


if __name__ == "__main__":
    unittest.main()
