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
from so100_builder.kinematics.kr10_r900_2.chain import (
    Unreachable,
    fk,
    ik,
    translate_holding_wrist,
)
from so100_builder.kinematics.kr10_r900_2.constants import CHAIN
from so100_builder.kinematics.kr10_r900_2.envelope import is_reachable
from so100_builder.kinematics.kr10_r900_2.grasp import orientation_from_stick_axis

# The two joint-space waypoints tuned by hand in RViz/real hardware,
# verbatim from kuka_pick_and_place/config/pick_and_place.yaml (degrees,
# CHAIN order). These are ground truth for the URDF this module is derived
# from -- this module must agree with them, not the other way around.
#
# Re-tuned on real hardware 2026-08-25, superseding the previous values
# (last: pregrasp (-84.0, -37.0, 129.0, 168.0, 2.0, 153.0), lower (-84.0,
# -32.0, 127.0, 175.0, 5.0, 146.0) -- themselves a coterminal +360 rebase of
# the original -207.0/-214.0 sim-tuned joint6 values once the real A6 axis
# was found not to move past +/-180deg). This round is NOT a coterminal
# rewrite: joint4/joint6 moved by ~18deg each in opposite directions between
# the old and new pregrasp, consistent with this pose sitting close to the
# wrist singularity (chain.py's own docstring: joint4/joint6 become mutually
# redistributable there for the same net orientation) rather than a
# different physical pose.
TUNED_POSES_DEG = {
    "pregrasp": (-83.45, -36.78, 128.10, 150.09, 0.86, 171.01),
    "lower": (-83.45, -32.25, 126.35, 173.05, 3.55, 148.04),
}

# gripper_tcp position at each tuned pose. Originally cross-checked
# 2026-08-21 against MoveIt's own compute_fk (KDL, reading the live
# robot_description) while deriving GRASP_OFFSET_M -- see
# KUKA_IMPLEMENTATION_PLAN.md Sec 3 Phase 2. Re-cross-checked 2026-08-25
# against the same live compute_fk for the re-tuned poses above: MoveIt's
# independent KDL solver agreed with this module's own fk() to 6 decimal
# places for both. This is what makes these regression tests, not just
# internal-consistency checks: an independent solver already agreed with
# these numbers.
# Re-derived 2026-09-09 after GRIPPER_TCP_Z_M's own re-derivation (see that
# constant's own comment in constants.py -- a finger-mesh-swap finding, 2mm
# then a further 1mm closer to the mount, same day) shifted gripper_tcp for
# every pose, TUNED_POSES_DEG included. Original values (valid for the OLD
# GRIPPER_TCP_Z_M=0.0805): pregrasp (0.047431, 0.399936, 0.051480), lower
# (0.047428, 0.399898, 0.021598).
EXPECTED_TCP_POSITION_M = {
    "pregrasp": (0.047412, 0.399968, 0.054480),
    "lower": (0.047409, 0.399930, 0.024598),
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
        # KQ5) at both independently-tuned poses, confirmed to <0.02deg
        # (grasp.py's own module docstring cites this as real evidence, not
        # a guess) -- still holds after 2026-08-25's real-hardware retune
        # (TUNED_POSES_DEG's own comment; this round was NOT a coterminal
        # rewrite, so this is a fresh empirical check, not a by-construction
        # guarantee).
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
        # 180] deg) representative. joint6's own real +/-180deg limit
        # (tightened 2026-08-25, TUNED_POSES_DEG's own comment) means a
        # legitimately-tuned pose should already land inside that range for
        # THIS robot -- but ik()'s own output is compared modulo 360 here
        # regardless, on general principle: this test is about recovering
        # the right SOLUTION, not about re-asserting a limit that belongs to
        # CHAIN/the URDF, and a wider-range future joint could legitimately
        # need this wrap again.
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


class TestTranslateHoldingWrist(unittest.TestCase):
    """K11-class safety net (motion.py's own _reject_wild_cartesian_swing):
    the fallback for a small local move when ik()/MoveIt's own numerical
    Cartesian planner would need to redistribute joint4/joint6 wildly to
    hold this arm's real orientation exactly near its wrist singularity.
    See translate_holding_wrist's own module docstring for the full
    reasoning -- these tests just hold it to the two properties that
    actually matter: joint4/5/6 truly never move, and the resulting
    position is close enough to be useful.
    """

    LOWER_DEG = (-84.0, -32.0, 127.0, 175.0, 5.0, 146.0)

    def test_wrist_joints_are_never_touched(self):
        joints = tuple(math.radians(v) for v in self.LOWER_DEG)
        result = translate_holding_wrist(joints, (0.0, 0.0, 0.05))
        self.assertEqual(result[3], joints[3])
        self.assertEqual(result[4], joints[4])
        self.assertEqual(result[5], joints[5])

    def test_converges_close_to_the_intended_position(self):
        # This exact pose is the one found live to need ~17mm correction
        # with zero refinement passes (translate_holding_wrist's own
        # docstring) -- the whole point of the default iteration count is
        # that this converges well past "close enough to plan a collision
        # check around", not that it's already small before iterating.
        joints = tuple(math.radians(v) for v in self.LOWER_DEG)
        pos0, _rot0 = fk(joints)
        delta = (0.0, 0.0, 0.05)
        result = translate_holding_wrist(joints, delta)
        pos1, _rot1 = fk(result)
        want = tuple(p + d for p, d in zip(pos0, delta))
        err_m = math.dist(pos1, want)
        self.assertLess(err_m, 0.001, "%.3fmm error" % (err_m * 1000.0))

    def test_more_iterations_converge_tighter(self):
        joints = tuple(math.radians(v) for v in self.LOWER_DEG)
        pos0, _rot0 = fk(joints)
        delta = (0.0, 0.0, 0.05)
        want = tuple(p + d for p, d in zip(pos0, delta))

        def err_after(n):
            pos, _rot = fk(translate_holding_wrist(joints, delta, iterations=n))
            return math.dist(pos, want)

        self.assertGreater(err_after(0), err_after(2))
        self.assertGreater(err_after(2), err_after(6))

    def test_away_from_the_singularity_orientation_stays_close(self):
        # Sanity check the approximation is actually reasonable when NOT
        # near the singularity that motivates it: joint5 far from 0 means
        # joint4/joint6 are well-determined (not mutually redistributable),
        # so the resulting ORIENTATION (not necessarily every raw joint
        # angle -- Euler parameterization is sensitive enough that a small
        # physical difference can still shuffle individual joint values a
        # couple degrees) should stay close to the exact,
        # orientation-preserving ik() answer for a small move.
        base = (0.0, -30.0, 60.0, 20.0, 45.0, -10.0)
        joints = tuple(math.radians(v) for v in base)
        delta = (0.0, 0.0, 0.02)
        approx = translate_holding_wrist(joints, delta)
        pos0, rot0 = fk(joints)
        target_pos = tuple(p + d for p, d in zip(pos0, delta))
        exact = ik(target_pos, rot0, elbow_up=True, wrist_flip=False)

        _pos_a, rot_a = fk(approx)
        _pos_e, rot_e = fk(exact)
        # Angle between the two resulting X axes as a proxy for overall
        # orientation drift (any column would do -- rotation matrices
        # rotate rigidly together).
        x_a = (rot_a[0][0], rot_a[1][0], rot_a[2][0])
        x_e = (rot_e[0][0], rot_e[1][0], rot_e[2][0])
        dot = sum(a * b for a, b in zip(x_a, x_e))
        drift_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        self.assertLess(drift_deg, 3.0, "%.2fdeg orientation drift" % drift_deg)

    def test_unreachable_delta_raises(self):
        joints = tuple(math.radians(v) for v in self.LOWER_DEG)
        with self.assertRaises(Unreachable):
            translate_holding_wrist(joints, (5.0, 5.0, 5.0))


if __name__ == "__main__":
    unittest.main()
