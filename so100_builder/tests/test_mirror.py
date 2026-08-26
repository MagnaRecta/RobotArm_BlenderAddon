"""core/mirror.py -- BLENDER_ADDON_PLAN.md Sec 10.4 / Phase E.

``joint_frames()`` is derived entirely from calling the vendored, tested
``fk()`` on successively longer joint prefixes (see its own docstring for
why that is exact, not a second FK implementation) -- so what is worth
testing here is not "does FK work" (already covered by
``test_kinematics_vendored.py``) but "is the derivation itself correct":
every consecutive pair of points is a RIGID link, so its length must be
identical across every pose, reachable or not. A bug in the EE_OFFSET
correction would show up as a segment length that moves with the pose --
exactly what these tests check for.
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core.mirror import joint_frames as _joint_frames  # noqa: E402
from so100_builder.kinematics import so_arm_100 as _kinematics  # noqa: E402
from so100_builder.kinematics.so_arm_100.chain import fk  # noqa: E402
from so100_builder.kinematics.so_arm_100.constants import CHAIN  # noqa: E402


def joint_frames(joint_angles_rad):
    # Multi-robot support (docs/STATUS.md, 2026-08-21): joint_frames() now
    # takes the robot's kinematics module explicitly -- this file only ever
    # tests so_arm_100, so it fixes that argument here rather than repeating
    # it at every call site below.
    return _joint_frames(_kinematics, joint_angles_rad)

# A handful of joint-space poses spanning the documented limits, plus the
# all-zero pose (not necessarily physically reachable by ik(), but fk()
# does not require reachability -- joint_frames() must not either).
_POSES_DEG = [
    (0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, -99.98, 85.94, 71.62, 90.0),     # 'home'
    (-93.0, 54.0, -7.0, -46.0, 90.0),      # 'lower'
    (0.0, 68.0, -17.0, -51.0, 90.0),       # 'place'
    (45.0, -20.0, 30.0, -10.0, -60.0),
]


def _rad(pose_deg):
    return tuple(math.radians(v) for v in pose_deg)


def _segment_lengths(points):
    return [math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)]


class TestJointFrames(unittest.TestCase):
    def test_returns_six_points_for_five_joints(self):
        points = joint_frames(_rad(_POSES_DEG[0]))
        self.assertEqual(len(points), 6)

    def test_the_first_point_is_the_base_link_origin(self):
        points = joint_frames(_rad(_POSES_DEG[0]))
        self.assertEqual(points[0], (0.0, 0.0, 0.0))

    def test_the_last_point_is_exactly_the_true_tcp(self):
        for pose_deg in _POSES_DEG:
            q = _rad(pose_deg)
            points = joint_frames(q)
            expected_pos, _rot = fk(q)
            for got, want in zip(points[-1], expected_pos):
                self.assertAlmostEqual(got, want, places=9, msg=pose_deg)

    def test_segment_lengths_are_pose_invariant(self):
        # Every point-to-point gap is a rigid link -- its length cannot
        # depend on the joint angles, reachable or not.
        reference = _segment_lengths(joint_frames(_rad(_POSES_DEG[0])))
        for pose_deg in _POSES_DEG[1:]:
            lengths = _segment_lengths(joint_frames(_rad(pose_deg)))
            for got, want in zip(lengths, reference):
                self.assertAlmostEqual(got, want, places=9, msg=pose_deg)

    def test_segment_lengths_are_invariant_under_random_sampling_too(self):
        random.seed(20260731)
        reference = _segment_lengths(joint_frames(_rad(_POSES_DEG[0])))
        for _ in range(200):
            q = tuple(random.uniform(lo, hi) for (_n, _xyz, _rpy, _ax, lo, hi) in CHAIN)
            lengths = _segment_lengths(joint_frames(q))
            for got, want in zip(lengths, reference):
                self.assertAlmostEqual(got, want, places=9, msg=q)

    def test_no_segment_is_degenerate(self):
        lengths = _segment_lengths(joint_frames(_rad(_POSES_DEG[0])))
        for length in lengths:
            self.assertGreater(length, 1e-4)

    def test_first_frame_ignores_joints_after_shoulder_rotation(self):
        # Shoulder_Rotation's own frame origin must not depend on the other
        # four joints' values -- it is the first link, nothing upstream of
        # it has been applied yet.
        base_q = _rad((30.0, 0.0, 0.0, 0.0, 0.0))
        other_q = _rad((30.0, 40.0, -20.0, 15.0, -60.0))
        p1 = joint_frames(base_q)[1]
        p2 = joint_frames(other_q)[1]
        for got, want in zip(p1, p2):
            self.assertAlmostEqual(got, want, places=9)


if __name__ == "__main__":
    unittest.main()
