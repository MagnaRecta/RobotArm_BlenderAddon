"""core/validate.py -- BLENDER_ADDON_PLAN.md Sec 7, Phase B.

Numbers here are cross-checked against the vendored, hardware-validated
kinematics module directly (fk()/ik()/tool_axis()), not just against this
module's own formulas -- see core/validate.py's module docstring for the
empirical findings this test suite locks in.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core import sticks as core_sticks  # noqa: E402
from so100_builder.core import validate as V  # noqa: E402
from so100_builder.core.transform import v_dot, v_sub  # noqa: E402
from so100_builder.kinematics.so_arm_100 import chain as kchain  # noqa: E402
from so100_builder.kinematics.so_arm_100 import envelope as kenvelope  # noqa: E402
from so100_builder.kinematics.so_arm_100.constants import GRASP_OFFSET_M  # noqa: E402


def make_stick(base, tip, stick_id="s_test"):
    return core_sticks.StickSpec(
        id=stick_id, base=base, tip=tip, length_m=v_dot(v_sub(tip, base), v_sub(tip, base)) ** 0.5,
        shared_ends=0,
    )


class TestGraspTarget(unittest.TestCase):
    def test_target_is_offset_from_base_toward_tip(self):
        stick = make_stick((0.0, -0.36, 0.0), (0.0, -0.36, 0.110))
        target = V.stick_grasp_target(stick)
        self.assertAlmostEqual(target[0], 0.0, places=9)
        self.assertAlmostEqual(target[1], -0.36, places=9)
        self.assertAlmostEqual(target[2], GRASP_OFFSET_M, places=9)

    def test_matches_the_known_lower_pose_relationship(self):
        # README/constants.py: GRASP_OFFSET_M was derived as (TCP height at
        # the tuned 'lower' pose) - (feeder stick's base height). The
        # formula here should reproduce that relationship for a vertical
        # stick whose base sits at the feeder's estimated height.
        base_z = 0.014
        stick = make_stick((0.0, 0.0, base_z), (0.0, 0.0, base_z + 0.100))
        target = V.stick_grasp_target(stick)
        self.assertAlmostEqual(target[2], base_z + GRASP_OFFSET_M, places=9)


class TestKnownVerticalPlacements(unittest.TestCase):
    """The common case (Sec 9.4: "largest envelope") -- must validate
    cleanly and match is_reachable()'s own envelope."""

    def test_a_vertical_stick_well_inside_the_envelope_is_buildable(self):
        stick = make_stick((0.0, -0.36, 0.0), (0.0, -0.36, 0.110))
        verdict = V.validate_stick(stick)
        self.assertTrue(verdict.buildable, verdict.reason)
        self.assertEqual(verdict.warnings, ())

    def test_a_vertical_stick_far_out_of_reach_is_impossible(self):
        stick = make_stick((2.0, 0.0, 0.0), (2.0, 0.0, 0.110))
        verdict = V.validate_stick(stick)
        self.assertFalse(verdict.buildable)
        self.assertIsNotNone(verdict.reason)

    def test_vertical_reachability_matches_the_envelope_sweep(self):
        # Cross-check against kinematics.sweep_envelope(), which sweeps TCP
        # position directly -- so a stick's BASE must sit GRASP_OFFSET_M
        # below the swept height for its grasp target to land there.
        tcp_z = 0.10
        min_r, max_r = kenvelope.sweep_envelope([tcp_z], tool_elevation_target_rad=0.0)[tcp_z]
        base_z = tcp_z - GRASP_OFFSET_M

        mid_r = (min_r + max_r) / 2.0
        stick = make_stick((mid_r, 0.0, base_z), (mid_r, 0.0, base_z + 0.100))
        self.assertTrue(V.validate_stick(stick).buildable)

        too_far = max_r + 0.10
        stick = make_stick((too_far, 0.0, base_z), (too_far, 0.0, base_z + 0.100))
        self.assertFalse(V.validate_stick(stick).buildable)

    def test_below_minimum_radius_is_impossible(self):
        # Sec 9.4: nothing closer than ~311mm can be placed vertically at
        # table height -- the near edge is as real a limit as the far edge.
        stick = make_stick((0.05, 0.0, 0.0), (0.05, 0.0, 0.100))
        verdict = V.validate_stick(stick)
        self.assertFalse(verdict.buildable)


class TestOutOfPlaneReachability(unittest.TestCase):
    """The disputed Sec 9.4 case: empirically reachable, contradicting the
    plan's table. Locking in the finding, not re-deriving it."""

    @staticmethod
    def _tilted_stick_at(target, elevation_rad, roll_rad):
        """Build a StickSpec whose grasp target reproduces `target`, using
        the known-reachable (elevation, roll) pair from a direct ik() call.
        stick.axis() is base->tip; the grip-to-base direction (what
        validate.py's fk() column actually represents) is its negation, so
        base = target + GRASP_OFFSET_M * grip_dir."""
        q = kchain.ik(target, tool_elevation_target_rad=elevation_rad, stick_roll_rad=roll_rad)
        _pos, rot = kchain.fk(q)
        grip_dir = (rot[0][2], rot[1][2], rot[2][2])
        base = tuple(t + GRASP_OFFSET_M * g for t, g in zip(target, grip_dir))
        tip = tuple(t - GRASP_OFFSET_M * g for t, g in zip(target, grip_dir))
        return make_stick(base, tip)

    def test_a_documented_out_of_plane_tilt_is_reachable(self):
        # From the probe that found this: target (0.30,-0.0452,0.10),
        # tool elevation 0, stick_roll -45deg gives a stick tilted out of
        # the arm's own vertical plane (nonzero tangential component) and
        # IS reachable per chain.ik() directly.
        stick = self._tilted_stick_at((0.30, -0.0452, 0.10), 0.0, math.radians(-45.0))
        verdict = V.validate_stick(stick)
        self.assertTrue(verdict.buildable, verdict.reason)

    def test_out_of_plane_tilt_carries_a_warning_even_when_buildable(self):
        stick = self._tilted_stick_at((0.30, -0.0452, 0.10), 0.0, math.radians(-45.0))
        verdict = V.validate_stick(stick)
        self.assertIn(V.WARN_OUT_OF_PLANE_TILT, verdict.warnings)

    def test_vertical_stick_never_carries_the_out_of_plane_warning(self):
        stick = make_stick((0.0, -0.36, 0.0), (0.0, -0.36, 0.110))
        verdict = V.validate_stick(stick)
        self.assertNotIn(V.WARN_OUT_OF_PLANE_TILT, verdict.warnings)


# The orientation solver itself (exact-root search, cardinal anchors,
# self-verification via fk()) now lives in kinematics/grasp.py and is
# exercised there -- see test_kinematics_vendored.py's grasp-specific
# classes, which vendor so_arm_100_kinematics/test/test_grasp.py's own
# regression cases verbatim. This file only covers validate.py's own UI-
# facing logic (Verdict wiring, warning codes, the flip suggestion).


class TestUnreachablePosition(unittest.TestCase):
    def test_target_on_the_shoulder_axis_is_reported_specifically(self):
        from so100_builder.kinematics.so_arm_100.constants import CHAIN
        shoulder = CHAIN[0][1]
        stick = make_stick((shoulder[0], shoulder[1], 0.0), (shoulder[0], shoulder[1], 0.100))
        verdict = V.validate_stick(stick)
        self.assertFalse(verdict.buildable)
        self.assertIn("azimuth undefined", verdict.reason)


class TestValidateSticks(unittest.TestCase):
    def test_validates_every_stick_independently_by_id(self):
        sticks = [
            make_stick((0.0, -0.36, 0.0), (0.0, -0.36, 0.110), "s_001"),
            make_stick((2.0, 0.0, 0.0), (2.0, 0.0, 0.110), "s_002"),
        ]
        verdicts = V.validate_sticks(sticks)
        self.assertEqual(set(verdicts), {"s_001", "s_002"})
        self.assertTrue(verdicts["s_001"].buildable)
        self.assertFalse(verdicts["s_002"].buildable)


class TestNearDegenerateElevationBranch(unittest.TestCase):
    """Regression for a real bug found 2026-07-29 via the user's own
    inverted-U test (two vertical uprights + one horizontal top, all
    110mm): the top stick's axis, after the mesh-expansion solve, is not
    exactly tangential -- off by a fraction of a degree, pure relaxation-
    solver noise, not a design intent. That was enough to make the exact
    elevation formula (numerically ill-conditioned right at the
    tangential case) pick a ~90deg-wrong branch demanding an impossible
    328mm reach, when the natural elevation=0 pose (Sec 9.4's own
    "horizontal tangential -- same as vertical") reaches it fine with
    <1deg of orientation error. See core/validate.py's Finding 4."""

    def test_the_inverted_u_top_stick_is_buildable_end_to_end(self):
        # Exactly the user's reproduction, through the full extraction +
        # validation pipeline, not just the orientation solver in isolation.
        y = -0.370
        points = [(-0.055, y, 0.0), (-0.055, y, 0.110), (0.055, y, 0.110), (0.055, y, 0.0)]
        result = core_sticks.extract_sticks(
            points, [(0, 1), (1, 2), (2, 3)], ["upright_l", "top", "upright_r"],
            ground_mode=core_sticks.GROUND_SLIDE,
        )
        verdicts = V.validate_sticks(result.sticks)
        for stick_id, verdict in verdicts.items():
            self.assertTrue(verdict.buildable, "%s: %s" % (stick_id, verdict.reason))

    def test_the_larger_asymmetry_case_warns_about_the_approximation(self):
        stick = make_stick(
            (0.06286285696057388, -0.37227827310562134, 0.11322574731245236),
            (-0.04713571584315837, -0.37227827310562134, 0.11327491791834149),
        )
        verdict = V.validate_stick(stick)
        self.assertTrue(verdict.buildable, verdict.reason)
        self.assertIn(V.WARN_ORIENTATION_APPROXIMATED, verdict.warnings)


class TestApproximationWarning(unittest.TestCase):
    def test_an_exact_root_solution_does_not_warn(self):
        # Vertical: solved exactly (elevation=0 IS the exact root here, not
        # an approximation), so no approximation warning.
        stick = make_stick((0.0, -0.36, 0.0), (0.0, -0.36, 0.110))
        verdict = V.validate_stick(stick)
        self.assertTrue(verdict.buildable)
        self.assertNotIn(V.WARN_ORIENTATION_APPROXIMATED, verdict.warnings)


class TestFlipSuggestion(unittest.TestCase):
    """Regression for a THIRD real case, same user, 2026-07-30: a
    near-horizontal stick whose "base = lower Z" tie-break (Sec 5.1) can
    land on either end for a nearly-perfectly-level stick, since a
    mesh-expansion-noise-scale Z difference decides it. Wrist_Roll's limit
    is asymmetric (~-157..+68 deg, WRIST_ROLL_AT_ZERO_STICK_ROLL_RAD=+90deg
    eating most of the positive headroom), so ONE end-assignment can be
    genuinely unreachable (needs +90ish deg roll) while the OTHER
    (negated grip direction, needs -90ish deg roll) is comfortably fine.
    Not a numerical-conditioning bug like Findings 4/5 -- both anchors
    here fail even at a 20deg tolerance; the fix is pointing at the
    existing per-stick Flip override, not accepting a wider error."""

    def test_the_flip_suggestion_case_is_reported_with_a_flip_hint(self):
        # Exact target/axis recovered from the user's own mesh: buildable
        # with THIS end as base is genuinely impossible (needs +90ish deg
        # roll, past Wrist_Roll's +68ish deg headroom) -- not a tolerance
        # issue, confirmed both cardinal anchors and exact roots all fail
        # even at a 20deg tolerance.
        target = (0.0038386544587053484, -0.37227827310562134, 0.11320022203672594)
        axis = (0.999999999999996, 0.0, 8.95262103240344e-08)
        stick = make_stick(
            (target[0] - 0.051 * axis[0], target[1], target[2]),
            (target[0] - 0.051 * axis[0] + 0.110 * axis[0], target[1], target[2]),
        )
        verdict = V.validate_stick(stick)
        self.assertFalse(verdict.buildable)
        self.assertIn("Flip", verdict.reason)

    def test_flipping_the_reported_case_actually_builds(self):
        target = (0.0038386544587053484, -0.37227827310562134, 0.11320022203672594)
        axis = (0.999999999999996, 0.0, 8.95262103240344e-08)
        base = (target[0] - 0.051 * axis[0], target[1], target[2])
        tip = (target[0] - 0.051 * axis[0] + 0.110 * axis[0], target[1], target[2])
        flipped = make_stick(tip, base)  # swap base/tip, exactly what the Flip toggle does
        verdict = V.validate_stick(flipped)
        self.assertTrue(verdict.buildable, verdict.reason)

    def test_a_genuinely_unreachable_stick_does_not_falsely_suggest_flip(self):
        stick = make_stick((2.0, 0.0, 0.0), (2.0, 0.0, 0.110))
        verdict = V.validate_stick(stick)
        self.assertFalse(verdict.buildable)
        self.assertNotIn("Flip", verdict.reason)


class TestOutOfPlaneWarningCalibration(unittest.TestCase):
    """Regression for the companion miscalibration found alongside the
    elevation bug: the warning fired on ALIGNMENT with tan_hat, which
    flags the safe, documented pure-tangential case (Sec 9.4: "same poses
    as vertical") as maximally risky. It should fire on a genuine MIX of
    in-plane and tangential lean instead -- angularly far from both
    extremes, not merely far from one."""

    def test_pure_tangential_does_not_warn(self):
        stick = make_stick((0.30, -0.0452 - 0.055, 0.10), (0.30, -0.0452 + 0.055, 0.10))
        verdict = V.validate_stick(stick)
        self.assertNotIn(V.WARN_OUT_OF_PLANE_TILT, verdict.warnings)

    def test_near_tangential_noise_does_not_warn(self):
        # The inverted-U's actual top stick: 0.71 deg off pure tangential.
        stick = make_stick(
            (0.05502588048152792, -0.37, 0.11319947840501104),
            (-0.0549741194806364, -0.37, 0.11320236351371533),
        )
        verdict = V.validate_stick(stick)
        self.assertNotIn(V.WARN_OUT_OF_PLANE_TILT, verdict.warnings)

    def test_pure_in_plane_tilt_does_not_warn(self):
        # X=0 for both ends keeps the stick (and, since the grasp offset is
        # applied along the stick's own axis, the grasp target too) exactly
        # in the shoulder axis's X=0 plane -- so the target's azimuth is
        # exactly along +-Y and tan_hat exactly along +-X, giving this
        # stick precisely zero tangential component, not merely close to it.
        stick = make_stick((0.0, -0.30, 0.0), (0.0, -0.36, 0.08))
        verdict = V.validate_stick(stick)
        self.assertNotIn(V.WARN_OUT_OF_PLANE_TILT, verdict.warnings)

    def test_a_genuine_45_degree_mix_still_warns(self):
        stick = TestOutOfPlaneReachability._tilted_stick_at(
            (0.30, -0.0452, 0.10), 0.0, math.radians(-45.0)
        )
        verdict = V.validate_stick(stick)
        self.assertIn(V.WARN_OUT_OF_PLANE_TILT, verdict.warnings)


class TestWireframeCubeValidation(unittest.TestCase):
    """Cross-check against the same cube fixture used in the Blender
    integration tests: the 4 vertical uprights are the common case (Sec
    9.4, "largest envelope") and must validate cleanly, independent of
    whatever the horizontal top/bottom-ring edges' fate turns out to be."""

    def test_vertical_uprights_are_always_buildable(self):
        edge = 0.110
        y = -0.370
        h = edge / 2.0
        points = [
            (-h, y - h, 0.0), (h, y - h, 0.0), (h, y + h, 0.0), (-h, y + h, 0.0),
            (-h, y - h, edge), (h, y - h, edge), (h, y + h, edge), (-h, y + h, edge),
        ]
        cube_edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                     (0, 4), (1, 5), (2, 6), (3, 7)]
        result = core_sticks.extract_sticks(points, cube_edges, ground_mode=core_sticks.GROUND_SLIDE)
        verdicts = V.validate_sticks(result.sticks)

        upright_ids = {result.topology.edges[i][0] for i in (8, 9, 10, 11)}
        for stick in result.sticks:
            if stick.id in upright_ids:
                self.assertTrue(verdicts[stick.id].buildable,
                               "%s: %s" % (stick.id, verdicts[stick.id].reason))


class TestPerformance(unittest.TestCase):
    def test_validating_150_sticks_is_fast(self):
        # Constraint B3: must not need chunking at realistic build sizes.
        # Measured ~0.09 ms/stick standalone; generous ceiling here to
        # absorb machine variance without becoming a flaky test.
        import random
        import time

        random.seed(11)
        sticks = []
        for i in range(150):
            az = random.uniform(-1.4, 1.4)
            r = random.uniform(0.30, 0.40)
            base = (r * math.cos(az), r * math.sin(az) - 0.045, random.uniform(0.0, 0.12))
            tilt = random.uniform(-0.3, 0.3)
            tip = (base[0] + 0.100 * math.sin(tilt), base[1], base[2] + 0.100 * math.cos(tilt))
            sticks.append(make_stick(base, tip, "s_%03d" % i))

        start = time.perf_counter()
        V.validate_sticks(sticks)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed, 1.0, "150-stick validation took %.3fs" % elapsed)


if __name__ == "__main__":
    unittest.main()
