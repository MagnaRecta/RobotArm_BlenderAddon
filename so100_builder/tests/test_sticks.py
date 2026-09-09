"""core/sticks.py -- BLENDER_ADDON_PLAN.md Sec 5, the addon's defining logic.

Numbers here are hand-checked against the spec's own worked examples and are
deliberately hardcoded: they are the contract, not incidental output.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core import sticks as S  # noqa: E402
from so100_builder.core.transform import v_dist  # noqa: E402
from so100_builder.kinematics.so_arm_100 import constants as kc  # noqa: E402

MM = 0.001


def mm(v):
    return v * 1000.0


# --- the Sec 5.2.1 worked example -------------------------------------------
# Inverted U, three physically-110 mm sticks, standing in the build volume.
U_POINTS = [
    (-0.055, -0.370, 0.000),  # 0  foot, left   (grounded)
    (-0.055, -0.370, 0.110),  # 1  top,  left
    (0.055, -0.370, 0.110),   # 2  top,  right
    (0.055, -0.370, 0.000),   # 3  foot, right  (grounded)
]
U_EDGES = [(0, 1), (1, 2), (2, 3)]
U_IDS = ["upright_l", "top", "upright_r"]


class TestWorkedInvertedU(unittest.TestCase):
    """Sec 5.2.1's table, confirmed with the user 2026-07-28 as 113.25 /
    113.25 / 116.5 mm (the request's 114.25 / 117.5 were off by 1 mm)."""

    def test_required_edge_lengths_match_the_spec_table(self):
        topology, _ = S.build_topology(U_POINTS, U_EDGES, U_IDS)
        required, shared = S.required_edge_lengths(topology, [0.110] * 3)

        self.assertEqual(shared, [1, 2, 1])
        self.assertAlmostEqual(mm(required[0]), 113.25, places=6)  # upright: top only
        self.assertAlmostEqual(mm(required[1]), 116.50, places=6)  # top: both ends
        self.assertAlmostEqual(mm(required[2]), 113.25, places=6)

    def test_foot_is_not_a_shared_end(self):
        # "An end that seats on the base plate is not shared and gets no
        # allowance" -- this is why an upright is 113.25 and not 116.5.
        topology, _ = S.build_topology(U_POINTS, U_EDGES, U_IDS)
        self.assertEqual(S.shared_end_flags(topology), [(False, True), (True, True), (True, False)])

    def test_the_u_solves_exactly_under_either_ground_mode(self):
        # Two feet on the plate make the plate behave as a fourth edge, so
        # this acyclic mesh takes the relaxation path. It still lands every
        # edge on target: a 4-vertex U has enough freedom to be exact, which
        # is why PIN and SLIDE agree here.
        for mode in (S.GROUND_SLIDE, S.GROUND_PIN):
            result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS, ground_mode=mode)
            self.assertEqual(len(result.sticks), 3)
            self.assertEqual(result.expansion.method, "relaxation")
            self.assertLess(result.expansion.max_residual_m, 0.001 * MM,
                            msg="ground_mode=%s" % mode)
            self.assertEqual([e[1] for e in result.errors], [])

    def test_the_uprights_tilt_to_absorb_the_widened_top_edge(self):
        # Sec 5.2.2's description of this exact case: widening the top edge
        # tilts the uprights by ~1.6 deg. Tilting shortens their vertical
        # extent and the solver raises the tops to compensate, so the tilt
        # costs no residual.
        for mode in (S.GROUND_SLIDE, S.GROUND_PIN):
            result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS, ground_mode=mode)
            upright = next(s for s in result.sticks if s.id == "upright_l")
            axis = upright.axis()
            tilt_deg = math.degrees(math.acos(min(1.0, abs(axis[2]))))
            self.assertAlmostEqual(tilt_deg, 1.65, delta=0.15, msg=mode)
            self.assertAlmostEqual(mm(upright.solved_edge_m), 113.25, places=3)

    def test_stick_lengths_are_untouched_by_the_expansion(self):
        # The whole point of Sec 5.2.1: the design grows, the sticks do not.
        result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS)
        for stick in result.sticks:
            self.assertAlmostEqual(mm(stick.length_m), 110.0, places=6)


# --- Phase A's "done when": a wireframe cube --------------------------------

CUBE_HALF = 0.055
CUBE_Y = -0.370


def cube_points(edge_m=0.110):
    h = edge_m / 2.0
    return [
        (-h, CUBE_Y - h, 0.0), (h, CUBE_Y - h, 0.0), (h, CUBE_Y + h, 0.0), (-h, CUBE_Y + h, 0.0),
        (-h, CUBE_Y - h, edge_m), (h, CUBE_Y - h, edge_m), (h, CUBE_Y + h, edge_m),
        (-h, CUBE_Y + h, edge_m),
    ]


CUBE_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),      # bottom ring (on the plate)
    (4, 5), (5, 6), (6, 7), (7, 4),      # top ring
    (0, 4), (1, 5), (2, 6), (3, 7),      # uprights
]


class TestWireframeCube(unittest.TestCase):
    """Phase A is done when a wireframe cube produces 12 sticks with correct
    base_link metre coordinates and correct stick lengths."""

    def test_twelve_sticks_all_110mm(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES)
        self.assertEqual(len(result.sticks), 12)
        for stick in result.sticks:
            self.assertAlmostEqual(mm(stick.length_m), 110.0, places=6)

    def test_every_vertex_has_valence_three_so_all_ends_are_shared(self):
        topology, _ = S.build_topology(cube_points(), CUBE_EDGES)
        self.assertEqual(len(topology.positions), 8)
        self.assertEqual(len(topology.edges), 12)
        for i in range(8):
            self.assertEqual(topology.degree(i), 3)
        for stick_shared in S.required_edge_lengths(topology, [0.110] * 12)[1]:
            self.assertEqual(stick_shared, 2)

    def test_all_twelve_required_edges_are_116_5mm(self):
        topology, _ = S.build_topology(cube_points(), CUBE_EDGES)
        required, _ = S.required_edge_lengths(topology, [0.110] * 12)
        for value in required:
            self.assertAlmostEqual(mm(value), 116.5, places=6)

    def test_coordinates_land_inside_the_build_volume(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_SLIDE)
        lo, hi = result.expanded_bounds
        for axis in range(3):
            self.assertGreaterEqual(hi[axis], kc.BUILD_VOLUME_MIN_M[axis] - 1e-9)
            self.assertLessEqual(lo[axis], kc.BUILD_VOLUME_MAX_M[axis] + 1e-9)

    def test_pinned_ground_cannot_expand_a_grounded_ring(self):
        # Why SLIDE became the default (decided 2026-07-28). Under PIN every
        # vertex of the bottom ring is immobile, so those four edges cannot
        # grow at all and miss their target by the full 6.5 mm -- a
        # perfectly buildable cube reports four impossible sticks.
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_PIN)
        residual_errors = [e for e in result.errors if e[1] == "residual_too_large"]
        self.assertEqual(len(residual_errors), 4)
        bottom = [s for s in result.sticks if s.base[2] < 1e-6 and s.tip[2] < 1e-6]
        for stick in bottom:
            self.assertAlmostEqual(mm(stick.residual_m), -6.5, places=3)

    def test_sliding_ground_solves_the_cube_exactly(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_SLIDE)
        self.assertTrue(result.expansion.converged)
        self.assertLess(result.expansion.max_residual_m, 0.01 * MM)
        self.assertEqual([e[1] for e in result.errors], [])

    def test_sliding_ground_keeps_the_first_layer_on_the_plate(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_SLIDE)
        for stick in result.sticks:
            if stick.base[2] < 1e-6:
                self.assertAlmostEqual(stick.base[2], 0.0, places=9)

    def test_the_built_cube_is_larger_than_the_design(self):
        # Sec 5.2.3: the built sculpture is larger than the mesh you drew.
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_SLIDE)
        design_lo, design_hi = result.design_bounds
        built_lo, built_hi = result.expanded_bounds
        design_height = design_hi[2] - design_lo[2]
        built_height = built_hi[2] - built_lo[2]
        self.assertAlmostEqual(mm(design_height), 110.0, places=6)
        self.assertAlmostEqual(mm(built_height), 116.5, places=3)


# --- Sec 5.2: the physical inset --------------------------------------------

STACK_POINTS = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.110), (0.0, -0.360, 0.220)]
STACK_EDGES = [(0, 1), (1, 2)]


class TestPhysicalEndpoints(unittest.TestCase):
    def test_tip_minus_base_always_equals_length(self):
        # BRIDGE_PROTOCOL.md Sec 6: the ROS2 side rejects a >1 mm
        # inconsistency between length_m and ||tip - base|| with bad_request.
        for points, edges in ((U_POINTS, U_EDGES), (cube_points(), CUBE_EDGES),
                              (STACK_POINTS, STACK_EDGES)):
            result = S.extract_sticks(points, edges, ground_mode=S.GROUND_SLIDE)
            for stick in result.sticks:
                self.assertAlmostEqual(
                    v_dist(stick.base, stick.tip), stick.length_m, places=9,
                    msg="stick %s violates ||tip-base|| == length_m" % stick.id,
                )

    def test_grounded_stick_starts_exactly_on_the_plate(self):
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES)
        lower = min(result.sticks, key=lambda s: s.base[2])
        self.assertAlmostEqual(lower.base[2], 0.0, places=9)
        self.assertAlmostEqual(mm(lower.tip[2]), 110.0, places=6)

    def test_both_sticks_inset_from_a_shared_vertex(self):
        # Two collinear sticks sharing one vertex: the exact walk puts that
        # vertex at 113.25 mm, and BOTH sticks stop 3.25 mm short of it, so
        # the glue gap is 6.5 mm.
        #
        # ⚠ SPEC INCONSISTENCY, reported rather than silently resolved:
        # BRIDGE_PROTOCOL.md Sec A.2's worked example shows this same stacked
        # pair with s_001.tip at z=0.110 and s_002.base at z=0.11325 -- a
        # 3.25 mm gap, i.e. only ONE end inset. That contradicts
        # BLENDER_ADDON_PLAN.md Sec 5.2 ("each stick must stop 3.25 mm short"
        # of a shared vertex) and Sec 5.2.1's required_edge formula, which the
        # user confirmed. The formula wins; the protocol's example numbers do
        # not follow it.
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES)
        self.assertEqual(result.expansion.method, "exact")
        lower, upper = sorted(result.sticks, key=lambda s: s.base[2])
        self.assertAlmostEqual(mm(lower.tip[2]), 110.0, places=6)
        self.assertAlmostEqual(mm(upper.base[2]), 116.5, places=6)
        self.assertAlmostEqual(mm(upper.base[2] - lower.tip[2]), 6.5, places=6)

    def test_free_top_end_is_not_inset(self):
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES)
        upper = max(result.sticks, key=lambda s: s.tip[2])
        self.assertAlmostEqual(mm(upper.tip[2]), 226.5, places=6)


# --- Sec 5.2.2: which solver runs -------------------------------------------


SQUARE_ON_THE_PLATE = [
    (-0.055, -0.425, 0.0), (0.055, -0.425, 0.0),
    (0.055, -0.315, 0.0), (-0.055, -0.315, 0.0),
]
SQUARE_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0)]


class TestFlatFirstLayer(unittest.TestCase):
    """A square drawn flat on the base plate -- four grounded vertices and no
    free ones. This is the case that decided the ground mode."""

    def test_slide_is_the_default(self):
        import inspect

        for function in (S.extract_sticks, S.solve_expansion):
            self.assertEqual(
                inspect.signature(function).parameters["ground_mode"].default,
                S.GROUND_SLIDE,
                msg=function.__name__,
            )

    def test_a_flat_square_solves_exactly_by_default(self):
        result = S.extract_sticks(SQUARE_ON_THE_PLATE, SQUARE_EDGES)
        self.assertEqual(len(result.sticks), 4)
        self.assertLess(result.expansion.max_residual_m, 0.01 * MM)
        self.assertEqual([e[1] for e in result.errors], [])

    def test_the_square_grows_by_one_allowance_per_end(self):
        result = S.extract_sticks(SQUARE_ON_THE_PLATE, SQUARE_EDGES)
        for stick in result.sticks:
            # The stick itself is untouched; the edge it spans grows by one
            # allowance at each of its two shared ends.
            self.assertAlmostEqual(mm(stick.length_m), 110.0, places=4)
            self.assertAlmostEqual(mm(stick.required_edge_m), 116.5, places=4)
            self.assertAlmostEqual(mm(stick.solved_edge_m), 116.5, places=2)

        # The bounding box is only approximately 116.5: relaxation constrains
        # edge lengths, not corner angles, so the square is free to rotate a
        # few hundredths of a degree. That is not an error -- the edges are
        # what the sticks have to match.
        lo, hi = result.expanded_bounds
        self.assertAlmostEqual(mm(hi[0] - lo[0]), 116.5, delta=0.2)
        self.assertAlmostEqual(mm(hi[1] - lo[1]), 116.5, delta=0.2)

    def test_the_square_stays_flat_on_the_plate(self):
        result = S.extract_sticks(SQUARE_ON_THE_PLATE, SQUARE_EDGES)
        for position in result.expansion.positions:
            self.assertAlmostEqual(position[2], 0.0, places=9)
        for stick in result.sticks:
            self.assertAlmostEqual(stick.base[2], 0.0, places=9)
            self.assertAlmostEqual(stick.tip[2], 0.0, places=9)

    def test_pinning_would_have_called_it_unbuildable(self):
        result = S.extract_sticks(SQUARE_ON_THE_PLATE, SQUARE_EDGES,
                                  ground_mode=S.GROUND_PIN)
        self.assertEqual(
            len([e for e in result.errors if e[1] == "residual_too_large"]), 4
        )


class TestBasePlateFloor(unittest.TestCase):
    """Nothing can be built below z=0 -- the plate is physical."""

    def test_a_stick_below_the_plate_is_reported(self):
        points = [(0.0, -0.360, -0.040), (0.0, -0.360, 0.070)]
        result = S.extract_sticks(points, [(0, 1)])
        self.assertIn("below_plate", [e[1] for e in result.errors])
        message = next(e[2] for e in result.errors if e[1] == "below_plate")
        self.assertIn("40.0 mm below", message)

    def test_a_grounded_vertex_is_never_pushed_below_the_plate(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES)
        self.assertNotIn("below_plate", [e[1] for e in result.errors])
        for position in result.expansion.positions:
            self.assertGreaterEqual(position[2], -1e-9)

    def test_below_plate_is_relative_to_a_raised_ground_height(self):
        # A design that sits happily on a raised plate (base at its own
        # Z=0.100) still reports below_plate once the plate is raised
        # PAST it -- the same physical check, just relative to the new
        # height rather than a hardcoded Z=0.
        points = [(0.0, -0.360, 0.100), (0.0, -0.360, 0.210)]
        result = S.extract_sticks(points, [(0, 1)], ground_height_m=0.100)
        self.assertNotIn("below_plate", [e[1] for e in result.errors])
        result = S.extract_sticks(points, [(0, 1)], ground_height_m=0.150)
        self.assertIn("below_plate", [e[1] for e in result.errors])


class TestExpansionSolver(unittest.TestCase):
    def test_a_tree_with_one_foot_is_solved_exactly(self):
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES)
        self.assertEqual(result.expansion.method, "exact")
        self.assertEqual(result.expansion.max_residual_m, 0.0)

    def test_the_exact_walk_preserves_the_design_angles(self):
        points = [(0.0, -0.370, 0.0), (0.0, -0.370, 0.110), (0.0, -0.290, 0.185)]
        edges = [(0, 1), (1, 2)]
        result = S.extract_sticks(points, edges)
        self.assertEqual(result.expansion.method, "exact")
        original = math.atan2(0.185 - 0.110, -0.290 + 0.370)
        arm = next(s for s in result.sticks if s.base[2] > 0.05)
        axis = arm.axis()
        self.assertAlmostEqual(math.atan2(axis[2], axis[1]), original, places=9)

    def test_a_loop_uses_relaxation_and_reports_residuals(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_SLIDE)
        self.assertEqual(result.expansion.method, "relaxation")
        self.assertEqual(len(result.expansion.residuals), 12)

    def test_solver_reports_every_edge_residual(self):
        result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS)
        for stick in result.sticks:
            self.assertAlmostEqual(
                stick.solved_edge_m - stick.required_edge_m, stick.residual_m, places=9
            )


# --- Sec 5.2.1 / N7: growth modes -------------------------------------------


class TestGrowthModes(unittest.TestCase):
    def test_uniform_mode_grows_every_edge_by_exactly_6_5mm(self):
        topology, _ = S.build_topology(U_POINTS, U_EDGES, U_IDS)
        required, shared = S.required_edge_lengths(
            topology, [0.110] * 3, growth_mode=S.GROWTH_UNIFORM
        )
        for value in required:
            self.assertAlmostEqual(mm(value), 116.5, places=6)
        # The reported shared count stays truthful even in uniform mode --
        # it goes into the build file and to the operator.
        self.assertEqual(shared, [1, 2, 1])

    def test_uniform_mode_makes_free_ends_overhang_harmlessly(self):
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES, growth_mode=S.GROWTH_UNIFORM)
        lower = min(result.sticks, key=lambda s: s.base[2])
        self.assertAlmostEqual(mm(lower.base[2]), 3.25, places=6)

    def test_uniform_mode_is_better_conditioned_on_a_loop(self):
        # Sec 5.2.2's stated reason for offering it.
        pinned = S.extract_sticks(cube_points(), CUBE_EDGES, growth_mode=S.GROWTH_PER_EDGE)
        uniform = S.extract_sticks(cube_points(), CUBE_EDGES, growth_mode=S.GROWTH_UNIFORM)
        self.assertLessEqual(uniform.expansion.max_residual_m,
                             pinned.expansion.max_residual_m + 1e-12)


# --- Sec 5.3: length modes ---------------------------------------------------


class TestLengthModes(unittest.TestCase):
    def test_design_driven_mode_uses_the_edge_as_drawn(self):
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.123)]
        result = S.extract_sticks(points, [(0, 1)])
        self.assertAlmostEqual(mm(result.sticks[0].length_m), 123.0, places=6)

    def test_snap_to_stock_picks_the_nearest_length(self):
        snapped, errors = S.snap_to_stock([0.083, 0.107, 0.149], [0.080, 0.100, 0.120, 0.150])
        self.assertEqual(snapped, [0.080, 0.100, 0.150])
        self.assertAlmostEqual(mm(errors[0]), -3.0, places=6)
        self.assertAlmostEqual(mm(errors[1]), -7.0, places=6)
        self.assertAlmostEqual(mm(errors[2]), 1.0, places=6)

    def test_an_exact_tie_snaps_to_the_shorter_stock_deterministically(self):
        # 110 mm is exactly between 100 and 120. Without an explicit
        # tie-break this is decided by float noise, so the twelve identical
        # edges of one cube snap to different stock lengths. The noise here
        # is the real float32 spread measured off a Blender cube.
        for noise in (0.0, 1.4305e-8, -5.96e-10, 3e-7, -3e-7):
            snapped, _ = S.snap_to_stock([0.110 + noise], [0.100, 0.120])
            self.assertAlmostEqual(snapped[0], 0.100, places=9,
                                   msg="noise=%g flipped the tie" % noise)

    def test_a_genuine_difference_still_wins_over_the_tie_break(self):
        # The tie-break must not swallow a real 1 mm preference.
        snapped, _ = S.snap_to_stock([0.111], [0.100, 0.120])
        self.assertAlmostEqual(snapped[0], 0.120, places=9)

    def test_snapping_happens_before_expansion(self):
        # Sec 5.3: snapping after the solve would re-break every edge length
        # the solver just satisfied. The proof is that the snapped length --
        # not the drawn one -- is what drives the required edge.
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.107), (0.0, -0.360, 0.214)]
        result = S.extract_sticks(
            points, [(0, 1), (1, 2)], stock_lengths_m=[0.080, 0.100, 0.120]
        )
        for stick in result.sticks:
            self.assertAlmostEqual(mm(stick.length_m), 100.0, places=6)
            self.assertAlmostEqual(mm(stick.required_edge_m), 103.25, places=6)
        self.assertAlmostEqual(mm(result.snap_errors_m[0]), -7.0, places=6)

    def test_stock_mode_still_honours_the_exact_length_invariant(self):
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.107), (0.0, -0.360, 0.214)]
        result = S.extract_sticks(
            points, [(0, 1), (1, 2)], stock_lengths_m=[0.100]
        )
        for stick in result.sticks:
            self.assertAlmostEqual(v_dist(stick.base, stick.tip), 0.100, places=9)


# --- Sec 5.4: length limits --------------------------------------------------


class TestLengthLimits(unittest.TestCase):
    def test_hard_floor_comes_from_the_kinematics_module(self):
        # Sec 5.4 / D13: read the grip-offset floor from the shared module,
        # never hardcode it -- Phase 0 confirms the real numbers with a ruler.
        self.assertAlmostEqual(
            S.hard_min_stick_length_m(),
            kc.MIN_GRASP_OFFSET_M + kc.JAW_CONTACT_HALF_LENGTH_M, places=12
        )
        self.assertAlmostEqual(mm(S.hard_min_stick_length_m()), 35.0, places=6)

    def test_min_length_below_the_hard_floor_is_refused(self):
        with self.assertRaises(ValueError):
            S.extract_sticks(STACK_POINTS, STACK_EDGES, min_stick_length_m=0.030)

    def test_short_stick_is_reported_with_its_actual_length(self):
        # Explicit min_stick_length_m: decouples this test's intent (the
        # error message names the stick's actual length) from wherever the
        # shared default (STICK_LENGTH_RANGE_M[0]) happens to sit.
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.070)]
        result = S.extract_sticks(points, [(0, 1)], min_stick_length_m=0.100)
        codes = [(e[0], e[1]) for e in result.errors]
        self.assertIn((result.sticks[0].id, "too_short"), codes)
        self.assertIn("70.0 mm", result.errors[0][2])

    def test_long_stick_is_reported(self):
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.160)]
        result = S.extract_sticks(points, [(0, 1)])
        self.assertIn("too_long", [e[1] for e in result.errors])

    def test_limits_apply_to_the_stick_not_the_expanded_edge(self):
        # A 148 mm stick with two shared ends expands to a 154.5 mm edge.
        # The 150 mm maximum applies to the stick, so this must pass.
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.148), (0.0, -0.360, 0.296)]
        result = S.extract_sticks(points, [(0, 1), (1, 2)], max_stick_length_m=0.150)
        self.assertEqual([e[1] for e in result.errors], [])


# --- Multi-robot (docs/STATUS.md 2026-08-21/2026-08-23) -----------------------


class TestMultiRobotGeometry(unittest.TestCase):
    """core/sticks.py's mesh-expansion geometry (joint gaps, stock section,
    length limits/floor) is now robot-aware -- previously every one of these
    silently used so_arm_100's own numbers regardless of ``robot_id``."""

    def test_hard_floor_differs_per_robot(self):
        from so100_builder.kinematics.kr10_r900_2 import constants as krc

        so_arm_100_floor = S.hard_min_stick_length_m("so_arm_100")
        kr10_floor = S.hard_min_stick_length_m("kr10_r900_2")
        self.assertAlmostEqual(mm(so_arm_100_floor), 35.0, places=6)
        self.assertAlmostEqual(
            kr10_floor, krc.MIN_GRASP_OFFSET_M + krc.JAW_CONTACT_HALF_LENGTH_M, places=12)
        # Deliberately duplicates what the line above already derives: a
        # canary that fires when a re-vendor moves the vendored constants,
        # so the change is noticed and reasoned about rather than absorbed
        # silently. It has fired once already -- 18.0mm until the 2026-09-09
        # re-vendor, when kr10_r900_2's gripper fingers were swapped for
        # shorter meshes (JAW_CONTACT_HALF_LENGTH_M 8.0 -> 5.5mm,
        # MIN_GRASP_OFFSET_M 10.0 -> 7.14mm). Update it, do not delete it.
        self.assertAlmostEqual(mm(kr10_floor), 12.64, places=6)
        self.assertLess(kr10_floor, so_arm_100_floor)

    def test_safe_bound_is_the_lowest_of_every_registered_robot(self):
        # properties.py's own static widget min= bound -- must never exceed
        # ANY registered robot's own real floor, or it would silently block
        # a value that robot could otherwise legitimately use.
        self.assertAlmostEqual(
            S.safe_min_stick_length_bound_m(), S.hard_min_stick_length_m("kr10_r900_2"),
            places=12)

    def test_a_stick_between_the_two_robots_floors_is_refused_for_so_arm_100(self):
        # 25mm clears kr10_r900_2's own floor (12.64mm) but not
        # so_arm_100's (35mm) -- the check must use the robot it is actually
        # asked about.
        with self.assertRaises(ValueError):
            S.extract_sticks(STACK_POINTS, STACK_EDGES, robot_id="so_arm_100",
                             min_stick_length_m=0.025)

    def test_the_same_stick_is_accepted_for_kr10(self):
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES, robot_id="kr10_r900_2",
                                  min_stick_length_m=0.025)
        self.assertEqual([e[1] for e in result.errors if e[1] == "too_short"], [])

    def test_joint_allowance_defaults_to_the_selected_robots_own_value(self):
        from so100_builder.kinematics.kr10_r900_2 import constants as krc

        topology, _ = S.build_topology(U_POINTS, U_EDGES, U_IDS)
        so_arm_100_required, _ = S.required_edge_lengths(topology, [0.110] * 3)
        kr10_required, _ = S.required_edge_lengths(
            topology, [0.110] * 3, joint_allowance_m=krc.JOINT_ALLOWANCE_M)
        # kr10_r900_2's real joint allowance (1mm/end) is much smaller than
        # so_arm_100's (3.25mm/end, square-stock formula) -- confirms this
        # is a genuinely different number being used, not a coincidence.
        self.assertLess(kr10_required[1] - 0.110, so_arm_100_required[1] - 0.110)

    def test_extract_sticks_uses_kr10s_own_defaults_when_not_overridden(self):
        from so100_builder.kinematics.kr10_r900_2 import constants as krc

        result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS, robot_id="kr10_r900_2")
        top = next(s for s in result.sticks if s.id == "top")
        # Required edge = stick + allowance * shared_ends (both ends shared
        # for "top"): 110 + 2 * kr10's own 1mm allowance = 112mm, NOT
        # so_arm_100's 116.5mm.
        self.assertAlmostEqual(mm(top.required_edge_m),
                               110.0 + 2000.0 * krc.JOINT_ALLOWANCE_M, places=3)
        self.assertAlmostEqual(top.section_m[0], krc.STICK_SECTION_M, places=9)

    def test_extract_sticks_still_defaults_to_so_arm_100_when_robot_id_omitted(self):
        # Byte-for-byte unchanged default behaviour.
        result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS)
        top = next(s for s in result.sticks if s.id == "top")
        self.assertAlmostEqual(mm(top.required_edge_m), 116.5, places=3)
        self.assertAlmostEqual(top.section_m[0], kc.STICK_SECTION_M, places=9)


# --- Sec 5.2.3 / 6.2: warnings -----------------------------------------------


class TestWarnings(unittest.TestCase):
    def test_angle_allowance_matches_the_spec_table(self):
        for degrees, expected_mm in ((90.0, 3.2), (60.0, 5.6), (45.0, 7.8), (30.0, 12.0)):
            got = S.angle_allowance_m(math.radians(degrees))
            self.assertAlmostEqual(mm(got), expected_mm, delta=0.1,
                                   msg="theta=%.0f deg" % degrees)

    def test_shallow_joint_raises_tight_clearance(self):
        # A narrow V: an upright, then a stick folding back down at 40 deg to
        # it. The interior angle at the shared apex is 40 deg, which needs
        # ~8.9 mm of allowance where only 3.25 mm is set.
        theta = math.radians(40.0)
        apex = (0.0, -0.370, 0.110)
        points = [
            (0.0, -0.370, 0.0),
            apex,
            (0.0, apex[1] + 0.110 * math.sin(theta), apex[2] - 0.110 * math.cos(theta)),
        ]
        result = S.extract_sticks(points, [(0, 1), (1, 2)])
        self.assertIn("tight_clearance", [w[1] for w in result.warnings])
        for stick in result.sticks:
            self.assertIn("tight_clearance", stick.warnings)

    def test_perpendicular_joint_does_not_warn(self):
        result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS)
        self.assertNotIn("tight_clearance", [w[1] for w in result.warnings])

    def test_floating_component_is_a_hard_error(self):
        points = [(0.0, -0.360, 0.100), (0.0, -0.360, 0.210)]
        result = S.extract_sticks(points, [(0, 1)])
        self.assertIn("floating_component", [e[1] for e in result.errors])

    def test_raising_the_build_plate_to_its_lowest_point_grounds_it(self):
        # Same stick as above -- its own lowest point is Z=0.100mm; a build
        # plate physically raised to meet it is no longer "floating" (the
        # plate is a real, height-adjustable object, 2026-08-23).
        points = [(0.0, -0.360, 0.100), (0.0, -0.360, 0.210)]
        result = S.extract_sticks(points, [(0, 1)], ground_height_m=0.100)
        self.assertNotIn("floating_component", [e[1] for e in result.errors])

    def test_the_floating_component_message_names_its_own_lowest_point(self):
        points = [(0.0, -0.360, 0.100), (0.0, -0.360, 0.210)]
        result = S.extract_sticks(points, [(0, 1)])
        message = next(e[2] for e in result.errors if e[1] == "floating_component")
        self.assertIn("100.0 mm", message)

    def test_ground_required_false_never_reports_floating_component(self):
        # A stick with no path to the plate at ANY height -- held by
        # something this addon does not model (2026-08-23 user request:
        # "I would put a stick's base... on shapes that are not a flat
        # base"). No plate to check against means nothing is ever floating.
        points = [(0.0, -0.360, 0.100), (0.0, -0.360, 0.210)]
        result = S.extract_sticks(points, [(0, 1)], ground_required=False)
        self.assertNotIn("floating_component", [e[1] for e in result.errors])

    def test_ground_required_false_never_reports_below_plate(self):
        # A stick that dips to negative Z -- would be below_plate under the
        # default plate-at-Z=0 assumption, but there is no plate here.
        points = [(0.0, -0.360, -0.040), (0.0, -0.360, 0.070)]
        result = S.extract_sticks(points, [(0, 1)], ground_required=False)
        self.assertNotIn("below_plate", [e[1] for e in result.errors])

    def test_ground_required_false_solves_the_u_exactly_not_by_relaxation(self):
        # The inverted-U's two feet both sit at Z=0, so under the default
        # ground_required=True the plate behaves as a fourth edge and it
        # takes the relaxation path (TestWorkedInvertedU, above). With no
        # plate to test against, nothing is "grounded" (0 <= the exact
        # solver's own <=1 requirement), so the SAME acyclic mesh takes the
        # exact path instead, picking an arbitrary root.
        result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS, ground_required=False)
        self.assertEqual(result.expansion.method, "exact")
        self.assertEqual(result.errors, [])
        self.assertLess(result.expansion.max_residual_m, 1e-9)

    def test_high_valence_vertex_is_flagged(self):
        centre = (0.0, -0.370, 0.100)
        points = [centre, (0.110, -0.370, 0.100), (-0.110, -0.370, 0.100),
                  (0.0, -0.260, 0.100), (0.0, -0.370, 0.210), (0.0, -0.370, 0.0)]
        edges = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5)]
        result = S.extract_sticks(points, edges)
        self.assertTrue(any("high_valence" in s.warnings for s in result.sticks))

    def test_residual_over_tolerance_is_reported_per_edge(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_PIN)
        offenders = [e for e in result.errors if e[1] == "residual_too_large"]
        self.assertTrue(offenders)
        self.assertIn("will not physically fit", offenders[0][2])


# --- topology ----------------------------------------------------------------


class TestTopology(unittest.TestCase):
    def test_coincident_vertices_merge_into_one_joint(self):
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.110),
                  (0.0, -0.360, 0.1100001), (0.0, -0.360, 0.220)]
        topology, _ = S.build_topology(points, [(0, 1), (2, 3)])
        self.assertEqual(len(topology.positions), 3)
        self.assertEqual(topology.degree(1), 2)

    def test_distinct_vertices_do_not_merge(self):
        points = [(0.0, -0.360, 0.0), (0.0, -0.360, 0.110), (0.0, -0.360, 0.115)]
        topology, _ = S.build_topology(points, [(0, 1), (1, 2)])
        self.assertEqual(len(topology.positions), 3)

    def test_degenerate_edge_is_dropped_and_reported(self):
        points = [(0.0, -0.360, 0.0), (0.0, -0.3600001, 0.0)]
        result = S.extract_sticks(points, [(0, 1)])
        self.assertEqual(result.sticks, [])
        self.assertIn("degenerate_edge", [e[1] for e in result.errors])

    def test_components_are_found_independently(self):
        points = list(STACK_POINTS) + [(0.100, -0.360, 0.0), (0.100, -0.360, 0.110)]
        topology, _ = S.build_topology(points, STACK_EDGES + [(3, 4)])
        self.assertEqual(len(topology.components()), 2)


# --- Sec 5.1 step 4: which end is base ---------------------------------------


class TestBaseEndSelection(unittest.TestCase):
    def test_base_is_the_lower_end(self):
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES)
        for stick in result.sticks:
            self.assertLess(stick.base[2], stick.tip[2])

    def test_flip_override_swaps_the_ends(self):
        result = S.extract_sticks(STACK_POINTS, STACK_EDGES)
        target = result.sticks[0].id
        flipped = S.extract_sticks(STACK_POINTS, STACK_EDGES, flips={target: True})
        original = next(s for s in result.sticks if s.id == target)
        swapped = next(s for s in flipped.sticks if s.id == target)
        self.assertEqual(original.base, swapped.tip)
        self.assertEqual(original.tip, swapped.base)


# --- Sec 5.5: cut list -------------------------------------------------------


class TestCutList(unittest.TestCase):
    def test_cut_list_rows_carry_what_the_human_needs(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_SLIDE)
        rows = S.cut_list(result.sticks)
        self.assertEqual(len(rows), 12)
        self.assertEqual(set(rows[0]), {"index", "id", "length_mm", "order", "shared_ends"})
        self.assertAlmostEqual(rows[0]["length_mm"], 110.0, places=6)

    def test_tally_collapses_to_what_you_want_at_a_saw(self):
        result = S.extract_sticks(cube_points(), CUBE_EDGES, ground_mode=S.GROUND_SLIDE)
        self.assertEqual(S.cut_tally(result.sticks), [(110.0, 12)])

    def test_csv_has_a_header_and_one_row_per_stick(self):
        result = S.extract_sticks(U_POINTS, U_EDGES, U_IDS)
        lines = S.cut_list_csv(result.sticks).strip().split("\n")
        self.assertEqual(len(lines), 4)
        self.assertTrue(lines[0].startswith("index,stick_id,stick_length_mm"))


class TestEmptyInput(unittest.TestCase):
    def test_no_edges_is_not_an_error(self):
        result = S.extract_sticks([], [])
        self.assertEqual(result.sticks, [])
        self.assertEqual(result.errors, [])


if __name__ == "__main__":
    unittest.main()
