"""core/order.py -- BLENDER_ADDON_PLAN.md Sec 6, Phase C.

Phase C is done when "a multi-layer test mesh produces an order that is
support-valid and fully buildable, and a deliberately-floating component is
correctly rejected" -- ``TestMultiLayerTower`` and ``TestFloatingComponent``
below are those two.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core import order as O  # noqa: E402
from so100_builder.core import sticks as core_sticks  # noqa: E402
from so100_builder.core import validate as V  # noqa: E402

MM = 0.001
Y = -0.370
L = 0.110


GAP = 0.0065


def extract(points, edges, ids=None, **kwargs):
    kwargs.setdefault("ground_mode", core_sticks.GROUND_SLIDE)
    return core_sticks.extract_sticks(points, edges, ids, **kwargs)


def ladder(levels, half=0.055, y=Y, length=L):
    """Stacked inverted-Us: two tangentially-separated uprights per level
    plus a tangential rung at each level above the plate.

    Deliberately all-tangential horizontals. A *radial* horizontal (one
    pointing at or away from the robot) needs the tool vertical, which
    Sec 9.4 says costs ~150 mm of reach -- verified here: a square-ring
    tower puts exactly its two radial rungs out of reach at every height
    tried, so a ring is the wrong shape for a "fully buildable" fixture.
    """
    points = []
    for level in range(levels):
        z = level * (length + GAP)
        points.extend([(-half, y, z), (half, y, z)])
    edges = []
    for level in range(levels - 1):
        base = level * 2
        edges.append((base, base + 2))
        edges.append((base + 1, base + 3))
    for level in range(1, levels):
        base = level * 2
        edges.append((base, base + 1))
    return extract(points, edges)


def order_of(result, **kwargs):
    return O.compute_order(result.sticks, **kwargs)


def assert_support_valid(testcase, order_result, ground_epsilon_m=0.0005):
    """C1: every stick's base must be grounded or at a vertex an EARLIER
    stick already established. This is the invariant the whole phase exists
    to guarantee, so it is checked structurally rather than trusted."""
    available = set()
    for stick in order_result.ordered:
        spec = stick.stick
        if spec.base[2] > ground_epsilon_m and spec.v_base not in available:
            testcase.fail(
                "%s (order %d) has base at vertex %d which nothing earlier "
                "established and which is not on the plate"
                % (spec.id, stick.order, spec.v_base))
        available.add(spec.v_base)
        available.add(spec.v_tip)


# --- geometry primitives ------------------------------------------------------


class TestSegmentDistance(unittest.TestCase):
    def test_parallel_segments(self):
        d = O.segment_distance((0, 0, 0), (1, 0, 0), (0, 0.5, 0), (1, 0.5, 0))
        self.assertAlmostEqual(d, 0.5, places=9)

    def test_crossing_segments_touch(self):
        d = O.segment_distance((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
        self.assertAlmostEqual(d, 0.0, places=9)

    def test_skew_segments(self):
        d = O.segment_distance((0, 0, 0), (1, 0, 0), (0, 0, 1), (0, 1, 1))
        self.assertAlmostEqual(d, 1.0, places=9)

    def test_endpoint_to_endpoint(self):
        d = O.segment_distance((0, 0, 0), (1, 0, 0), (3, 0, 0), (4, 0, 0))
        self.assertAlmostEqual(d, 2.0, places=9)

    def test_degenerate_point_segments(self):
        # A "segment" can legitimately be a point; must not divide by zero.
        d = O.segment_distance((0, 0, 0), (0, 0, 0), (3, 4, 0), (3, 4, 0))
        self.assertAlmostEqual(d, 5.0, places=9)

    def test_is_symmetric(self):
        a, b = (0.1, 0.2, 0.3), (0.5, -0.2, 0.9)
        c, d = (-0.3, 0.4, 0.1), (0.7, 0.1, -0.2)
        self.assertAlmostEqual(O.segment_distance(a, b, c, d),
                               O.segment_distance(c, d, a, b), places=12)


class TestJawGeometry(unittest.TestCase):
    def test_jaw_segment_is_centred_on_the_grip_point(self):
        stick = core_sticks.StickSpec(
            id="s", base=(0.0, Y, 0.0), tip=(0.0, Y, L), length_m=L, shared_ends=0)
        low, high = O.jaw_segment(stick)
        from so100_builder.kinematics.constants import GRASP_OFFSET_M
        self.assertAlmostEqual((low[2] + high[2]) / 2.0, GRASP_OFFSET_M, places=9)
        self.assertAlmostEqual(high[2] - low[2], O.JAW_LENGTH_M, places=9)

    def test_grip_height_comes_from_the_kinematics_module(self):
        # Sec 5.4: never hardcode it here -- Phase 0's ruler check must
        # propagate automatically.
        from so100_builder.kinematics.constants import GRASP_OFFSET_M
        stick = core_sticks.StickSpec(
            id="s", base=(0.0, Y, 0.0), tip=(0.0, Y, L), length_m=L, shared_ends=0)
        low, _high = O.jaw_segment(stick, grasp_offset_m=GRASP_OFFSET_M + 0.010)
        self.assertAlmostEqual(low[2], GRASP_OFFSET_M + 0.010 - O.JAW_LENGTH_M / 2.0,
                               places=9)

    def test_no_placed_sticks_is_always_clear(self):
        stick = core_sticks.StickSpec(
            id="s", base=(0.0, Y, 0.0), tip=(0.0, Y, L), length_m=L, shared_ends=0)
        ok, _gap, offender = O.jaw_clearance(stick, [])
        self.assertTrue(ok)
        self.assertIsNone(offender)

    def test_a_parallel_neighbour_at_the_grip_height_is_a_clash(self):
        stick = core_sticks.StickSpec(
            id="new", base=(0.0, Y, 0.0), tip=(0.0, Y, L), length_m=L, shared_ends=0)
        # 5 mm away, running alongside -- well inside jaw half-width + section.
        neighbour = core_sticks.StickSpec(
            id="old", base=(0.005, Y, 0.0), tip=(0.005, Y, L), length_m=L, shared_ends=0)
        ok, gap, offender = O.jaw_clearance(stick, [neighbour])
        self.assertFalse(ok)
        self.assertEqual(offender, "old")
        self.assertAlmostEqual(gap, 0.005, places=6)

    def test_a_distant_neighbour_is_clear(self):
        stick = core_sticks.StickSpec(
            id="new", base=(0.0, Y, 0.0), tip=(0.0, Y, L), length_m=L, shared_ends=0)
        far = core_sticks.StickSpec(
            id="old", base=(0.10, Y, 0.0), tip=(0.10, Y, L), length_m=L, shared_ends=0)
        ok, _gap, _offender = O.jaw_clearance(stick, [far])
        self.assertTrue(ok)

    def test_a_stick_stacked_directly_below_does_not_clash(self):
        # The supporting stick a new one seats on must NOT read as a clash:
        # the jaws grip ~51 mm up, far above the joint.
        lower = core_sticks.StickSpec(
            id="lower", base=(0.0, Y, 0.0), tip=(0.0, Y, L), length_m=L, shared_ends=1)
        upper = core_sticks.StickSpec(
            id="upper", base=(0.0, Y, L + 0.0065), tip=(0.0, Y, 2 * L + 0.0065),
            length_m=L, shared_ends=1)
        ok, _gap, _offender = O.jaw_clearance(upper, [lower])
        self.assertTrue(ok)

    def test_the_sweep_catches_an_obstacle_only_above_the_final_pose(self):
        # An obstacle the jaws pass THROUGH on the way down, but which is
        # clear at the final pose, must still be caught (Sec 7.2 approach).
        stick = core_sticks.StickSpec(
            id="new", base=(0.0, Y, 0.0), tip=(0.0, Y, L), length_m=L, shared_ends=0)
        from so100_builder.kinematics.constants import GRASP_OFFSET_M
        overhead = core_sticks.StickSpec(
            id="over",
            base=(-0.05, Y, GRASP_OFFSET_M + O.APPROACH_CLEARANCE_M),
            tip=(0.05, Y, GRASP_OFFSET_M + O.APPROACH_CLEARANCE_M),
            length_m=0.10, shared_ends=0)
        ok_swept, _gap, _offender = O.jaw_clearance(stick, [overhead])
        ok_final_only, _g2, _o2 = O.jaw_clearance(
            stick, [overhead], approach_clearance_m=0.0, approach_samples=1)
        self.assertTrue(ok_final_only, "final pose alone should be clear here")
        self.assertFalse(ok_swept, "the approach sweep should catch it")


# --- support graph ------------------------------------------------------------


class TestSupportGraph(unittest.TestCase):
    def test_grounded_vertices_are_found_from_physical_ends(self):
        result = extract([(0.0, Y, 0.0), (0.0, Y, L)], [(0, 1)])
        grounded = O.grounded_vertices(result.sticks)
        self.assertEqual(len(grounded), 1)

    def test_components_split_disconnected_designs(self):
        points = [(0.0, Y, 0.0), (0.0, Y, L), (0.09, Y, 0.0), (0.09, Y, L)]
        result = extract(points, [(0, 1), (2, 3)])
        self.assertEqual(len(O.components(result.sticks)), 2)

    def test_a_connected_design_is_one_component(self):
        points = [(-0.055, Y, 0.0), (-0.055, Y, L), (0.055, Y, L), (0.055, Y, 0.0)]
        result = extract(points, [(0, 1), (1, 2), (2, 3)])
        self.assertEqual(len(O.components(result.sticks)), 1)


class TestFloatingComponent(unittest.TestCase):
    """Phase C "done when", half two: a deliberately-floating component is
    correctly rejected."""

    def _design(self):
        # A grounded upright, plus a completely detached pair floating in
        # mid-air with no path to the plate.
        points = [
            (0.0, Y, 0.0), (0.0, Y, L),                 # grounded
            (0.09, Y, 0.100), (0.09, Y, 0.210),         # floating
        ]
        return extract(points, [(0, 1), (2, 3)], ["grounded", "floating"])

    def test_the_floating_stick_is_reported_as_an_error(self):
        result = order_of(self._design())
        codes = [code for _id, code, _msg in result.errors]
        self.assertIn(O.ERROR_FLOATING_COMPONENT, codes)

    def test_the_floating_stick_is_named_in_the_message(self):
        result = order_of(self._design())
        message = next(m for _i, c, m in result.errors
                       if c == O.ERROR_FLOATING_COMPONENT)
        self.assertIn("floating", message)

    def test_the_floating_stick_is_not_ordered(self):
        result = order_of(self._design())
        self.assertIn("floating", result.unordered)
        self.assertNotIn("floating", [entry.id for entry in result.ordered])

    def test_the_grounded_part_is_still_ordered(self):
        result = order_of(self._design())
        self.assertIn("grounded", [entry.id for entry in result.ordered])

    def test_the_result_is_not_marked_complete(self):
        result = order_of(self._design())
        self.assertFalse(result.complete)


# --- ordering -----------------------------------------------------------------


class TestSimpleStack(unittest.TestCase):
    """Three sticks stacked vertically: the order is forced, bottom-up."""

    def _design(self):
        points = [(0.0, Y, 0.0), (0.0, Y, L), (0.0, Y, 2 * L), (0.0, Y, 3 * L)]
        return extract(points, [(0, 1), (1, 2), (2, 3)], ["a", "b", "c"])

    def test_all_three_are_ordered_bottom_up(self):
        result = order_of(self._design())
        self.assertEqual([entry.id for entry in result.ordered], ["a", "b", "c"])

    def test_the_order_is_support_valid(self):
        result = order_of(self._design())
        assert_support_valid(self, result)

    def test_order_indices_are_dense_and_match_position(self):
        # BRIDGE_PROTOCOL.md A.2: "`order` must be dense and match array
        # position".
        result = order_of(self._design())
        self.assertEqual([entry.order for entry in result.ordered],
                         list(range(len(result.ordered))))

    def test_no_backtracking_was_needed(self):
        result = order_of(self._design())
        self.assertEqual(result.backtracks, 0)

    def test_the_result_is_complete(self):
        result = order_of(self._design())
        self.assertTrue(result.complete)
        self.assertEqual(result.unordered, [])


class TestInvertedU(unittest.TestCase):
    """The user's own shape. Both uprights must precede the top, and the top
    is a loop closure once they are up."""

    def _design(self):
        points = [(-0.055, Y, 0.0), (-0.055, Y, L), (0.055, Y, L), (0.055, Y, 0.0)]
        return extract(points, [(0, 1), (1, 2), (2, 3)],
                       ["upright_l", "top", "upright_r"])

    def test_the_top_is_placed_last(self):
        result = order_of(self._design())
        self.assertEqual(result.ordered[-1].id, "top")

    def test_the_order_is_support_valid(self):
        assert_support_valid(self, order_of(self._design()))

    def test_the_top_is_flagged_as_a_loop_closure(self):
        result = order_of(self._design())
        top = result.by_id()["top"]
        self.assertIn(O.WARN_LOOP_CLOSURE, top.warnings)

    def test_every_stick_is_ordered_and_complete(self):
        result = order_of(self._design())
        self.assertEqual(len(result.ordered), 3)
        self.assertTrue(result.complete)

    def test_the_top_gets_the_orientation_that_actually_validates(self):
        # Both ends are supported by the time the top goes on, so the solver
        # is free to choose -- and must choose the reachable one. This is the
        # Finding 6 case, reproduced structurally rather than by auto-flip.
        result = order_of(self._design())
        top = result.by_id()["top"]
        self.assertTrue(V.validate_stick(top.stick).buildable)


class TestMultiLayerTower(unittest.TestCase):
    """Phase C "done when", half one: a multi-layer mesh produces an order
    that is support-valid and fully buildable."""

    def _design(self):
        # Three levels: 4 uprights and 2 rungs, so the solver has genuine
        # choices at each layer rather than one forced chain.
        return ladder(3)

    def test_every_stick_is_ordered(self):
        design = self._design()
        result = order_of(design)
        self.assertEqual(len(result.ordered), len(design.sticks))

    def test_the_order_is_support_valid(self):
        assert_support_valid(self, order_of(self._design()))

    def test_lower_layers_come_before_upper_layers(self):
        # C2: build bottom-up. The cost function's primary key is max_z, so
        # the heights of successive placements must be non-decreasing.
        result = order_of(self._design())
        heights = [max(e.stick.base[2], e.stick.tip[2]) for e in result.ordered]
        for earlier, later in zip(heights, heights[1:]):
            self.assertLessEqual(earlier, later + 1e-9,
                                 "build order jumps back down: %r" % (heights,))

    def test_the_result_is_complete_with_no_floating_errors(self):
        result = order_of(self._design())
        self.assertTrue(result.complete, result.summary())
        self.assertEqual(
            [c for _i, c, _m in result.errors if c == O.ERROR_FLOATING_COMPONENT], [])

    def test_no_stick_had_to_be_force_placed(self):
        result = order_of(self._design())
        self.assertEqual(result.forced, 0, result.summary())


# --- Sec 6.2 warnings ---------------------------------------------------------


class TestPlacementWarnings(unittest.TestCase):
    def test_a_leaning_one_anchor_stick_is_a_cantilever(self):
        # 45 deg from vertical, glued only at its base.
        lean = L * math.sin(math.radians(45.0))
        rise = L * math.cos(math.radians(45.0))
        points = [(0.0, Y, 0.0), (0.0, Y, L), (lean, Y, L + rise)]
        result = order_of(extract(points, [(0, 1), (1, 2)], ["upright", "arm"]))
        arm = result.by_id()["arm"]
        self.assertIn(O.WARN_CANTILEVER, arm.warnings)

    def test_a_vertical_one_anchor_stick_is_not_a_cantilever(self):
        points = [(0.0, Y, 0.0), (0.0, Y, L), (0.0, Y, 2 * L)]
        result = order_of(extract(points, [(0, 1), (1, 2)], ["lower", "upper"]))
        self.assertNotIn(O.WARN_CANTILEVER, result.by_id()["upper"].warnings)

    def test_a_two_anchor_stick_is_a_loop_closure_not_a_cantilever(self):
        points = [(-0.055, Y, 0.0), (-0.055, Y, L), (0.055, Y, L), (0.055, Y, 0.0)]
        result = order_of(extract(points, [(0, 1), (1, 2), (2, 3)],
                                  ["l", "top", "r"]))
        top = result.by_id()["top"]
        self.assertIn(O.WARN_LOOP_CLOSURE, top.warnings)
        self.assertNotIn(O.WARN_CANTILEVER, top.warnings)


# --- chunking (constraint B3) -------------------------------------------------


class TestChunkedExecution(unittest.TestCase):
    def _design(self):
        return ladder(3)

    def test_step_returns_false_until_done_then_true(self):
        solver = O.OrderSolver(self._design().sticks)
        seen_false = False
        for _ in range(2000):
            if solver.step(budget=1):
                break
            seen_false = True
        self.assertTrue(solver.done)
        self.assertTrue(seen_false, "a budget of 1 should not finish immediately")

    def test_a_tiny_budget_reaches_the_same_answer_as_one_shot(self):
        chunked_solver = O.OrderSolver(self._design().sticks)
        while not chunked_solver.step(budget=1):
            pass
        one_shot = O.compute_order(self._design().sticks)
        self.assertEqual([e.id for e in chunked_solver.result.ordered],
                         [e.id for e in one_shot.ordered])

    def test_chunking_does_not_change_the_warnings_either(self):
        chunked_solver = O.OrderSolver(self._design().sticks)
        while not chunked_solver.step(budget=1):
            pass
        one_shot = O.compute_order(self._design().sticks)
        self.assertEqual(chunked_solver.result.warnings, one_shot.warnings)
        self.assertEqual(chunked_solver.result.errors, one_shot.errors)

    def test_progress_reaches_one(self):
        # NOT asserted monotonic: backtracking genuinely un-places sticks,
        # so progress can move backwards. See OrderSolver.progress.
        solver = O.OrderSolver(self._design().sticks)
        while not solver.step(budget=2):
            self.assertGreaterEqual(solver.progress, 0.0)
            self.assertLessEqual(solver.progress, 1.0)
        self.assertAlmostEqual(solver.progress, 1.0, places=9)

    def test_stepping_after_done_is_harmless(self):
        solver = O.OrderSolver(self._design().sticks)
        solver.solve()
        placed = len(solver.result.ordered)
        self.assertTrue(solver.step())
        self.assertEqual(len(solver.result.ordered), placed)

    def test_an_empty_design_finishes_immediately(self):
        solver = O.OrderSolver([])
        self.assertTrue(solver.step())
        self.assertTrue(solver.result.complete)
        self.assertEqual(solver.result.ordered, [])


class TestUnreachableStickDoesNotThrash(unittest.TestCase):
    """Phase B reachability depends only on a stick's own geometry, so if
    every orientation of a stick fails, no build order can rescue it and
    backtracking over it is pure waste. Found while building this phase: a
    4-level ladder (whose top is genuinely out of reach) burned the entire
    500-backtrack budget before giving up. See OrderSolver._is_hopeless."""

    def test_an_out_of_reach_top_does_not_exhaust_the_backtrack_budget(self):
        result = O.compute_order(ladder(4).sticks)
        self.assertLess(result.backtracks, 50,
                        "solver thrashed: %d backtracks" % result.backtracks)

    def test_it_still_orders_everything_and_reports_the_bad_stick(self):
        result = O.compute_order(ladder(4).sticks)
        design = ladder(4)
        self.assertEqual(len(result.ordered), len(design.sticks))
        self.assertGreater(result.forced, 0)
        self.assertIn(O.ERROR_FORCED_PLACEMENT,
                      [code for _id, code, _msg in result.errors])

    def test_the_reachable_lower_levels_are_untouched_by_the_bad_top(self):
        result = O.compute_order(ladder(4).sticks)
        forced_ids = {stick_id for stick_id, code, _m in result.errors
                      if code == O.ERROR_FORCED_PLACEMENT}
        for entry in result.ordered:
            if entry.id in forced_ids:
                continue
            self.assertTrue(V.validate_stick(entry.stick).buildable)


class TestPerformance(unittest.TestCase):
    def test_a_realistic_design_orders_quickly(self):
        # Constraint B3's real target: ordering is the phase with an IK call
        # in its inner loop, so this is the one that needed chunking.
        import time

        points, index = [], {}
        bays, levels, rows = 3, 3, 2
        for row in range(rows):
            y = Y + row * (L + GAP)
            for bay in range(bays + 1):
                x = (bay - bays / 2.0) * L
                for level in range(levels):
                    index[(row, bay, level)] = len(points)
                    points.append((x, y, level * (L + GAP)))
        edges = []
        for row in range(rows):
            for bay in range(bays + 1):
                for level in range(levels - 1):
                    edges.append((index[(row, bay, level)], index[(row, bay, level + 1)]))
            for bay in range(bays):
                for level in range(1, levels):
                    edges.append((index[(row, bay, level)], index[(row, bay + 1, level)]))
        design = extract(points, edges)

        start = time.perf_counter()
        result = O.compute_order(design.sticks)
        elapsed = time.perf_counter() - start
        self.assertEqual(len(result.ordered), len(design.sticks))
        self.assertGreaterEqual(len(design.sticks), 24, "fixture got smaller")
        self.assertLess(elapsed, 10.0, "ordering took %.2fs" % elapsed)


if __name__ == "__main__":
    unittest.main()
