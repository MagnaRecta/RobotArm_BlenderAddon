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
        from so100_builder.kinematics.so_arm_100.constants import GRASP_OFFSET_M
        self.assertAlmostEqual((low[2] + high[2]) / 2.0, GRASP_OFFSET_M, places=9)
        self.assertAlmostEqual(high[2] - low[2], O.JAW_LENGTH_M, places=9)

    def test_grip_height_comes_from_the_kinematics_module(self):
        # Sec 5.4: never hardcode it here -- Phase 0's ruler check must
        # propagate automatically.
        from so100_builder.kinematics.so_arm_100.constants import GRASP_OFFSET_M
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
        from so100_builder.kinematics.so_arm_100.constants import GRASP_OFFSET_M
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


class TestLayerBoundaries(unittest.TestCase):
    """Sec 6 C2's "within a layer", added 2026-09-09 -- see
    ``layer_boundaries``'s own docstring for why raw Z was not one."""

    def layers(self, heights, tolerance_m=O.DEFAULT_LAYER_TOLERANCE_M):
        import bisect
        bounds = O.layer_boundaries(heights, tolerance_m)
        return [bisect.bisect_right(bounds, h) for h in heights]

    def test_no_heights_has_no_boundaries(self):
        self.assertEqual(O.layer_boundaries([]), [])

    def test_one_height_is_a_single_layer(self):
        self.assertEqual(O.layer_boundaries([0.05]), [])

    def test_expansion_noise_stays_one_layer(self):
        # The real failure: a physically flat course arrives as a spread of
        # distinct floats because mesh expansion nudged every vertex.
        course = [0.0662, 0.0664, 0.0665, 0.0667, 0.0671, 0.0673]
        self.assertEqual(self.layers(course), [0] * len(course))

    def test_separate_courses_land_in_separate_layers(self):
        heights = [0.0318, 0.0320, 0.0662, 0.0673, 0.1015, 0.1027]
        self.assertEqual(self.layers(heights), [0, 0, 1, 1, 2, 2])

    def test_layers_are_numbered_upward(self):
        self.assertEqual(self.layers([0.20, 0.10, 0.30]), [1, 0, 2])

    def test_a_layer_never_spans_more_than_the_tolerance(self):
        # Complete linkage, not nearest-neighbour: a shallow ramp of closely
        # spaced heights must NOT chain into one enormous layer.
        ramp = [i * 0.002 for i in range(50)]
        assigned = self.layers(ramp, tolerance_m=0.010)
        self.assertGreater(max(assigned), 1)
        for layer in set(assigned):
            members = [h for h, a in zip(ramp, assigned) if a == layer]
            self.assertLessEqual(max(members) - min(members), 0.010 + 1e-12)

    def test_a_bigger_tolerance_merges_courses(self):
        heights = [0.0318, 0.0662, 0.1015]
        self.assertEqual(self.layers(heights, tolerance_m=0.001), [0, 1, 2])
        self.assertEqual(self.layers(heights, tolerance_m=0.100), [0, 0, 0])


class TestAccessibilityWithinALayer(unittest.TestCase):
    """Sec 6 C2: a layer is built from the far side back toward the robot, so
    the arm never walls off somewhere it still has to reach at that height.

    Before layers existed this was untestable *and* untested: every stick's
    raw Z differed, so the height key never tied and this term never ran
    (2026-09-09 user report -- a voxel lattice built each course's outer ring
    before its core, which is the one order the arm cannot do)."""

    def _row_of_uprights(self, radii, z0=0.0):
        """Free-standing uprights at a range of distances from the robot,
        all topping out in the same layer."""
        points, edges = [], []
        for index, radius in enumerate(radii):
            points.extend([(0.0, -radius, z0), (0.0, -radius, z0 + L)])
            edges.append((2 * index, 2 * index + 1))
        return extract(points, edges)

    # so_arm_100 reaches a free-standing upright between roughly 0.32 m and
    # 0.46 m out, so every radius here stays inside that band -- a stick the
    # arm cannot reach at all is force-placed last, which would make these
    # pass for entirely the wrong reason.
    def test_the_far_side_of_a_layer_is_built_first(self):
        radii = [0.34, 0.38, 0.42, 0.46]
        result = self._row_of_uprights(radii)
        order = O.OrderSolver(result.sticks).solve()
        self.assertTrue(order.complete)
        self.assertEqual(order.forced, 0)

        placed = [
            math.hypot(entry.stick.base[0], entry.stick.base[1])
            for entry in order.ordered
        ]
        self.assertEqual(placed, sorted(placed, reverse=True))

    def test_the_layer_below_is_finished_before_the_one_above(self):
        # Height still wins over accessibility: the near stick of a layer
        # goes up before ANY stick of the layer above, never the other way
        # round -- otherwise "far side first" would climb the far column all
        # the way to the top and leave the near one to reach past.
        points = [
            (0.0, -0.34, 0.0), (0.0, -0.34, L),   # 0-1  near, layer 0
            (0.0, -0.42, 0.0), (0.0, -0.42, L),   # 2-3  far,  layer 0
            (0.0, -0.42, 2 * L),                  # 4    stacked on the far
        ]
        result = extract(points, [(0, 1), (2, 3), (3, 4)])
        solver = O.OrderSolver(result.sticks)
        order = solver.solve()
        self.assertTrue(order.complete)
        self.assertEqual(order.forced, 0)

        layers = [solver.layer_of(entry.stick) for entry in order.ordered]
        self.assertEqual(layers, sorted(layers))
        self.assertEqual(layers[-1], max(layers))


class TestSupportGraph(unittest.TestCase):
    def test_grounded_vertices_are_found_from_physical_ends(self):
        result = extract([(0.0, Y, 0.0), (0.0, Y, L)], [(0, 1)])
        grounded = O.grounded_vertices(result.sticks)
        self.assertEqual(len(grounded), 1)

    def test_ground_height_m_shifts_which_end_counts_as_grounded(self):
        # A stick sitting entirely above Z=0 has no grounded end by default,
        # but does once the plate is raised to meet its own base.
        result = extract([(0.0, Y, 0.100), (0.0, Y, 0.100 + L)], [(0, 1)])
        self.assertEqual(len(O.grounded_vertices(result.sticks)), 0)
        self.assertEqual(
            len(O.grounded_vertices(result.sticks, ground_height_m=0.100)), 1)

    def test_components_split_disconnected_designs(self):
        points = [(0.0, Y, 0.0), (0.0, Y, L), (0.09, Y, 0.0), (0.09, Y, L)]
        result = extract(points, [(0, 1), (2, 3)])
        self.assertEqual(len(O.components(result.sticks)), 2)

    def test_arbitrary_anchor_vertices_seeds_one_per_component(self):
        points = [(0.0, Y, 0.0), (0.0, Y, L), (0.09, Y, 0.0), (0.09, Y, L)]
        result = extract(points, [(0, 1), (2, 3)])
        anchors = O.arbitrary_anchor_vertices(result.sticks)
        # One anchor vertex for each of the two disconnected sticks -- no
        # plate involved, so this works identically whether the design
        # actually touches Z=0 or not.
        self.assertEqual(len(anchors), 2)
        for stick in result.sticks:
            self.assertIn(stick.v_base, anchors)

    def test_arbitrary_anchor_vertices_is_deterministic(self):
        points = [(0.0, Y, 0.0), (0.0, Y, L)]
        result = extract(points, [(0, 1)])
        first = O.arbitrary_anchor_vertices(result.sticks)
        second = O.arbitrary_anchor_vertices(result.sticks)
        self.assertEqual(first, second)

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

    def test_raising_the_build_plate_to_the_floating_group_grounds_it(self):
        # The "floating" pair's own lowest point is Z=0.100 -- raise the
        # plate to meet it and it stops being floating (2026-08-23: the
        # plate is a real, height-adjustable object).
        result = order_of(self._design(), ground_height_m=0.100)
        self.assertNotIn("floating", result.unordered)
        self.assertIn("floating", [entry.id for entry in result.ordered])
        codes = [code for _id, code, _msg in result.errors]
        self.assertNotIn(O.ERROR_FLOATING_COMPONENT, codes)

    def test_a_component_only_grounds_within_epsilon_of_the_raised_plate(self):
        # 0.099 misses the floating group's own Z=0.100 lowest point by 1mm,
        # outside the default 0.5mm epsilon -- still floating.
        result = order_of(self._design(), ground_height_m=0.099)
        self.assertIn("floating", result.unordered)

    def test_the_message_names_the_floating_groups_own_lowest_point(self):
        result = order_of(self._design())
        message = next(m for _i, c, m in result.errors
                       if c == O.ERROR_FLOATING_COMPONENT)
        self.assertIn("100.0 mm", message)

    def test_ground_required_false_orders_the_floating_part_too(self):
        # 2026-08-23 user request: a design held by something this addon
        # does not model (a jig, a non-flat fixture) -- no plate to check
        # against means nothing is ever floating, regardless of height.
        result = order_of(self._design(), ground_required=False)
        self.assertEqual(result.unordered, [])
        ids = [entry.id for entry in result.ordered]
        self.assertIn("grounded", ids)
        self.assertIn("floating", ids)
        codes = [code for _id, code, _msg in result.errors]
        self.assertNotIn(O.ERROR_FLOATING_COMPONENT, codes)

    def test_ground_required_false_still_produces_a_support_valid_order(self):
        # Relaxing the PLATE requirement must not relax the topological
        # one: every stick still has to attach to its own component's
        # arbitrary anchor or an earlier stick -- sticks are never placed
        # in thin air relative to EACH OTHER.
        design = self._design()
        available = set(O.arbitrary_anchor_vertices(design.sticks))
        result = order_of(design, ground_required=False)
        for entry in result.ordered:
            spec = entry.stick
            self.assertTrue(
                spec.v_base in available,
                "%s (order %d) attaches to nothing established yet"
                % (spec.id, entry.order))
            available.add(spec.v_base)
            available.add(spec.v_tip)

    def test_ground_required_false_warns_rather_than_silently_certifying(self):
        result = order_of(self._design(), ground_required=False)
        codes = [code for _id, code, _msg in result.warnings]
        self.assertIn(O.WARN_NO_BUILD_PLATE, codes)


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


class TestReplay(unittest.TestCase):
    """``OrderSolver.replay()`` -- 2026-08-23 user request: "After computing
    a build order, I want to be able to move the steps before or after its
    position." Reuses ``TestSimpleStack``'s straight 3-stick tower (a
    forced, unambiguous bottom-up order: a's base is the only grounded
    vertex, b needs a's tip, c needs b's tip) so every scenario below is
    deterministic -- no ties for the search to break arbitrarily."""

    def _design(self):
        points = [(0.0, Y, 0.0), (0.0, Y, L), (0.0, Y, 2 * L), (0.0, Y, 3 * L)]
        return extract(points, [(0, 1), (1, 2), (2, 3)], ["a", "b", "c"])

    def test_replaying_the_original_sequence_reproduces_it(self):
        solver = O.OrderSolver(self._design().sticks)
        result = solver.replay(["a", "b", "c"])
        self.assertEqual([entry.id for entry in result.ordered], ["a", "b", "c"])
        self.assertEqual(result.errors, [])
        self.assertEqual(result.forced, 0)
        assert_support_valid(self, result)

    def test_moving_a_stick_before_its_own_support_is_flagged(self):
        # "b" needs "a"'s tip -- moving it BEFORE "a" (the example's own
        # "move before" case) leaves it with nothing to attach to yet.
        solver = O.OrderSolver(self._design().sticks)
        result = solver.replay(["b", "a", "c"])
        self.assertEqual([entry.id for entry in result.ordered], ["b", "a", "c"])
        self.assertEqual(result.forced, 1)
        by_id = result.by_id()
        self.assertIsNotNone(by_id["b"].reason)
        self.assertIn("b", [stick_id for stick_id, _c, _m in result.errors])
        # "a" and "c" were not themselves moved and still validate cleanly.
        self.assertIsNone(by_id["a"].reason)
        self.assertIsNone(by_id["c"].reason)

    def test_a_later_stick_still_attaches_to_the_forced_ones_endpoints(self):
        # Even though "b" was placed with an error, its endpoints are real
        # 3D positions -- "c" (which needs b's tip) still attaches fine,
        # exactly as force_place() already does for the automatic search.
        solver = O.OrderSolver(self._design().sticks)
        result = solver.replay(["b", "a", "c"])
        self.assertTrue(
            all(entry.id != "c" or entry.reason is None for entry in result.ordered))

    def test_orientation_is_not_re_decided(self):
        # replay() must use each stick's EXISTING base/tip -- never silently
        # re-flip to make a broken position "work".
        design = self._design()
        original = {s.id: (s.v_base, s.v_tip) for s in design.sticks}
        solver = O.OrderSolver(design.sticks)
        result = solver.replay(["c", "b", "a"])
        for entry in result.ordered:
            self.assertEqual(
                (entry.stick.v_base, entry.stick.v_tip), original[entry.id])

    def test_a_stick_missing_from_the_sequence_is_still_placed(self):
        # "b" omitted -- new since the sequence was last stored, say --
        # must not simply vanish from the result.
        solver = O.OrderSolver(self._design().sticks)
        result = solver.replay(["a", "c"])
        self.assertEqual(len(result.ordered), 3)
        self.assertIn("b", [entry.id for entry in result.ordered])

    def test_replay_never_backtracks(self):
        solver = O.OrderSolver(self._design().sticks)
        result = solver.replay(["b", "a", "c"])
        self.assertEqual(result.backtracks, 0)

    def test_a_floating_component_stays_excluded_even_if_named_in_the_sequence(self):
        points = [
            (0.0, Y, 0.0), (0.0, Y, L),               # grounded
            (0.09, Y, 0.100), (0.09, Y, 0.210),       # floating
        ]
        design = extract(points, [(0, 1), (2, 3)], ["grounded", "floating"])
        solver = O.OrderSolver(design.sticks)
        # Even asked for first, a genuinely floating component cannot be
        # placed -- excluded up front in __init__, same as solve().
        result = solver.replay(["floating", "grounded"])
        self.assertIn("floating", result.unordered)
        self.assertNotIn("floating", [entry.id for entry in result.ordered])
        self.assertIn("grounded", [entry.id for entry in result.ordered])


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


# --- Multi-robot (docs/STATUS.md 2026-08-21/2026-08-23) -----------------------


class TestKr10BuildOrder(unittest.TestCase):
    """Regression test for the exact bug report that motivated threading
    robot_id through OrderSolver: every reachability check inside the
    solver used to run so_arm_100's own kinematics regardless of the
    selected robot, so a kr10_r900_2 design sitting well within THAT
    robot's own build volume -- but well beyond so_arm_100's own, much
    smaller, documented reach (max validated Y is -450mm) -- would report
    every candidate unreachable and fail to produce any order at all, with
    no clue that the wrong robot's kinematics was actually being checked.
    """

    # Deep in kr10_r900_2's own build zone (Y in [-600, -300] mm,
    # core/robots.py) but well past so_arm_100's own max documented reach
    # (Y = -450mm) -- deliberately NOT in the two robots' overlap region,
    # so a pass here can only be explained by the solver genuinely using
    # kr10_r900_2's own kinematics, not so_arm_100's by accident.
    KUKA_Y = -0.550

    def _design(self):
        return ladder(3, y=self.KUKA_Y)

    def test_every_stick_is_ordered_for_kr10(self):
        design = self._design()
        result = order_of(design, robot_id="kr10_r900_2")
        self.assertEqual(len(result.ordered), len(design.sticks), result.summary())
        self.assertTrue(result.complete, result.summary())
        self.assertEqual(result.forced, 0, result.summary())

    def test_the_same_design_is_hopeless_for_so_arm_100(self):
        # Confirms the fixture actually distinguishes the two robots --
        # without that, the test above wouldn't prove anything. so_arm_100
        # can't reach any of it, so every stick gets force-placed with a
        # reachability error rather than genuinely ordered (`.complete`
        # only reflects "nothing was excluded as floating", not "no
        # errors" -- forced placements still count as complete).
        design = self._design()
        result = order_of(design)  # default robot_id: so_arm_100
        self.assertEqual(result.forced, len(design.sticks), result.summary())

    def test_the_order_is_support_valid_for_kr10(self):
        assert_support_valid(self, order_of(self._design(), robot_id="kr10_r900_2"))

    def test_cost_heuristic_uses_kr10s_own_shoulder_point_not_so_arm_100s(self):
        # _horizontal_reach's reference point differs between the two
        # robots' own CHAIN[0][1] -- confirm the solver actually picked
        # kr10_r900_2's, not so_arm_100's (which would silently still "work"
        # here since it only affects ordering priority, not correctness).
        from so100_builder.core import robots as core_robots

        solver = O.OrderSolver(self._design().sticks, robot_id="kr10_r900_2")
        self.assertEqual(
            solver._shoulder_axis_point,
            core_robots.get_robot("kr10_r900_2").kinematics.constants.CHAIN[0][1],
        )


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
